#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# HazemeyerIocaste_PS.py
# This file is part of tango-ds (http://sourceforge.net/projects/tango-ds/)
#
# tango-ds is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# tango-ds is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with tango-ds.  If not, see <http://www.gnu.org/licenses/>.

'''TANGO DS for Hazemeyer power supplies controlled by iocaste control unit
   over Modbus/TCP.
'''

META = u"""
    $URL: https://svn.code.sf.net/p/tango-ds/code/Servers/PowerSupply/HazemeyerIocaste_PS/tags/1.11.1/HazemeyerIocaste_PS.py $
    $LastChangedBy: sergiblanch $
    $Date: 2012-11-09 16:51:17 +0100 (Fri, 09 Nov 2012) $
    $Rev: 5765 $
    License: GPL3+
    Author: Lothar Krause <lkrause@cells.es>
""".encode('latin1')

# Python standard library
import traceback
from pprint import pformat, pprint
from copy import deepcopy
import struct
import logging
from functools import partial
from types import StringType
from time import time, sleep,strftime
from numpy import ndarray
import subprocess
import re
import array

# TANGO Imports
from PyTango import DevVoid, DevString, DevState, DevFailed, DevDouble, \
    Except, DevShort, Util, DevLong, DevUShort, \
    DevVarStringArray, DevVarShortArray, DeviceProxy, AttrQuality, SCALAR, \
    SPECTRUM, READ, READ_WRITE, Database, Device_3Impl, DeviceClass, \
    DevVarLongArray, DevBoolean, DevLong64, DevULong, DispLevel

# extra Imports
from PowerSupply.standard import (AQ_VALID, AQ_ALARM, AQ_INVALID, AQ_CHANGING,
    PS_Exception, PowerSupply, PowerSupply_Class,CommandExc, AttrExc,
    gen_cmd_list,  gen_property_list, gen_attr_list, tango_main)
from PowerSupply.util import *

# HazemeyerIocaste_PS
from state import *

maxStepCyclingRetries = 3

class HazemeyerIocaste_PS(PowerSupply):
    """Device server will allow to interact with ALBA HazemeyerIocaste_PS connected using Modbus over TCP.
    """

    ### Device Properties ###
    ModbusDevice = None
    ModbusTimeout = None
    Type = None

    # maps each attribute name to a function that returns the value of the
    # corresponding attributes. these attributes are then read in UpdateState()
    # and change events generated
    _READ = []
    PUSHED_ATTR = ( 'State', 'Status', 'Current', 'CurrentSetpoint', 'Voltage',
    )
    # kludge is used to enable / disabled the workaround for apply-setpoint-bug
    kludge = True

    #
    _modbus_err_count = 0

    def __init__(self, cl, name):
        PowerSupply.__init__(self, cl, name)
        self.init_device(cl, name)
        self.commandWait = None
        #kludge auxiliars for commands
        self.commandKludge = [None,None]#[from,to]
        self.kludgeCommandTimestamp = None
        self.kludgeCommandWait = {'OffStandby':7,
                                  'StandbyOn':1,
                                  'OnStandby':1,
                                  'StandbyOff':1}#seconds
        #the index on the nested lists says the number of retries needed
        self.kludgeCommandCounter = {'POWEROFF':[0,0],'STANDBY':[0,0],'POWERON':[0,0]}
        self.kludgeCommandCurrentTry = 0
        #kludge auxiliars for setpoints
        self.setpointKludge = False
        self.kludgeSetpointTimestamp = None
        self.kludgeSetpointWait = 0.5#s
        self.kludgeSetpointCounter = [0,0]
        self.kludgeSetpointCurrentTry = 0

    def init_device(self, cl=None, name=None):
        PowerSupply.init_device(self, cl, name)

        # cache contains what the DS considers to be the current read values
        # of each attribute, updated from UpdateState
        self.cache = {}
        self.vdq_cache = {}
        # initializes internal modbus data to fill pattern
        self.ALL = [ USHORT_FILL ] * NREGISTER
        # all bits are False, neither power on nor off
        self.ALL[ADDR_STATE] = 0

        self._next_read_time = time() #< when next read might be attempted
        self._read_t = time() #< when last successful read was done
        self._pend_oo(None)

        #cycling vbles
        self.cache['_isCycling'] = False
        self.cache['_cycling'] = {}
        #configuration
        self.nCyclingSteps = 6
        self.cache['_cycling']['minValue'] = None
        self.cache['_cycling']['maxValue'] = None
        self.cache['_cycling']['finalValue'] = None
        #step description
        self.cache['_cycling']['stepIdx'] = None
        #motion area
        self.cache['_cycling']['stepIsMoving'] = False
        self.cache['_cycling']['stepRampStartTS'] = None
        self.cache['_cycling']['stepRampRetry'] = 0
        self.cache['_cycling']['expectedRampTime'] = None
        #wait area
        self.cache['_cycling']['stepWaitStartTS'] = None
        self.cache['_cycling']['expectedWaitTime'] = 30#s FIXME: this value should be a property, but now can be mod using Exec()
        
        #self.cache['_cycling_stepWaitTimestamp'] = None
        #self.cache['_cycling_rampStartTimestamp'] = None
        #self.cache['_cycling_expectedRampTime'] = None
        #self.cache['_cycling_stepWaitTime'] = 30#s FIXME: this value should be a property, but now can be mod using Exec()
        self.cache['_cycling']['logger_hackish'] = 0
        self.cache['_cycling']['alarm'] = False

        # initializes connection
        try:
            self.modbus = DeviceProxy(self.ModbusDevice)
            self.modbus.set_timeout_millis(self.ModbusTimeout)
        except Exception,e:
            self.modbus = None
            self.log.debug('Exception in modbus bind: %s'%e)

        # guesses Type device property if not set
        if not self.Type:
            self.Type = self.get_name().rpartition('/')[-1][0]
            self.log.info('guessing Type is %r' % self.Type)

        # rkudge in order to repeat the apply n times
        #self.KludgeApplyRepeat = 5

        #  customize external interlock messages
        self.STAT = IocasteStateLogic(self, self.Type)
        msg = self.STAT.msg_interlock1
        for i,x in enumerate(self.STAT.xi):
            ianame = "Interlock%d" % (i+1)
            msg[x] = getattr(self, ianame)

        for aname,rfun in self._READ:
            self.set_change_event(aname, True)
        self.set_change_event('isCycling',True)
        self.STAT.INITIALIZED()

    def _check_ramp(self):
        '''Checks value of current ramp, when set to 0.0 the value of the
           'local' mode ramp is set. This function is called in UpdateState.
        '''
        C = self.cache
        if not 'CurrentRamp' in C:
            return
        if C["CurrentRamp"]==0.0:# and 'CurrentRampLocal' in C:
            msg = 'remote current ramp not configured (using Alba\'s default value %6.3f)'%(ALBAS_DEFAULT_CURRENT_RAMP)
            self.log.info(msg)
            self._write_float(ADDR_CURRENT_RAMP,ALBAS_DEFAULT_CURRENT_RAMP)
            

    def _check_setpoint(self):
        '''Check if the value of the setpoint have been applied by checking the
           known error ways. By now, when "current setpoint local" don't show
           what has been send to "current set" but "current setpoint readback"
           confirms.
        '''
        if self.kludge and bit(self.ALL[ADDR_STATE],BIT_POWERON) and \
           self.cache.get('RemoteMode'):
            if not 'CurrentSetpoint' in self.cache: return
            currentSetpointLocal = self._get_float(ADDR_IREF_READ)
            currentSetpointReadback = self._get_float(ADDR_IREF_REMOTE_READ)
            currentSetpoint = self.cache['CurrentSetpoint']
            if self.setpointKludge:
                if time() < self.kludgeSetpointTimestamp+self.kludgeSetpointWait:
                    return
                if currentSetpoint != currentSetpointLocal:
                    self.kludgeSetpointCurrentTry += 1
                    self.log.debug("_check_setpoint ? "\
                                   "currentSetpoint(%6.3f) == Iocaste Local(%6.3f)"\
                                   %(currentSetpoint,currentSetpointLocal))
                    self._update_current()
                    self.log.warn("Detected ignored local setpoint, resending it!")
                elif currentSetpointLocal != currentSetpointReadback:
                    self.kludgeSetpointCurrentTry += 1
                    self.log.debug("_check_setpoint ? "\
                                   "Iocaste Local(%6.3f) == IocasteReadback(%6.3f)"\
                                   %(currentSetpointLocal,currentSetpointReadback))
                    self._command_bit(BIT_RAMPSTART)
                    self.log.warn("Detected ignored readback setpoint, resending it!")
                else:
                    self._appendSetpointCounter()
                    self.log.debug("setpoint %6.3f well applied, nothing else to do"%(currentSetpointLocal))
                    self.setpointKludge = False
            elif not (currentSetpointLocal == currentSetpointReadback == currentSetpoint):
                self.log.error("Weird, unconsistend registers but no setpoint from the device")
                self.setpointKludge = True
                self.kludgeSetpointTimestamp = time()
                self.kludgeSetpointCurrentTry = 0
    
    def _check_commandIsApplied(self):
        if self.kludge and not self.kludgeCommandTimestamp==None:
            st = self.ALL[ADDR_STATE]
            #only check if command is well applied if enough time is lapsed
            if time() >= self.kludgeCommandTimestamp+self.commandWait:
                self.log.debug('_check_commandIsApplied ?')
                if self.commandKludge[1] == BIT_POWERON:
                    if not bit(st, BIT_POWERON):
                        self.kludgeCommandCurrentTry += 1
                        self._onoff(BIT_POWERON)
                        self.kludgeCommandTimestamp = time()
                        self.log.warn("Detected ignored command On(), resending it!")
                    else:
                        self._appendCommandCounter('POWERON')
                        self.log.debug("Command On() well applied, nothing else to do")
                        self.kludgeCommandTimestamp = None
                elif self.commandKludge[1] == BIT_POWEROFF:
                    if not bit(st, BIT_POWEROFF):
                        self.kludgeCommandCurrentTry += 1
                        self._onoff(BIT_POWEROFF)
                        self.kludgeCommandTimestamp = time()
                        self.log.warn("Detected ignored command Off(), resending it!")
                    else:
                        self._appendCommandCounter('POWEROFF')
                        self.log.debug("Command Off() well applied, nothing else to do")
                        self.kludgeCommandTimestamp = None
                elif self.commandKludge[1] == BIT_STANDBY:
                    if not bit(st, BIT_STANDBY):
                        self.kludgeCommandCurrentTry += 1
                        self.Standby()
                        #timestamp set in the command
                        self.log.warn("Detected ignored command Standby(), resending it!")
                    else:
                        self._appendCommandCounter('STANDBY')
                        self.log.debug("Command Standby() well applied, nothing else to do")
                        self.kludgeCommandTimestamp = None
                else:
                    self.log.error("Ops! this should not happen! [_check_commandIsApplied()] command %d"%self.commandKludge)
                    self.kludgeCommandTimestamp = None

    def _setCommandWait(self):
        '''for the kludge to force the device to review if a command has 
           been applied, different times are need depending on the command
        '''
        #off->standby
        if self.commandKludge == [BIT_POWEROFF,BIT_STANDBY]:
            self.commandWait = self.kludgeCommandWait['OffStandby']
        #standby->on
        elif self.commandKludge == [BIT_STANDBY,BIT_POWERON]:
            self.commandWait = self.kludgeCommandWait['StandbyOn']
        #on->standby
        elif self.commandKludge == [BIT_POWERON,BIT_STANDBY]:
            self.commandWait = self.kludgeCommandWait['OnStandby']
        #standby->off
        elif self.commandKludge == [BIT_STANDBY,BIT_POWEROFF]:
            self.commandWait = self.kludgeCommandWait['StandbyOff']
        #off->standby->on
        elif self.commandKludge == [BIT_POWEROFF,BIT_POWERON]:
            self.commandWait = self.kludgeCommandWait['OffStandby'] + self.kludgeCommandWait['StandbyOn']
        #on->standby-off
        elif self.commandKludge == [BIT_POWERON,BIT_POWEROFF]:
            self.commandWait = self.kludgeCommandWait['OnStandby'] + self.kludgeCommandWait['StandbyOff']
        #if destination is the same than the origin
        elif self.commandKludge[0] == self.commandKludge[1]:
            self.commandWait = 0
        else:
            raise PS_Exception("Command wait time cannot be establish %s"%self.commandKludge)

    def _appendCommandCounter(self,commandType):
        while len(self.kludgeCommandCounter[commandType]) <= self.kludgeCommandCurrentTry:
            self.kludgeCommandCounter[commandType].append(0)
        self.kludgeCommandCounter[commandType][self.kludgeCommandCurrentTry] += 1

    def _appendSetpointCounter(self):
        while len(self.kludgeSetpointCounter) <= self.kludgeSetpointCurrentTry:
            self.kludgeSetpointCounter.append(0)
        self.kludgeSetpointCounter[self.kludgeSetpointCurrentTry] += 1
        
    def _getCommandState(self):
        st = self.ALL[ADDR_STATE]
        if bit(st, BIT_STANDBY):
            return BIT_STANDBY
        elif bit(st, BIT_POWEROFF):
            return BIT_POWEROFF
        elif bit(st, BIT_POWERON):
            return BIT_POWERON
        else:
            raise PS_Exception("Cannot establish current state")

    def _check_remote(self):
        '''Generates a sensible error message when trying to control a
           power supply that is in Local Mode.
        '''
        if not self.cache.get('RemoteMode'):
            msg = "can not control power supply when in local mode or offline"
            raise PS_Exception(msg)

    def _onoff(self, b):
        '''Turns power supply on or off, depending which command bit 'b'.
           When necessary the command will be repeated after 5 seconds,
           to switch from On to Off or from Off to On states.
        '''
        bits = (b, BIT_RAMPSTART)
        self._command_bit( *bits )
        st = self.ALL[ADDR_STATE]
        if self.STAT.type_code==4 and not bit(st, b) and not bit(st, BIT_STANDBY):
            self._pend_oo(bits)

    ### Commands ###
    @CommandExc
    def On(self):
        '''Turns power supply on.'''
        self._check_remote()
        if self.kludge:
            self.commandKludge = [self._getCommandState(),BIT_POWERON]
            self._setCommandWait()
            self.kludgeCommandTimestamp = time()
            self.kludgeCommandCurrentTry = 0
        self._onoff(BIT_POWERON)

    @CommandExc
    def Off(self):
        '''Turns power supply off.'''
        self._check_remote()
        if self.cache['_isCycling']:
            self._set_cycling_trace("Off() command called: aborting cycling")
            self.AbortCycle()
        if self.kludge:
            self.commandKludge = [self._getCommandState(),BIT_POWEROFF]
            self._setCommandWait()
            self.kludgeCommandTimestamp = time()
            self.kludgeCommandCurrentTry = 0
        self._onoff(BIT_POWEROFF)

    def _pend_oo(self, bits, timeout=10.0):
        '''Set a pending on-or-off operation.
            @param b specifies the command bit that should be set.
                Give None to cancel pending oo-operations.
            @param timeout specifies after how many seconds the operation is considered to timeout
                   (old default was 7.5 seconds)
        '''
        self._pending_bits = bits
        self._pending_latest = time()+timeout

    @CommandExc
    def Standby(self):
        '''Switches the power supply to standby mode.
        '''
        self._check_remote()
        if self.cache['_isCycling']:
            self._set_cycling_trace("Standby() command called: aborting cycling")
            self.AbortCycle()
            #The final state will be standby due to the Abort.
        if self.kludge:
            self.commandKludge = [self._getCommandState(),BIT_STANDBY]
            self._setCommandWait()
            self.kludgeCommandTimestamp = time()
            self.kludgeCommandCurrentTry = 0

        # cancels pending command On/Off writes
        self._pend_oo(None)
        st = self.ALL[ADDR_STATE]

        if bit(st, BIT_STANDBY):
            pass # does nothing when already on standby

        elif bit(st, BIT_POWERON):
            self._command_bit( BIT_POWEROFF, BIT_RAMPSTART )

        elif bit(st, BIT_POWEROFF):
            self._command_bit(BIT_POWERON, BIT_RAMPSTART )

        else:
            desc = "neither on, off nor standby bit was set!"
            raise PS_Exception(desc)

    @CommandExc
    def ResetInterlocks(self):
        '''Resets errors messages, in particular related to interlocks.
        '''
        PowerSupply.ResetInterlocks(self)
        now = time()
        # this causes communications to be reattempted in next UpdateState
        if self._next_read_time > now:
            self.STAT.COMM_RESET()
            self._next_read_time = now
        if self.cache['_cycling']['alarm'] == True:
            self.cache['_cycling']['alarm']=False

        # only resets interlocks if communication is working,
        # this way the user has a chance to identify the cause of the problem
        # before it is resetted
        else:
            try:
                self._command_bit(BIT_RESET_FAULT)
            except Tg.DevFailed, df:
                if df[-1].reason=='API_DeviceTimedOut':
                    raise PS.PS_Exception('modbus device not responding (CORBA call timed out)')
                else:
                    raise

    def read_Version(self, attr):
        text = ( 'project id {proj}\niocaste id {io}\npower supply id {ps}\n' +
            'MB441 program revision {mb441}\n' +
            'acquisition program revision {acq}\n' +
            'regulation program revision {reg}\nip address {ip}'
            ).format(
                proj=self._get_ulong(ADDR_PROJECT_ID),
                io=self._get_ulong(ADDR_IOCASTE_ID),
                ps=self.ALL[ADDR_POWERSUPPLY_ID],
                mb441=self.ALL[ADDR_MB441_PROGRAM_REVISION],
                acq=self.ALL[ADDR_REG445_ACQ_PROGRAM_REVISION],
                reg=self.ALL[ADDR_REG445_REG_PROGRAM_REVISION],
                ip=self._get_ipaddress()
            )
        attr.set_value_date_quality(text, self._read_t, AQ_VALID)


    def _start_ramp(self):
        '''Starts ramping of the current.
        '''
        self._command_bit(BIT_RAMPSTART)
        #if self.kludge:
        #    for r in range(self.KludgeApplyRepeat):
        #      self._command_bit(BIT_RAMPSTART)
        #      sleep(0.025)
        self.STAT.move_min_t = time()+0.1
        if not self.cache['_isCycling']:
            self.STAT.CURRENT_ADJUST()

    ### Conversion Functions ###
    _get_float = lambda self, addr: (
        struct.unpack('f', struct.pack('HH', *self.ALL[addr:addr+2] ) )[0])

    _get_bit = lambda self, addr, bit: (
        bool(self.ALL[addr] & (1<<bit)) )

    _get_binstring = lambda self, addr: (
        binstr(self.ALL[addr]) )

    _get_ulong = lambda self, addr: (
        int(self.ALL[addr+1])*2**16 + int(self.ALL[addr]) )

    ### Attributes ###
    @AttrExc
    def write_I(self, wattr):
        if self.cache['_isCycling']:
            Except.throw_exception("Not available during cycling",
                                   "Write I not possible during cycling",
                                   "write_I")
        Iref = wattr.get_write_value()
        self._write_iref(Iref)

    @AttrExc
    def read_CurrentSetpoint(self, attr):
        if self._check_invalid(attr): return
        if self.KludgeCurrentSetpointReadback:
            addr = ADDR_IREF_REMOTE_READ
        else:
            addr = ADDR_IREF_READ
        Iref = self._get_float(addr)
        v = self._correct_attr('CurrentSetpoint', attr, Iref, inv=True)
        try:
            attr.set_write_value(v)
        except DevFailed, fail:
            if fail[0].reason == 'API_WAttrOutsideLimit':
                pass
            else:
                raise

    @AttrExc
    def write_CurrentSetpoint(self, wattr):
        '''Writes the current setpoint.
        '''
        if not self.cache['_isCycling'] and \
           not self.get_state() in [DevState.ON,DevState.MOVING]:
            Except.throw_exception("Setpoint not possible!",
                                   "Write CurrentSetpoint not possible in state %s"%self.get_state(),
                                   "write_CurrentSetpoint")
        if self.cache['_isCycling']:
            Except.throw_exception("Not available during cycling",
                                   "Write CurrentSetpoint not possible during cycling",
                                   "write_CurrentSetpoint")
        sp = wattr.get_write_value()
        self.__writeSetpoint(sp)

    def __writeSetpoint(self,sp):
        try:
            self.setpointKludge = True
            self.kludgeSetpointTimestamp = time()
            self.kludgeSetpointCurrentTry = 0
            self.cache['CurrentSetpoint'] = sp
            self.log.debug("Write setpoint %6.3f"%sp)
            self._update_current()
        except Exception,e:
            msg = traceback.format_exc()
            # _setpoint_exc is a dictionary that counts
            # how often each exception occured
            if not hasattr(self, '_setpoint_exc'):
                self._setpoint_exc = {}
            self._setpoint_exc.setdefault(msg, 0)
            self._setpoint_exc[msg] += 1

    def _update_current(self):
        '''Updates the reference current (Iref) from the current setpoint
           taking offset and scaling factor in account.
        '''

        sp = self.cache.get('CurrentSetpoint', NAN)
        Iref = self._correct('CurrentSetpoint', sp)
        self._write_iref(Iref)

    def _write_iref(self, Iref):
        '''Writes reference current to power supply and starts the current ramp,
           if power supply signal readiness and power is on.
        '''
        # writing nans will not be attempted, confuses the power supply
        if str(Iref)=="nan": return
        self._write_float(ADDR_IREF_WRITE, Iref )
        self._start_ramp()

    def _check_invalid(self, attr):
        if self._next_read_time:
            attr.set_quality(AQ_INVALID)
            return True

    @AttrExc
    def read_CurrentOffset(self, attr):
        if self._check_invalid(attr): return
        v = self._get_attr_correction('CurrentSetpoint')['Offset']
        attr.set_value(v)

    @AttrExc
    def write_CurrentOffset(self, wattr):
        if self.cache['_isCycling']:
            Except.throw_exception("Not available during cycling",
                                   "Write CurrentOffset not possible during cycling",
                                   "write_CurrentOffset")
        new_offset = wattr.get_write_value()
        self._set_attr_correction('CurrentSetpoint', Offset=new_offset)
        self._update_current()

    @AttrExc
    def read_ALL(self, attr):
        if self._check_invalid(attr): return
        attr.set_value(self.ALL)

    @AttrExc
    def read_StateCode64(self, attr):
        sw = self.cache['StateWords']
        value = encode_state_u64(sw)
        attr.set_value(value)

    @AttrExc
    def read_StateCode(self, attr):
        sw = int(self.cache['StateWords'][0])
        attr.set_value(sw)

    @AttrExc
    def read_ErrorCode(self, attr):
        if self.cache.has_key('StateWords'):
            sw = self.cache['StateWords']
            value = encode_error_i32(sw)
            attr.set_value(value)

    @CommandExc
    def ApplyCurrent(self):
        self._start_ramp()

    @CommandExc
    def UpdateState(self):
        #kludge checks
        self._check_commandIsApplied()
        self._check_setpoint()

        # updates registers and state machine
        #info = self.STAT

        #cycling stage
        if self.cache['_isCycling']:
            self.__isCyclingMotionDone()
            if self.__isStepWaitDone():
                self._doNextCycleStep()
            try:
                #not more often than ten seconds 
                #(assuming a polling on the UpdateState() of 250ms)
                if self.cache['_cycling']['logger_hackish'] == 4*10:
                    self._set_cycling_trace("UpdateState()"\
                                            " step=%d, current=%6.3f (voltage=%6.3f),"\
                                            " setpoint=%6.3f,"\
                                            " min=%6.3f, max=%6.3f, final=%6.3f,"\
                                            " step wait timestamp=%s"
                                            %(self.cache['_cycling']['stepIdx'],
                                              current,self.cache['Voltage'],setp,
                                              self.cache['_cycling']['minValue'],
                                              self.cache['_cycling']['maxValue'],
                                              self.cache['_cycling']['finalvalue']))
                    self.cache['_cycling']['logger_hackish'] = 0
                else:
                    self.cache['_cycling']['logger_hackish'] += 1
            except:
                pass#ignore if there is an exception, this is only for log debug.

        # test whether its okay to read again
        if self._next_read_time > time(): return
        # executes pending on/off command, if any
        # self.log.debug("pending %s @ %s <= %s", self._pending_bits, self._pending_latest, time())
        if self._pending_bits is not None:

            state = self.get_state()
            # discards pending op when timing out
            if time() > self._pending_latest:
                self._alarm('pending timeout %s' % [self._pending_bits] )
                self._pend_oo(None)

            # waits until ready to execute pending operation
            elif state in (DevState.ON, DevState.OFF):
                pass

            # executes pending operation when ready
            elif state==DevState.STANDBY:
                # sleep(0.05) #< could be useful to increase chance of Standby switching to succeed
                try:
                    self._command_bit(*self._pending_bits)
                except Exception,e:
                    self._exception()
                finally:
                    self._pend_oo(None)

            # discards operation when switched to local mode
            elif not self.cache.get('RemoteMode'):
                self._pend_oo(None)

            # otherwise the pending operation is silently discarded
            # for example when state is ALARM, FAULT
            else:
                self._pend_oo(None)
                
        try:
            if self.modbus == None:
                self.modbus = DeviceProxy(self.ModbusDevice)
                self.modbus.set_timeout_millis(self.ModbusTimeout)
                #self._set_communications_trace("Comunication recovered")
        except Exception,e:
            self.STAT.UNKNOWN("Cannot bind to the modbus device")
            return

        try:
            start_time = time()
            regs = self.modbus.ReadHoldingRegisters( [0,NREGISTER] )
            self.read_t = (time()+start_time) / 2.0
            if self.get_state()==DevState.FAULT:
                self._set_communications_trace("Fault recovered")
                #self.sticky_messages.clear()

            #TODO: better way to differenciate between pytango versions
            if type(regs) == list:#tango 7.1 or below
                self.ALL = ndarray(shape=(len(regs),),dtype='uint32',buffer=array.array('i',regs))
            else:#tango 7.2 or above
                self.ALL = ndarray(shape=regs.shape, dtype='uint16', buffer=regs)
            self._next_read_time = 0
            for (aname, rfun) in HazemeyerIocaste_PS._READ:
                try:
                    self.cache[aname] = value = rfun(self)
                    self.push_change_event(aname, value, self.read_t, AQ_VALID)
                except Exception:
                    self.log.exception('UpdateState '+aname)
            self.cache["pending bits"] = self._pending_bits

            # pushes change events for attributes that are NOT in _READ
            for aname in ('CurrentSetpoint', 'Current', 'Voltage'):
                self.vdq_cache[aname] = PA = self._read(aname)
                self.cache[aname] = PA.value
                self.push_change_event(aname, *PA.triple)

#            self._check_apply()
            word = self.cache['StateWords'] = self.ALL[ADDR_STATE:ADDR_STATE+5]
            self._check_ramp()
            self.STAT.update(word,self.cache)

        except DevFailed, dx:
            self._read_t = None
            self._modbus_err_count += 1
            now = time()
            time_to_fail = now - start_time
            reason = dx[-1].reason
            desc = dx[-1].desc
            if reason=='API_DeviceTimedOut':
                iphost = self.modbus.get_property('Iphost').get('Iphost', [])
                if iphost:
                    msg = 'host %s timed out'%(iphost[0])
                else:
                    msg = 'timeout?'

            else:
                msg = "%s"%(dx[-1].desc)
            self.log.error('UpdateState: %s:%s' %  (reason,desc))
            self._next_read_time = now + RECONNECT_DELAY_s
            #self._next_read_time = float('inf')
            if not self.get_state()==DevState.FAULT:
                self._set_communications_trace(msg)
            self.STAT.COMM_FAULT(msg)
            self.modbus.set_timeout_millis(self.ModbusTimeout)

        except Exception, exc:
            self.STAT.SERVER_ERROR(exc)
            raise

    @CommandExc
    def Cycle(self):
        if self.cache['_isCycling']:
            #desc = "Magnet is already cycling."
            #raise PS_Exception(desc)
            pass#no message if it's already cycling
            self._append_new_cycling()
            self._set_cycling_trace("Cycle() called but already cycling")
        elif not self.get_state() in [DevState.ON,DevState.MOVING]:
            desc = "Magnet can only be cycled in on state."
            self._append_new_cycling()
            self._set_cycling_trace("Cycle() called but %s"%desc)
            raise PS_Exception(desc)
        elif not self.cache['_isCycling']:
            #TODO: only in on state
            #Get the range on the currentSetpoint Attr
            self.__prepare2cycle()

    @CommandExc
    def AbortCycle(self):
        self._append_new_cycling()
        self._set_cycling_trace("AbortCycle() called")
        if self.cache['_isCycling']:
            cycling = self.cache['_cycling']
            #prepare the setpoint to be where it was
            self.cache['CurrentSetpoint'] = cycling['finalValue']
            #clean the internal cycling vbles
            cycling['minValue'] = None
            cycling['maxValue'] = None
            cycling['finalValue'] = None
            cycling['stepIdx'] = None
            cycling['stepIsMoving'] = False
            cycling['stepRampStartTS'] = None
            cycling['stepRampRetry'] = 0
            cycling['expectedRampTime'] = None
            cycling['stepWaitStartTS'] = None
            #write the new setpoint to the iocaste
            self._update_current()
            #mark that this is not cycling
            self.cache['_isCycling'] = False
            self.push_change_event('isCycling',False)
            self.Standby()
        else:
            #desc = "There is no cycling running to be aborted."
            #raise PS_Exception(desc)
            pass#no message if cancels something that is not cycling

    ####
    # cycling methods
    ####
    def __prepare2cycle(self):
        cycling = self.cache['_cycling']
        multiattr=self.get_device_attr()
        setpAttr = multiattr.get_attr_by_name('CurrentSetpoint')
        try:
            self.__setupCyclingRange(setpAttr)
            #Those assigments are out to be set by the previous method
            #cycling['maxValue'] = float(setpAttr.get_max_value())
            #cycling['minValue'] = float(setpAttr.get_min_value())
        except DevFailed, e:
            Except.re_throw_exception(e,"Cycle command failed",
                                      "Non usable range defined for cycling",
                                      "Cycle()")
        #backup the current setpoint to go back at the end of the cycling procedure
        cycling['finalValue'] = self._get_float(ADDR_IREF_WRITE)
        #point to the first step
        cycling['stepIdx'] = 0
        #Mark to start test in UpdateState()
        self.cache['_isCycling'] = True
        self.push_change_event('isCycling',True)
        #start new log:
        self._append_new_cycling()
        self._set_cycling_trace("Cycle() called with %6.3f current: "\
                                "min=%6.3f and max=%6.3f"
                                %(cycling['finalValue'],
                                  cycling['minValue'],
                                  cycling['maxValue']))
        self._doNextCycleStep()
        
    def __setupCyclingRange(self,setpAttr):
        '''The min/max of the cycling is defined hierarcically from if the 
           warning value is defined, if not try with the alarm, and if not
           try with the max value it self.
           In case, no one of those are configured, and exception is raised
           and the cycling cannot be launched.
        '''
        cycling = self.cache['_cycling']
        attrProp = setpAttr.get_properties_3()
        try:
            cycling['maxValue'] = float(attrProp.att_alarm.max_warning)
        except:
            try:
                cycling['maxValue'] = float(attrProp.att_alarm.max_alarm)
            except:
                cycling['maxValue'] = float(attrProp.max_value)
                #If exception in this read of convertion, parent method will catch the exception
        try:
            cycling['minValue'] = float(attrProp.att_alarm.min_warning)
        except:
            try:
                cycling['minValue'] = float(attrProp.att_alarm.min_alarm)
            except:
                cycling['minValue'] = float(attrProp.min_value)
                #If exception in this read of convertion, parent method will catch the exception
        
    def _doNextCycleStep(self):
        try:
            cycling = self.cache['_cycling']
            cycling['stepIdx'] += 1
            nextSetpoint = self.__setpoint4thisStep()
            if cycling['stepIdx'] <= self.nCyclingSteps:
                self.__setCyclingSetpoint(nextSetpoint)
            if cycling['stepIdx'] > self.nCyclingSteps:
                #clean up vbles:
                self.cache['_isCycling'] = False
                self.push_change_event('isCycling',False)
                cycling['minValue'] = None
                cycling['maxValue'] = None
                cycling['finalValue'] = None
                cycling['stepIdx'] = None
                cycling['stepIsMoving'] = False
                cycling['stepRampStartTS'] = None
                cycling['expectedRampTime'] = None
                cycling['stepWaitStartTS'] = None
                #don't change the expectedWaitTime
                cycling['logger_hackish'] = 0
        except Exception,e:
            self._set_cycling_trace("doNextCycleStep() Exception during an step: %s"%e)
            traceback.print_exc()
            self.cache['_isCycling'] = False
            self.push_change_event('isCycling',False)
            #write the setpoint as where it was
            self.__writeSetpoint(self.cache['_cycling']['finalValue'])
            #change to standby to allow the user this had a problem.
            self.Standby()

    def __setCyclingSetpoint(self,setpoint):
        cycling = self.cache['_cycling']
        cycling['stepRampRetry'] += 1
        cycling['stepIsMoving'] = True
        cycling['stepRampStartTS'] = time()
        biggervalue = max(self.cache['Current'],setpoint)
        smallervalue = min(self.cache['Current'],setpoint)
        cycling['expectedRampTime'] = max((biggervalue-smallervalue)/self.cache['CurrentRamp'],1)#at least 1s of ramp time
        self._set_cycling_trace("setCyclingSetpoint() step %d, expected ramp time "\
                                "between origin (%6.3fA,%6.3fV) and destination (%6.3fA)"\
                                "is %6.3fs with a ramp of %6.3fA/s"\
                                %(cycling['stepIdx'],
                                  self.cache['Current'],self.cache['Voltage'],
                                  setpoint,
                                  cycling['expectedRampTime'],
                                  self.cache['CurrentRamp']))
        self.__writeSetpoint(setpoint)
        cycling['stepWaitStartTS'] = None

    def __isCyclingMotionDone(self):
        cycling = self.cache['_cycling']
        if self.cache['_isCycling'] and cycling['stepIsMoving']:
            lapsetime = time() - cycling['stepRampStartTS']
            if lapsetime >= cycling['expectedRampTime']:
                isRegulationOk = self._get_bit(ADDR_STATE,BIT_REGOK)
                current = self._get_float(ADDR_I_MEASURE)
                setp = self.__setpoint4thisStep()
                #check if regulationOk register works
                if isRegulationOk:
                    self._set_cycling_trace("__isCyclingMotionDone() regulationOk flag "\
                                            "detected! (Motion should be done).")
                    #self.__cycling_markMotionDone()
                #if the current is close to the setpoint, we decide the motion is done.
                if (current > setp-(self.RegulationPrecision*20) and \
                      current < setp+(self.RegulationPrecision*20)):
                    self._set_cycling_trace("__isCyclingMotionDone() current=%6.3f "\
                                            "(voltage=%6.3f) near "\
                                            "setpoint=%6.3f (Motion done)."
                                            %(current,self.cache['Voltage'],setp))
                    
                    self.__cycling_markMotionDone()
                else:
                    if lapsetime >= 2*cycling['expectedRampTime']:
                        self._set_cycling_trace("__isCyclingMotionDone() after the "\
                                                "expected ramp time (%6.3f s), it "\
                                                "would be done but not yet."\
                                                " (current=%6.3f,voltage=%6.3f;setpoint=%6.3f) "\
                                                "retrying..."
                                                %(lapsetime,current,self.cache['Voltage'],setp))
                        self.__setCyclingSetpoint(setp)
                    if cycling['stepRampRetry'] > maxStepCyclingRetries:
                        self._set_cycling_trace("__isCyclingMotionDone() after %d "\
                                                "motion retries: "\
                                                "aborting the cycling!"%(maxStepCyclingRetries))
                        cycling['alarm'] = True
                        self.AbortCycle()
            return cycling['stepIsMoving']
        return False

    def __isStepWaitDone(self):
        '''Method used to check if the wait on the step is done, to decide if
           can be moved to next step.
        '''
        cycling = self.cache['_cycling']
        if self.cache['_isCycling'] and not cycling['stepIsMoving']:
            lapsetime = time() - cycling['stepWaitStartTS']
            if cycling['expectedWaitTime'] <= lapsetime:
                return True
        return False

    def __cycling_markMotionDone(self):
        cycling = self.cache['_cycling']
        cycling['stepIsMoving'] = False
        cycling['stepRampStartTS'] = None
        cycling['stepRampRetry'] = 0
        cycling['expectedRampTime'] = None
        if cycling['stepIdx'] == self.nCyclingSteps:
            cycling['stepWaitStartTS'] = time()-cycling['expectedWaitTime']
        else:
            cycling['stepWaitStartTS'] = time()
        

    def __setpoint4thisStep(self):
        cycling = self.cache['_cycling']
        if cycling['stepIdx'] < self.nCyclingSteps:
            if cycling['stepIdx']%2 == 1:#in [1,3,5]:
                sp = cycling['maxValue']
            elif cycling['stepIdx']%2 == 0:# in [2,4]:
                sp = cycling['minValue']
        else:#if cycling['stepIdx'] in [0,6]:
            sp = cycling['finalValue']
        return sp
    ####
    # done cycling methods
    ####

    ### generic utilities ###
    def _write_float(self, addr, val):
        val_byte = struct.pack('f', val)
        val_short = struct.unpack('hh', val_byte)
        args = [addr, len(val_short)] + list(val_short)
        self.modbus.PresetMultipleRegisters(args)

    def _command_bit(self, *nmo):
        '''Writes one or more command bits.
        '''
        cmd_byte = 0
        for n in nmo:
            assert 0 <= n <= 15, "bit index must be between 0 and 15"
            cmd_byte |= 1<<n

        self.log.debug('_command_bit %r', nmo)
        self.modbus.PresetSingleRegister( [ADDR_COMMAND, cmd_byte])

    @AttrExc
    def read_Voltage(self, attr):
        if self._check_invalid(attr): return
        self._correct_attr('Voltage', attr, self.cache.get('V', NAN))

    @AttrExc
    def read_Current(self, attr):
        if self._check_invalid(attr): return
        v = self._correct_attr('Current', attr, self.cache.get('I', NAN))
        self.cache['Current'] = v
        delta = abs(v-self.cache['CurrentSetpoint'])
        if attr.get_quality()==AQ_VALID and delta > self.RegulationPrecision:
            if self.get_state()==DevState.MOVING:
                attr.set_quality(AQ_CHANGING)
            elif self.get_state()==DevState.ON:
                attr.set_quality(AQ_ALARM)
            else:
                attr.set_quality(AQ_VALID)

    def _get_ipaddress(self):
        low,high = self.ALL[ADDR_IP_ADDRESS:ADDR_IP_ADDRESS+2]
        ip = low>>8, (low & 0xFF), high>>8, high & 0xFF
        return ".".join(map(str,ip))

    @AttrExc
    def read_StatusBits(self, attr):
        if self._check_invalid(attr): return
        st,i1,i2,i3,i4 = self.cache['StateWords']
        NL = ' 0x%x\n'
        y = ('\n'+
            'bit        fedc ba98 7654 3210\n'+
            'state      '+binstr(st) + NL % st +
            'interlock1 '+binstr(i1) + NL % i1 +
            'interlock2 '+binstr(i2) + NL % i2 +
            'regulation '+binstr(i3) + NL % i3 +
            'internal   '+binstr(i4) + NL % i4
        )
        attr.set_value(y)


    @AttrExc
    def read_MAC(self, attr):
        ip = self.modbus.get_property( 'Iphost')['Iphost'][0]
        proc = subprocess.Popen(['/sbin/arping', '-c3', '-f', ip], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output = proc.stdout.read()
        r = proc.wait()
        if r:
            raise PS.PS_Exception(proc.stderr.read())
        m = re.search(r'\[(..:..:..:..:..:..)\]', output)
        if m:
            attr.set_value(m.group(1))
        else:
            attr.set_quality(AQ_INVALID)

    @AttrExc
    def read_SendingFailed_Setpoints(self, attr):
        if self.kludge:
            failed = 0.
            for i in range(1,len(self.kludgeSetpointCounter)):
                failed += self.kludgeSetpointCounter[i]
            total = self.kludgeSetpointCounter[0] + failed
            try:
                if total == 0:
                    percentage = 0.
                else:
                    percentage = (failed / total)*100
            except:
                percentage = NAN
            attr.set_value(percentage)
        else:
            attr.set_value(NAN)

    @AttrExc
    def read_SendingFailed_Setpoints_detail(self,attr):
        if self.kludge:
            attr.set_value(self.kludgeSetpointCounter,len(self.kludgeSetpointCounter))
        else:
            attr.set_value([0],1)

    @AttrExc
    def read_SendingFailed_Commands(self, attr):
        if self.kludge:
            failed = 0.
            total = 0
            for k in ['POWEROFF','POWERON','STANDBY']:
                for i in range(1,len(self.kludgeCommandCounter[k])):
                    failed += self.kludgeCommandCounter[k][i]
                total += self.kludgeCommandCounter[k][0] + failed
            try:
                if total == 0:
                    percentage = 0
                else:
                    percentage = (failed / total)*100
            except:
                percentage = NAN
            attr.set_value(percentage)
        else:
            attr.set_value(NAN)

    @AttrExc
    def read_SendingFailed_Commands_detail(self,attr):
        if self.kludge:
            #prepare an auxiliar list where add all the command sub-statistics
            l = []
            for k in self.kludgeCommandCounter.keys():
                for i in range(len(self.kludgeCommandCounter[k])):
                    while len(l) <= i:
                        l.append(0)
                    l[i] += self.kludgeCommandCounter[k][i]
            attr.set_value(l,len(l))
        else:
            attr.set_value([0],1)

    @AttrExc
    def read_isCycling(self,attr):
        attr.set_value(self.cache['_isCycling'])

    @CommandExc
    def ResetStatistics(self):
        self.kludgeCommandCounter = {'POWEROFF':[0,0],'STANDBY':[0,0],'POWERON':[0,0]}
        self.kludgeSetpointCounter = [0,0]


class HazemeyerIocaste_PS_Class(PowerSupply_Class):#DeviceClass):

    class_property_list = gen_property_list(
        ('RegulationPrecision','ModbusTimeout'),
        XI = (1,4) )
#    class_property_list['RegulationPrecision'][2] = 0.0225
    class_property_list['RegulationPrecision'][2] = 0.05
    class_property_list['ModbusTimeout'][2] = 1000

    device_property_list = gen_property_list( ('ModbusDevice', ), cpl=class_property_list)
    device_property_list['Type'] = [ DevString, "2, 4 or 6", [] ]
    device_property_list['#Code'] = [ DevString, "equipment code", 'none' ]
    device_property_list['KludgeCurrentSetpointReadback'] = [ DevBoolean, 'whether you want to use register 26 or register 46 for current setpoint readback', False ]

    cmd_opt = ('Standby', 'ApplyCurrent', 'UpdateState', 'Cycle', 'AbortCycle',
               'CleanCommunicationsTrace')
    cmd_list = gen_cmd_list(opt=cmd_opt)
    cmd_list['UpdateState'][2]['polling period'] = 250

    #    Attribute definitions
    attr_opt = ('CurrentOffset', 'Resistance', 'CyclingTrace',
                'CommunicationsTrace','HaveCommunicationsTrace')
    attr_list = gen_attr_list(max_err=32, opt=attr_opt)
    attr_list['ALL'] =  [ [ DevUShort, SPECTRUM, READ, NREGISTER ] ]
    attr_list['MAC'] = [ [ DevString, SCALAR, READ], { 'description' : 'read MAC'} ]
    attr_list['SendingFailed_Setpoints'] = [[DevDouble,SCALAR,READ],
                                      {'description':'Statistic about the '\
                                                     'setpoint send failures',
                                       'label':'Setpoint Fails',
                                       'unit':'%',
                                       'Display level':DispLevel.EXPERT}]
    attr_list['SendingFailed_Commands'] = [[DevDouble,SCALAR,READ],
                                      {'description':'Statistic about the '\
                                                     'command send failures',
                                       'label':'Command Fails',
                                       'unit':'%',
                                       'Display level':DispLevel.EXPERT}]
    cmd_list['ResetStatistics'] = [[DevVoid,
                                    'Reset the dictionaries who collects the '\
                                    'SendignFailed statistics'],
                                   [DevVoid, ''], { }
                                  ]
    attr_list['SendingFailed_Setpoints_detail'] = [[DevLong,SPECTRUM,READ,100],
                                                   {'description':'number of resends per setpoint',
                                                    'label':'Setpoint retries',
                                                    'Display level':DispLevel.EXPERT}]
    attr_list['SendingFailed_Commands_detail'] = [[DevLong,SPECTRUM,READ,100],
                                                  {'description':'number of resends per command',
                                                   'label':'Command retries',
                                                   'Display level':DispLevel.EXPERT}]
    attr_list['isCycling'] = [[DevBoolean,SCALAR,READ],
                             {'description':'Boolean to know if this magnet is in cycling mode',
                              'label':'is cycling',
                              }]
#factory.add_cmd('ResetStatistics','Reset the dictionaries who collects the SendignFailed statistics')

### Attribute Machinery ###
def add_attr(aname, tp, rfun=None, wfun=None, dim=SCALAR, x=0, extra={},
    rwfun=None
):
    if rwfun is None:
        rwfun = nop
    read_fun_name = "read_"+aname
    write_fun_name = "write_"+aname

    rw = READ if wfun is None else READ_WRITE

    # all attributes are obtained using 'rfun' and stored into cache dictionary
    if not hasattr(HazemeyerIocaste_PS, read_fun_name):
        def read_fun(self, attr):
            if self._check_invalid(attr): return
            value = self.cache.get(aname)
            if value is None:
                reason = 'value of %r unknown' % aname
                if self.ALL[0]==USHORT_FILL:
                    reason += ", probably because no data has been read, yet."
                raise PS_Exception(reason)
            else:
                attr.set_value(value)

            w_value = rwfun(self)
            if not w_value is None:
                attr.set_write_value(w_value)

        setattr(HazemeyerIocaste_PS, read_fun_name, read_fun)
        if rfun is not None:
            HazemeyerIocaste_PS._READ.append( (aname, rfun) )

    if rw==READ_WRITE and not hasattr(HazemeyerIocaste_PS, write_fun_name):
        def write_fun(self, attr):
            data = []
            attr.get_write_value(data)
            value = data[0]
            wfun(self, value)

        setattr(HazemeyerIocaste_PS, write_fun_name, write_fun)

    if dim==SCALAR:
        attr_desc = [ [ tp, SCALAR, rw ], extra ]
    elif dim==SPECTRUM:
        attr_desc = [ [ tp, SPECTRUM, rw, x ], extra ]
    else:
        print "dim == ?", dim
    HazemeyerIocaste_PS_Class.attr_list[aname] = attr_desc

def add_state_bit(aname, bitno, extra={}):
    rfun = partial(HazemeyerIocaste_PS._get_bit, addr=ADDR_STATE, bit=bitno)
    add_attr(aname, DevBoolean, rfun, extra=extra)

def add_double(aname, addr=None, waddr=None, rwaddr=None, extra={}):
    rfun = partial(HazemeyerIocaste_PS._get_float, addr=addr)
    if waddr is None:
        rwfun = None
        wfun = None
    else:
        rwfun = partial(HazemeyerIocaste_PS._get_float, addr=rwaddr)
        def wfun(self, value):
            HazemeyerIocaste_PS._write_float(self, waddr, value)

    add_attr(aname, DevDouble, rfun, wfun, SCALAR, extra=extra, rwfun=rwfun)

def add_binstring(aname, addr=None, extra={}):
    rfun = partial(HazemeyerIocaste_PS._get_binstring, addr=addr)
    add_attr(aname, DevString, rfun, extra=extra)

def add_ushort_array(aname, addr, n, extra={}):
    def rfun(self):
        return self.ALL[addr:addr+n]
    add_attr(aname, DevUShort, rfun, dim = SPECTRUM, x=n, extra=extra)

def add_ushort(aname, addr, extra={}):
    def rfun(self):
        return self.ALL[addr]
    add_attr(aname, DevUShort, rfun, extra=extra)

def add_ulong(aname, addr, extra={}):
    def rfun(self):
        return self._get_ulong(addr)
    add_attr(aname, DevLong, rfun, extra=extra)

def add_short(aname, addr, extra={}):
    def rfun(self):
        return int(self.ALL[addr])
    add_attr(aname, DevShort, rfun, extra=extra)

add_ushort('PowerSupplyId', ADDR_POWERSUPPLY_ID)
add_ushort('VersionMB441', ADDR_MB441_PROGRAM_REVISION, extra={'description':'MB441 programm revision'})
add_ushort('VersionAcquisition', ADDR_REG445_ACQ_PROGRAM_REVISION, extra={'description':'Acquisition programm revision'})
add_ushort('VersionRegulation', ADDR_REG445_REG_PROGRAM_REVISION, extra={'description':'Regulation programm revision'})
add_attr('Version', DevString, ADDR_VERSION, 3,
    extra={'description':'Summary of version info and other useful'}
    )

add_attr('IpAddress', DevString, rfun=HazemeyerIocaste_PS._get_ipaddress)
add_ulong('ProjectId', ADDR_PROJECT_ID)
add_ulong('IocasteId', ADDR_IOCASTE_ID)
add_attr('StateCode64', DevLong64, extra=dict(format='%x'))
add_attr('StateCode', DevLong, extra=dict(format='%x'))
add_attr('ErrorCode', DevLong, extra=dict(format='%x'))

add_short('Interlock1', ADDR_INTERLOCK1)
add_short('Interlock2', ADDR_INTERLOCK2)
add_short('Interlock3', ADDR_REGULATION_FAULT)
add_short('Interlock4', ADDR_INTERNAL_FAULT)

# attributes will be evaluated in order of definition!
A_extra = dict(unit='A', format='%7.4f')
add_double('I', extra=A_extra, addr=ADDR_I_MEASURE, waddr=ADDR_IREF_WRITE, rwaddr=ADDR_IREF_READ)
add_attr('CurrentOffset', DevDouble, extra={ 'format':'%7.4f', "unit":'A'}, rfun=None, wfun=True)
add_attr('CurrentSetpoint', DevDouble, extra=A_extra, wfun=True)
add_attr('Current', DevDouble, extra=A_extra, rfun=None)
add_attr('StatusBits', DevString)
add_double('Voltage',  extra={ "unit":'V'}, addr=ADDR_VOLTAGE_MEASURE)
add_double('V',  extra={ "unit":'V'}, addr=ADDR_VOLTAGE_MEASURE)
add_double('CurrentRampLocal',  extra={ 'unit':'A/s'}, addr=ADDR_CURRENT_RAMP_LOCAL)
add_double('CurrentRamp',  extra={ 'unit':'A/s', 'display level' : True},
    addr=ADDR_CURRENT_RAMP, waddr=ADDR_CURRENT_RAMP, rwaddr=ADDR_CURRENT_RAMP,
)

add_state_bit('RemoteMode', BIT_REMOTE)
add_state_bit('Ready', BIT_READY)
add_state_bit('Fault', BIT_GLOBAL_FAULT)
add_state_bit('RegulationOk', BIT_REGOK)

if __name__ == '__main__':
    tango_main(HazemeyerIocaste_PS)
