#!/usr/bin/env python
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

class Release:
    author = "Lothar Krause <lkrause@cells.es> for CELLS / ALBA synchrotron"
    date = "2009-05-27"
    hexversion = 0x010200
    credits= "ALBA controls"


# Python standard library
import sys
import time
import traceback
from pprint import pformat, pprint
from copy import deepcopy
import struct
import logging
from functools import partial
from types import StringType

# TANGO Imports
from PyTango import DevVoid, DevString, DevState, DevFailed, DevDouble, \
    Except, DevShort, Util, DevLong, DevUShort, \
    DevVarStringArray, DevVarShortArray, DeviceProxy, AttrQuality, SCALAR, \
    SPECTRUM, READ, READ_WRITE, Database, Device_3Impl, DeviceClass, \
    DevVarLongArray, DevBoolean

# Extra Imports
import ps_standard as PS

# HazemeyerIocaste_PS
from ps_util import *
from state import *

class HazemeyerIocaste_PS(PS.PowerSupply):
    """Device server will allow to interact with ALBA HazemeyerIocaste_PS connected using Modbus over TCP.
    """
    # maps each attribute name to a function that returns the value of the corresponding attributes

    # list of attributes that should be read in UpdateState() command
    _READ = []

    def __init__(self, cl, name):
        PS.PowerSupply.__init__(self, cl, name)
        self.init_device(cl, name)

    def init_device(self, cl=None, name=None):
        if cl is None:
            cl = self.get_device_class().get_name()
            name = self.get_name()
        PS.PowerSupply.init_device(self, cl, name)

        self.cache = {}
        # initializes internal modbus data to fill pattern
        self.ALL = [ USHORT_FILL ] * NREGISTER
        # all bits are False, neither power on nor off
        self.ALL[ADDR_STATE] = 0

        self._next_read_time = 0 #< when next read might be attempted
        self._pend_oo(None)

        # The check_polled argument allows sub-classes to customize polled attribute verification
        # replace by regular device properties

        # initializes connection
        self._mandate_property('ModbusDevice')
        self.modbus = DeviceProxy(self.ModbusDevice)
        modbus_timeout = int(self.ModbusTimeout)
        self.modbus.set_timeout_millis(modbus_timeout)

        # initializes interpretation of error messages
        # checks type property
        dt = "2 (dipole), 4 (quadrupole) or 6 (sextupole)"
        desc = "Valid values are %s." % dt
        self._mandate_property('Type', config_hint=desc)
        pctype = TYPE_TRANS[self.Type.lower()]

        msg,mask = PSTYPE_INFO[pctype]
        # allow customization of external interlock messages
        msg[1] = copy(msg[1])
        for i,x in enumerate(XI_INDICES):
            msg[1][x] = getattr(self, "Interlock%d" % (i+1) )
        self.STAT = IocasteStateLogic(self, msg, mask)
        self.STAT.INITIALIZED()

        if False:
            self.Type = None
            self.Resolution = None
            self.ModbusDevice = None
            self.ModbusTimeout = None

    def _check_ramp(self):
        '''Checks value of current ramp, when set to 0.0 the value of the
           CurrentRampDefault property is set.
           The function is called at strategic places, when the power supply is
           switched on to ensure that the ramp is set to the 'good value'.
        '''
        C = self.cache
        if not 'CurrentRamp' in C:
            return
        if C["CurrentRamp"]==0.0 and 'CurrentRampLocal' in C:
            self._write_float(ADDR_CURRENT_RAMP, C['CurrentRampLocal'] )

    def _check_remote(self):
        '''Generates a sensible error message when trying to control a
           power supply that is in Local Mode.
        '''
        if not self.cache.get('RemoteMode'):
            msg = "can not control power supply when in local mode or offline"
            raise PS.PS_Exception(msg)

    def _onoff(self, b):
        '''Turns power supply on or off, depending which command bit 'b'.
           When necessary the command will be repeated after 5 seconds,
           to switch from On to Off or from Off to On states.
        '''

        bits = (b, BIT_RAMPSTART)
        self._command_bit( *bits )
        st = self.ALL[ADDR_STATE]
        if not bit(st, b) and not bit(st, BIT_STANDBY):
            self._pend_oo(bits)

    ### Commands ###
    @PS.ExceptionHandler
    def On(self):
        '''Turns power supply on.'''
        self._check_remote()
        self._onoff(BIT_POWERON)

    @PS.ExceptionHandler
    def Off(self):
        '''Turns power supply off.'''
        self._check_remote()
        self._onoff(BIT_POWEROFF)

    def _pend_oo(self, bits, timeout=7.5):
        '''Set a pending on-or-off operation.
            @param b specifies the command bit that should be set.
                Give None to cancel pending oo-operations.
            @param timeout specifies after how many seconds the operation is considered to timeout
        '''
        self._pending_bits = bits
        self._pending_latest = time.time()+timeout

    @PS.ExceptionHandler
    def Standby(self):
        '''Switches the power supply to standby mode.
        '''
        self._check_remote()
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
            raise PS.PS_Exception(desc)

    @PS.ExceptionHandler
    def ResetInterlocks(self):
        '''Resets errors messages, in particular related to interlocks.
        '''
        PS.PowerSupply.ResetInterlocks(self)
        self._command_bit(BIT_RESET_FAULT)

    def _start_ramp(self):
        '''Starts current ramping.
        '''
        self._command_bit(BIT_RAMPSTART)
        self.set_state(DevState.MOVING)

    ### Conversion Functions ###
    _get_float = lambda self, addr: \
        struct.unpack('f', struct.pack('HH', *self.ALL[addr:addr+2] ) )[0]

    _get_bit = lambda self, addr, bit: \
        bool(self.ALL[addr] & (1<<bit))

    _get_binstring = lambda self, addr: \
        binstr(self.ALL[addr])

    ### Attributes ###

    @PS.ExceptionHandler
    def write_Iref(self, wattr):
        '''Directly writes the current reference of the power supply, bypassing
           the CurrentSetpoint attribute and eventual trigger modes.
        '''
        data = []
        wattr.get_write_value( data )
        Iref = data[0]
        self._write_iref(Iref)

    @PS.ExceptionHandler
    def write_CurrentSetpoint(self, wattr):
        '''Writes the current setpoint.
        '''
        data = []
        wattr.get_write_value( data )
        self.cache['CurrentSetpoint'] = data[0]
        self._update_current()

    def _update_current(self):
        Iset = self.cache['CurrentSetpoint']
        Iref = self._correct('CurrentSetpoint', Iset)
        self._write_iref(Iref)

    def _write_iref(self, Iref):
        '''Writes reference current to power supply and starts the current ramp,
           if power supply signal readiness and power is on.
        '''
        # writing nans will not be attempted, confuses the power supply
        if str(Iref)=="nan": return
        self._write_float( ADDR_CURRENT_REFERENCE, Iref )
        self._start_ramp()


    @PS.ExceptionHandler
    def read_CurrentOffset(self, attr):
        v = self._get_attr_correction('CurrentSetpoint')['Offset']
        attr.set_value(v)

    @PS.ExceptionHandler
    def write_CurrentOffset(self, wattr):
        data = []
        wattr.get_write_value(data)
        self._set_attr_correction('CurrentSetpoint', Offset=data[0])
        self._update_current()

    @PS.ExceptionHandler
    def read_ALL(self, attr):
        attr.set_value(self.ALL)

    @PS.ExceptionHandler
    def ApplyCurrent(self):
        self._start_ramp()

    @PS.ExceptionHandler
    def UpdateState(self):
        # updates registers and state machine
        info = self.STAT
        self.log.debug('update state...')

        # executes pending on/off command, if any
        self.log.info("pending %s @ %s <= %s", self._pending_bits, self._pending_latest, time.time())
        if self._pending_bits is not None:

            state = self.get_state()
            # discards pending op when timing out
            if time.time() > self._pending_latest:
                self._fault('pending timeout %s' % [self._pending_bits] )
                self._pend_oo(None)

            # waits until ready to execute pending operation
            elif state in (DevState.ON, DevState.OFF):
                pass

            # executes pending operation when ready
            elif state==DevState.STANDBY:
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


        # test whether its okay to read again
        if self._next_read_time > time.time(): return

        try:
            start_time = time.time()
            regs = self.modbus.ReadHoldingRegisters( [0,NREGISTER] )
            reg_str = struct.pack('h'*NREGISTER, *regs)
            self.ALL = list(struct.unpack('H'*NREGISTER, reg_str ))
            self._next_read_time = 0
            for (aname, rfun) in HazemeyerIocaste_PS._READ:
                self.cache[aname] = value = rfun(self)
                self.log.debug("    %s = %s", aname, value)
                self.push_change_event(aname, value)

            self.log.debug('')
            self.cache["pending_bits"] = self._pending_bits
            self._check_ramp()
            info.update(self.ALL[ADDR_STATE:ADDR_STATE+5], self.cache)

        except DevFailed, dx:
            now = time.time()
            time_to_fail = now - start_time
            reason = dx[-1]["reason"]
            desc = dx[-1]["desc"]
            self.log.error("%s after %s because %s ", reason, time_to_fail, desc)
            self._next_read_time = now + RECONNECT_DELAY_s

            mute = reason in ('TimeWAITBetweenRetries', 'API_CommandFailed', 'API_DeviceTimedOut', 'API_CantConnectToDevice')
            if mute:
                self._alarm('communication with control unit via %r failed' % self.ModbusDevice)
            else:
                self._fault("unexpected communication problem %r!\n%s" % (reason,desc) )

    def _write_float(self, addr, val):
        val_byte = struct.pack('f', val)
        val_short = struct.unpack('hh', val_byte)
        args = [ addr, 2 ] + list(val_short)
        self.modbus.PresetMultipleRegisters( args )

    def _command_bit(self, *nmo):
        '''Writes one or more command bits.
        '''
        cmd_byte = 0
        for n in nmo:
            assert 0 <= n <= 15, "bit index must be between 0 and 15"
            cmd_byte |= 1<<n
        args = [ADDR_COMMAND, cmd_byte]
        self.modbus.PresetSingleRegister( args )

    @PS.ExceptionHandler
    def read_Voltage(self, attr):
        self._correct_attr('Voltage', attr, self.cache.get('Umeas'))

    @PS.ExceptionHandler
    def read_Current(self, attr):
        self._correct_attr('Current', attr, self.cache.get('Imeas'))

    @PS.ExceptionHandler
    def read_CurrentTrigger(self, attr):
        attr.set_value(self.cache.get('CurrentTrigger', 2))

    @PS.ExceptionHandler
    def write_CurrentTrigger(self, wattr):
        data = []
        wattr.get_write_value(data)
        self.cache['CurrentTrigger'] = data[0]

    def _get_ipaddress(self):
        low,high = self.ALL[ADDR_IP_ADDRESS:ADDR_IP_ADDRESS+2]
        ip = low>>8, (low & 0xFF), high>>8, high & 0xFF
        return ".".join(map(str,ip))

class HazemeyerIocaste_PS_Class(DeviceClass):

    class_property_list = PS.gen_property_list( ('Tolerance','ModbusTimeout'), XI = (1,4) )
    class_property_list['ModbusTimeout'][2] = 50

    device_property_list = PS.gen_property_list( ('ModbusDevice', ))
    device_property_list.update(class_property_list)

    device_property_list['Type'] = [ DevString, "2, 4 or 6", [] ]


    cmd_opt = ('Standby', 'ApplyCurrent', 'UpdateState')
    cmd_list = PS.gen_cmd_list(opt=cmd_opt)
    cmd_list['UpdateState'][2]['polling period'] = 250

    #    Attribute definitions
    attr_opt = ('CurrentOffset', 'CurrentTrigger', 'Resistance')
    attr_list = PS.gen_attr_list(max_err=(1+4)+16, opt=attr_opt)
    attr_list['ALL'] =  [ [ DevUShort, SPECTRUM, READ, NREGISTER ] ]

### Attribute Machinery ###
def add_attr(aname, tp, rfun, wfun=None, dim=SCALAR, x=0, extra={}):
    read_fun_name = "read_"+aname
    write_fun_name = "write_"+aname

    rw = READ if wfun is None else READ_WRITE

    # all attributes are obtained using 'rfun' and stored into cache dictionary
    if not read_fun_name in HazemeyerIocaste_PS.__dict__:
        def read_fun(self, attr):
            if not aname in self.cache:
                reason = 'value of %r unknown' % aname
                if self.ALL[0]==USHORT_FILL:
                    reason += ", probably because no data has been read, yet."
                raise PS.PS_Exception(reason)
            else:
                attr.set_value( self.cache[aname] )
        setattr(HazemeyerIocaste_PS, read_fun_name, read_fun)
        if rfun is not None:
            HazemeyerIocaste_PS._READ.append( (aname,rfun) )

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

def add_double(aname, addr=None, waddr=None, extra={}):
    rfun = partial(HazemeyerIocaste_PS._get_float, addr=addr)
    if waddr is None:
        wfun = None
    else:
        def wfun(self, value):
            HazemeyerIocaste_PS._write_float(self, waddr, value)
    add_attr(aname, DevDouble, rfun, wfun, SCALAR, extra=extra)

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


add_ushort('HazemeyerCode', ADDR_HAZEMEYER_CODE)
add_ushort('PowerSupplyId', ADDR_POWERSUPPLY_ID)
add_ushort('IocasteCode', ADDR_IOCASTE_CODE)
add_ushort_array('Version', ADDR_VERSION, 3,
    extra={'description':'MB441 and REG445 ACQ, REG programm revisions'}
    )

add_attr('IpAddress', DevString, rfun=HazemeyerIocaste_PS._get_ipaddress)

add_binstring('Interlock1', ADDR_INTERLOCK1)
add_binstring('Interlock2', ADDR_INTERLOCK2)
add_binstring('RegulationFault', ADDR_REGULATION_FAULT)
add_binstring('InternalFault', ADDR_INTERNAL_FAULT)
add_binstring('StateBits', ADDR_STATE)

# attributes will be evaluated in order of definition!
A_extra = dict(unit='A', format='%7.4f')
add_attr('CurrentOffset', DevDouble, extra={ 'format':'%7.4f', "unit":'A'}, rfun=None, wfun=True)
add_double('Imeas', extra=A_extra, addr=ADDR_CURRENT_MEASURE)
add_double('Iref', extra=A_extra, addr=ADDR_CURRENT_REFERENCE_READBACK, waddr=ADDR_CURRENT_REFERENCE)
gcss = lambda self: self._correct('CurrentSetpoint',self.cache.get('Iref'), inv=True)
add_attr('CurrentSetpoint', DevDouble, extra=A_extra, rfun=gcss, wfun=True)
add_attr('Current', DevDouble, extra=A_extra, rfun=None)
add_double('Umeas',  extra={ "unit":'V'}, addr=ADDR_VOLTAGE_MEASURE)
add_double('CurrentRampLocal',  extra={ 'unit':'A/s'}, addr=ADDR_CURRENT_RAMP_LOCAL)
add_double('CurrentRamp',  extra={ 'unit':'A/s'}, addr=ADDR_CURRENT_RAMP)

add_state_bit('RemoteMode', BIT_REMOTE)
add_state_bit('Ready', BIT_READY)
add_state_bit('Fault', BIT_GLOBAL_FAULT)
add_state_bit('Busy', BIT_NOBUSY)

if __name__ == '__main__':
    PS.tango_main(HazemeyerIocaste_PS)
