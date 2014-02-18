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
    $URL$
    $LastChangedBy$
    $Date$
    $Rev$
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
import subprocess
from time import time,sleep

# TANGO Imports
from PyTango import DevVoid, DevString, DevState, DevFailed, DevDouble, \
    Except, DevShort, Util, DevLong, DevUShort, \
    DevVarStringArray, DevVarShortArray, DeviceProxy, AttrQuality, SCALAR, \
    SPECTRUM, READ, READ_WRITE, Database, Device_3Impl, DeviceClass, \
    DevVarLongArray, DevBoolean

# extra Imports
import PowerSupply.standard as PS
from PowerSupply.util import *

# HazemeyerIocaste_PS
from state import *

class HazemeyerIocaste_PS(PS.PowerSupply):
    """Device server will allow to interact with ALBA HazemeyerIocaste_PS connected using Modbus over TCP.
    """

    ### Device Properties ###
    ModbusDevice = None
    ModbusTimeout = None

    # maps each attribute name to a function that returns the value of the
    # corresponding attributes. these attributes are then read in UpdateState()
    # and change events generated
    _READ = []
    PUSHED_ATTR = ( 'State', 'Status', 'Current', 'CurrentSetpoint', 'Voltage',
    )

    #
    _modbus_err_count = 0

    def __init__(self, cl, name):
        PS.PowerSupply.__init__(self, cl, name)
        self.init_device(cl, name)

    def init_device(self, cl=None, name=None):
        PS.PowerSupply.init_device(self, cl, name)

        # cache contains what the DS considers to be the current read values
        # of each attribute, updated from UpdateState
        self.cache = {}
        # initializes internal modbus data to fill pattern
        self.ALL = [ USHORT_FILL ] * NREGISTER
        # all bits are False, neither power on nor off
        self.ALL[ADDR_STATE] = 0

        self._next_read_time = time() #< when next read might be attempted
        self._pend_oo(None)

        # initializes connection
        self.modbus = DeviceProxy(self.ModbusDevice)
        self.modbus.set_timeout_millis(self.ModbusTimeout)

        # checks type property
        if not self.Type:
            self.Type = self.get_name().rpartition('/')[-1][0]
            self.log.info('guessing Type is %r' % self.Type)

        pctype = TYPE_TRANS[self.Type.lower()]

        msg,mask,xi = PSTYPE_INFO[pctype]

        #  customize external interlock messages
        msg[1] = copy(msg[1])
        for i,x in enumerate(xi):
            msg[1][x] = getattr(self, "Interlock%d" % (i+1) )
        self.STAT = IocasteStateLogic(self, msg, mask)
        self.STAT.INITIALIZED()

    def _check_ramp(self):
        '''Checks value of current ramp, when set to 0.0 the value of the
           'local' mode ramp is set. This function is called in UpdateState.
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
    @PS.CommandExc
    def On(self):
        '''Turns power supply on.'''
        self._check_remote()
        self._onoff(BIT_POWERON)

    @PS.CommandExc
    def Off(self):
        '''Turns power supply off.'''
        self._check_remote()
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

    @PS.CommandExc
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

    @PS.CommandExc
    def ResetInterlocks(self):
        '''Resets errors messages, in particular related to interlocks.
        '''
        PS.PowerSupply.ResetInterlocks(self)
        now = time()
        if self._next_read_time > now:
            self.STAT.COMM_RESET()
            self._next_read_time = now
        else:
            self._command_bit(BIT_RESET_FAULT)


    def _start_ramp(self):
        '''Starts ramping of the current.
        '''
        self._command_bit(BIT_RAMPSTART)
        # 0.3 must be slightly bigger than the polling time
        self.STAT.move_min_t = time()+0.1
        self.STAT.CURRENT_ADJUST()

    ### Conversion Functions ###
    _get_float = lambda self, addr: \
        struct.unpack('f', struct.pack('HH', *self.ALL[addr:addr+2] ) )[0]

    _get_bit = lambda self, addr, bit: \
        bool(self.ALL[addr] & (1<<bit))

    _get_binstring = lambda self, addr: \
        binstr(self.ALL[addr])

    ### Attributes ###
    @PS.AttrExc
    def write_I(self, wattr):
        Iref = wattr.get_write_value()
        self._write_iref(Iref)

    @PS.AttrExc
    def read_CurrentSetpoint(self, attr):
        if self._check_invalid(attr): return
        Iref = self._get_float(ADDR_IREF_READ)
        v = self._correct_attr('CurrentSetpoint', attr, Iref, inv=True)
        attr.set_write_value(v)

    @PS.AttrExc
    def write_CurrentSetpoint(self, wattr):
        '''Writes the current setpoint.
        '''
        try:
          sp = wattr.get_write_value()
          self.cache['CurrentSetpoint'] = sp
          self._update_current()
        except Exception:
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
            attr.set_quality(PS.AQ_INVALID)
            return True

    @PS.AttrExc
    def read_CurrentOffset(self, attr):
        if self._check_invalid(attr): return
        v = self._get_attr_correction('CurrentSetpoint')['Offset']
        attr.set_value(v)

    @PS.AttrExc
    def write_CurrentOffset(self, wattr):
        new_offset = wattr.get_write_value()
        self._set_attr_correction('CurrentSetpoint', Offset=new_offset)
        self._update_current()

    @PS.AttrExc
    def read_ALL(self, attr):
        if self._check_invalid(attr): return
        attr.set_value(self.ALL)

    @PS.CommandExc
    def ApplyCurrent(self):
        self._start_ramp()

    @PS.CommandExc
    def UpdateState(self):
        # checks whether host can be reach by ping

        # updates registers and state machine
        info = self.STAT

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
            start_time = time()
            regs = self.modbus.ReadHoldingRegisters( [0,NREGISTER] )

            reg_str = struct.pack('h'*NREGISTER, *regs)
            self.ALL = list(struct.unpack('H'*NREGISTER, reg_str ))
            self._next_read_time = 0
            for (aname, rfun) in HazemeyerIocaste_PS._READ:
                try:
                    self.cache[aname] = value = rfun(self)
                    self.push_change_event(aname, value)
                except Exception:
                    traceback.print_exc()
            self.cache["pending_bits"] = self._pending_bits 
            # generates push change events for attributes that are NOT in _READ
            for aname in ('Current', 'CurrentSetpoint', 'Voltage'):
                PA = self._read(aname)
                self.push_change_event(aname, *PA.triple)
            self._check_ramp()
#            self._check_apply()
            info.update(self.ALL[ADDR_STATE:ADDR_STATE+5], self.cache)

        except DevFailed, dx:
            self._modbus_err_count += 1
            now = time()
            time_to_fail = now - start_time
            reason = dx[-1].reason
            desc = dx[-1].desc
            if reason=='API_DeviceTimedOut':
                iphost = self.modbus.get_property('Iphost').get('Iphost', [])
                if iphost:
                    msg = 'host %s timed out' % iphost[0]
                else:
                    msg = 'timeout?'

            else:
                msg = dx[-1].desc
            self.log.error('UpdateState: %s:%s' %  (reason,desc))
            self._next_read_time = now + RECONNECT_DELAY_s
            self._next_read_time = float('inf')
            self.STAT.COMM_FAULT(msg)
            self.modbus.set_timeout_millis(self.ModbusTimeout)

        except Exception, exc:
            self.STAT.SERVER_ERROR(exc)
            raise


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
        sleep(0.05)
        self.modbus.PresetSingleRegister( [ADDR_COMMAND, cmd_byte])

    @PS.AttrExc
    def read_Voltage(self, attr):
        if self._check_invalid(attr): return
        self._correct_attr('Voltage', attr, self.cache.get('V', NAN))

    @PS.AttrExc
    def read_Current(self, attr):
        if self._check_invalid(attr): return
        v = self._correct_attr('Current', attr, self.cache.get('I', NAN))
        self.cache['Current'] = v

    def _get_ipaddress(self):
        low,high = self.ALL[ADDR_IP_ADDRESS:ADDR_IP_ADDRESS+2]
        ip = low>>8, (low & 0xFF), high>>8, high & 0xFF
        return ".".join(map(str,ip))

    @PS.AttrExc
    def read_StatusBits(self, attr):
        if self._check_invalid(attr): return
        y = '\n' + \
            'bit        fedc ba98 7654 3210\n' + \
            'state      '+binstr(self.ALL[ADDR_STATE])+ '\n' + \
            'interlock1 '+binstr(self.ALL[ADDR_INTERLOCK1]) + '\n' + \
            'interlock2 '+binstr(self.ALL[ADDR_INTERLOCK2]) + '\n' + \
            'regulation '+binstr(self.ALL[ADDR_REGULATION_FAULT]) + '\n' + \
            'internal   '+binstr(self.ALL[ADDR_INTERNAL_FAULT])
        attr.set_value(y)

class HazemeyerIocaste_PS_Class(DeviceClass):

    class_property_list = PS.gen_property_list(
        ('RegulationPrecision','ModbusTimeout'),
        XI = (1,4) )
    class_property_list['RegulationPrecision'][2] = 0.005
    class_property_list['ModbusTimeout'][2] = 1000

    device_property_list = PS.gen_property_list( ('ModbusDevice', ), cpl=class_property_list)
    device_property_list['Type'] = [ DevString, "2, 4 or 6", [] ]
    device_property_list['#Code'] = [ DevString, "equipment code", 'none' ]

    cmd_opt = ('Standby', 'ApplyCurrent', 'UpdateState')
    cmd_list = PS.gen_cmd_list(opt=cmd_opt)
    cmd_list['UpdateState'][2]['polling period'] = 250

    #    Attribute definitions
    attr_opt = ('CurrentOffset', 'Resistance')
    attr_list = PS.gen_attr_list(max_err=32, opt=attr_opt)
    attr_list['ALL'] =  [ [ DevUShort, SPECTRUM, READ, NREGISTER ] ]

### Attribute Machinery ###
def add_attr(aname, tp, rfun=None, wfun=None, dim=SCALAR, x=0, extra={}, rwfun=None):
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
                raise PS.PS_Exception(reason)
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

def add_short(aname, addr, extra={}):
    def rfun(self):
        return self.ALL[addr]
    add_attr(aname, DevShort, rfun, extra=extra)

add_ushort('HazemeyerCode', ADDR_HAZEMEYER_CODE)
add_ushort('PowerSupplyId', ADDR_POWERSUPPLY_ID)
add_ushort('IocasteCode', ADDR_IOCASTE_CODE)
add_ushort_array('Version', ADDR_VERSION, 3,
    extra={'description':'MB441 and REG445 ACQ, REG programm revisions'}
    )

add_attr('IpAddress', DevString, rfun=HazemeyerIocaste_PS._get_ipaddress)

add_short('Interlock1', ADDR_INTERLOCK1)
add_short('Interlock2', ADDR_INTERLOCK2)
add_short('Interlock3', ADDR_REGULATION_FAULT)
add_short('Interlock4', ADDR_INTERNAL_FAULT)

# attributes will be evaluated in order of definition!
A_extra = dict(unit='A', format='%7.4f')
add_attr('CurrentOffset', DevDouble, extra={ 'format':'%7.4f', "unit":'A'}, rfun=None, wfun=True)
add_double('I', extra=A_extra, addr=ADDR_I_MEASURE, waddr=ADDR_IREF_WRITE, rwaddr=ADDR_IREF_READ)
add_attr('CurrentSetpoint', DevDouble, extra=A_extra, wfun=True)
add_attr('Current', DevDouble, extra=A_extra, rfun=None)
add_attr('StatusBits', DevString)
add_double('Voltage',  extra={ "unit":'V'}, addr=ADDR_VOLTAGE_MEASURE)
add_double('V',  extra={ "unit":'V'}, addr=ADDR_VOLTAGE_MEASURE)
add_double('CurrentRampLocal',  extra={ 'unit':'A/s'}, addr=ADDR_CURRENT_RAMP_LOCAL)
add_double('CurrentRamp',  extra={ 'unit':'A/s'}, addr=ADDR_CURRENT_RAMP)

add_state_bit('RemoteMode', BIT_REMOTE)
add_state_bit('Ready', BIT_READY)
add_state_bit('Fault', BIT_GLOBAL_FAULT)
add_state_bit('RegulationOk', BIT_REGOK)

if __name__ == '__main__':
    PS.tango_main(HazemeyerIocaste_PS)

