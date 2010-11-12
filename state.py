#!/usr/bin/env python
# -*- coding: utf-8 -*-

# state.py
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

'''Contains mostly StateLogic and global constants shared between device server
   for Hazemeyer power supplies and modbus-print, diagnostic utility.
'''

from copy import copy, deepcopy
from time import time
from PyTango import DevState

from PowerSupply.util import bit, bits, bit_filter_msg, binstr, UniqList
import PowerSupply.standard as PS

TYPE_TRANS = {
    '2' : 2,
    'bend' : 2,
    'b' : 2,
    'd' : 2,
    'di' : 2,
    'dipole' : 2,
    '4' : 4,
    'q' : 4,
    'quad' : 4,
    'quadrupole' : 4 ,
    '6' : 6,
    's' : 6,
    'sext' : 6,
    'sextupole' : 6,
}

# in contrast to 'mapping' documentation our indices start at 0, so that
# address of nth register is n-1
ADDR_COMMAND = 0
ADDR_CURRENT_RAMP = 1
ADDR_IREF_WRITE = 5

ADDR_STATE = 15
ADDR_INTERLOCK1 = 16
ADDR_INTERLOCK2 = 17
ADDR_REGULATION_FAULT = 18
ADDR_INTERNAL_FAULT = 19
ADDR_I_MEASURE = 24
ADDR_IREF_REMOTE_READ = 26
ADDR_VOLTAGE_MEASURE = 28
ADDR_CURRENT_RAMP_LOCAL = 34
ADDR_PROJECT_ID = 36

ADDR_CODE = \
ADDR_HAZEMEYER_CODE = 38
ADDR_IOCASTE_CODE = 39
ADDR_POWERSUPPLY_ID = 40

ADDR_VERSION = \
ADDR_MB441_PROGRAM_REVISION = 41
ADDR_REG445_ACQ_VERSION = 42
ADDR_REG445_REG_VERSION = 43

ADDR_IP_ADDRESS = 44
ADDR_IREF_READ = 46
NREGISTER = 48

# command bits
BIT_POWERON = 0
BIT_POWEROFF = 1
BIT_RESET_FAULT = 2  #< resets error flags
BIT_RAMPSTART = 3

# state bits
BIT_READY = 2
BIT_LOCAL = 3
BIT_REMOTE = 4
BIT_GLOBAL_FAULT = 6
BIT_STANDBY = 7
BIT_REGOK = 8
BIT_INTERLOCK_BYPASS = 15

# useful masks for testing states
MASK_ALARM = 1<<BIT_GLOBAL_FAULT
MASK_POWER = 1<<BIT_POWERON | 1<<BIT_POWEROFF
MASK_INV_MAINS_FAULT = 0xe

# message lists for each interlock register are defined as found in
# documentation, starting with bit 15 counting down to 0
# to be accessible by bit index the lists are reversed after their definition.
MSG_STATE = [ "interlock bypass on" ] + \
    [ "unused state bit %s" % r for r in range(14,6,-1) ] + \
    [ "global fault",
      "no busy",
      "local control",
      "remote control",
      "ready",
      "power off",
      "power on"
    ]
MSG_STATE.reverse()

# 'None' values indicate that it is no error for the bit to be set
MSG_INTERLOCK2 = [ 'interlock2 bit %s' % r for r in range(0,16) ]

MSG_REGULATION_FAULT = [
    "IAC fault",
    "regulation fault bit 1",
    "regulation fault bit 2",
    "earth current fault",
    "regulation fault bit 4",
    "over current",
    "over voltage",
] + [ "regulation fault, unused bit %s" % r for r in range(7,16) ]

MSG_INTERNAL_FAULT = [
    "internal fault, bit %s" % r for r in range(15,6,-1) ] + [
    "display fault",
    "REG Com",
    "EEPROM timeout",
    "EEPROM data fault",
    "Def Card Fonction Fault",
    "Intor FOnction Fault",
    "IAC Fonction Fault"
]
MSG_INTERNAL_FAULT.reverse()

MASK = [ 0 ] * 5

# basic messages
MSG = [ MSG_STATE, None, MSG_INTERLOCK2 , MSG_REGULATION_FAULT, MSG_INTERNAL_FAULT ]
MSG_KEYS = [ 'State' , 'Interlock1', 'Interlock2', 'RegulationFault', 'InternalFault' ]
MSG_DICT = dict(zip(MSG_KEYS,MSG))


# Quadrupoles
MSG_QUAD = deepcopy(MSG)
MSG_QUAD[1] = [
    "over temperature L2",
    "over temperature L3",
    "over temperature L4",
    "over temperature L5",
    "over temperature high frequency transformer",
    "over temperature coldplate",
    "ambient temperature too high",
    "phases fault",
    PS.MSG.EXTERNAL_INTERLOCK.format(1),
    PS.MSG.EXTERNAL_INTERLOCK.format(2),
    PS.MSG.EXTERNAL_INTERLOCK.format(3),
    PS.MSG.EXTERNAL_INTERLOCK.format(4),
    "over temperature BF",
    "interlock1 bit 13",
    "IGBT driver fault, see documentation of SKHI22A",
    "interlock 1 bit 15"
]
MSG_QUAD[2][0:4] = [
    PS.MSG.DOOR,
    PS.MSG.CABINET_WATER,
    PS.MSG.EMERGENCY_STOP,
    'SD breaker fault'
]

MASK_QUAD = deepcopy(MASK)
MASK_QUAD[2] = 0
MASK_QUAD[3] = 9

# error messages for sextupoles magnets power supplies
MSG_SEXT = deepcopy(MSG)
MSG_SEXT[1] = [
    PS.MSG.DOOR,
    PS.MSG.EMERGENCY_BUTTON_PUSHED,
    PS.MSG.EXTERNAL_INTERLOCK.format(1),
    "coils temperature",
    "transfo temperature",
    "thermal relay",
    "rectifier temperature",
    "capa fuses",
    "circuit breaker",
    "phase",
    PS.MSG.CABINET_WATER,
    PS.MSG.EXTERNAL_INTERLOCK.format(2),
    PS.MSG.EXTERNAL_INTERLOCK.format(3),
    PS.MSG.EXTERNAL_INTERLOCK.format(4),
    "optical 1",
    "optical 2"
]

MASK_SEXT = deepcopy(MASK)
MASK_SEXT[2] = 0x000F
MASK_SEXT[3] = 0x000F

# Bending Magnet Definitions identical to sextupoles
MSG_BEND = deepcopy(MSG_SEXT)
# only difference
MSG_BEND[1][9] = "DC overcurrent"
MASK_BEND = MASK_SEXT

PSTYPE_INFO = {
    2 : ( MSG_BEND, MASK_BEND, (2,11,12,13) ),
    4 : ( MSG_QUAD, MASK_QUAD, (8,9,10,11) ),
    6 : ( MSG_SEXT, MASK_SEXT, (2,11,12,13))
}
def msg_check():
    for k,(msg,mask,xi) in PSTYPE_INFO.iteritems():
        for m in msg:
            assert len(m)==16, 'msg {0} has length {1}\n{2}'.format(k, len(m),m)
msg_check()

# global constants

# a random fill value with a characteristic pattern to aid in debugging
USHORT_FILL = 0x5555

RECONNECT_DELAY_s = 5.0

class IocasteStateLogic(PS.StateLogic):
    '''Class handles generation of State, Status and Erros attributes.'''

    def __init__(self, device, msgs=MSG, masks=MASK):
        PS.StateLogic.__init__(self, device)
        if device is None:
                self.alarms = UniqList()
        else:
                self.alarms = device.alarms
        self.set_msg(msgs, masks)
        self.move_min_t = 0

    def set_msg(self, msgs, mask):
        self.msg_state = msgs[0]
        self.msg_interlock1 = msgs[1]
        self.msg_interlock2 = msgs[2]
        self.msg_regulation_fault = msgs[3]
        self.msg_internal_fault = msgs[4]
        self.mask = mask

    def update(self, state_words, values):
        sw = tuple(state_words)
        values.setdefault('Current', 0.0)
        values.setdefault('CurrentSetpoint', 0.0)
        values.setdefault('pending_bits', None)
        self._last_state_words = state_words
        alarms = self.alarms
        alarms.clear()

        sw_masked = [ sw[i] ^ M for i,M in enumerate(self.mask) ]
        sw_masked[3] = sw_masked[3] & 0xFFFE
        values['StateWords'] = sw_masked
        st, ione, itwo, regf, inff = sw_masked

        # translate error bits set in state byte to error message
        if bit(st, BIT_INTERLOCK_BYPASS):
            alarms += [ self.msg_state[BIT_INTERLOCK_BYPASS] ]

        # global interlock bit is silently ignored
        ilk_msg = bit_filter_msg(ione, self.msg_interlock1) + \
                  bit_filter_msg(itwo, self.msg_interlock2) + \
                  bit_filter_msg(regf, self.msg_regulation_fault) + \
                  bit_filter_msg(inff, self.msg_internal_fault)

        alarms += ilk_msg
        values["Alarm"] = ", ".join(ilk_msg)
        alarms += bit_filter_msg(regf, self.msg_regulation_fault)
        alarms += bit_filter_msg(inff, self.msg_internal_fault)

        self.power_on = bit(st, BIT_POWERON)

        if self.power_on and (not bit(st, BIT_REGOK) or time()<self.move_min_t):
                self.CURRENT_ADJUST()

        elif self.power_on:
                self.ON()

        elif len(ilk_msg):
            self.ALARMS(alarms)

        elif bit(st, BIT_GLOBAL_FAULT):
            self.ALARM('interlock or fault had been detected, please acknowledge')

        elif bit(st, BIT_STANDBY):
            self.STANDBY()

        elif bit(st, BIT_POWEROFF):
            self.OFF()

        else:
            ststr = '%04x '*5 % sw
            msg = "internal error, unforseen state words " + ststr
            self.device._fault(msg)

        pend_bit = values["pending_bits"]
        if  pend_bit == BIT_POWERON:
            self.SWITCH_ON(**values)
        elif pend_bit == BIT_POWEROFF:
            self.SWITCH_OFF(**values)


def format_state(state_words):
    state_words += [ 0 ] * 5
    state_words = state_words[0:5]
    info = IocasteStateLogic(None)
    info.update( state_words, { } )
    err_msg = "\n".join(info.alarms)
    return  "\n".join(map(binstr, state_words)) + \
            "\nstate %s\n%s\n%s" % (info.state, info.status, err_msg)


if __name__== "__main__" :
    import sys
    vals =  map( eval, sys.argv[1:] )
    ## state.py 0x52 0xc02 0xf 0xf
    print format_state(vals)

