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

from PowerSupply.util import bit, bits, bit_filter_msg, binstr, UniqList, bitpack, bitunpack
import PowerSupply.standard as PS
PM = PS.MSG

TYPE_TRANS = {
    2 : 2,
    '2' : 2,
    'bend' : 2,
    'b' : 2,
    'd' : 2,
    'di' : 2,
    'dipole' : 2,
    4 : 4,
    '4' : 4,
    'q' : 4,
    'quad' : 4,
    'quadrupole' : 4 ,
    6 : 6,
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
ADDR_IOCASTE_ID = 38
ADDR_POWERSUPPLY_ID = 40

ADDR_VERSION = \
ADDR_MB441_PROGRAM_REVISION = 41
ADDR_REG445_ACQ_PROGRAM_REVISION = 42
ADDR_REG445_REG_PROGRAM_REVISION = 43

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

# number of bits important of each register
BITCOUNT = ( 2, 16, 4, 7, 0 )
BITOVER =  ( 0,  0, 1, 1, 1 )

# other global variables
# a random fill value with a characteristic pattern to aid in debugging
USHORT_FILL = 0x5555
RECONNECT_DELAY_s = 5.0
ALBAS_DEFAULT_CURRENT_RAMP = 20.0

# basic messages
UNDOCUMENTED = PM.UNDOCUMENTED_BIT
MSG_STATE = [
    "power on", 'power off', 'ready', 'local control', 'remote control',
    UNDOCUMENTED.format('state', 5), 'fault',
    'standby', 'regulation ok'
] + [
    UNDOCUMENTED.format('state', r) for r in range(9, 15) ] + [
    'interlock bypass on'
]

# 'None' values indicate that it is no error for the bit to be set
MSG_INTERLOCK2 = [ UNDOCUMENTED.format('interlock 2', r) for r in range(16)]
MSG_INTERLOCK2[0] += ' / standby'

REGF = UNDOCUMENTED.format('regulation fault', '{0}')
MSG_REGULATION_FAULT = [
    "I AC fault",
    'I AC 1',
    'I AC 2',
    'earth current fault',
    REGF.format(4),
    'over voltage',
    'over current',
] + [ REGF.format(r) for r in range(7,16) ]


MSG_INTERNAL_FAULT = [
    'IAC Fonction Fault',
    'Intor FOnction Fault',
    'EEPROM data fault',
    'EEPROM timeout',
    'REG Com',
    'display fault',
] + [ UNDOCUMENTED.format('internal fault', r) for r in xrange(6,16) ]


# template for message definition
MSG = [ MSG_STATE,
        None, MSG_INTERLOCK2 , MSG_REGULATION_FAULT, MSG_INTERNAL_FAULT ]
MSG_KEYS = [ 'State' , 'Interlock1', 'Interlock2', 'RegulationFault', 'InternalFault' ]
MASK = [ 0 ] * 5

# Quadrupoles
MSG_QUAD = deepcopy(MSG)
MSG_QUAD[1] = [
    PM.OVERTEMP.format('L2'),
    PM.OVERTEMP.format('L3'),
    PM.OVERTEMP.format('L4'),
    PM.OVERTEMP.format('L5'),
    PM.OVERTEMP.format('high frequency transformer'),
    PM.OVERTEMP.format('cold plate'),
    PM.AMBIENT_TEMP,
    'phases fault',
    PM.EXTERNAL_INTERLOCK.format(1),
    PM.EXTERNAL_INTERLOCK.format(2),
    PM.EXTERNAL_INTERLOCK.format(3),
    PM.EXTERNAL_INTERLOCK.format(4),
    PM.OVERTEMP.format('BF'),
    UNDOCUMENTED.format('interlock 1', 13),
    "IGBT driver fault, see documentation of SKHI22A",
    UNDOCUMENTED.format('interlock 1', 15),
]
MSG_QUAD[2][0:4] = [
    PM.DOOR,
    PM.CABINET_WATER,
    PM.EMERGENCY_STOP,
    'SD breaker fault'
]

MASK_QUAD = deepcopy(MASK)
MASK_QUAD[2] = 0
MASK_QUAD[3] = 9

# error messages for sextupoles magnets power supplies
MSG_SEXT = deepcopy(MSG)
MSG_SEXT[1] = [
    PM.DOOR,
    PM.EMERGENCY_BUTTON_PUSHED,
    PM.EXTERNAL_INTERLOCK.format(1),
    "coils temperature",
    "transfo temperature",
    "thermal relay",
    "rectifier temperature",
    "capa fuses",
    "circuit breaker",
    "phase",
    PM.CABINET_WATER,
    PM.EXTERNAL_INTERLOCK.format(2),
    PM.EXTERNAL_INTERLOCK.format(3),
    PM.EXTERNAL_INTERLOCK.format(4),
    "optical 1",
    "optical 2"
]
MSG_SEXT[2][0] += ' (ignored)'

MASK_SEXT = deepcopy(MASK)
MASK_SEXT[2] = 0x000F
MASK_SEXT[3] = 0x000F

# Bending Magnet Definitions identical to sextupoles
MSG_BEND = deepcopy(MSG_SEXT)
# only difference
MSG_BEND[1][9] = "DC overcurrent"
MASK_BEND = MASK_SEXT

PSTYPE_INFO = {
    2 : ( 'bending', MSG_BEND, MASK_BEND, 1, (2,11,12,13) ),
    4 : ( 'quadrupole', MSG_QUAD, MASK_QUAD, 0, (8,9,10,11)  ),
    6 : ( 'sextupole', MSG_SEXT, MASK_SEXT, 1, (2,11,12,13) )
}
def msg_check():
    for k,(name,msg,mask,ign,xi) in PSTYPE_INFO.iteritems():
        for m in msg:
            assert len(m)==16, 'msg type {0} has length {1}\n{2}'.format(k, len(m),m)
msg_check()


class IocasteStateLogic(PS.StateLogic):
    '''Class handles generation of State, Status and Erros attributes.'''

    def __init__(self, device, key):
        PS.StateLogic.__init__(self, device)
        key = str(key).lower()
        self.type_code = type_code = TYPE_TRANS[key]
        self.name, msgs, self.mask, ign, self.xi = PSTYPE_INFO[type_code]
        msgs = deepcopy(msgs)

        if device is None:
            self.alarms = UniqList()
            self.reg_prec_moving = 0.1
            self.reg_prec_static = 0.1
        else:
            self.alarms = device.transient_messages
            # a better solution would be to use an extra instability time
            # after the move (ask Tiago for this)
            self.reg_prec_static = device.RegulationPrecision
            self.reg_prec_moving = device.RegulationPrecision / 3.0
        self.move_min_t = 0

        # setup messages
        self.msg_state = msgs[0]
        self.msg_interlock1 = msgs[1]
        self.msg_interlock2 = msgs[2]
        self.msg_regulation_fault = msgs[3]
        self.msg_internal_fault = msgs[4]
        self.ignore_interlock2 = ign

    def update(self, state_words, values):#,isCycling=False,cyclingStep=None,cyclingTS=None):
        values.setdefault('pending bits', 0)
        sw = tuple(state_words)
        alarms = UniqList() #< this are NEW alarms

        sw_masked = [ sw[i] ^ M for i,M in enumerate(self.mask) ]
        sw_masked[2] &= ~self.ignore_interlock2
        # its hardcoded because it seems fix for the different types
        sw_masked[3] &= 0xFFFE
        st, ione, itwo, regf, inff = values['StateWords cooked'] = sw_masked

        # translate error bits set in state byte into message
        if bit(st, BIT_INTERLOCK_BYPASS):
            alarms += [ self.msg_state[BIT_INTERLOCK_BYPASS] ]

        # global interlock bit is silently ignored
        ilk_msg = bit_filter_msg(ione, self.msg_interlock1) + \
                bit_filter_msg(itwo, self.msg_interlock2) + \
                bit_filter_msg(regf, self.msg_regulation_fault) + \
                bit_filter_msg(inff, self.msg_internal_fault)

        alarms += ilk_msg

        # shortcuts variables used below, mostly for clarity
        PEND_BIT = values['pending bits']


        # if the regulation bit provided state register works use this
        # REG_OK = bit(st, BIT_REGOK)

        if 'CurrentSetpoint' in values and 'Current' in values:
            Idelta = abs(values['CurrentSetpoint'] - values['Current'])
            if self.state == PS.Tg.DevState.MOVING:
                REG_OK = Idelta < self.reg_prec_moving
            else:
                REG_OK = Idelta < self.reg_prec_static
        else:
            REG_OK = True

        if values.get('CurrentRamp',None)==0.0:
            self.FAULT('current ramp not configured')
            
        elif values['_isCycling']:
            msg = "magnet is cycling"
            cycling = values['_cycling']
            if cycling['stepIsMoving']:
                laspseRampTime = time() - cycling['stepRampStartTS']
                remainingTime = cycling['expectedRampTime'] - laspseRampTime
                msg += ", changing current (%ds left)"%(int(remainingTime))
                if cycling['stepRampRetry']>1:
                    msg += "(ramp retry %d)"%(cycling['stepRampRetry']-1)
                self.MOVING(msg)
            elif cycling['stepWaitStartTS'] == None:
                self.DISABLE(msg)
            elif cycling['stepIdx'] == None:
                msg += ', unknow step'
                self.ALARM(msg)
            else:
                if cycling['stepIdx'] in [1,3,5]:
                    msg += ", waiting on maximum"
                elif cycling['stepIdx'] in [2,4]:
                    msg += ", waiting on minimum"
                lapseStepTime = time() - cycling['stepWaitStartTS']
                remainingTime = cycling['expectedWaitTime'] - lapseStepTime
                msg += " (%ds left)"%(int(remainingTime))
                self.DISABLE(msg)
        elif values['_cycling']['alarm']:
            self.ALARM('error detected during cycling, please acknowledge')

        elif bit(st, BIT_POWERON):
            if not REG_OK or time() < self.move_min_t:
                self.CURRENT_ADJUST()
            elif PEND_BIT ==BIT_POWEROFF:
                self.SWITCHING_OFF()
            else:
                self.ON()

        elif len(ilk_msg):
            self.ALARMS(alarms)

        elif bit(st, BIT_GLOBAL_FAULT):
            self.ALARM('interlock or fault had been detected, please acknowledge')

        elif bit(st, BIT_STANDBY):
            self.STANDBY()

        elif bit(st, BIT_POWEROFF):
            if PEND_BIT==BIT_POWERON:
                self.SWITCHING_ON()
            else:
                self.OFF()

        else:
            msg = "internal error, unforseen state words "
            try:
                ststr = ('%x '*len(sw)) % sw
                msg += ststr
            except Exception, exc:
                msg += str(sw) + " (also %s)" % str(exc)
            self.FAULT(msg)


    def strstate(self, word):
#        NAME, MSG, MASK, IGN, XI = PSTYPE_INFO[TYPE_TRANS[type_code]]
#        logic = IocasteStateLogic(None, MSG, MASK, IGN)
#        if self.IGN==1:
#            logic.ignore_interlock2 = False
#        elif IGN==0:
#            pass
#        else:
#            raise Exception('does not know how to handle IGN='+str(IGN))
        values = {}
        self.update(word, values)
        msg = UniqList()
        msg.extend(bit_filter_msg(word[0], self.msg_state))
        msg.extend(self.device.messages)
        return msg

    def strerror_i32(self, code):
        word, overflow = decode_error_i32(code, extra=True)
        msg = self.strstate(word)
        msg += [ m for flag,m in zip(overflow[2:], [
            UNDOCUMENTED.format('interlock 2', '5 to 15'),
            REGF.format('7 to 15'),
            'internal fault'] ) if flag ]
        return word, msg


def encode_error_i32(sw):
    '''Packs interlock and error related bits into a signed 32 bit value
       into an unsigned 31 bit value, the sign indicates an unspecified
       internal error.
       All status words sw should be positive
    '''
    assert (s >= 0 for s in sw), 'status words must not be negative'
    st_fault = sw[0] >> BIT_GLOBAL_FAULT & 1
    st_inhibit = sw[0] >> BIT_INTERLOCK_BYPASS & 1
    word = copy(sw)
    word[0] = st_fault | st_inhibit << 1
    code_u32 = bitpack(BITCOUNT, BITOVER, word)
    code_u31, internal_fault = code_u32 >> 1, code_u32 & 1
    if internal_fault:
        return -code_u31
    else:
        return code_u31

def decode_error_i32(code, extra=False):
    '''Attempts to convert the error code back into register values.
       :see: assemble_error_code for limitation of data recovered
    '''
    internal_fault = int(code<0)
    code = abs(code) << 1
    code |= internal_fault
    word, overflow = bitunpack(BITCOUNT, BITOVER, code)
    st_fault = word[0] & 1
    st_inhibit = 1 if word[0] & 2 else 0
    word[0] = st_inhibit << BIT_INTERLOCK_BYPASS | st_fault << BIT_GLOBAL_FAULT

    if extra:
        return word, overflow
    else:
        for idx, (o, b) in enumerate(zip(overflow, BITCOUNT)):
            word[idx] |= o * (2**b-1)
        return word

def encode_state_u64(sw):
    '''Packs state, error and interlock register bits into a single
       64 bit value. Only lowest 46 bits are used.
    '''
    # sign counts as one bit
    i30 = encode_error_i32(sw) >> 2
    if i30>=0:
        u30 = i30
    else:
        # convert sign bit into bit 30
        u30 = -i30 | 2**30

    return sw[0] | u30 << 16

def decode_state_u64(code):
    st = code & 0xFFFF
    u30 = code>>16
    sw = decode_error_i32(u30)
    sw[0] = st
    return sw

def format_hex(ls):
    return '{0:4x} {1:4x} {2:2x} {3:2x} {4:x}'.format(*ls)

if __name__== "__main__" :
    import sys
    args = sys.argv[1:]
    if not len(args) in (2,6):
        print 'usage: explain TYPE STATE INTERLOCK1 INTERLOCK2 REGF INTERNAL'
        print '(hexadecimal)'
        sys.exit(1)

    logic = IocasteStateLogic(None, args[0])
    print 'Hazemeyer Iocaste {0} power supply\n'.format(logic.name), 'ErrorCode',
    if len(args)==2:
        code = eval(args[1])
        print hex(code)
        word, msg = logic.strerror_i32(code)
    elif len(args)==6:
        word0 = [ int(r,16) for r in args[1:] ]
        code = encode_error_i32(word0)
        print hex(code)
        print '  input word', format_hex(word0)
        word = decode_error_i32(code)
        msg = logic.strstate(word0)
    else:
        print 'usage error!'
        sys.exit(1)
    print 'decoded word', format_hex(word)
    if len(msg)==0:
        msg.append('okay')
    print '----'
    print '\n'.join(msg)
