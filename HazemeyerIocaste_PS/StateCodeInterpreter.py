#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# StateCodeInterpreter.py
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

META = u"""
    $URL: https://tango-ds.svn.sourceforge.net/svnroot/tango-ds/Servers/PowerSupply/HazemeyerIocaste_PS/tags/1.11.1/HazemeyerIocaste_PS.py $
    $LastChangedBy: sergiblanch $
    $Date: 2011-08-30 16:50:31 +0200 (Tue, 30 Aug 2011) $
    $Rev: 3894 $
    License: GPL3+
    Author: Sergi Blanch <sblanch@cells.es>
""".encode('latin1')

import sys
import PyTango
from state import *
from numpy import ndarray

class StateCodeInterpreter(PyTango.Device_4Impl):
    def __init__(self,cl, name):
        PyTango.Device_4Impl.__init__(self,cl,name)
        StateCodeInterpreter.init_device(self)

    def init_device(self):
        self.debug_stream("In ", self.get_name(), "::init_device()")
        self.change_state(PyTango.DevState.INIT)
        self.get_device_properties(self.get_device_class())
        
        self.change_state(PyTango.DevState.ON)
        self.StateWords = (0,0,0,0,0)
        self.number = 0
        self.cache = {}
        self.MagnetType = None

    def _decode(self):
        self.cache = {}#clean if there is a previous value interpreted
        bar = decode_state_u64(self.number)
        st,i1,i2,i3,i4 = self.StateWords = bar
        #prepare all the output attributes:
        NL = ' 0x%x\n'
        self.cache['StatusBits'] = ('\n'+
                                    'bit        fedc ba98 7654 3210\n'+
                                    'state      '+binstr(st) + NL % st +
                                    'interlock1 '+binstr(i1) + NL % i1 +
                                    'interlock2 '+binstr(i2) + NL % i2 +
                                    'regulation '+binstr(i3) + NL % i3 +
                                    'internal   '+binstr(i4) + NL % i4
                                   )

        #get messages structure
        if self.MagnetType == 2: messages = MSG_BEND
        elif self.MagnetType == 4: messages = MSG_QUAD
        elif self.MagnetType == 6: messages = MSG_SEXT

        for i,element in enumerate(['PowerStatus',
                                    'Interlock1',
                                    'Interlock2',
                                    'Regulation',
                                    'Internal']):
            try:
                self.cache[element] = self.StateWords[i]
                s,b = self.populateList(self.StateWords[i], messages[i])
                self.cache[element+'Str'] = s
                self.cache[element+'Array'] = b
            except Exception,exc:
                self.error_stream("In %s::_decode(%s) '%s' Exception: %s"\
                              %(self.get_name(),self.number,element,exc))
    #end _decode()
    
    def populateList(self,stWord,msgList):
        strMsgs = []
        booleans = []
        for i in range(len(msgList)):
            b = bool(stWord >> i & 1)
            if b: strMsgs.append(msgList[i])
            booleans.append(b)
        return strMsgs,booleans

    def Encode_StateCode64(self,argin):
        self.debug_stream("In %s::Encode_StateCode64(%s)"%(self.get_name(),argin))
        try:
            self.change_state(PyTango.DevState.RUNNING)
            self.StateWords = argin
            argout = encode_state_u64(argin)
            self.change_state(PyTango.DevState.ON)
            return argout
        except:
            self.change_state(PyTango.DevState.FAULT)

    def is_Encode_StateCode64_allowed(self):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True

    def Decode_StateCode64(self,argin):
        self.debug_stream("In %s::Decode_StateCode64(%s)"%(self.get_name(),argin))
        try:
            self.change_state(PyTango.DevState.RUNNING)
            self.number = argin
            self._decode()
            argout = self.StateWords
            self.change_state(PyTango.DevState.ON)
            return argout
        except:
            self.change_state(PyTango.DevState.FAULT)

    def is_Decode_StateCode64_allowed(self):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True


    def Encode_ErrorCode(self,argin):
        self.debug_stream("In %s::Encode_ErrorCode(%s)"%(self.get_name(),argin))
        try:
            self.change_state(PyTango.DevState.RUNNING)
            argout = encode_error_i32(argin)
            self.change_state(PyTango.DevState.ON)
            return argout
        except:
            self.change_state(PyTango.DevState.FAULT)

    def is_Encode_ErrorCode_allowed(self):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True

    def Decode_ErrorCode(self,argin):
        self.debug_stream("In %s::Decode_ErrorCode(%s)"%(self.get_name(),argin))
        try:
            self.change_state(PyTango.DevState.RUNNING)
            argout = decode_error_i32(argin)
            self.change_state(PyTango.DevState.ON)
            return argout
        except:
            self.change_state(PyTango.DevState.FAULT)

    def is_Decode_ErrorCode_allowed(self):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True

    def SetStatusBits(self,argin):
        self.debug_stream("In %s::SetStatusBits(%s)"%(self.get_name(),argin))
        try:
            self.change_state(PyTango.DevState.RUNNING)
            self.number = argin
            self._decode()
            #x = decode_state_u64(argin)
            #st,i1,i2,i3,i4 = self.StateWords = x
            #NL = ' 0x%x\n'
            #argout = ('\n'+
            #          'bit        fedc ba98 7654 3210\n'+
            #          'state      '+binstr(st) + NL % st +
            #          'interlock1 '+binstr(i1) + NL % i1 +
            #          'interlock2 '+binstr(i2) + NL % i2 +
            #          'regulation '+binstr(i3) + NL % i3 +
            #          'internal   '+binstr(i4) + NL % i4
            #         )
            argout = self.cache['StatusBits']
            print argout
            self.change_state(PyTango.DevState.ON)
            return argout
        except Exception,e:
            print e
            self.change_state(PyTango.DevState.FAULT)

    def is_SetStatusBits_allowed(self):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True

#------------------------------------------------------------------
#    Read PowerStatus{,Str,Array} attributes
#------------------------------------------------------------------
    def read_PowerStatus(self, attr):
        self.debug_stream("In %s::read_PowerStatus()"%self.get_name())
        if self.cache.has_key('PowerStatus'):
            attr.set_value(self.cache['PowerStatus'])
        else:
            attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
#---- PowerStatus attribute State Machine -----------------
    def is_PowerStatus_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True
    def read_PowerStatusStr(self, attr):
        self.debug_stream("In %s::read_PowerStatusStr()"%self.get_name())
        if self.cache.has_key('PowerStatusStr'):
            argout = self.cache['PowerStatusStr']
            attr.set_value(argout,len(argout))
        else:
            attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
    def is_PowerStatusStr_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True
    def read_PowerStatusArray(self, attr):
        self.debug_stream("In %s::read_PowerStatusArray()"%self.get_name())
        if self.cache.has_key('PowerStatusArray'):
            argout = self.cache['PowerStatusArray']
            attr.set_value(argout,len(argout))
        else:
            attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
    def is_PowerStatusArray_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True

#------------------------------------------------------------------
#    Read Interlock1{,Str,Array} attributes
#------------------------------------------------------------------
    def read_Interlock1(self, attr):
        self.debug_stream("In %s::read_Interlock1()"%self.get_name())
        if self.cache.has_key('Interlock1'):
            argout = self.cache['Interlock1']
            attr.set_value(argout)
        else:
            attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
    def is_Interlock1_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True
    def read_Interlock1Str(self, attr):
        self.debug_stream("In %s::read_Interlock1Str()"%self.get_name())
        if self.cache.has_key('Interlock1Str'):
            argout = self.cache['Interlock1Str']
            attr.set_value(argout)
        else:
            attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
    def is_Interlock1Str_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True
    def read_Interlock1Array(self, attr):
        self.debug_stream("In %s::read_Interlock1Array()"%self.get_name())
        if self.cache.has_key('Interlock1Array'):
            argout = self.cache['Interlock1Array']
            attr.set_value(argout)
        else:
            attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
    def is_Interlock1Array_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True

#------------------------------------------------------------------
#    Read Interlock2{,Str,Array} attributes
#------------------------------------------------------------------
    def read_Interlock2(self, attr):
        self.debug_stream("In %s::read_Interlock2()"%self.get_name())
        if self.cache.has_key('Interlock2'):
            argout = self.cache['Interlock2']
            attr.set_value(argout)
        else:
            attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
    def is_Interlock2_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True
    def read_Interlock2Str(self, attr):
        self.debug_stream("In %s::read_Interlock2Str()"%self.get_name())
        if self.cache.has_key('Interlock2Str'):
            argout = self.cache['Interlock2Str']
            attr.set_value(argout)
        else:
            attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
    def is_Interlock2Str_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True
    def read_Interlock2Array(self, attr):
        self.debug_stream("In %s::read_Interlock2Array()"%self.get_name())
        if self.cache.has_key('Interlock2Array'):
            argout = self.cache['Interlock2Array']
            attr.set_value(argout)
        else:
            attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
    def is_Interlock2Array_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True

#------------------------------------------------------------------
#    Read Regulation{,Str,Array} attributes
#------------------------------------------------------------------
    def read_Regulation(self, attr):
        self.debug_stream("In %s::read_Regulation()"%self.get_name())
        if self.cache.has_key('Regulation'):
            argout = self.cache['Regulation']
            attr.set_value(argout)
        else:
            attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
    def is_Regulation_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True
    def read_RegulationStr(self, attr):
        self.debug_stream("In %s::read_RegulationStr()"%self.get_name())
        if self.cache.has_key('RegulationStr'):
            argout = self.cache['RegulationStr']
            attr.set_value(argout)
        else:
            attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
    def is_RegulationStr_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True
    def read_RegulationArray(self, attr):
        self.debug_stream("In %s::read_RegulationArray()"%self.get_name())
        if self.cache.has_key('RegulationArray'):
            argout = self.cache['RegulationArray']
            attr.set_value(argout)
        else:
            attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
    def is_RegulationArray_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True

#------------------------------------------------------------------
#    Read Internal{,Str,Array} attributes
#------------------------------------------------------------------
    def read_Internal(self, attr):
        self.debug_stream("In %s::read_Internal()"%self.get_name())
        if self.cache.has_key('Internal'):
            argout = self.cache['Internal']
            attr.set_value(argout)
        else:
            attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
    def is_Internal_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True
    def read_InternalStr(self, attr):
        self.debug_stream("In %s::read_InternalStr()"%self.get_name())
        if self.cache.has_key('InternalStr'):
            argout = self.cache['InternalStr']
            attr.set_value(argout)
        else:
            attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
    def is_InternalStr_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True
    def read_InternalArray(self, attr):
        self.debug_stream("In %s::read_InternalArray()"%self.get_name())
        if self.cache.has_key('InternalArray'):
            argout = self.cache['InternalArray']
            attr.set_value(argout)
        else:
            attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
    def is_InternalArray_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True

#------------------------------------------------------------------
#    Read/Write MagnetType attributes
#------------------------------------------------------------------
    def read_MagnetType(self, attr):
        self.debug_stream("In %s::read_MagnetType()"%self.get_name())

        if self.MagnetType == 2: strType = 'bending'
        elif self.MagnetType == 4: strType = 'quadrupole'
        elif self.MagnetType == 6: strType = 'sextupole'
        else: strType = None

        if not strType == None:
            attr.set_value(strType)
        else:
            attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
    def write_MagnetType(self,attr):
        self.debug_stream("In %s::write_MagnetType()"%self.get_name())
        data=[]
        attr.get_write_value(data)
        data = str(data[0])
        self.debug_stream("Attribute value = ", data)
        self.MagnetType = TYPE_TRANS.get(data,None)
        self._decode()
    def is_Internal_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.FAULT]:
            return False
        return True

###
# auxiliar methods
    def change_state(self,newstate):
        self.debug_stream("In %s::change_state(%s)"%(self.get_name(),str(newstate)))
        self.set_state(newstate)
        self.push_change_event('State',newstate)
# end auxiliar methods
###

class StateCodeInterpreterClass(PyTango.DeviceClass):
    #    Command definitions
    cmd_list = {
        'Encode_StateCode64':
            [[PyTango.DevVarUShortArray, "List of errors (PowerStatus,Interlock1,Interlock2,regulation,internal)"],
             [PyTango.DevLong64, "StateCode64 value"]
            ],
        'Decode_StateCode64':
            [[PyTango.DevLong64, "StateCode64 value"],
             [PyTango.DevVarUShortArray, "List of errors (PowerStatus,Interlock1,Interlock2,regulation,internal)"]
            ],
        'Encode_ErrorCode':
            [[PyTango.DevVarUShortArray, ""],
             [PyTango.DevLong, ""]
            ],
        'Decode_ErrorCode':
            [[PyTango.DevLong, ""],
             [PyTango.DevVarUShortArray, ""]
            ],
        'SetStatusBits':
            [[PyTango.DevLong64,""],
             [PyTango.DevString,""]
            ],
        }
    attr_list = {
        'PowerStatus':
            [[PyTango.DevLong,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'label':"Power Status",
                'description':"booleans of the register 15 of the Iocaste",
            } ],
        'PowerStatusStr':
            [[PyTango.DevString,
            PyTango.SPECTRUM,
            PyTango.READ,16],
            {
                'label':"Power Status strings",
                'description':"strings with the meaning of the register 15 of the Iocaste",
            } ],
        'PowerStatusArray':
            [[PyTango.DevBoolean,
            PyTango.SPECTRUM,
            PyTango.READ,16],
            {
                'label':"Power Status boolean list ",
                'description':"booleans array of the register 15 of the Iocaste",
            } ],
        'Interlock1':
            [[PyTango.DevLong,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'label':"Interlock1",
                'description':"booleans of the register 16 of the Iocaste",
            } ],
        'Interlock1Str':
            [[PyTango.DevString,
            PyTango.SPECTRUM,
            PyTango.READ,16],
            {
                'label':"Interlock1 strings",
                'description':"strings with the meaning of the register 16 of the Iocaste",
            } ],
        'Interlock1Array':
            [[PyTango.DevBoolean,
            PyTango.SPECTRUM,
            PyTango.READ,16],
            {
                'label':"Interlock1 boolean list",
                'description':"booleans array of the register 16 of the Iocaste",
            } ],
        'Interlock2':
            [[PyTango.DevLong,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'label':"Interlock2",
                'description':"booleans of the register 17 of the Iocaste",
            } ],
        'Interlock2Str':
            [[PyTango.DevString,
            PyTango.SPECTRUM,
            PyTango.READ,16],
            {
                'label':"Interlock2 strings",
                'description':"strings with the meaning of the register 17 of the Iocaste",
            } ],
        'Interlock2Array':
            [[PyTango.DevBoolean,
            PyTango.SPECTRUM,
            PyTango.READ,16],
            {
                'label':"Interlock2 boolean list",
                'description':"booleans array of the register 17 of the Iocaste",
            } ],
        'Regulation':
            [[PyTango.DevLong,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'label':"Regulation",
                'description':"booleans of the register 18 of the Iocaste",
            } ],
        'RegulationStr':
            [[PyTango.DevString,
            PyTango.SPECTRUM,
            PyTango.READ,16],
            {
                'label':"Regulation strings",
                'description':"strings with the meaning of the register 18 of the Iocaste",
            } ],
        'RegulationArray':
            [[PyTango.DevBoolean,
            PyTango.SPECTRUM,
            PyTango.READ,16],
            {
                'label':"Regulation boolean list",
                'description':"booleans array of the register 18 of the Iocaste",
            } ],
        'Internal':
            [[PyTango.DevLong,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'label':"Internal",
                'description':"booleans of the internal state of the Iocaste",
            } ],
        'InternalStr':
            [[PyTango.DevString,
            PyTango.SPECTRUM,
            PyTango.READ,16],
            {
                'label':"Internal strings",
                'description':"strings with the meaning of the internal state of the Iocaste",
            } ],
        'InternalArray':
            [[PyTango.DevBoolean,
            PyTango.SPECTRUM,
            PyTango.READ,16],
            {
                'label':"Internal boolean list",
                'description':"booleans array of the internal state of the Iocaste",
            } ],
        'MagnetType':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ_WRITE],
            {
                'label':"Magnet type",
                'description':"Type of magnet to know who to interpret reg16&17. 2:b,bend,dipole 4:q,quad,quadrupole 6:s,sext,sextupole",
            } ],
        }

    def __init__(self, name):
        PyTango.DeviceClass.__init__(self, name)
        self.set_type(name);
        print "In StateCodeInterpreter_Class  constructor"

if __name__ == '__main__':
    try:
        py = PyTango.Util(sys.argv)
        py.add_TgClass(StateCodeInterpreterClass,
                       StateCodeInterpreter,
                       'StateCodeInterpreter')

        U = PyTango.Util.instance()
        U.server_init()
        U.server_run()

    except PyTango.DevFailed,e:
        print '-------> Received a DevFailed exception:',e
    except Exception,e:
        print '-------> An unforeseen exception occured....',e
