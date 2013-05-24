#!/usr/bin/env python

import PyTango
import time

default_path="/homelocal/sicilia/tmp"

default_devName = "lab/pc/hz01"
default_pollingPeriod = 3000#ms
default_ntimes = 3
default_cycles = 1000

class TestLogger:
    def __init__(self,path=default_path,suffix=""):
        self.__fileName = path+"/"+time.strftime("%Y%m%d_%H%M_")+suffix+".log"
    def log(self,msg):
        f = open(self.__fileName,'a')
        f.write(msg+"\n")
        print(msg)
        f.close()

class HazemeyerIocaster_Tester(TestLogger):
    """This class has been prepared to test the Write Single/Multiple Registers to the
       plc (modbus subdevice) using the HazemeyerIocaster_PS device server.
       The purpose is to do full cycle transitions of OFF <-> STANDBY <-> ON, and
       log if any of the commands is being "lost"
    """
    def __init__(self,
                 devName=default_devName,
                 pollingPeriod=default_pollingPeriod,
                 ntimes=default_ntimes,
                 transitions=default_cycles*4,
                 useWsingle=True):
        tag = "Wsingle" if useWsingle else "Wmultiple"
        TestLogger.__init__(self,suffix="HazemeyerIocaster_Tester_%s"%tag)
        self.__devName = devName
        self._dev = PyTango.DeviceProxy(devName)
        self.__pollingPeriod = pollingPeriod
        self.__ntimes = ntimes
        rest = transitions%4
        if not rest == 0: transitions+=rest#make mandatori transitions-1%4 = 3, finish off
        self.__useWsingle = useWsingle
        self.log("HazemeyerIocaster test using %s device"%(self.__devName))
        self.__transitions = transitions
        self.__backup = {}
        self.__statistics = {}
        #execution
        self.__preTest()
        self.__useWriteSingleRegister()
        self.__test()
        self.__postTest()

    def __preTest(self):
        """Backup and setup"""
        self.__backup["UpdateState"] = self._dev.get_command_poll_period("UpdateState")
        if not self.__backup["UpdateState"] == self.__pollingPeriod:
            self.log("Changing the polling period of the 'UpdateState' command, from %d to %d (ms)"\
                     %(self.__backup["UpdateState"],self.__pollingPeriod))
            self._dev.poll_command("UpdateState",self.__pollingPeriod)
        else:
            self.log("Using current polling period of %d in the 'UpdateState' command"%(self.__backup["UpdateState"]))
        self.__backup["kludge"] = self._dev.Exec("self.kludge")
        if self.__backup["kludge"] == True:
            self.log("Forcing to only send commands one time (the kludge)")
            self._dev.Exec("self.kludge = False")
        else:
            self.log("The 'kludge' was already 'False'")
        self.log("Wait time between command and checker is %d ms (polling %d * %d)"\
                 %(self.__pollingPeriod*self.__ntimes,self.__pollingPeriod,self.__ntimes))
        self.__statistics["ok"] = 0
        self.__statistics["nok"] = 0
        self.sureItsOff()


    def __useWriteSingleRegister(self):
        self.__backup["commandBitWriteSingle"] = self._dev.Exec("self.commandBitWriteSingle")
        if self.__useWsingle:
            self.log("Using PresetSingleRegister")
            self._dev.Exec("self.commandBitWriteSingle = True")
        else:
            self.log("Using PresetMultipleRegisters")
            self._dev.Exec("self.commandBitWriteSingle = False")

    def __test(self):
        iteration = 0
        retries = 0
        self.log("Starting %d state transitions"%(self.__transitions))
        while (iteration < self.__transitions):
            substep = iteration%4
            if substep in [0,2]:
                cmd = "Standby"
                self._dev.Standby()
            elif substep == 1:
                cmd = "On"
                self._dev.On()
            else:#if substep == 3:
                cmd = "Off"
                self._dev.Off()
            seconds2sleep = (self.__pollingPeriod/1000.)*self.__ntimes
            time.sleep(seconds2sleep)
            state = self._dev.State()
            current,I,V = self._dev.read_attributes(["Current","I","V"])
            self.log("transition:%3d(%1d) Cmd=%9s States= %7s Current=%6.3fA (I=%6.3f,V=%6.3f)"\
                     %(iteration,substep,cmd,state,current.value,I.value,V.value))
            if self.__IsWhereItShouldBe(self._dev.State(),substep):
                iteration += 1
                retries = 0
                self.__statistics["ok"] += 1
            elif self._dev.State() == PyTango.DevState.ALARM:
                self.log("Detected an ALARM at %s with message '%s'"%(time.strftime("%Y%m%d %H%M"),self._dev.Status()))
                dump = {}
                for k,v in enumerate(list(self._dev['all'].value)):
                    dump[hex(k)] = hex(v)
                self.log("Dump registers = %s"%dump)
                return
            else:
                retries += 1
                self.__statistics["nok"] += 1

    def __IsWhereItShouldBe(self,state,substep):
        if state == PyTango.DevState.STANDBY and substep in [0,2]: return True
        elif state in [PyTango.DevState.ON,PyTango.DevState.MOVING] and substep == 1: return True
        elif state == PyTango.DevState.OFF and substep == 3: return True
        else: return False

    def __postTest(self):
        """Restore and statistics"""
        self.log("Total commands well send %d, total ignored %d"%(self.__statistics["ok"],self.__statistics["nok"]))
        self._dev.poll_command("UpdateState",self.__backup["UpdateState"])
        self._dev.Exec("self.kludge = %s"%self.__backup["kludge"])
        if self.__backup.has_key("commandBitWriteSingle"):
            self._dev.Exec("self.commandBitWriteSingle = %s"%self.__backup["commandBitWriteSingle"])
        self.sureItsOff()

    def sureItsOff(self):
        #make sure that this ends in off state
        self._dev.Off()
        time.sleep((self.__pollingPeriod/1000.)*self.__ntimes)#wait n times the polling
        self._dev.Off()

#commands:
PWR_ON  = 0x001 #0b0000000000000001
PWR_OFF = 0x002 #0b0000000000000010
RESET   = 0x002 #0b0000000000000100
RAMP    = 0x008 #0b0000000000001000
#readbacks:
ON      = 0x001 #0b0000000000000001
OFF     = 0x002 #0b0000000000000010
READY   = 0x004 #0b0000000000000100
LOCAL   = 0x008 #0b0000000000001000
REMOTE  = 0x010 #0b0000000000010000
FAULT   = 0x080 #0b0000000010000000
STANDBY = 0x100 #0b0000000100000000
REG_OK  = 0x200 #0b0000001000000000
class Modbus_Tester(TestLogger):
    def __init__(self,
                 devName=default_devName+"-modbus",
                 pollingPeriod=default_pollingPeriod,
                 ntimes=default_ntimes,
                 transitions=default_cycles*4,
                 useWsingle=True):
        tag = "Wsingle" if useWsingle else "Wmultiple"
        TestLogger.__init__(self,suffix="Modbus_Tester_%s"%tag)
        if not devName.endswith("-modbus"): devName = devName+"-modbus"
        self.__devName = devName
        self._dev = PyTango.DeviceProxy(devName)
        self.__waitTime = (pollingPeriod/1000.)*2
        self.__ntimes = ntimes
        rest = transitions%4
        if not rest == 0: transitions+=rest#make mandatori transitions-1%4 = 3, finish off
        self.__transitions = transitions
        self.__useWsingle = useWsingle
        self.log("Modbus test using %s device"%(self.__devName))
        self.__statistics = {}
        self.__statistics["ok"] = 0
        self.__statistics["nok"] = 0
        self.test()
        self.log("Total commands well send %d, total ignored %d"%(self.__statistics["ok"],self.__statistics["nok"]))
        self.sureItsOff()
    #I/O to the Iocaste
    def write(self,addr,value):
        if self.__useWsingle:
            self._dev.PresetSingleRegister([addr,value])
        else:
            self._dev.PresetMultipleRegisters([addr,1,value])
    def On(self):
        self.write(0x00,RAMP|PWR_ON)
        reg00 = self.read(0x00)
        reg0F = self.read(0x0F)
        self.log("OnCmd  0x00=%s,0x0F=%s"%(bin(reg00),bin(reg0F)))
    def Off(self):
        self.write(0x00,RAMP|PWR_OFF)
        reg00 = self.read(0x00)
        reg0F = self.read(0x0F)
        self.log("OffCmd 0x00=%s,0x0F=%s"%(bin(reg00),bin(reg0F)))
    def read(self,addr,nbites=1):
        return self._dev.ReadHoldingRegisters([addr,nbites])
    #boolean checkers
    def IsOff(self):
        reg00 = self.read(0x00)
        reg0F = self.read(0x0F)
        self.log("IsOff?     0x00=%s,0x0F=%s"%(bin(reg00),bin(reg0F)))
        return bool(reg0F & OFF)
    def IsStandby(self):
        reg00 = self.read(0x00)
        reg0F = self.read(0x0F)
        self.log("IsStandby? 0x00=%s,0x0F=%s"%(bin(reg00),bin(reg0F)))
        return bool(reg0F & STANDBY)
    def IsOn(self):
        reg00 = self.read(0x00)
        reg0F = self.read(0x0F)
        self.log("IsOn?      0x00=%s,0x0F=%s"%(bin(reg00),bin(reg0F)))
        return bool(reg0F & ON)
    def test(self):
        iteration = 0
        retries = 0
        self.log("Starting %d state transitions"%(self.__transitions))
        while (iteration < self.__transitions):
            substep = iteration%4
            if substep in [0,2]:
                cmd = "Standby"
                if substep == 0:
                    self.On()
                    time.sleep(self.__waitTime*self.__ntimes)
                else:
                    self.Off()
                    time.sleep(self.__waitTime)
                for i in range(self.__ntimes):
                    if self.IsStandby():
                        self.log("transition:%3d(%1d) Cmd=%9s retries=%d"%(iteration,substep,cmd,retries))
                        iteration += 1
                        retries = 0
                        self.__statistics["ok"] += 1
                        break
                    else:
                        retries += 1
                        self.__statistics["nok"] += 1
            elif substep == 1:
                cmd = "On"
                self.On()
                time.sleep(self.__waitTime)
                for i in range(self.__ntimes):
                    if self.IsOn():
                        self.log("transition:%3d(%1d) Cmd=%9s retries=%d"%(iteration,substep,cmd,retries))
                        iteration += 1
                        retries = 0
                        self.__statistics["ok"] += 1
                        break
                    else:
                        retries += 1
                        self.__statistics["nok"] += 1
            else:#if substep == 3:
                cmd = "Off"
                self.Off()
                time.sleep(self.__waitTime)
                for i in range(self.__ntimes):
                    if self.IsOff():
                        self.log("transition:%3d(%1d) Cmd=%9s retries=%d"%(iteration,substep,cmd,retries))
                        iteration += 1
                        retries = 0
                        self.__statistics["ok"] += 1
                        break
                    else:
                        retries += 1
                        self.__statistics["nok"] += 1
    def sureItsOff(self):
        #make sure that this ends in off state
        self.Off()
        time.sleep(self.__waitTime*self.__ntimes)#wait n times the polling
        self.Off()

def main():
    import sys
    import json
    import taurus.core.util.argparse as argparse

    parser = argparse.get_taurus_parser()
    parser.set_usage("%prog [options] <devName>")
    parser.set_description("This is a test script for the Iocaste of Hazemeyer Power supplies.")
    parser.add_option("--pollingPeriod")
    parser.add_option("--waitTime")
    parser.add_option("--transitions")
    parser.add_option("--realExecution")

    parser, options, args = argparse.init_taurus_args(parser=parser)

    if not len(args) == 1:
        sys.stderr.write("Need to supply one (and only one) device Name\n")
        sys.exit(1)

    print("args = %s"%(args))
    optionsstr = "%s"%options
    optionsdict = eval(optionsstr)

    print("optionsdict = %s"%(optionsdict))

    if optionsdict['pollingPeriod'] == None: pollinglist = 250
    else: pollinglist = json.loads(optionsdict['pollingPeriod'])

    if optionsdict['transitions'] == None: transitions = 1000
    else: transitions = int(optionsdict['transitions'])

    if optionsdict['realExecution'] == None: realExecution = True
    else: realExecution = eval(optionsdict['realExecution'])

    for pollperiod in pollinglist:
        for Wsingle in [True,False]:
            if optionsdict['waitTime'] == None: waitTime = (10*1000)/pollperiod #mean 10s
            else: waitTime = (int(optionsdict['waitTime'])*1000)/pollperiod
            print("HazemeyerIocaster_Tester(devName=%s,"\
                  "pollingPeriod=%s,ntimes=%s,transitions=%s,"\
                  "useWsingle=%s"\
                  %(args[0],pollperiod,waitTime,transitions,Wsingle))
            if realExecution:
                tester = HazemeyerIocaster_Tester(args[0],pollperiod,waitTime,transitions,Wsingle)


def oldMain():
    import sys
    args = sys.argv[1:]
    if not len(args) in [0,1,4]:
        print 'usage parameters: device pollingPeriod ntimeswait transitions'
        sys.exit(1)
    if len(args) == 1:
        for polling in range(250,1050,50):
            tester = HazemeyerIocaster_Tester(devName=args[0],
                                              pollingPeriod=polling,
                                              ntimes=(10*1000)/polling,
                                              transitions=1000,
                                              useWsingle=False)
            tester = HazemeyerIocaster_Tester(devName=args[0],
                                              pollingPeriod=polling,
                                              ntimes=(10*1000)/polling,
                                              transitions=1000,
                                              useWsingle=True)
        #tester = Modbus_Tester(devName=args[0],useWsingle=False)
        #tester = Modbus_Tester(devName=args[0],useWsingle=True)
    else:
        tester = HazemeyerIocaster_Tester(devName=args[0],
                                          pollingPeriod=int(args[1]),
                                          ntimes=int(args[2]),
                                          transitions=int(args[3]),
                                          useWsingle=False)
        tester = HazemeyerIocaster_Tester(devName=args[0],
                                          pollingPeriod=int(args[1]),
                                          ntimes=int(args[2]),
                                          transitions=int(args[3]),
                                          useWsingle=True)
#        tester = Modbus_Tester(devName=args[0],
#                               pollingPeriod=int(args[1]),
#                               ntimes=int(args[2]),
#                               transitions=int(args[3]),
#                               useWsingle=False)
#        tester = Modbus_Tester(devName=args[0],
#                               pollingPeriod=int(args[1]),
#                               ntimes=int(args[2]),
#                               transitions=int(args[3]),
#                               useWsingle=True)

if __name__ == '__main__':
    main()