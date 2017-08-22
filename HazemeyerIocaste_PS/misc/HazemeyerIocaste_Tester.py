#!/usr/bin/env python

# HazemeyerIocaste_Tester.py
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

import sys
import PyTango
import time
import traceback
import struct
import signal

default_path="./"#"/homelocal/sicilia/tmp"

#exit error codes:
EXIT_NORMAL = 0
ERROR_ARGUMENTS = 1
ERROR_PRETEST = 2
ERROR_INTEST = 3
ERROR_POSTTEST = 4

ADDR_CURRENTSETPOINT = 5
ADDR_CURRENTREADBACKSETPOINT = 26
ADDR_CURRENTMEASURE = 24
ADDR_CURRENTSETPOINTLOCAL = 46

class TestLogger:
    def __init__(self,path=default_path,suffix="",debug=False):
        self._path = path
        self._suffix = suffix
        self.__fileName = self._path+"/"+time.strftime("%Y%m%d_%H%M%S_")+self._suffix+".log"
        self._debug = debug
        if self._debug: print("DEBUG: This flag is true.")
    def log(self,msg):
        f = open(self.__fileName,'a')
        f.write(msg+"\n")
        print(msg)
        f.close()
    def cycleLog(self,suffix=""):
        self._suffix = suffix
        self.__fileName = self._path+"/"+time.strftime("%Y%m%d_%H%M_")+self._suffix+".log"
    def debug(self,msg):
        if self._debug: print("DEBUG: %s"%(msg))
    def sleep(self,t):
        self.debug("Sleep %6.3f"%t)
        time.sleep(t)

class Tester(TestLogger):
    def __init__(self,debug=False):
        TestLogger.__init__(self,suffix="HazemeyerIocaster_Tester",debug=debug)
        self.__devName = None
        self.__devProxy = None
        self.__cmdLst = ['Standby','On']
        self.__cmdSleep = [7]*len(self.__cmdLst)
        self.__pollingList = [250]
        self.__ntransitions = 1000
        self.__setpLst = [20]
        self.__realExec = True
        self.__statistics = {}
        self.__backup = {}
        self.__epsilon = 1.0
        self.__kludge = False
        signal.signal(signal.SIGINT, self.signal_handler)
        self._ctrlC = False
        self.__currentPollingPeriod = None

    def signal_handler(self,signal,frame):
        self.log("Pressed Ctrl+C, aborting!")
        self._ctrlC = True

    def setDevice(self,devName):
        self.debug("Tester.setDevice(%s)"%(devName))
        try:
            self.__devName = devName
            self.__devProxy = PyTango.DeviceProxy(devName)
        except:
            sys.stderr.write("Cannot make connections to the device\n")
            sys.exit(ERROR_PRETEST)

    def setCommandList(self,cmdLst,cmdSleep=None):
        self.debug("Tester.setCommandList(%s,%s)"%(cmdLst,cmdSleep))
        try:
            for cmd in cmdLst:
                if not cmd.lower() in ['off','standby','on']:
                    sys.stderr.write("Cannot build the command list with %s"%cmd)
                    sys.exit(ERROR_PRETEST)
            self.__cmdLst = cmdLst
        except:
            sys.stderr.write("Exception building command list.\n")
        try:
            if not cmdSleep == None:
                if len(cmdSleep) == 1:
                    self.__cmdSleep = [cmdSleep]*len(self.__cmdLst)
                else:
                    self.__cmdSleep = cmdSleep
            else:
                self.__cmdSleep = [7]*len(self.__cmdLst)
        except:
            sys.stderr.write("Exception building command sleep time list.\n")

    def setPollingList(self,pollingLst):
        self.debug("Tester.setPollingList(%s)"%(pollingLst))
        try:
            if type(pollingLst) == int: pollingLst = [pollingLst]
            self.__pollingList = pollingLst
        except:
            sys.stderr.write("Exception building the polling list.\n")

    def setNumTransitions(self,ntrans):
        self.debug("Tester.setNumTransitions(%s)"%(ntrans))
        try:
            self.__ntransitions = ntrans
        except:
            sys.stderr.write("Exception building setting the number of transitions.\n")

    def setCurrentSetpoints(self,setpLst):
        self.debug("Tester.setCurrentSetpoints(%s)"%(setpLst))
        try:
            self.__setpLst = setpLst
        except:
            sys.stderr.write("Exception building setting the list of setpoints.\n")

    def is_realExecution(self,real):
        self.debug("Tester.is_realExecution(%s)"%(real))
        try:
            self.__realExec = real
        except:
            sys.stderr.write("Cannot undertand if the executions must be real or not.\n")
            sys.exit(ERROR_PRETEST)

    def setEpsilon(self,epsilon):
        self.__epsilon = epsilon

    def setKludge(self,kludge):
        self.__kludge = kludge

    def make(self):
        self.debug("Tester.make()")
        self.debug("device: %s"%self.__devName)
        msgCmdLst = "command list (and sleep time):"
        for i,e in enumerate(self.__cmdLst):
            msgCmdLst += "%s(%s)"%(e,self.__cmdSleep[i])
        self.debug(msgCmdLst)
        self.debug("Polling periods to test: %s"%(self.__pollingList))
        self.debug("Each test will do %d state transitions"%(self.__ntransitions))
        self.debug("When ON state, will send %s setpoints"%self.__setpLst)
        for pollingPeriod in self.__pollingList:
            self.__currentPollingPeriod = pollingPeriod
            if self.__kludge : modes = [True]
            else: modes = [True,False]
            for Wsingle in modes:
                bar = "HazemeyerIocaster_Tester_p%s_%s"%(pollingPeriod,"WSR" if Wsingle else "WMR")
                print bar
                self.cycleLog(suffix=bar)#open another file
                self.__useWsingle = Wsingle
                self.__preTest(pollingPeriod)
                self.__test()
                self.__postTest()
                if self._ctrlC: break
            if self._ctrlC: break

    def __preTest(self,currentPollingPeriod):
        """Backup and setup"""
        self.debug("Tester.__preTest(%s)"%(currentPollingPeriod))
        if self.__realExec:
            self.__backup["UpdateState"] = self.__devProxy.get_command_poll_period("UpdateState")
            if not self.__backup["UpdateState"] == currentPollingPeriod:
                self.log("Changing the polling period of the 'UpdateState' command, from %d to %d (ms)"\
                         %(self.__backup["UpdateState"],currentPollingPeriod))
                self.__devProxy.poll_command("UpdateState",currentPollingPeriod)
            else:
                self.log("Using current polling period of %d in the 'UpdateState' command"%(self.__backup["UpdateState"]))
            #kludge
            self.__backup["kludge"] = bool(self.__devProxy.Exec("self.kludge"))
            if self.__backup["kludge"] == self.__kludge:
                self.log("The 'kludge' was already %s"%(self.__kludge))
            else:
                self.log("Changing the 'kludge' from %s to %s"%(self.__backup["kludge"],self.__kludge))
                self.__devProxy.Exec("self.kludge = %s"%(self.__kludge))
            #command bit write
            self.__backup["commandBitWriteSingle"] = self.__devProxy.Exec("self.commandBitWriteSingle")
            if self.__useWsingle:
                self.log("Using PresetSingleRegister")
                self.__devProxy.Exec("self.commandBitWriteSingle = True")
            else:
                self.log("Using PresetMultipleRegisters")
                self.__devProxy.Exec("self.commandBitWriteSingle = False")
#        self.log("Wait time between command and checker is %d ms (polling %d * %d)"\
#                 %(self.__pollingPeriod*self.__ntimes,self.__pollingPeriod,self.__ntimes))
        self.__statistics["ok"] = 0
        self.__statistics["nok"] = 0
        self.__statistics["setp_ok"] = 0
        self.__statistics["setp_nok"] = 0
        self.__statistics["setp_slow"] = 0
        self.__statistics["alarms_global"] = 0
        self.__statistics["alarms_continuous"] = 0
        self.__statistics["faults_global"] = 0
        self.__statistics["faults_continuous"] = 0
        self.__goFirstState()
        
    def __goFirstState(self):
        #make sure It's already in the state of the first command
        for i in range(10):
            if self.__realExec:
                self.__devProxy.command_inout(self.__cmdLst[0])
            self.sleep(self.__cmdSleep[0])#remember, the wait time
            if self.__isOnCmdState(self.__cmdLst[0]):
                break
        if i == 4:
            sys.stderr.write("cannot make sure is in the inicial stage.\n")
            sys.exit(ERROR_PRETEST)

    def __test(self):
        self.debug("Tester.__test()")
        self.log("Starting %d state transitions"%(self.__ntransitions))
        iteration = 0
        retries = 0
        try:
            while (iteration < self.__ntransitions):
                for step,cmd in enumerate(self.__cmdLst):#test the different cmds
                    while True:
                        if self.__realExec:
                            self.__devProxy.command_inout(cmd)
                        self.sleep(self.__cmdSleep[step])
                        if self.__realExec:
                            if hasattr(self.__devProxy,'I'):
                                state,current,I,V,currRamp = self.__devProxy.read_attributes(["State","Current","I","V","CurrentRamp"])
                                self.log("iteration:%3d(%1d) Cmd=%9s States= %7s Current=%6.3fA (I=%6.3f,V=%6.3f,ramp=%6.3f)"\
                                         %(iteration,step,cmd,state.value,current.value,I.value,V.value,currRamp.value))
                            else:
                                state,current,currRamp = self.__devProxy.read_attributes(["State","Current","CurrentRamp"])
                                self.log("iteration:%3d(%1d) Cmd=%9s States= %7s Current=%6.3fA (ramp=%6.3f)"\
                                         %(iteration,step,cmd,state.value,current.value,currRamp.value))
                            state = state.value
                        else:
                            state=cmd
                            self.log("iteration:%3d(%1d) Cmd=%9s"%(iteration,step,cmd))
                        if self.__isOnCmdState(cmd,state):
                            retries = 0
                            self.__statistics["ok"] += 1
                            break
                        if self.__kludge:
                            msg = " ! possible issue with the command (not ready after %3.1fs)"%self.__cmdSleep[step]
                            self.log(msg)
                            self.sleep(self.__cmdSleep[step])
                        else:
                            retries += 1
                            self.__statistics["nok"] += 1
                        if self._ctrlC: break
                    if not self.__realExec and cmd.lower()=='on':
                        state = PyTango.DevState.ON
                    if state == PyTango.DevState.ON or\
                       state == PyTango.DevState.MOVING:
                        sp_retries = 0
                        for setp in self.__setpLst:#setpoint test
                            try:
                                if self.__realExec:
                                    self.__devProxy.write_attribute("CurrentSetpoint",setp)
                                    maxv = max(current.value,setp)
                                    minv = min(current.value,setp)
                                    t_movement = (maxv-minv)/currRamp.value
                                    if not self.__currentPollingPeriod == None and \
                                       t_movement < self.__currentPollingPeriod*4:
                                        t_movement = (self.__currentPollingPeriod*4)/1000.#to seconds
                                    else:
                                        t_movement*=2
                                    self.sleep(t_movement)
                                else: self.sleep(1)
                                if self.__kludge:
                                    #in this case the device is responsible to 
                                    #resend the setpoint until is applied
                                    if not self.__isOnCurrentSetpoint(setp):
                                        msg = "                 ! Setpoint to apply the hackish (not in place after %3.1fs)"%t_movement
                                        self.log(msg)
                                        self.__statistics["setp_slow"] += 1
                                        self.sleep(t_movement)
                                    i = 1
                                    while not self.__isOnCurrentSetpoint(setp):
                                        msg = "                 ! not yet in the setpoint (after %3.1fs)"%(i*t_movement)
                                        self.log(msg)
                                        self.sleep(t_movement)
                                        if self._ctrlC: break
                                        i += 1
                                        if i == 10:
                                            self.__statistics["setp_nok"] += 1
                                            msg = "                !! wait 10 times the expected time (after %3.1fs)"%(i*t_movement)
                                            break
                                else:
                                    #when kludge false, the test will distinguish 
                                    #between slow and ignored setpoints
                                    if self.__realExec and not self.__devProxy.read_attribute("RegulationOk").value:
                                        self.__statistics["setp_slow"] += 1
                                        msg = "                 ! Possible slow apply of a setpoint (after %3.1fs)"%t_movement
                                        maxv = max(current.value,setp)
                                        minv = min(current.value,setp)
                                        t_movement = (maxv-minv)/currRamp.value
                                        if not self.__currentPollingPeriod == None and \
                                        t_movement <= self.__currentPollingPeriod*8:
                                            t_movement = (self.__currentPollingPeriod*8)/1000.#to seconds
                                        else:
                                            t_movement*=2
                                        msg +=", wait '~twice' (%3.1fs))"%t_movement
                                        self.log(msg)
                                        self.sleep(t_movement)
                                if self.__isOnCurrentSetpoint(setp) == 1:#logs of this part inside
                                    self.__statistics["setp_ok"] += 1
                                    if self.__realExec:
                                        current = self.__devProxy.read_attribute("Current")
                                else:#if it's slow by a second time, log it as nok
                                    self.__statistics["setp_nok"] += 1
                                    #don't read the current because it didn't move and
                                    #next step will be longer
                                if self._ctrlC: break
                            except Exception,e:
                                self.log(str(e))
                                traceback.print_exc()
                iteration += 1
                try: cmd_percentage = self.__statistics["nok"]*100./(self.__statistics["nok"]+self.__statistics["ok"])
                except: cmd_percentage = 0.0
                self.log("Partial statistics: commands well send %d, total ignored %d (%4.2f)"
                         %(self.__statistics["ok"],
                           self.__statistics["nok"],
                           cmd_percentage))
                total_setp = self.__statistics["setp_ok"]+self.__statistics["setp_nok"]+self.__statistics["setp_slow"]
                setp_percentage = self.__statistics["setp_nok"]*100./(total_setp) or float('nan')
                slow_percentage = self.__statistics["setp_slow"]*100./(total_setp) or float('nan')
                self.log("Partial statistics: setpoints well send %d, ignored %d (%4.2f); slow setpoints %d (%4.2f)"
                         %(self.__statistics["setp_ok"],
                           self.__statistics["setp_nok"],
                           setp_percentage,
                           self.__statistics["setp_slow"],
                           slow_percentage))
                self.log("Partial statistics: alarms %d"%(self.__statistics["alarms_global"]))
                self.log("Partial statistics: faults %d"%(self.__statistics["faults_global"]))
                if self._ctrlC: break
        except Exception,e:
            self.log(str(e))
            traceback.print_exc()
            if self.__realExec and hasattr(self.__devProxy,'all'):
                dump = {}
                for k,v in enumerate(list(self.__devProxy['all'].value)):
                    dump[hex(k)] = hex(v)
                self.log("Dump registers = %s"%dump)
            sys.exit(ERROR_INTEST)

    def __isOnCmdState(self,cmd,state=None):
        #self.debug("Tester.__isOnCmdState(%s,%s)"%(cmd,state))
        if not self.__realExec: return True
        if state == None:
            state = self.__devProxy.State()
        bar = repr(state).split('.')
        bar.reverse()
        if bar[0].lower() in ['alarm','fault']:
            #FIXME: resetInterlock, with a counter to abort if too many
            self.log("Detected an %s at %s with message '%s'"%(bar[0].upper(),time.strftime("%Y%m%d %H%M"),self.__devProxy.Status()))
            if bar[0].lower() == 'alarm':
                self.__devProxy.ResetInterlocks()
                self.__statistics['alarms_global'] += 1
                self.__statistics["alarms_continuous"] += 1
                if self.__statistics["alarms_continuous"] == 100:
                    raise Exception("Too many alarms, reset interlocks is not enough. Aborting.")
            elif bar[0].lower() == 'fault':
                self.__devProxy.Init()
                self.__statistics['faults_global'] += 1
                self.__statistics["faults_continuous"] += 1
                if self.__statistics["faults_continuous"] == 100:
                    raise Exception("Too many faults, Init() device is not enough. Aborting.")
            self.__goFirstState()
            self.sleep(1)
        elif bar[0].lower() == cmd.lower() or (bar[0].lower() == 'moving' and cmd.lower() == 'on'):
            self.__statistics["alarms_continuous"] = 0
            return True
        return False

    def __isOnCurrentSetpoint(self,sp,epsilon=None):
        #self.debug("Tester.__isOnCurrentSetpoint(%s,%s)"%(sp,epsilon))
        if epsilon == None:
            epsilon = self.__epsilon
        if self.__realExec:
            c = self.__devProxy.read_attribute("Current").value
            regOk = self.__devProxy.read_attribute("RegulationOk").value
        else:
            c = sp
            regOk = True
        #msg = "CurrentSetpoint = %6.3f => current = %6.3f epsilon=%6.3f"%(sp,c,epsilon)
        msg = "Attr CurrentSetpoint = %6.3f, Attr Current = %6.3f, "%(sp,c)
        if self.__realExec:
            addr2read = {'CurrentSetpoint':ADDR_CURRENTSETPOINT,
                         'CurrentReadBackSetpoint':ADDR_CURRENTREADBACKSETPOINT,
                         'CurrentMeasure':ADDR_CURRENTMEASURE,
                         'CurrentSetpointLocal':ADDR_CURRENTSETPOINTLOCAL}
            for k in addr2read:
                msg += "Reg %s(%d) = %6.3f, "%(k,addr2read[k],self.__readRegister(addr2read[k]))
        msg += "RegOk = %s, "%(regOk)
        msg += "epsilon = %6.3f"%(epsilon)
        if c > sp-epsilon and c < sp+epsilon:
            self.log("                 + %s"%(msg))
            return True
        if not regOk:
            self.log("                 ? %s"%(msg))
            return None
        self.log("                 - %s"%(msg))
        return False

    def __readRegister(self,addr):
        regs = self.__devProxy['All']
        return struct.unpack('f', struct.pack('HH',*regs.value[addr:addr+2]))[0]

    def __postTest(self):
        """Restore and statistics"""
        self.debug("Tester.__postTest()")
        try:
            self.log("Total commands well send %d, total ignored %d (%4.2f)"
                     %(self.__statistics["ok"],
                       self.__statistics["nok"],
                       self.__statistics["nok"]*100./(self.__statistics["nok"]+self.__statistics["ok"])))
            total_setp = self.__statistics["setp_ok"]+self.__statistics["setp_nok"]+self.__statistics["setp_slow"]
            setp_percentage = self.__statistics["setp_nok"]*100./(total_setp)
            slow_percentage = self.__statistics["setp_slow"]*100./(total_setp)
            self.log("Total setpoints well send %d, ignored %d (%4.2f), slow setpoints %d (%4.2f)"
                     %(self.__statistics["setp_ok"],
                       self.__statistics["setp_nok"],
                       setp_percentage,
                       self.__statistics["setp_slow"],
                       slow_percentage))
            self.log("Total alarms %d"%(self.__statistics["alarms_global"]))
            self.log("Total faults %d"%(self.__statistics["faults_global"]))
            if self.__realExec:
                self.__devProxy.poll_command("UpdateState",self.__backup["UpdateState"])
                self.__devProxy.Exec("self.kludge = %s"%self.__backup["kludge"])
                if self.__backup.has_key("commandBitWriteSingle"):
                    self.__devProxy.Exec("self.commandBitWriteSingle = %s"%self.__backup["commandBitWriteSingle"])
                self.__sureItsOff()
        except Exception,e:
            self.log(str(e))
            sys.exit(ERROR_POSTTEST)

    def __sureItsOff(self):
        #make sure that this ends in off state
        if self.__realExec:
            self.__devProxy.Off()
            self.sleep(5)
            self.__devProxy.Off()

def main():
    import json
    import os, getopt

    try:
        possibleOptions = {'command-transit-list=':'List of commands to transit up and down. Default: standby,on',
                           'command-sleep-time=':'List of sleep time (seconds) per each command or one integer for all of them. Default: 7',
                           'update-state-polling=':'List of pollings for the command UpdateState. Default: [250]',
                           'ntransitions=':'Number of times that the command list will be transited. Default: 1000',
                           'currentsetpoints=':'List of setpoints to test when a transition passes by an ON state. Default: [20]',
                           'debug':'Get more text in the console (not in the log file).',
                           'fake-execution':'Debug option, no real commands will be sent to the device.',
                           'epsilon=':'Threashold for the current setpoint comparison',
                           'kludge=':'Boolean to test with the hackishes in the device (Default False)'}
        opt_pairs, args = getopt.getopt(sys.argv[1:], "h",
                                             ['help']+possibleOptions.keys())
        currentOptions = {}
        for k,v, in opt_pairs:
            currentOptions[k]=v
        #print("currentOptions: %s"%currentOptions)
    except getopt.GetoptError, e:
        print str(e)
        sys.exit(ERROR_ARGUMENTS)

    if currentOptions.has_key('--help') or currentOptions.has_key('-h'):
        description = "Description: This is a test script for the Iocaste of Hazemeyer Power supplies.\n"
        usage = "Usage: %s [options] <devName>\n"%(sys.argv[0])
        options = "Options:\n"
        for k in possibleOptions:
            spaces = " "*(25-len(k))
            options += "\t--%s%s%s\n"%(k,spaces,possibleOptions[k])
        param = "<devName>: A Tango device name of a HazemeyerIocaste_PS device server"
        print("\n%s\n%s\n%s\n%s\n"%(description,usage,options,param))
        #...
        sys.exit(EXIT_NORMAL)
    
    if not len(args) == 1:
        sys.stderr.write("\nNeed to supply one (and only one) device Name\nCheck --help\n\n")
        sys.exit(ERROR_ARGUMENTS)

    if currentOptions.has_key('--debug'):
        test = Tester(debug=True)
    else: test = Tester()
    test.setDevice(args[0])
    
    if currentOptions.has_key('--command-transit-list'):
        cmdLst = currentOptions['--command-transit-list'].split(',')
        if currentOptions.has_key('--command-sleep-time'):
            test.setCommandList(cmdLst,json.loads(currentOptions['--command-sleep-time']))
        else:
            test.setCommandList(cmdLst)
    if currentOptions.has_key('--update-state-polling'):
        test.setPollingList(json.loads(currentOptions['--update-state-polling']))
    if currentOptions.has_key('--ntransitions'):
        test.setNumTransitions(int(currentOptions['--ntransitions']))
    if currentOptions.has_key('--currentsetpoints'):
        test.setCurrentSetpoints(json.loads(currentOptions['--currentsetpoints']))
    if currentOptions.has_key('--fake-execution'):
        test.is_realExecution(False)
    if currentOptions.has_key('--epsilon'):
        test.setEpsilon(float(currentOptions['--epsilon']))
    if currentOptions.has_key('--kludge'):
        test.setKludge(bool(currentOptions['--kludge']))
    
    
    test.make()
    sys.exit(EXIT_NORMAL)


if __name__ == '__main__':
    main()