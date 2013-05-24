#!/usr/bin/python
# -*- coding: utf-8 -*-

import PyTango as Tg
import sys
from time import sleep, time
import os
# related to time or to frequency?

DB = Tg.Database()
Istep = 0.25
MAX_DIFF = Istep / 3.0
Istart = 25.0
Imax = 200.0

class StepLost(Exception):
    def __init__(self, device, Iref, timeout=None):
        Iset = device['I'].w_value
        Exception.__init__(self, '%s lost step %4.4f (set point is %4.4f)' % (device.dev_name(), Iref, Iset))
        self.device = device
        self.Iref = Iref
        self.timeout = timeout

class MyDeviceProxy(Tg.DeviceProxy):

    def __init__(self, name):
        Tg.DeviceProxy.__init__(self, name)
        prop = self.get_property( ('RegulationPrecision',))
        self.RegulationPrecision = prop.get('RegulationPrecision', 0.02)

    def set_Iref(self, I):
        self._start_t = time()
        self.write_attribute('I', I)

def try_dev(name):
    dev = Tg.DeviceProxy(name)
    dev.ping()
    dev.ResetInterlocks()
    dev['I']
    dev.State()
    if not dev['RemoteMode'].value:
         raise Exception('not in remote mode')
    return dev.Status()

def try_server(name):
    ds = []
    if not '/' in name:
        name = 'HazemeyerIocaste_PS/'+name

    print name
    ls = DB.get_device_class_list(name)
    for d,c in zip(ls[::2], ls[1::2]):
        try:
            if c=='HazemeyerIocaste_PS':
               s = try_dev(d)
               ds.append(d)
               print d, s
        except Exception, exc:
               print 'fail', d, c, exc
    return ds



def wait_I(d, Iref, start_t, timeout=5.0):
    sys.stdout.flush()
    Iref_real = d['CurrentSetpoint'].value
    if abs(Iref_real-Iref)>Istep/MAX_DIFF:
        print "warning device has setpoint %f which is not %f" % (Iref_real, Iref)
    while True:
        sleep(0.2)
        Iread = d['Current'].value
        diff = abs(Iread - Iref)
        if diff < MAX_DIFF: break
        if time()-start_t > timeout:
#            d.ApplyCurrent()
            raise StepLost(d, Iref, timeout)
    print d.dev_name().split('/')[-1],

def main(args):

    print args
    device_names = []
    for name in args:
        try:
            if name.count('/')==2:
                try_dev(name)
                device_names.append(name)
            else:
                ds = try_server(name)
                device_names.extend(ds)
        except Exception, exc:
            print exc

    print device_names
    devices = [ MyDeviceProxy(d) for d in device_names ]

    def fmt_dev(d):
        if d.State()!=Tg.DevState.ON:
            d.On()
        return "%s %s %s" % (d.dev_name(), 'R' if d['RemoteMode'].value else 'x', d.Status())

    sleep(1)
    x = [ fmt_dev(d) for d in devices]
    print '\n'.join(x)
    if not devices:
        print 'no suitable devices!'
        return


    print Istart,
    sys.stdout.flush()
    for d in devices:
        d['CurrentSetpoint'] = Istart
    start_t = time()
    for d in devices:
        wait_I(d, Istart, start_t, timeout=60.0)

    print
    try:
      for step in xrange(0, 600):
          Iref = Istart + step*Istep
          if Iref>Imax:
                print 'okay, good.'
                return
          print "%10f %7.4f" % (time(),Iref),
          sys.stdout.flush()
          for d in devices:
              d['CurrentSetpoint'] = Iref
          start_t = time()
          for d in devices:
              wait_I(d, Iref, start_t)
          diff_t = time()-start_t
          print "%4.2f s" % diff_t
    except StepLost:
          # os.system('killall Modbus')
          raise

main(sys.argv)

