#!/usr/bin/python

import PyTango as Tg
import traceback
import os
import sys

print 'TANGO_HOST=', os.getenv('TANGO_HOST')
DB = Tg.Database()
for serv in DB.get_server_list('HazemeyerIocaste_PS/*'):
  print serv
  dcl = DB.get_device_class_list(serv)
  for dev,class_ in zip(dcl[::2],dcl[1::2]):
    try:
      if class_=='HazemeyerIocaste_PS':
        d = Tg.DeviceProxy(dev)
        code = DB.get_device_property(dev, ('#Code') ).get('#Code')
        if code:
            code = code[0]
        print dev, code, 
        r = d.Exec('self._modbus_err_count')
        if r=='0': 
            r = ''
        else:
            r += ' modbus errors'
        sys.stdout.flush()
        print d.State(), d.Status(), r
    except Tg.DevFailed, f:
#        traceback.print_exc()
        print f[-1].desc
  print 
