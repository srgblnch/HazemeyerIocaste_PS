#!/usr/bin/python

import PyTango as Tg

DB = Tg.Database()

for serv in DB.get_server_list('HazemeyerIocaste_PS/*'):
  print serv
  dcl = DB.get_device_class_list(serv)
  for d,c in zip(dcl[::2], dcl[1::2]):
    if c=='HazemeyerIocaste_PS':
        ps = Tg.DeviceProxy(d)
        if not ps.is_command_polled('UpdateState'):
            ps.poll_command('UpdateState',250)
            print d
  print