#!/usr/bin/python
# -*- coding: utf-8 -*-

import PyTango as Tg

DB = Tg.Database()
def cfg_ev_ps(dev):
    props = {}
    cfg_change_ev(props, 'Current', abs=0.005)
    cfg_change_ev(props, 'CurrentSetpoint', abs=0.005)
    cfg_change_ev(props, 'Voltage', abs=0.05)
    print dev,props
    DB.put_device_attribute_property(dev, props)

def cfg_change_ev(props, aname, abs=None, rel=None):
    '''Updates props for attribute 'aname' for generating change events
       from the numerical abs and rel changes passed.
    '''
    ap =  {'abs_change' : '', 'rel_change' : ''}

    if abs:
        ap['abs_change'] = str(abs)

    if rel:
        ap['rel_change'] = str(rel)
    props[aname] =  ap

for serv in DB.get_server_list('HazemeyerIocaste_PS/sr03'):
  print serv
  dcl = DB.get_device_class_list(serv)
  for d,c in zip(dcl[::2], dcl[1::2]):
      if c=='HazemeyerIocaste_PS':
        print d
        cfg_ev_ps(d)

