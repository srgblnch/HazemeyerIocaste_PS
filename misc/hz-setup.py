# -*- coding: utf-8 -*-
import math
import csv
import serial
import sys
import time
import pprint
import re
# import winsound

MAC_PAT = re.compile(r"(.|\n)*MAC : (.*)", re.M)
RAFAM_PAT = re.compile(r'..-..-(.+)-RKA([^-]+)', re.I)

# qh05 of 08D02 was not installed?
# qh05 of 08D02 was not installed?

class Netinfo(object):
    
    def __init__(self, code, ip, mac=None, hostname=None):
        self.code = code
        self.ip = ip
        self.mac = mac
        s = code.split('-')
        sector = s[3][3:5]
        family = s[2].lower()
        self.hostname = 'dpcsr'+sector+family
    
def import_network_info():
    info = {}
    with open('equip_report_network.csv') as nin:
        nin.readline()
        for row in csv.reader(nin):
            code = row[0]
            info[code] = Netinfo(code, row[5], row[4], row[2])
    return info 

def tohex(ch):
    c2 = ch.strip().upper()
    if len(c2)<2:
        c2 = '0'+c2
    return c2
 
class Configu(dict):
    def __init__(self, info, port='COM3'):
        self.info = info
        self.serial = serial.Serial()
        self.serial.port = port
        
    def p(self, *args, **kwargs):
        sep = kwargs.get('sep', ' ')
        eol = kwargs.get('eol', '')
        msg = sep.join(args)
        sys.stdout.write(msg) 
        sys.stdout.flush()
        if eol:
            sys.stdout.write(eol)
        
    def command(self, cmd):
        txt = cmd+'\r'
        self.p(txt, eol='\n')
        self.serial.write(txt)
        r = self.serial.readline(eol='>')
        self.p(r)
        return r
        
    def wait_open(self):
        count = 0
        last = 0
        while not self.serial.isOpen():
            try:
                self.serial.open()
            except Exception:
                count += 1
                v = int(16*math.log(count))
                if  v > last:
                    self.p('.')
                    last = v 
                time.sleep(0.1)
        
    def setup(self, netinfo):
        self.wait_unplug()
        rack,fam = rack_of(netinfo.code)
#        winsound.MessageBeep()
        self.p('plug %s of rack %s!\n' % (fam,rack) )
        self.p('setup', netinfo.code, eol='\n')
        try:
            self.wait_open()
        except KeyboardInterrupt:
            print 'skipping %s -- press RETURN to continue with next' % netinfo.code
            raw_input()
            return
        self.serial.flushInput()
        self.serial.flushOutput()
        self.serial.timeout = 10
        self.command('')
        self.p('\n')
        self.p('serial line %r open' % (self.serial.port, ) )
        self.command('')
        mac_reply = str(self.command('read mac'))
        hzmac = MAC_PAT.match(mac_reply)
        if hzmac is None:
            raise Exception('%r did not match pattern %r' % (mac_reply, MAC_PAT))
        else:
            hzmac_parts = ( tohex(p) for p in hzmac.group(2).split('-') )
        netinfo.mac = ':'.join(hzmac_parts)
        self.p('mac is %r\n' % netinfo.mac)
        self.command('ipad 0'+netinfo.ip)
        gateway = netinfo.ip.rpartition('.')[0]+'.254'
        self.command('mask 255.255.255.0')
        self.command('gate 0'+gateway)
        winsound.MessageBeep()
        self.command('save eeprom')
        self.serial.write('restart\r')
        self.p('setup %s finished\n\n' % netinfo.code)
        
    def wait_unplug(self):
        self.p('Please unplug cable!\n')
        while self.serial.isOpen():
            try:
                time.sleep(0.1)
                # purpose of this line is to fail when USB cable is disconnected
                # without having to read / write anything to the device
                self.serial.timeout = 0.1
            except Exception:
                self.serial.close()

def update_mac(info, fname):
    with open(fname) as mac_in:
        for row in csv.reader(mac_in):
            if row:
              code = row[0]
              info[code].mac = row[2]

def rack_of(code):
    mat = RAFAM_PAT.match(code)
    if mat is None:
        raise Exception('code %r does not match regular expression RACK_PAT')
    else:
        return mat.group(2), mat.group(1)
                
def cmp_ni(a,b):
    racka,fama = rack_of(a.code)
    rackb,famb = rack_of(b.code)
    if cmp(racka, rackb)==0:
        return cmp(fama,famb)
    else:
        return cmd(racka,rackb)
  
def cmp_code(a,b):
    racka,fama = rack_of(a)
    rackb,famb = rack_of(b)
    if cmp(racka, rackb)==0:
        return cmp(fama,famb)
    else:
        return cmp(racka,rackb)
  
def extract_racks(info):
    rack_info = {}
    for code,n in info.iteritems():
        rack = code.split('-')[3][3:]
        ls = rack_info.setdefault(rack, [])
        ls.append(n)
    for r in rack_info.values():
        r.sort(cmp=cmp_ni)
        
    return rack_info
            
class MyDialect(csv.excel_tab):
    lineterminator = '\n'
            
def write_mac(info, fname):
    with open(fname, 'w') as mac_out:
        writer = csv.writer(mac_out)
        writer.writerow( ('Equipment Code', 'hostname', 'MAC', 'IP') ) 
        for code,i in info.iteritems():
            if i.mac:
                row = (i.code, i.hostname, i.mac, i.ip)
                writer.writerow(row)
            
def main():
    info = import_network_info()
    cfg = Configu(info)
    update_mac(info, 'mac3.csv')
    for c in sorted(info, cmp_mac):
      x = info[c]
      print '%s,%s,%s,%s' % (x.code, x.hostname, x.ip, x.mac)
    return
    try:
        racks = extract_racks(info)
        for r in sys.argv[1:]:
            r = r.upper()
            if RAFAM_PAT.match(r):
                cfg.setup(info[r])
            else:
                for eq in racks[r.upper()]:
                    cfg.setup(eq)
    finally:
        write_mac(info, 'mac-list2.csv')
        write_mac(info, 'mac/list-%.2f.csv' % time.time())
        
main()