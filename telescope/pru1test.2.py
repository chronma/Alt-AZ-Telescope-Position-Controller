import ctypes
from ctypes import *

import os
import mmap
import time

# need to run script from command line as sudo,sudo python3 ctypestest.py
Task= ['getbit','setbit', 'clearbit']

def bitmagic(value, bitnumber,task):
    if task == 'getbit':
        return ((value>>bitnumber)&1)
    if task == 'setbit':
        return (value | (1<<bitnumber))
    if task == 'clearbit':
        return (value & ~(1<<bitnumber))
    
    if task =='TogleBit':
        return (value ^((value>>bitnumber)&1))
    
a=0x010000
b=0x4a300000
c=(a+b)
print (c)
a1steps=1000
a1pulselength=int(20000000/4)

a2steps=1000
a2pulselength=int(8754000/2)

a3steps=1000
a3pulselength=int(8754000*1.5)

fd = os.open("/dev/mem", os.O_SYNC | os.O_RDWR)
mem = mmap.mmap(fd, mmap.PAGESIZE, flags=mmap.MAP_SHARED, offset=c)
os.close(fd)
# command bits are at 10100
command_bits = ctypes.c_uint32.from_buffer(mem, 0x100).value # 0x00 is the command bit

ctypes.c_uint32.from_buffer(mem, 0x400).value = a1steps
ctypes.c_uint32.from_buffer(mem, 0x404).value = a1pulselength 
ctypes.c_uint32.from_buffer(mem, 0x408).value = a2steps
ctypes.c_uint32.from_buffer(mem, 0x40C).value = a2pulselength 
ctypes.c_uint32.from_buffer(mem, 0x410).value = a3steps
ctypes.c_uint32.from_buffer(mem, 0x414).value = a3pulselength 


t=command_bits
t=0
t=bitmagic(t,19,'setbit')
t=bitmagic(t,20,'setbit')
t=bitmagic(t,21,'setbit')
#t=bitmagic(t,5,'setbit')
#t=bitmagic(t,3,'setbit')

print (hex(t))
ctypes.c_uint32.from_buffer(mem, 0x100).value = t 

command_bits = ctypes.c_uint32.from_buffer(mem, 0x100).value # 0x00 is the command bit
print (hex(command_bits))


