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
    
a=0x10000
b=0x4a300000
c=(a+b)
print (c)

pulses=3000
pulselengths=2
direction =1
fd = os.open("/dev/mem", os.O_SYNC | os.O_RDWR)
mem = mmap.mmap(fd, mmap.PAGESIZE, flags=mmap.MAP_SHARED, offset=c)
os.close(fd)

v = ctypes.c_uint32.from_buffer(mem, 0x00).value # 0x00 is the command bit
t=v
if (bitmagic(t,0,'getbit')==0):
    ctypes.c_uint32.from_buffer(mem, 0x04).value = pulses
    ctypes.c_uint32.from_buffer(mem, 0x08).value = pulselengths
   
    #if (direction==0):
    #    t=bitmagic(t,3,'clearbit')
    #    print ("direction 0")
    #else:
    #    t=bitmagic(t,3,'setbit')
    #    print ('direction 1')
    t=bitmagic(t,0,'setbit')
    for i in range (31,0,-1):
        print(bitmagic(t,i,'getbit'), end='')
    print(' ')
    
    ctypes.c_uint32.from_buffer(mem, 0x00).value = t
v = ctypes.c_uint32.from_buffer(mem, 0x00).value # 0x00 is the command bit

for i in range (31,0,-1):
        print(bitmagic(v,i,'getbit'), end='')
print(' ')