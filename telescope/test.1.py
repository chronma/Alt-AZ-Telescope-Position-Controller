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
    
a=0x000000
b=0x4a300000
c=(a+b)
print (c)

a=0x2000
b=0x4a300000
d=(a+b)
print (hex(c),hex(d))




fd = os.open("/dev/mem", os.O_SYNC | os.O_RDWR)
mem = mmap.mmap(fd, mmap.PAGESIZE, flags=mmap.MAP_SHARED, offset=c)
os.close(fd)

fd = os.open("/dev/mem", os.O_SYNC | os.O_RDWR)
mem1 = mmap.mmap(fd, mmap.PAGESIZE, flags=mmap.MAP_SHARED, offset=d)
os.close(fd)

#v = ctypes.c_uint32.from_buffer(mem, 0x28).value
#t=v
#for i in range (11,12):
#        t=bitmagic(t,i,'clearbit')
        #t=bitmagic(t,i,'setbit')

#ctypes.c_uint32.from_buffer(mem, 0x28).value = t

while (1):
    v = ctypes.c_uint32.from_buffer(mem, 0x200).value
    w = ctypes.c_uint32.from_buffer(mem1, 0x200).value
    #w = ctypes.c_uint32.from_buffer(mem, 0x32).value
    #x = ctypes.c_uint32.from_buffer(mem, 0x00).value
   
#ctypes.c_uint32.from_buffer(mem, 0x4).value = z  
#ctypes.c_uint32.from_buffer(mem, 0x8).value = s 
#    t=v
#    print ('incoming',v,'  ',t)
    print("pru0 ", end=' ')
    for i in range (31,-1,-1):
        print(bitmagic(v,i,'getbit'), end='')
    print(' ')
    print("pru1 ", end=' ')
    for i in range (31,-1,-1):
        print(bitmagic(w,i,'getbit'), end='')
    print(' ')
   # print("test bitval ")
    #for i in range (31,0,-1):
     #   print(bitmagic(w,i,'getbit'), end='')
    #print(' ')
    #print("test prucmmd ")
    #for i in range (31,0,-1):
    #    print(bitmagic(x,i,'getbit'), end='')
    #print(' ')
#    for i in range (0,31):
#        t=bitmagic(t,i,'clearbit')
        #t=bitmagic(t,i,'setbit')

    #ctypes.c_uint32.from_buffer(mem, 0x28).value = t 

#    print ('out',t,'  ','')
#    for i in range (0,31):
#        print(bitmagic(t,i,'getbit'), end='')
#    print(' ')
#    print ('verified bits',v,'  ','')
#    for i in range (0,31):
#        print(bitmagic(v,i,'getbit'), end='')
#    print(' ')
    time.sleep (.05)



    