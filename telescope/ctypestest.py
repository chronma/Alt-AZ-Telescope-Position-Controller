import ctypes
from ctypes import *

import os
import mmap
import time

# need to run script from command line as sudo,sudo python3 ctypestest.py

i=c_int(42)
print (i, i.value)

a=0x10000
b=0x4a300000
c=(a+b)
print (c)
count = 0
fd = os.open("/dev/mem", os.O_SYNC | os.O_RDWR)
mem = mmap.mmap(fd, mmap.PAGESIZE, flags=mmap.MAP_SHARED, offset=c)
os.close(fd)
print(os.getuid())
print (os.getenv("USER"))
print (os.getenv("SUDO_USER"))
#value = ctypes.c_int.from_address(a)
#print (value)
step =0x00
while (count <10):
    count =count+1
    #print (step)
    v = ctypes.c_uint32.from_buffer(mem, step).value
    print (v,'  ',hex(v))
    


#t=0x11111111
z=300
s=20
#print (hex(t))
#ctypes.c_uint32.from_buffer(mem, 0x4).value |= t    
  
ctypes.c_uint32.from_buffer(mem, 0x4).value = z  
ctypes.c_uint32.from_buffer(mem, 0x8).value = s 
#ctypes.c_uint32.from_buffer(mem, 0x0).value = t  