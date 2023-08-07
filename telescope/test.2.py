import ctypes
from ctypes import *
import subprocess
import os
import mmap
import time

#PRU 0
subprocess.run("config-pin P9_31 pruout", shell=True, check=True)
subprocess.run("config-pin P9_29 pruout", shell=True, check=True)
subprocess.run("config-pin P9_30 pruout", shell=True, check=True)
subprocess.run("config-pin P9_28 pruout", shell=True, check=True)
subprocess.run("config-pin P8_12 pruout", shell=True, check=True)
subprocess.run("config-pin P8_11 pruout", shell=True, check=True)
subprocess.run("config-pin P9_27 pruin", shell=True, check=True)
subprocess.run("config-pin P9_24 pruin", shell=True, check=True)
subprocess.run("config-pin P8_15 pruin", shell=True, check=True)
#pru 1
subprocess.run("config-pin P8_45 pruout", shell=True, check=True)
subprocess.run("config-pin P8_46 pruout", shell=True, check=True)
subprocess.run("config-pin P8_43 pruout", shell=True, check=True)
subprocess.run("config-pin P8_44 pruout", shell=True, check=True)
subprocess.run("config-pin P8_41 pruout", shell=True, check=True)
subprocess.run("config-pin P8_42 pruout", shell=True, check=True)
subprocess.run("config-pin P8_39 pruin", shell=True, check=True)
subprocess.run("config-pin P8_40 pruin", shell=True, check=True)
subprocess.run("config-pin P8_27 pruin", shell=True, check=True)




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
    
a=0x0000000
b=0x4a300000
c=(a+b)
print (c)

a=0x0002000
b=0x4a300000
d=(a+b)
print (d)
a1steps=100000
a1pulselength=int(1877446*1)

a2steps=10
a2pulselength=int(8964143*1.5)

a3steps=10
a3pulselength=int(8964143*.75)

a4steps=10
a4pulselength=int(8964143*1)

a5steps=160000
a5pulselength=int(10756972*1)

a6steps=10
a6pulselength=int(8964143*.75)



fd = os.open("/dev/mem", os.O_SYNC | os.O_RDWR)
mem = mmap.mmap(fd, mmap.PAGESIZE, flags=mmap.MAP_SHARED, offset=c)
os.close(fd)

fd = os.open("/dev/mem", os.O_SYNC | os.O_RDWR)
mem1 = mmap.mmap(fd, mmap.PAGESIZE, flags=mmap.MAP_SHARED, offset=d)
os.close(fd)



command_bits = ctypes.c_uint32.from_buffer(mem, 0x200).value # 0x00 is the command bit

c

t=0

    
t=bitmagic(t,0,'setbit')
t=bitmagic(t,1,'setbit')
t=bitmagic(t,2,'setbit')
t=bitmagic(t,5,'setbit')
t=bitmagic(t,3,'setbit')
print (hex(t))
for x in range(0,1,1):
    ctypes.c_uint32.from_buffer(mem, 0x300).value = a1steps
    ctypes.c_uint32.from_buffer(mem, 0x304).value = a1pulselength 
    ctypes.c_uint32.from_buffer(mem, 0x308).value = a2steps
    ctypes.c_uint32.from_buffer(mem, 0x30C).value = a2pulselength 
    ctypes.c_uint32.from_buffer(mem, 0x310).value = a3steps
    ctypes.c_uint32.from_buffer(mem, 0x314).value = a3pulselength 
    ctypes.c_uint32.from_buffer(mem1, 0x300).value = a4steps
    ctypes.c_uint32.from_buffer(mem1, 0x304).value = a4pulselength 
    ctypes.c_uint32.from_buffer(mem1, 0x308).value = a5steps
    ctypes.c_uint32.from_buffer(mem1, 0x30C).value = a5pulselength 
    ctypes.c_uint32.from_buffer(mem1, 0x310).value = a6steps
    ctypes.c_uint32.from_buffer(mem1, 0x314).value = a6pulselength 
    
    ctypes.c_uint32.from_buffer(mem, 0x200).value = t 
    ctypes.c_uint32.from_buffer(mem1, 0x200).value = t 

    command_bits = ctypes.c_uint32.from_buffer(mem, 0x200).value # 0x00 is the command bit
    print (hex(command_bits))
    command_bits1 = ctypes.c_uint32.from_buffer(mem1, 0x200).value # 0x00 is the command bit
    print (hex(command_bits))
    h=0
    h1=0

    for x in range (0,3):
        if (bitmagic(command_bits,x,"getbit")==1):
            h=bitmagic(h,x,'setbit')
            h1=bitmagic(h1,x,'setbit')
        else:    
            h=bitmagic(h,x,'clearbit')
            h1=bitmagic(h1,x,'clearbit')
    print(hex(h1), hex(h))
    
    while ((h1!=0) or (h!=0)):
        time.sleep(.001)
        
        command_bits = ctypes.c_uint32.from_buffer(mem, 0x200).value # 0x00 is the command bit
        command_bits1 = ctypes.c_uint32.from_buffer(mem1, 0x200).value # 0x00 is the command bit
        for x in range (0,3):
            if (bitmagic(command_bits,x,"getbit")==1):
                h=bitmagic(h,x,'setbit')
                h1=bitmagic(h1,x,'setbit')
            else:    
                h=bitmagic(h,x,'clearbit')
                h1=bitmagic(h1,x,'clearbit')
        
    time.sleep(5)
       
t=0
ctypes.c_uint32.from_buffer(mem, 0x200).value = t 
ctypes.c_uint32.from_buffer(mem1, 0x200).value = t
