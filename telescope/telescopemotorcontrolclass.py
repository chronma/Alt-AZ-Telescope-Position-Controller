import ctypes
from ctypes import *
import subprocess
import os
import mmap
import time
import board
import adafruit_lsm303_accel
import adafruit_lis2mdl
import adafruit_mma8451
import busio
import serial
import math
import numpy

import Adafruit_BBIO.GPIO as GPIO

#I2C COMS TO Sensors
subprocess.run("config-pin P9_24 uart", shell=True, check=True)
subprocess.run("config-pin P9_26 uart", shell=True, check=True)

i2c = board.I2C()
accel = adafruit_mma8451.MMA8451(i2c)

mag = adafruit_lis2mdl.LIS2MDL(i2c)

sensor = adafruit_lsm303_accel.LSM303_Accel(i2c) # not used yet could check base level

hardiron_calibration = [[-67.05, -20.849999999999998], [-12.45, 26.55], [-38.85, -32.699999999999996]]


def normalize(_magvals,hi_calibration):

    ret = [0, 0, 0]

    for i, axis in enumerate(_magvals):

        minv, maxv = hi_calibration[i]

        axis = min(max(minv, axis), maxv)  # keep within min/max calibration

        ret[i] = (axis - minv) * 200 / (maxv - minv) + -100

    return ret



# globals

Task= ['getbit','setbit', 'clearbit']
#acess pru/arm shared memory   
DMEM_Offset=0x0002000  #pru DMEM
PRU1_Address=0x4a300000  #pru1 base address
Address=(DMEM_Offset+PRU1_Address)

fd = os.open("/dev/mem", os.O_SYNC | os.O_RDWR)
mem1 = mmap.mmap(fd, mmap.PAGESIZE, flags=mmap.MAP_SHARED, offset=Address)
os.close(fd)
command_bits=0
command_bits0=0
command_bits1=0
command_bits2=0

#Axis 4 - azimuth axis
Axis4_microsteps=16
Axis4_ratio=50*10  #gearbox*gear ratio
Axis4_motor_deg_per_step=1.8
Axis4_direction_bit=3
Axis4_command_bit=0
Axis4_deg_per_step=Axis4_motor_deg_per_step/Axis4_microsteps/Axis4_ratio

#Axis 5 - alt axis
Axis5_microsteps=16
Axis5_ratio=50*10  #gearbox*gear ratio
Axis5_motor_deg_per_step=1.8
Axis5_direction_bit=4
Axis5_command_bit=1
Axis5_deg_per_step=Axis5_motor_deg_per_step/Axis5_microsteps/Axis5_ratio

#Axis 6 - focuser axis
Axis6_microsteps=16
Axis6_motor_deg_per_step=1.8
Axis6_max_turns=3.25
Axis6_direction_bit=5
Axis6_command_bit=2
Axis6_deg_per_step=Axis6_motor_deg_per_step/Axis6_microsteps



def bitmagic(value, bitnumber,task):
    if task == 'getbit':
        return ((value>>bitnumber)&1)
    if task == 'setbit':
        return (value | (1<<bitnumber))
    if task == 'clearbit':
        return (value & ~(1<<bitnumber))
    
    if task =='TogleBit':
        return (value ^((value>>bitnumber)&1))

class motorcontrol(object):
    #pru 1
    
    # initialize GPIO at creation
    def __init__(self):
        
        subprocess.run("config-pin P8_45 pruout", shell=True, check=True)
        subprocess.run("config-pin P8_46 pruout", shell=True, check=True)
        subprocess.run("config-pin P8_43 pruout", shell=True, check=True)
        subprocess.run("config-pin P8_44 pruout", shell=True, check=True)
        subprocess.run("config-pin P8_41 pruout", shell=True, check=True)
        subprocess.run("config-pin P8_42 pruout", shell=True, check=True)
        subprocess.run("config-pin P8_39 pruin", shell=True, check=True)
        subprocess.run("config-pin P8_40 pruin", shell=True, check=True)
        subprocess.run("config-pin P8_27 pruin", shell=True, check=True)
        
        subprocess.run("echo 'stop' > /sys/class/remoteproc/remoteproc2/state", shell=True, check=False)
        subprocess.run("sudo cp /home/debian/bin/Steppercontrolpru1.out /lib/firmware/Steppercontrolpru1.out", shell=True, check=True)
        subprocess.run("echo 'Steppercontrolpru1.out' > /sys/class/remoteproc/remoteproc2/firmware", shell=True, check=True)
        subprocess.run("echo 'start' > /sys/class/remoteproc/remoteproc2/state", shell=True, check=True)
    
        # GPIO SET UP inputs keypad
    
        subprocess.run("config-pin P8_28 gpio_input", shell=True, check=True) #ALT+
        subprocess.run("config-pin P8_29 gpio_input", shell=True, check=True) #ALT-
        subprocess.run("config-pin P8_30 gpio_input", shell=True, check=True) #AZ CCW
        subprocess.run("config-pin P8_07 gpio_input", shell=True, check=True) #AZ CW
        subprocess.run("config-pin P8_08 gpio_input", shell=True, check=True) #zoom in
        subprocess.run("config-pin P8_09 gpio_input", shell=True, check=True) #zoom out
        subprocess.run("config-pin P8_10 gpio_input", shell=True, check=True) #capture alt az (move to known target reset angles)

        GPIO.setup("P8_28", GPIO.IN)
        GPIO.setup("P8_29", GPIO.IN)
        GPIO.setup("P8_30", GPIO.IN)
        GPIO.setup("P8_7", GPIO.IN)
        GPIO.setup("P8_8", GPIO.IN)
        GPIO.setup("P8_9", GPIO.IN)
        GPIO.setup("P8_10", GPIO.IN)
        GPIO.setup("P9_27", GPIO.OUT) #pin P9-27 enable steppers high disables
        
        GPIO.add_event_detect("P8_28", GPIO.FALLING)
        GPIO.add_event_detect("P8_29", GPIO.FALLING)
        GPIO.add_event_detect("P8_30", GPIO.FALLING)
        GPIO.add_event_detect("P8_7", GPIO.FALLING)
        GPIO.add_event_detect("P8_8", GPIO.FALLING)
        GPIO.add_event_detect("P8_9", GPIO.FALLING)
        GPIO.add_event_detect("P8_10", GPIO.FALLING)
        
        GPIO.output("P9_27",0) #pin P9-27 enable steppers high disables
        
    def __del__(self):
        
        GPIO.output("P9_27",1) #pin P9-27 enable steppers high disables
        
    def disable_stepper_drivers(self):
        
        GPIO.output("P9_27",1) #pin P9-27 enable steppers high disables

    def enable_stepper_drivers(self):
        
        GPIO.output("P9_27",0) #pin P9-27 enable steppers high disables

    def calibrate_compass(self):
        # turn base a few times and normalize mag sensor readings
        pru1_command_bits=0
        a4steps=int((abs(2*360))/Axis4_deg_per_step) #run the 
        a4pulselength=int((1/4)*Axis4_deg_per_step*200000000) #set frequency
        a4adsteps=int(math.sqrt( (8*(.1*1000000000)+a4pulselength)/(4*a4pulselength))+.5) #no accel /decel
        ctypes.c_uint32.from_buffer(mem1, 0x300).value = a4steps
        ctypes.c_uint32.from_buffer(mem1, 0x304).value = a4pulselength 
        ctypes.c_uint32.from_buffer(mem1, 0x318).value = a4adsteps
        pru1_command_bits=bitmagic(pru1_command_bits,Axis4_direction_bit,'setbit') # 
        pru1_command_bits=bitmagic(pru1_command_bits,Axis4_command_bit,'setbit') # 
        ctypes.c_uint32.from_buffer(mem1, 0x200).value=pru1_command_bits
        
        start_time = time.monotonic()
        # Update the high and low extremes
        while time.monotonic() - start_time < 120.0:
            magval = mag.magnetic
            print("Calibrating - X:{0:10.2f}, Y:{1:10.2f}, Z:{2:10.2f} uT".format(*magval))
            
            for i, axis in enumerate(magval):
                hardiron_calibration[i][0] = min(hardiron_calibration[i][0], axis)
                hardiron_calibration[i][1] = max(hardiron_calibration[i][1], axis)
                
        print("Calibration complete:")
        print("hardiron_calibration =", hardiron_calibration)
        return hardiron_calibration
        
    def AZ_home(self,pru1_command_bits):
        pru1_command_bits=0
        print("hardiron_calibration =", hardiron_calibration)
        for i in range (0,10,1):
            magvals = mag.magnetic
            print("Magnetometer (micro-Teslas)): X=%0.3f Y=%0.3f Z=%0.3f"%magvals)
            normvals = normalize(magvals,hardiron_calibration)
            compass_heading = int(math.atan2(normvals[1], normvals[0]) * 180.0 / math.pi)
            compass_heading += 180
            print("compass_heading: ", compass_heading)
            time.sleep(.25)
        #home is 0
        
        if compass_heading < 180:
            xangletohome=compass_heading
        else:
            xangletohome=-360+compass_heading    
        
        print("angletohome az",xangletohome)
        a4steps=int((abs(xangletohome))/Axis4_deg_per_step) #run the 
        print ('steps', a4steps)
        a4pulselength=int((1/4)*Axis4_deg_per_step*200000000) #set speen (1/deg/sec*deg/step*1/200MHz)
        a4adsteps=30 #no accel /decel
        ctypes.c_uint32.from_buffer(mem1, 0x300).value = a4steps
        ctypes.c_uint32.from_buffer(mem1, 0x304).value = a4pulselength 
        ctypes.c_uint32.from_buffer(mem1, 0x318).value = a4adsteps
        if (xangletohome>0):
            pru1_command_bits=bitmagic(pru1_command_bits,Axis4_direction_bit,'setbit') # 
        else:
             pru1_command_bits=bitmagic(pru1_command_bits,Axis4_direction_bit,'clearbit')
        pru1_command_bits=bitmagic(pru1_command_bits,Axis4_command_bit,'setbit') # 
        
        time.sleep(.1)
        
        return (pru1_command_bits)

    def AZ_rotate(self,pru1_command_bits,a4degrees,a4degreespersec,a4acceltime):
        pru1_command_bits=0
        a4steps=int((abs(a4degrees))/Axis4_deg_per_step) #run the 
        print ('steps', a4steps)
        a4pulselength=int((1/a4degreespersec)*Axis4_deg_per_step*200000000) #set frequency
        print('pulselength',a4pulselength)
        a4adsteps=int(math.sqrt( (8*(a4acceltime*1000000000)+a4pulselength)/(4*a4pulselength))+.5) #no accel /decel
        print('accel steps ',a4adsteps)
        ctypes.c_uint32.from_buffer(mem1, 0x300).value = a4steps
        ctypes.c_uint32.from_buffer(mem1, 0x304).value = a4pulselength 
        ctypes.c_uint32.from_buffer(mem1, 0x318).value = a4adsteps
        if (a4degrees<0):
            pru1_command_bits=bitmagic(pru1_command_bits,Axis4_direction_bit,'setbit') # 
        else:
             pru1_command_bits=bitmagic(pru1_command_bits,Axis4_direction_bit,'clearbit')
        
        pru1_command_bits=bitmagic(pru1_command_bits,Axis4_command_bit,'setbit') # 
        print(hex(pru1_command_bits))
        time.sleep(.1)
        return (pru1_command_bits)


    def ALT_home(self,pru1_command_bits):
        pru1_command_bits=0
        print("Acceleration (m/s^2): X=%0.3f Y=%0.3f Z=%0.3f"%accel.acceleration)
        time.sleep(1)
        for i in range (10):
            Accx,Accy,Accz=accel.acceleration # tupple from sensor
            time.sleep(.2)
            print("Acceleration (m/s^2): X=%0.3f Y=%0.3f Z=%0.3f"%(Accx,Accy,Accz))
        
        angletohome=math.degrees(math.atan2(round(Accz,2),-round(Accx,2)))-2.5
        print("angletohome az",angletohome)
        a5steps=int((abs(angletohome))/Axis5_deg_per_step) #run the 
        print ('steps', a5steps)
        a5pulselength=int((1/1)*Axis5_deg_per_step*200000000) #set speen (1/deg/sec*deg/step*1/200MHz)
        a5adsteps=30 #no accel /decel
        ctypes.c_uint32.from_buffer(mem1, 0x308).value = a5steps
        ctypes.c_uint32.from_buffer(mem1, 0x30C).value = a5pulselength 
        ctypes.c_uint32.from_buffer(mem1, 0x31C).value = a5adsteps
        if (angletohome<0):
            pru1_command_bits=bitmagic(pru1_command_bits,Axis5_direction_bit,'setbit') # 
        else:
             pru1_command_bits=bitmagic(pru1_command_bits,Axis5_direction_bit,'clearbit')
        pru1_command_bits=bitmagic(pru1_command_bits,Axis5_command_bit,'setbit') # 
        print(hex(pru1_command_bits))
        time.sleep(.1)
        print("Acceleration (m/s^2): X=%0.3f Y=%0.3f Z=%0.3f"%accel.acceleration)
        return (pru1_command_bits)

    def ALT_move(self,pru1_command_bits,a5degrees,a5degreespersec,a5acceltime):
        pru1_command_bits=0
        a5steps=int((abs(a5degrees))/Axis5_deg_per_step) #run the 
        print ('steps', a5steps)
        a5pulselength=int((1/a5degreespersec)*Axis5_deg_per_step*200000000) #set frequency
        print('pulselength',a5pulselength)
        a5adsteps=int(math.sqrt( (8*(a5acceltime*1000000000)+a5pulselength)/(4*a5pulselength))+.5) #no accel /decel
        print('accel steps ',a5adsteps)
        ctypes.c_uint32.from_buffer(mem1, 0x308).value = a5steps
        ctypes.c_uint32.from_buffer(mem1, 0x30C).value = a5pulselength 
        ctypes.c_uint32.from_buffer(mem1, 0x31C).value = a5adsteps
        if (a5degrees<0):
            pru1_command_bits=bitmagic(pru1_command_bits,Axis5_direction_bit,'setbit') # 
        else:
             pru1_command_bits=bitmagic(pru1_command_bits,Axis5_direction_bit,'clearbit')
        
        pru1_command_bits=bitmagic(pru1_command_bits,Axis5_command_bit,'setbit') # 
        print(hex(pru1_command_bits))
        time.sleep(.1)
        return (pru1_command_bits)

    def focuser_home(self,pru1_command_bits):
        pru1_command_bits=0
        a6steps=int((Axis6_microsteps*Axis6_max_turns*360)/Axis6_motor_deg_per_step) #run the focuser in until it cycles the max turns (the focuser)
        print ('steps', a6steps)
        time_home=3*(1000000000/5) #homing run time
        print('time home',time_home)
        a6pulselength=int(time_home/a6steps) #set frequency
        a6adsteps=1 #no accel /decel
        ctypes.c_uint32.from_buffer(mem1, 0x310).value = a6steps
        ctypes.c_uint32.from_buffer(mem1, 0x314).value = a6pulselength 
        ctypes.c_uint32.from_buffer(mem1, 0x320).value = a6adsteps
        pru1_command_bits=bitmagic(pru1_command_bits,Axis6_direction_bit,'setbit') # 
        pru1_command_bits=bitmagic(pru1_command_bits,Axis6_command_bit,'setbit') # 
        print(hex(pru1_command_bits))
        time.sleep(.1)
        return (pru1_command_bits)

    def focuser_move(self,pru1_command_bits, a6degrees,a6degreespersec,a6accelrate):
        pru1_command_bits=0
        a6steps=int((Axis6_microsteps*abs(a6degrees))/Axis6_motor_deg_per_step) #run the focuser in until it cycles the max turns (the focuser)
        print ('steps', a6steps)
        a6pulselength=int((1/a6degreespersec)*Axis6_deg_per_step*200000000) #set frequency
        a6adsteps=a6accelrate #no accel /decel
        ctypes.c_uint32.from_buffer(mem1, 0x310).value = a6steps
        ctypes.c_uint32.from_buffer(mem1, 0x314).value = a6pulselength 
        ctypes.c_uint32.from_buffer(mem1, 0x320).value = a6adsteps
        if (a6degrees<0):
            pru1_command_bits=bitmagic(pru1_command_bits,Axis6_direction_bit,'setbit') # 
        else:
             pru1_command_bits=bitmagic(pru1_command_bits,Axis6_direction_bit,'clearbit')
        
        pru1_command_bits=bitmagic(pru1_command_bits,Axis6_command_bit,'setbit') # 
        print(hex(pru1_command_bits))
        time.sleep(.1)
        return (pru1_command_bits)





