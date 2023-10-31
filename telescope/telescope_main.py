# ALT - AZ Telescope Tracking
# written (poorly) by chris martin 10/30/2023 (several months work)
# Uses lots of bits and pieces from the internet
# Python Script is for Python 3
# should use BB image (debian10) bone-eMMC-flasher-debian-10.3-iot-armhf-2020-04-06-4gb.img(2).xz
# last one that still has Cloud9 IDE

# Uses BB PRU1 This puts the stepper outputs on the same pins used for the processor boot sequence pins.
# this requires unpluging the stepper io plugs (these P8 pins cannot be grounded or held high) need a solution here.
# using PRU 0 would stop this (P9) but on the BBG the wifi chip blocks these pins and this program really needs wifi
# the Pru program is separate and must be placed into /debian/home/bin folder 
# "sudo cp /home/debian/bin/Steppercontrolpru1.out /lib/firmware/Steppercontrolpru1.out"

# the camera is a SVBONY 305. I am using indigo server instead of having it embedded. Port :7624
# sudo indigo_server to start server

#Telescope web page is on port :5000 

# this script must be run sudo.
# sudo python3 telescope_main.py


import ctypes  #ctypes is used to pass information to the PRU / BBG shared memory
from ctypes import *
import subprocess # automate config-pin calls and starting PRU program
import os
import mmap # access pru shared ram
import time # clock-- BBG time needs to be accurate for tracking.
import board # uses adafruit BLinka
import adafruit_lsm303_accel #ADAFRUIT LSM303AGR I2C coms I'm not impressed with the accuracy of the compass and the Accelerometer is jittery
import adafruit_lis2mdl #ADAFRUIT LSM303AGR I2C coms I'm not impressed with the accuracy of the compass and the Accelerometer is jittery
import adafruit_mma8451 #ADAFRUIT Accelerometer I2C coms
import busio
import serial # Serial link to GPS (UART)
import math
import numpy
from inputimeout import inputimeout, TimeoutOccurred # Teminal Input() Menu times out to keep things moving
import Adafruit_BBIO.GPIO as GPIO # Used run required gpio pins 
import inquirer # pretty neat menu selector for terminal input
from telescopemotorcontrolclass import motorcontrol # Class to run the telescope steppers 
from pynmeagps import NMEAReader # used to parse GPS DATA

i2c = board.I2C()  #Adafruit blinka create board
sensor = adafruit_lsm303_accel.LSM303_Accel(i2c) # start base accelerometer sensor
mag = adafruit_lis2mdl.LIS2MDL(i2c) # start base compass sensor
accel = adafruit_mma8451.MMA8451(i2c) # start tube accelerometer sensor
accel.data_rate=adafruit_mma8451.DATARATE_100HZ 
gps = serial.Serial(port = "/dev/ttyO1", baudrate=9600) #connect to gps sensor uart
gps.close()
gps.open()  


# GLOBALS  ( there are better ways to do this, but this works)
telescope_motion = motorcontrol() # Instance of motor control class for the steppers
command_bits=0  # The command bits are how the PRU program and this script pass data and provide for the handshake

Fcurrentangle=0 # degrees - this is the position of the focuser my focuser has 3.25 turns for is range of motion 1170 deg
anglemanualmove=2 # degrees - this is the default increment that the base (AZ) or scope (ALT) turns when manually moved or bumps when in auto(tracking)
anglemanaulmovespeed=2 #degrees/sec - this is the default speed setting
Alt_angle=90.000 # telescope is straight up
Az_angle=0.000 #base is pointing north
AZoffset=0.0 # degrees - used to correct the tracking position (fine tune in flight)
ALToffset=0.0 # degrees - used to correct the tracking position (fine tune in flight)
mode=0 # tracks what mode the telescope is in
latitude = 44.194 #deg default location change this to your location
longitude= 88.447 #deg default location change this to your location
elevation= 51.2 #meters
DMEM_Offset=0x0002000  #pru DMEM
PRU1_Address=0x4a300000  #pru1 base address
Address=(DMEM_Offset+PRU1_Address)
fd = os.open("/dev/mem", os.O_SYNC | os.O_RDWR)
mem1 = mmap.mmap(fd, mmap.PAGESIZE, flags=mmap.MAP_SHARED, offset=Address)
os.close(fd)

hardiron_calibration = [[-68.7, -20.849999999999998], [-12.45, 26.55], [-40.8, -32.699999999999996]] #would be nice to store this in a file after calibration

def normalize(_magvals):

    ret = [0, 0, 0]

    for i, axis in enumerate(_magvals):

        minv, maxv = hardiron_calibration[i]

        axis = min(max(minv, axis), maxv)  # keep within min/max calibration

        ret[i] = (axis - minv) * 200 / (maxv - minv) + -100

    return ret

from skyfield.api import load  # skyfield library https://rhodesmill.org/skyfield/
from skyfield.api import N, W, wgs84 # sets targets realtime
from skyfield.api import Star, load


planets = load('de421.bsp')  #load the ephe data for planets
ts = load.timescale() # get time
planet = [
  inquirer.List('size',
                message="Select planet",
                choices=['sun', 'earth','moon' ,'MERCURY BARYCENTER', 'VENUS BARYCENTER', 'MARS BARYCENTER', 'JUPITER BARYCENTER',
                'SATURN BARYCENTER','URANUS BARYCENTER', 'NEPTUNE BARYCENTER'],
            ),
]  # terminal drop down list for planets

star = [
  inquirer.List('size',
                message="Select star",
                choices=['polaris', 'vega', 'Andromeda_M31','pleiades','mizar','m101','mirphak','deneb' ],
            ),
] # terminal drop down list for a couple of stars

sun = planets['sun']
home = planets['earth'] + wgs84.latlon(latitude * N, longitude * W)

polaris = Star(ra_hours=(3,3,12.6), dec_degrees=(89,21,38.7))
vega= Star(ra_hours=(18,37,44.2), dec_degrees=(38,48,34.0))
Andromeda_M31=Star(ra_hours=(00,43,03.3), dec_degrees=(41,24,02.4))
pleiades=Star(ra_hours=(3,48,25.8), dec_degrees=(24,11,29.4))
mizar=Star(ra_hours=(13,24,51.0), dec_degrees=(54,48,4.0))
m101=Star(ra_hours=(14,4,1.2), dec_degrees=(54,14,6.0))
mirphak=Star(ra_hours=(3,26,3.0), dec_degrees=(49,56,43.5))
deneb=Star(ra_hours=(20,42,14.0), dec_degrees=(45,22,08.9))
target=polaris # default target is the north star

def bitmagic(value, bitnumber,task):  # utility to set individual bits this is used for the commandbit dialog between the program and PRU
    if task == 'getbit':
        return ((value>>bitnumber)&1)
    if task == 'setbit':
        return (value | (1<<bitnumber))
    if task == 'clearbit':
        return (value & ~(1<<bitnumber))
    if task =='TogleBit':
        return (value ^((value>>bitnumber)&1))

from flask import Flask, render_template, session, request, Response  # Web page handler
from flask_socketio import SocketIO, emit, disconnect

from threading import Thread  # program uses 3 threads... 1. web page/ socketio 2. main program 3. display data routine
thread=5 # max threads not used

import logging #this section allows the log data to not spam the terminal window
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

async_mode = None

app=Flask(__name__,template_folder='templates') #sets location of the index.html file (gets lost otherwise)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, async_mode=async_mode, logger=False, engineio_logger=False, log_output=False)

@app.route('/') #home page set at server root directory
def index():
    return render_template('index.html', async_mode=socketio.async_mode, log_output=False)
   

@socketio.on('my_event' ) # inital handshake to the web page
def test_message(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    emit('my_response',
         {'data': message['data'], 'count': session['receive_count']})
    print("connected")     

@socketio.on('panright_event') # turn the base CW viewed from top
def panright(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    global anglemanaulmovespeed # access to global default stepper speed
    global mode # access to global program mode
    global command_bits # access to global stepper command word
    global Az_angle # access to global position tracking var
    global AZoffset # access to global position tracking var
    moveangle=float(message['data']) #get move angle from web page
    movespeed=anglemanaulmovespeed
    if mode=="m":  #if manual mode then move the turn base at the default speed x degrees
        print (moveangle, " ", movespeed)
        if ((bitmagic(command_bits,9,"getbit")==0) and (bitmagic(command_bits,10,"getbit")==0) and (bitmagic(command_bits,11,"getbit")==0) ): #make sure the pru is not already running a motor
            command_bits = telescope_motion.AZ_rotate(command_bits,-moveangle,anglemanaulmovespeed,0)
            Az_angle-=moveangle #track angle
            ctypes.c_uint32.from_buffer(mem1, 0x200).value=command_bits #send command word to the PRU shared memory
    elif mode=="a": #if auto mode then move bump AZ position 
            AZoffset-=moveangle
    else:
        emit('my_response',
         {'data': 'not in man or auto', 'count': session['receive_count']},
         broadcast=True) #send warning to web page if you try to rotate the base when not in the right mode
        time.sleep(2) #holds warning fro a couple of seconds before being over written
    
    emit('my_response', 
         {'data': message['data'], 'count': session['receive_count']},
         broadcast=True)

@socketio.on('panleft_event') # turn the base CCW viewed from top
def panleft(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    global anglemanaulmovespeed # access to global default stepper speed
    global mode # access to global program mode
    global command_bits # access to global stepper command word
    global Az_angle # access to global position tracking var
    global AZoffset # access to global position tracking var
    moveangle=float(message['data']) #get move angle from web page
    movespeed=anglemanaulmovespeed
    if mode=="m": #if manual mode then move the turn base at the default speed x degrees
        print (moveangle, " ", movespeed)
        if ((bitmagic(command_bits,9,"getbit")==0) and (bitmagic(command_bits,10,"getbit")==0) and (bitmagic(command_bits,11,"getbit")==0) ):
            command_bits = telescope_motion.AZ_rotate(command_bits,moveangle,anglemanaulmovespeed,0)
            Az_angle+=moveangle #track angle
            ctypes.c_uint32.from_buffer(mem1, 0x200).value=command_bits #send command word to the PRU shared memory
    elif mode=="a": #if auto mode then move bump AZ position 
            AZoffset+=moveangle
    else:
        emit('my_response',
         {'data': 'not in man or auto', 'count': session['receive_count']},
         broadcast=True) #send warning to web page if you try to rotate the base when not in the right mode
        time.sleep(2)
    emit('my_response',
         {'data': message['data'], 'count': session['receive_count']},
         broadcast=True)

@socketio.on('altup_event') # raise the telescope
def altup(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    global anglemanaulmovespeed # access to global default stepper speed
    global mode # access to global program mode
    global command_bits # access to global stepper command word
    global Alt_angle  # access to global position tracking var
    global ALToffset  # access to global position tracking var
    moveangle=float(message['data']) #get move angle from web page
    movespeed=anglemanaulmovespeed
    if mode=="m": #if manual mode then move the turn base at the default speed x degrees
        print (moveangle, " ", movespeed)
        if ((bitmagic(command_bits,9,"getbit")==0) and (bitmagic(command_bits,10,"getbit")==0) and (bitmagic(command_bits,11,"getbit")==0) ):
            command_bits = telescope_motion.ALT_move(command_bits,-moveangle,anglemanaulmovespeed,0)
            Alt_angle-=moveangle  #track angle
            ctypes.c_uint32.from_buffer(mem1, 0x200).value=command_bits
    elif mode=="a": #if auto mode then move bump ALT position
            ALToffset-=moveangle
    else:
        emit('my_response',
         {'data': 'not in man or auto', 'count': session['receive_count']},
         broadcast=True)
        time.sleep(2)
    emit('my_response',
         {'data': message['data'], 'count': session['receive_count']},
         broadcast=True)

@socketio.on('altdown_event') # raise the telescope
def altup(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    global anglemanaulmovespeed # access to global default stepper speed
    global mode # access to global program mode
    global command_bits # access to global stepper command word
    global Alt_angle  # access to global position tracking var
    global ALToffset  # access to global position tracking var
    moveangle=float(message['data']) #get move angle from web page
    movespeed=anglemanaulmovespeed
    if mode=="m": #if manual mode then move the turn base at the default speed x degrees
        print (moveangle, " ", movespeed)
        if ((bitmagic(command_bits,9,"getbit")==0) and (bitmagic(command_bits,10,"getbit")==0) and (bitmagic(command_bits,11,"getbit")==0) ):
            command_bits = telescope_motion.ALT_move(command_bits,moveangle,anglemanaulmovespeed,0)
            Alt_angle+=moveangle
            ctypes.c_uint32.from_buffer(mem1, 0x200).value=command_bits
    elif mode=="a": #if auto mode then move bump ALT position
            ALToffset+=moveangle
    else:
        emit('my_response',
         {'data': 'not in man or auto', 'count': session['receive_count']},
         broadcast=True)
        time.sleep(2)
    emit('my_response',
         {'data': message['data'], 'count': session['receive_count']},
         broadcast=True)

@socketio.on('focusin_event') # move focuser in
def focusin_event(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    focdeg=float(message['data'])
    global command_bits
    global Fcurrentangle
    command_bits=telescope_motion.focuser_move(command_bits,-focdeg,30,0) #focuser moves focdeg at 30 deg/seg with 0 AD steps
    ctypes.c_uint32.from_buffer(mem1, 0x200).value=command_bits
    Fcurrentangle+=-focdeg
    emit('my_response',
         {'data': message['data'], 'count': session['receive_count']},
         broadcast=True)

@socketio.on('focusout_event') # move focuser in
def focusin_event(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    focdeg=float(message['data']) 
    global command_bits
    global Fcurrentangle
    command_bits=telescope_motion.focuser_move(command_bits,focdeg,30,0) #focuser moves focdeg at 30 deg/seg with 0 AD steps
    ctypes.c_uint32.from_buffer(mem1, 0x200).value=command_bits
    Fcurrentangle+=focdeg
    emit('my_response',
         {'data': message['data'], 'count': session['receive_count']},
         broadcast=True)

@socketio.on('turning_rate_event')  #change the base and tilt motor speed, don't exceed 6 deg /sec
def turning_rate_event(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    global anglemanaulmovespeed
    anglemanaulmovespeed=float(message['data'])
    emit('my_response',
         {'data': 'speed set', 'count': session['receive_count']},
         broadcast=True)


@socketio.on('latlong_event') # get new location from web page
def latlong_event(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    global latitude
    global longitude
    latitude=float(message['data0'])
    longitude=float(message['data1'])
    elevation=float(message['data2'])
    print("lat, lon, elev: ",latitude,", ",longitude,", ", elevation)
    emit('my_response',
         {'data': 'stopping', 'count': session['receive_count']})  


@socketio.on('setposition_request') # set the current position of the scope as the tracking target
def setposition_request():
    session['receive_count'] = session.get('receive_count', 0) + 1
    global Alt_angle
    global Az_angle
    global target
    
    home = planets['earth'] + wgs84.latlon(latitude * N, longitude * W)  #skyfield topo home position
    t=ts.now() # need the current time (epoch)
    direction = home.at(t).from_altaz(alt_degrees=Alt_angle, az_degrees=Az_angle) # scope position in ALT AZ 
    rah,dech,dist=direction.radec() # contvert to RA DEC 
    print(rah,dech,dist)
    target=Star(ra=rah, dec=dech)    # set current position to be the target
    print(target)
    emit('my_response',
         {'data': 'set position', 'count': session['receive_count']})     

@socketio.on('stop_request') #stop the scope and enter idle mode"
def stop_request():
    session['receive_count'] = session.get('receive_count', 0) + 1
    global mode
    print ("stop")
    mode="s"
    print (mode)
    emit('my_response',
         {'data': 'stopping', 'count': session['receive_count']})     
         
@socketio.on('Manual_request') # put the scope in manual mode
def Manual_request():
    session['receive_count'] = session.get('receive_count', 0) + 1
    global mode
    print ("manual")
    mode='m'
    print (mode)
    emit('my_response',
         {'data': 'manual', 'count': session['receive_count']})   

@socketio.on('Auto_request')  # put the scope in auto mode (start tracking)
def Auto_request():
    session['receive_count'] = session.get('receive_count', 0) + 1
    global mode
    print ("auto")
    mode='a'
    print (mode)
    emit('my_response',
         {'data': 'auto', 'count': session['receive_count']})  

@socketio.on('ALThome_request') # home the ALT axis of the scope straight up and down
def ALT_home_request():
    session['receive_count'] = session.get('receive_count', 0) + 1
    global mode
    print ("Homing ALT")
    mode='l'
    print (mode)
    emit('my_response',
         {'data': 'home ALT', 'count': session['receive_count']}) 

@socketio.on('AZhome_request') # home the AZ axis of the scope based on the compass set north
def ALT_home_request():
    session['receive_count'] = session.get('receive_count', 0) + 1
    global mode
    print ("Homing AZ")
    mode='z'
    print (mode)
    emit('my_response',
         {'data': 'home AZ', 'count': session['receive_count']}) 

@socketio.on('calibratecompass_request') # calibrate the compass do this out side
def calibratecompass_request():
    session['receive_count'] = session.get('receive_count', 0) + 1
    global mode
    print ("Compass calibration")
    mode='c'
    print (mode)
    emit('my_response',
         {'data': 'calibrate compass', 'count': session['receive_count']}) 

@socketio.on('Focuserhome_request') # send the focuser home
def Focuserhome_request():
    session['receive_count'] = session.get('receive_count', 0) + 1
    global command_bits
    global Fcurrentangle
    print ("Focuser home request")
    command_bits=telescope_motion.focuser_home(command_bits)
    ctypes.c_uint32.from_buffer(mem1, 0x200).value=command_bits
    Fcurrentangle=0
    print ('homing')
    done=1
    while done==1:
        command_bits = ctypes.c_uint32.from_buffer(mem1, 0x200).value
        done=bitmagic(command_bits,2,'getbit')
        time.sleep(.5)
    print ("focuser @ home")    
    emit('my_response',
         {'data': 'focuser @ home', 'count': session['receive_count']}) 


@socketio.on('my_ping') # legacy webpage verification on connection
def ping_pong():
    emit('my_pong', broadcast=False,log_output=False)


@socketio.on('connect' )  # not used plan to start main program regardless of web page connection
def test_connect():
    global thread
    #if thread is None:
    #    thread = socketio.start_background_task(target=background_thread,log_output=False)
    emit('my_response', {'data': 'Connected', 'count': 1})
    print("ok")

@socketio.on('target_event') # change target to coordinates entered on webpage
def target_event(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    global target
    
    RA_deg=float(message['data0'])
    RA_min=float(message['data1'])
    RA_sec=float(message['data2'])
    Dec_deg=float(message['data3'])
    Dec_min=float(message['data4'])
    Dec_sec=float(message['data5'])
    
    target=Star(ra_hours=(RA_deg,RA_min,RA_sec), dec_degrees=(Dec_deg,Dec_min,Dec_sec))
    print("target = ", target)    
    emit('my_response',
         {'data': 'target', 'count': session['receive_count']},
         broadcast=True)

@socketio.on('target_planet_event') #change target to planet from webpage list
def target_planet_event(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    global target
    target=planets[message['data']]
    print("target = ", target)
    emit('my_response',
         {'data': 'speed set', 'count': session['receive_count']},
         broadcast=True)

@socketio.on('target_star_event') #change target to star from webpage list
def target_start_event(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    global target
    target=globals()[message['data']]
    print("target = ", target)
    emit('my_response',
         {'data': 'speed set', 'count': session['receive_count']},
         broadcast=True)

def Display_cmd(cmdstring, cmdstring1=None):  # send status to web page
    subprocess.run("clear", shell=True, check=True)
    data=[]
        
    if cmdstring1 is None:
        print("command: ", cmdstring)
        socketio.emit('recieve_server_data_event', data=(str(cmdstring)) )
            
    else:
        print("command1: ", cmdstring)
        print("command2: ", cmdstring1)
        socketio.emit('recieve_server_data_event', data=(cmdstring,cmdstring1) )    

def Display_data():  # send telescope data to web page
    data=[]
    global Fcurrentangle
    global Alt_angle
    global Az_angle
    global mode
    global target
    silent=True

    while True:
        # mode
        if mode=="z" or mode =="Z":
            mode_str="Homing Azimuth Mode:"
        if mode=="c" or mode =="C":
            mode_str="Compass Calibration Mode"
        if mode=="l" or mode =="L":
            mode_str="Homing Altitude Mode"
        if mode=="q" or mode =="Q":
            mode_str="cut power"
        if mode=="M" or mode=="m":
            mode_str="Manual Mode"
        if (mode=="A" or mode =="a"):
            mode_str="Auto Mode"
        if (mode=="T" or mode =="t"):
            mode_str="Set target from terminal Mode"    
        if mode==0:
            mode_str="Idle Mode"
        #Mode end
        if not silent:
            print (mode_str)
        
        #position data
        pos_data_str=str("Alt: "+str(Alt_angle)+" AZ: "+str(Az_angle)+" Focuser pos: "+str(Fcurrentangle))
        target_str=str("Target: "+str(target))
        
        # tube sensor
        Accx,Accy,Accz=accel.acceleration # tupple from sensor
        angletohome=math.degrees(math.atan2(round(Accz,2),-round(Accx,2)))
        tube_sensor_str=str("tube angle: Acceleration (m/s^2): X=%0.5f Y=%0.5f Z=%0.5f"%accel.acceleration+"...Tube angle from vertical:%0.5f"%(angletohome-2.85))
        
        if not silent:
            print (tube_sensor_str)
        # tube sensor end
        
        # Base Level
        Accx,Accy,Accz=sensor.acceleration # tupple from sensor
        absvector=math.sqrt(Accx*Accx+Accy*Accy+Accz*Accz)
        Axn,Ayn,Azn=Accx/absvector,Accy/absvector,Accz/absvector
        Anormal=(Axn,Ayn,Azn) # normalize sensor vector
        Aup=(0,0,1) # up vector
        vectordotproduct=numpy.dot(Anormal,Aup) #angle between vectors
        vectorcrossproduct=numpy.cross(Aup,Anormal) #not used cross product give vector normal to two vectors
        anglebetweenvectors=math.degrees(math.acos(vectordotproduct/(1*1)))
        base_sensor_str=str("Base: Normal X=%0.5f Y=%0.5f Z=%0.5f"%Anormal+"...Base angle from vertical: =%0.5f"%anglebetweenvectors)
        
        if not silent:
            print(base_sensor_str)
        #Base Level end 
       
        #Compass
        magvals = mag.magnetic
        normvals = normalize(magvals)
        compass_heading = int(math.atan2(normvals[1], normvals[0]) * 180.0 / math.pi)
        # this translates it to be between 0 and 360
        compass_heading += 180
        compass_str=str("Heading (N=0, E=90, S=180, W=270) :"+str(compass_heading)+" deg")
        if not silent:
            print(compass_str)
        #Compass end
        
        #GPS
        gpsmsg1 = NMEAReader.parse( (gps.readline()))
        gpsmsg2 = NMEAReader.parse( (gps.readline()))
        gpsmsg3 = NMEAReader.parse( (gps.readline()))
        if not silent:
            print(gpsmsg1)
            print(gpsmsg2)
            print(gpsmsg3)
        #GPS end
    
        socketio.emit('recieve_current_data_event', data=(mode_str,pos_data_str,target_str,tube_sensor_str,base_sensor_str,compass_str,str(gpsmsg1),str(gpsmsg2),str(gpsmsg3)) )
        time.sleep(2)


def background_thread(): #main program
    
    #local varibles
    command_bits0=0
    command_bits1=0
    command_bits2=0
    done = False
    Fmanualmove=10
    Fmanualspeed=360
    Offsetincrement=.1
    
    #global varible call outs
    global target
    global ALToffset
    global AZoffset
    global Alt_angle
    global Az_angle
    global anglemanualmove
    global anglemanaulmovespeed
    global mode
    global Fcurrentangle
    
    command_bits = ctypes.c_uint32.from_buffer(mem1, 0x200).value # grab command bits to start

    while not done:  #main loop
        print ("mode: ",mode)
        time.sleep(1) 
        if ((bitmagic(command_bits,9,"getbit")==0) and (bitmagic(command_bits,10,"getbit")==0) and (bitmagic(command_bits,11,"getbit")==0) ): #make sure nothing is running before a new command issues
            subprocess.run("clear", shell=True, check=True)
            try:
                mode = inputimeout(prompt="enter mode Select (T)arget (A)uto, (M)anual, (C)ompass calibration, A(Z) home,  A(L)T home, (Q)uit :", timeout=5)
            except TimeoutOccurred:
                print(mode)  #terminal abridged menu
            
            if mode=="z" or mode =="Z": # home the base to the compass direction
                command_bits0 = telescope_motion.AZ_home(command_bits0)
                Display_cmd("AZ home")
                Az_angle=0.000 #base is pointing north
                mode=0
            if mode=="c" or mode =="C": # calibrate the compass
                hardiron_calibration=telescope_motion.calibrate_compass()
                Display_cmd("compass calibration")
                mode=0
            if mode=="l" or mode =="L": 
                command_bits1 = telescope_motion.ALT_home(command_bits1)
                Display_cmd("ALT home")
                Alt_angle=90.000 # telescope is straight up
                mode=0
            if mode=="q" or mode =="Q": # home the telescope tube straight up
                telescope_motion.disable_stepper_drivers()
                done=True
                Display_cmd("cut power")
                mode=0
            command_bits=command_bits0|command_bits1|command_bits2
            print ('out ',hex(command_bits))
            ctypes.c_uint32.from_buffer(mem1, 0x200).value=command_bits
            
            if mode=="M" or mode=="m": #manual mode
                mode="m"
                manual = True
                
                while manual:
                    
                    command_bits = ctypes.c_uint32.from_buffer(mem1, 0x200).value # 0x00 is the command bit
                    if ((bitmagic(command_bits,9,"getbit")==0) and (bitmagic(command_bits,10,"getbit")==0) and (bitmagic(command_bits,11,"getbit")==0) ):
                        
                        #the bbg has a control pad from a cnc machine for local adjustments to position and scope.
                        if GPIO.event_detected("P8_28"):
                            Display_cmd("y+ pressed")
                            command_bits = telescope_motion.ALT_move(command_bits,-anglemanualmove,anglemanaulmovespeed,0)
                            Alt_angle=Alt_angle-anglemanualmove
                            
                        if GPIO.event_detected("P8_29"):
                            Display_cmd("y- pressed")
                            command_bits = telescope_motion.ALT_move(command_bits,anglemanualmove,anglemanaulmovespeed,0)
                            Alt_angle=Alt_angle-anglemanualmove
                        if GPIO.event_detected("P8_30"):
                            Display_cmd("x- pressed")
                            command_bits = telescope_motion.AZ_rotate(command_bits,-anglemanualmove,anglemanaulmovespeed,0)
                            Az_angle=Az_angle-anglemanualmove
                            
                        if GPIO.event_detected("P8_7"):
                            Display_cmd("x+ pressed")
                            command_bits = telescope_motion.AZ_rotate(command_bits,anglemanualmove,anglemanaulmovespeed,0)
                            Az_angle=Az_angle+anglemanualmove
                            
                        if GPIO.event_detected("P8_8"):
                            Display_cmd("Z- zoom in pressed") 
                            command_bits = telescope_motion.focuser_move(command_bits,-Fmanualmove,Fmanualspeed,0)
                            
                        if GPIO.event_detected("P8_9"):
                            Display_cmd("Z+ zoom out pressed") 
                            command_bits = telescope_motion.focuser_move(command_bits,Fmanualmove,Fmanualspeed,0)
                            
                        if GPIO.event_detected("P8_10"):
                            Display_cmd("A- set actual postion") 
                            
                        if ((bitmagic(command_bits,6,"getbit")==1) or (bitmagic(command_bits,7,"getbit")==1) or (bitmagic(command_bits,8,"getbit")==1) or mode=="s" ):
                            manual=False
                            mode=0
                            
                        ctypes.c_uint32.from_buffer(mem1, 0x200).value=command_bits
                
                        time.sleep(.1)        
            
            if (mode=="A" or mode =="a"): #auto mode
                mode="a"
                automatic =True
                
                while automatic:
                    command_bits = ctypes.c_uint32.from_buffer(mem1, 0x200).value # 0x00 is the command bit
                    if ((bitmagic(command_bits,6,"getbit")==1) or (bitmagic(command_bits,7,"getbit")==1) or (bitmagic(command_bits,8,"getbit")==1) or mode=="s" ):
                        automatic=False
                        mode=0
                    if ((bitmagic(command_bits,9,"getbit")==0) and (bitmagic(command_bits,10,"getbit")==0) and (bitmagic(command_bits,11,"getbit")==0) ):
                        subprocess.run("clear", shell=True, check=True)
                        t=ts.now()
                        astrometric = home.at(t).observe(target)
                        alt, az, d = astrometric.apparent().altaz()
                        
                        
                        if GPIO.event_detected("P8_28"):
                            Display_cmd("y+ pressed")
                            ALToffset=ALToffset-Offsetincrement
                            
                        if GPIO.event_detected("P8_29"):
                            Display_cmd("y- pressed")
                            ALToffset=ALToffset+Offsetincrement
                            
                        if GPIO.event_detected("P8_30"):
                            Display_cmd("x- pressed")
                            AZoffset=AZoffset-Offsetincrement
                            
                        if GPIO.event_detected("P8_7"):
                            Display_cmd("x+ pressed")
                            AZoffset=AZoffset+Offsetincrement
                        
                        #print("Target >> Alt = ", alt.degrees," AZ = ",az.degrees)
                        #print("focuser at :", Fcurrentangle)
                        message1 =str("Target >> Alt = " + str(alt.degrees)+" AZ = "+ str(az.degrees) +" :: "+" focuser at :" +str(Fcurrentangle))
                        
                        Alt_angle=(Alt_angle-alt.degrees)+ALToffset
                        Az_angle=Az_angle-az.degrees+AZoffset
                        ALToffset=0
                        AZoffset=0
                        if (abs(Alt_angle)>10.00):
                            altspeed=4
                            altadtime=1
                        elif (abs(Alt_angle)>1.00):
                            altspeed=2
                            altadtime=0
                        elif(abs(Alt_angle)>0.02):
                            altspeed=.5
                            altadtime=0
                        else:
                            altspeed=.25
                            altadtime=0   
                        
                        if (abs(Az_angle)>10.00):
                            azspeed=4
                            azadtime=1
                        elif (abs(Az_angle)>1.00):
                            azspeed=2
                            azadtime=0
                        elif(abs(Az_angle)>0.02):
                            azspeed=.5
                            azadtime=0
                        else:
                            azspeed=.25
                            azadtime=0    
                        
                        command_bits = telescope_motion.AZ_rotate(command_bits,Az_angle,azspeed,azadtime)  #AZ
                        command_bits |=telescope_motion.ALT_move(command_bits,Alt_angle,altspeed,altadtime)  #Alt
                        
                        message2=str("Moving >> Alt = "+str(Alt_angle)+" at speed, accel time: "+str(altspeed)+" , "+str(altadtime)+
                        " AZ = "+str(Az_angle)+" at speed, accel time: "+str(azspeed)+", "+ str(azadtime))
                        Display_cmd(message1,message2)
                        
                        if GPIO.event_detected("P8_8"):
                            Display_cmd("Z- zoom in pressed") 
                            command_bits |= telescope_motion.focuser_move(command_bits,-Fmanualmove,Fmanualspeed,0)
                            Fcurrentangle=Fcurrentangle-Fmanualmove
                            if (Fcurrentangle < 0):
                                Fcurrentangle=0
                        
                        if GPIO.event_detected("P8_9"):
                            Display_cmd("Z+ zoom out pressed") 
                            command_bits |= telescope_motion.focuser_move(command_bits,Fmanualmove,Fmanualspeed,0)
                            Fcurrentangle=Fcurrentangle-Fmanualmove
                            if Fcurrentangle > 360*3.25:
                                Fcurrentangle=360*3.25
                        Alt_angle=alt.degrees
                        Az_angle=az.degrees
                    
                        ctypes.c_uint32.from_buffer(mem1, 0x200).value=command_bits
                        
                        time.sleep(.1)
            if (mode=="T" or mode =="t"): #set target
                
                target_type=input("(p)lanet, (s)tar, or (C)oordinates: ")
                
                if (target_type=="P" or target_type =="p"):
                    get = inquirer.prompt(planet)
                    target=planets[get["size"]]
                    print("target = ", target)
                
                if (target_type=="S" or target_type =="s"):
                    get = inquirer.prompt(star)
                    target=globals()[get["size"]]
                    print("target = ", target)
                    
                if (target_type=="C" or target_type =="c"):
                    RA_deg=float(input("Enter RA (Deg):"))
                    RA_min=float(input("Enter RA (min):"))
                    RA_sec=float(input("Enter RA (sec):"))
                    Dec_deg=float(input("Enter Dec (Deg):"))
                    Dec_min=float(input("Enter Dec (min):"))
                    Dec_sec=float(input("Enter Dec (sec):"))
                    target=Star(ra_hours=(RA_deg,RA_min,RA_sec), dec_degrees=(Dec_deg,Dec_min,Dec_sec))
                    print("target = ", target)    
                mode=0

def start_flask_app(): # web page socket i/o thread
    
    if __name__ == '__main__':    
        socketio.run(app, host='0.0.0.0',port =5000, debug=False,log_output=False)
    pass

t1 = Thread(target=background_thread)
t2 = Thread(target=start_flask_app)
t3 = Thread(target=Display_data)
t1.start()
t2.start()
t3.start()

# need to start indigo server
#sudo indigo_server




