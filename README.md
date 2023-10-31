# Beaglebone based go to control of an ALt AZ telescope
Control of stepper motors via the PRU micro controllers on the Beaglebone platform.  
The program for the PRU's was written and compiled using TI Code Composer Studio and the main program is Python3 on th earm side.  
PLease note the pru pins are problamatic to access for the Beagle Bone Green most of the PRU0 pins are tied up by the on board wifi chip and the pins for 
PRU1 mostly overlap the boot sequence pins for the arm processor, until a wayt to isolate the pins on start up is found the stepper driuver io wires cannot be connected until
after the BB has booted (unplug the io plugs from the stepper drivers)  

Move the compiled pru program from the debug directory to the /debian/home/bin folder. The program will copy it to the firmware folder.  
Reference "sudo cp /home/debian/bin/Steppercontrolpru1.out /lib/firmware/Steppercontrolpru1.out"  

Dependencies: make sure yoru IDE is set to use Python 3, update apt and pip3  
Ctypes (sudo pip3 install ctypes) install for all users  
Adafruit Blinka (sudo pip3 install Adafruit-Blinka) isntall for all users   
Pyserial (sudo pip3 install pyserial)  
Numpy (sudo pip3 install numpy)  
inputimeout 1.0.4 (sudo pip3 install inputimeout)  
Adafruit-BBIO 1.2.0  (sudo pip3 install Adafruit-BBIO)  
inquirer 3.1.3 (sudo pip3 install inquirer)  
pynmeagps 1.0.29 (sudo pip3 install pynmeagps)  
Skyfield (sudo pip3 install skyfield)  
Flask 3.0.0 (sudo pip3 install Flask)  
  
Usage.  
Home the scope and target something easy to find. set in auto mode and fine tune the position with the key pad or web page. once locked on a target it should switchto other 
targets fairly accurately.  

