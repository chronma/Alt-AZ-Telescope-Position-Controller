# Beaglebone-PRU-Stepper-Motor-Driver
Control stepper motors via the PRU micro controllers on the Beaglebone platform.  
C programs to run 6 outputs and 3 inputs on the 2 PRU microcontrollers. for a project i'm working on I need non-integer stepper drive frequencies.  the PRU's are fast enough to approximate this.
The program does not use the remproc dialog or the older UIO libraries, instead (still in progess) it uses command bits and bytes in each PRU* DMEM areas. these can be written and read from the PRU and from the ARM.

The program for the PRU's was written and compiled using TI Code Composer Studio and the Python to test on the arm side.




Some useful notes:
BBG nonwifi,

after sharing usb over internet in windows 
there is set up to do on the BBG
each restart may require these to alllow acces to the internet.
sudo /sbin/route add default gw 192.168.7.1

ping google.com to check.
steps..
https://www.digikey.com/en/maker/blogs/how-to-connect-a-beaglebone-black-to-the-internet-using-usb
ping 192.168.7.1
sudo /sbin/route add default gw 192.168.7.1
sudo echo nameserver 8.8.8.8 | sudo tee /etc/resolv.conf

to download projects from cloud nine
install zip

sudo apt-get install zip


Code Composer Studio

follow this 
https://community.element14.com/products/devtools/single-board-computers/next-genbeaglebone/b/blog/posts/beaglebone-control-stepper-motors-with-pru---part-1-intentions to get set up.

also this is helpful

https://www.glennklockwood.com/embedded/beaglebone-pru.html

they both use rpmsg to talk between the ar335x and PRUs,


command to show pru state (remoteproc1 /2)
cat /sys/class/remoteproc/remoteproc1/state


I made two CSS projects that test setting Pins P9_29, P9_31 on PRU0 and  P8_45, P8_46 on PRU1
These are 

bb_PRU_STEPPER00 and bb_PRU_STEPPER01

config-pin p9.29 pruout
config-pin p9.31 pruout   * need to be careful with what type of BB you have.. wifi screws with this use uEnv to turn stuff off
 ls /sys/devices/platform/ocp/ | grep pinmux* will list pins that are open.

How to move build file from CSS to PRU

1. use WinSCP to put the file in the /home/debian/bin directory on your BB

2.These command in order move the file into the pru and start it running.
echo 'stop' > /sys/class/remoteproc/remoteproc1/state
sudo cp /home/debian/bin/PRU_access_const_table.out /lib/firmware/PRU_access_const_table.out
or
sudo cp /home/debian/bin/Steppercontrolpru0.out /lib/firmware/Steppercontrolpru0.out
echo 'PRU_access_const_table.out' > /sys/class/remoteproc/remoteproc1/firmware
or
echo 'Steppercontrolpru0.out' > /sys/class/remoteproc/remoteproc1/firmware
echo 'start' > /sys/class/remoteproc/remoteproc1/state

Prudebug is very useful for look at the PRU memory
see yoder's cook book for install
sudo ./prudebug -p AM335x 

