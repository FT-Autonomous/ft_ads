As seen in the computing flow chart in this folder the esp32's need to be powered off a usb hub as there are not enough ports on the Pi. This means that they will be communicating over serial (indicated in blue) to the Pi. 

The USB hub allegedly needs 5V at 500 mA and this gets it from the Pi, which is on 5V and max 3A

There are four things that need to be communicated with over Ethernet:

 - Steering Motor Driver
 - Jetson - power rating of 60 W
 - Pi
 - LiDAR

The rest of the modules just need an esp-32 to control everything and these will communicate with the Pi through serial.

The ASMS switch is going to be connected to the Jetson with a USB and that is all that we know for now. 