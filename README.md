# odas_ros
ODAS ROS package: Tabletop version by Marc-Antoine Maheux at [IntRoLab - Université de Sherbrooke](https://introlab.3it.usherbrooke.ca)

This package is a ROS package for [ODAS](https://github.com/introlab/odas)

## Prerequisites
You will need CMake, GCC and the following external libraries:
```
sudo apt-get install cmake gcc libfftw3-dev libconfig-dev libasound2-dev
```

## Installation
First, you need to clone the repository and prepare it for execution. Clone it in the `src` directory of your catkin workspace.
```
git clone https://github.com/introlab/odas_ros.git
```
You can then build the 

## Hardware configuration
For ODAS to locate and track sound sources, it needs to know what is the physical configuration of all microphones. There is a file (`configuration.cfg`) that is used to provide ODAS with all the information it needs. You will need the position and direction of each microphones. See [ODAS Configuration](https://github.com/introlab/odas/wiki/Configuration) for details on each part of the file. 

Here are the important steps:

### Sound card configuration
At this part of the configuration file, you need to set the correct card and dev.
```
# Input with raw signal from microphones
    interface: {    #"arecord -l" OR "aplay --list-devices" to see the devices
        type = "soundcard_name";
        devicename = "hw:CARD=1,DEV=0";
```
To know what is your card number, plug it in your computer and run `arecord -l` in a terminal. The output should look something like this:
```
**** List of CAPTURE Hardware Devices ****
card 0: PCH [HDA Intel PCH], device 0: ALC294 Analog [ALC294 Analog]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 1: 8_sounds_usb [16SoundsUSB Audio 2.0], device 0: USB Audio [USB Audio]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
```
In this case, the card number is 1 and the device is 0 for the 16SoundsUSB audio card. So the device name should be: `"hw:CARD=1,DEV=0";`.

