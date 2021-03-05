# odas_ros
ODAS ROS package: Tabletop version by Marc-Antoine Maheux at [IntRoLab - Université de Sherbrooke](https://introlab.3it.usherbrooke.ca)

This package is a ROS package for [ODAS](https://github.com/introlab/odas)

## Prerequisites
You will need CMake, GCC and the following external libraries:
```
sudo apt-get install cmake gcc build-essential libfftw3-dev libconfig-dev libasound2-dev
```

ODAS ROS uses the audio utilities from [AudioUtils](https://github.com/introlab/audio_utils) so it should be installed in your catkin workspace. If it is not already, here is how to do so:

Clone AudioUtils in your catkin workspace:
```
git clone https://github.com/introlab/audio_utils.git
```
Install dependencies:
```
sudo apt-get install gfortran texinfo
```

In the cloned directory of audio_utils:
```
git submodule update --init --recursive
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
At this part of the configuration file, you need to set the correct card and device number.
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

### Microphone configuration
For ODAS to precisely locate and track a sound source, it needs to know precisely the microphone position. The frame of reference used to measure the microphones position will end up being the one used for the sound tracking. Here is an example of a microphone configuration. It is easier if the reference point is located in the center of the microphones.
```
# Microphone 1 #TODO
        {
            mu = ( 0.122376, 0.08144437, 0.042 );
            sigma2 = ( +1E-6, 0.0, 0.0, 0.0, +1E-6, 0.0, 0.0, 0.0, +1E-6 );
            direction = ( -0.08144, -0.12238, 0.0 );
            angle = ( 80.0, 100.0 );
        },
```

For Microphone 1, `mu` is the position in x, y and z from the reference point. `sigma2` is the position variance in `xx, xy xz, yx, yy, yz, zx, zy, zz` this setting should mainly remain untouched. The `direction` parameter is the direction of the microphone. It should be a unit vector pointing in the direction that the microphone is pointing relative to the reference frame. The `angle` parameter is the maximum angle at which gain is 1 and minimum angle at which gain is 0. 

ODAS can't provide the tracked object distance but a coordinate in a unit sphere.
