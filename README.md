# odas_ros
ODAS ROS package: Tabletop version by Marc-Antoine Maheux at [IntRoLab - Université de Sherbrooke](https://introlab.3it.usherbrooke.ca)
Modified by: Olivier Roy

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
sudo pip install libconf
```

In the cloned directory of `audio_utils`, run this line to install all submodules:
```
git submodule update --init --recursive
```

If you get errors when building with `catkin_make`, you can modify the cmake file of audio_utils to add C++ 14 compiler option.
```
add_compile_options(-std=c++14)
```

## Installation
First, you need to clone the repository in your catkin workspace.
```
git clone https://github.com/introlab/odas_ros.git
```
In the cloned directory of `odas_ros`, run this line to install all submodules:
```
git submodule update --init --recursive
```

## Hardware configuration
For ODAS to locate and track sound sources, it needs to be configured. There is a file (`configuration.cfg`) that is used to provide ODAS with all the information it needs. You will need the position and direction of each microphones. See [ODAS Configuration](https://github.com/introlab/odas/wiki/Configuration) for details.

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

### Mapping 
Depending on your configuration, you will need to map the microphones from the soundcard to the software. If you wish to use all microphones, then you can map all of them. For example, if there is 16 microphones and wish to use them all:
```
mapping:
{
    map: (1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16);
};
```
But if only certain microphones should be used, they can be mapped. For example, if I only wish to use microphones 1 to 4 and 6 to 10 and 12:
```
mapping:
{
    map: (1,2,3,4,6,7,8,9,10,12);
};
```
They will be mapped to microphones 1 to 10.

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

### Sound Source Localization, Tracking and Separation
ODAS can output the sound source localization, the source source tracking and the sound source separation:
* Sound Source Localization: All the potential sound sources in the unit sphere. Each sound source position on the unit sphere and its energy.
* Sound Source Tracking: The most probable location of the sound source is provided (xyz position on the unit sphere).
* Sound Source Separation: An Audio Frame of the isolated sound source is provided.

Depending on what type of information will be used, the configuration file needs to be modified. For example, if I need only the Sound Source Tracking, the configuration file should be modified. The only thing that should be changed is the format and interface for each type of data. The required format if it is enabled is `json` and the interface type should be `socket`. If it is disabled, the format can be set to `undefined` and the interface type to `blackhole`.

For example, if the only data that will be used is the sound source tracking:
* In the `# Sound Source Localization` section, this should be modified to look like this:
```
potential: {

        #format = "json";

        #interface: {
        #    type = "socket";
        #    ip = "127.0.0.1";
        #    port = 9002;
        #};

        format = "undefined";

        interface: {
           type = "blackhole";
        };
```

* In the `# Sound Source Tracking` section, this should be modified to look like this:
```
# Output to export tracked sources
    tracked: {

        format = "json";

        interface: {
            type = "socket";
            ip = "127.0.0.1";
            port = 9000;
        };
    };
```

* In the `# Sound Source Separation` section, this should be modified to look like this:
```
separated: { #packaging and destination of the separated files

        fS = 44100;
        hopSize = 256;
        nBits = 16;

        interface: {
           type = "blackhole";
        };

        #interface: {
        #    type = "socket";
        #    ip = "127.0.0.1";
        #    port = 9001;
        #}        
    };
 ```
