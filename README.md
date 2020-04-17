# ODAS -> ROS
This project is being developped by the [IntRoLab](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/index.php/Main_Page) team.

## Prerequisites
You will need CMake, GCC and the following external libraries:
FFTW:
```
sudo apt-get install libfftw3-dev
```
LibConfig:
```
sudo apt-get install libconfig-dev
```
ALSA:
```
sudo apt-get install libasound2-dev
```

## Installation
First, you need to clone the repository and prepare it for execution.
```
cd ~/catkin_ws/src
git clone https://github.com/introlab/odas_ros.git

cd ~/catkin_ws
catkin_make
```
To get more information on how to use [odas_ros](https://github.com/introlab/odas_ros/tree/redesign_modular/odas_ros) and [soundtracking](https://github.com/introlab/odas_ros/tree/redesign_modular/sound_tracking), visit the README.md of the corresponding folders. 

## Repository
### odas_msgs
Contains the [message](https://github.com/introlab/odas_ros/tree/redesign_modular_v2/odas_msgs/msg) declaration used by odas_server and sound_tracking. 

### odas_server
Contains everything you need to use [odas](https://github.com/introlab/odas) through a [ROS](https://www.ros.org/) architecture.

### sound_traking
Uses odas_ros's nodes to calculate the [twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html) command for a robot to orient itself toward the loudest speech source.
