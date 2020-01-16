# ODAS -> ROS
This project is being developped by the [IntRoLab](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/index.php/Main_Page) team.

## Installation
First, you need to clone the repository and prepare it for execution.
```
cd ~/catkin_ws/src
git clone https://github.com/introlab/odas_ros.git

cd ~/catkin_ws
catkin_make
```
To get more information on how to use [odas_ros](https://github.com/introlab/odas_ros/tree/redesign_modular/odas_ros) and [soundtracking](https://github.com/introlab/odas_ros/tree/redesign_modular/sound_tracking), more information is given in the README.md of the corresponding folders. 

## Repository
### odas_ros
This folder contains everything you need to use [odas](https://github.com/introlab/odas) through a [ROS](https://www.ros.org/) architecture.

### sound_traking
This folder uses odas_ros's nodes to calculate the [twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html) command for a robot to orient itself toward the loudest speech source.
