# odas_ros

## Get started
1. Make sure you have the right [configuration file](https://github.com/introlab/odas_ros/tree/redesign_modular/odas_ros/odas_ros/odas/config) selected for your hardware setup. To do this you can change the line in [odas.launch](https://github.com/introlab/odas_ros/blob/redesign_modular/odas_ros/odas_ros/launch/odas.launch) with your own configuration file
```
<param name="config_file" value="$(find odas_ros)/odas/config/securbot_turtle_ros.cfg" />
```
2. Start ROS
```
roslaunch odas_ros odas.launch
```
3. With `rostopic list` you should be able to see this:
```
/rosout
/rosout_agg
/sounds/tracked_sources
```

## Repository
There are two folders, odas_ros and odas_msgs, which are separated for modularity purposes.

## odas_msgs
odsa_msgs contains the files required to use odas's custom ROS messages that are used by odas_ros and sound_tracking.
Theses messages contains the position of a source (x,y,z), if the source is talking, and the probability of activity.

## odas_ros
odas_ros is the package containing the scripts and nodes to use odas_core_node and server.py to run the odas library.

### launch
This is where the ROS launch files are located.

### odas
This folder contains the programs of the odas library.
  - Hardware configuration of the microphones
  - All the code from the [original librairy](https://github.com/introlab/odas)

### scripts
This folder contains the programs that converts the data from odas to ROS messages.

### src
This folder contains the programs to run odas_core_node.

