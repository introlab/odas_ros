# sound_tracking

sound_tracking is an algorithm that uses ODAS and the tracked sources to return the direction of a source. An example is a robot with a microphone array that uses odas_ros and sound_tracking to locate different sources and orient itself in the direction of one depending on the sound_tracking mode. Currently it gets the speech with most confidence and orients to it.
It is made to be dependent of [odas_ros](https://github.com/introlab/odas_ros/tree/redesign_modular/odas_ros).

## Get Started
To use sound_tracking with odas_ros, you need to have a terminal running [odas.launch](https://github.com/introlab/odas_ros/blob/redesign_modular/odas_ros/odas_ros/launch/odas.launch) or a node publishing topics under `/sounds/tracked_sources`. Running the launch file will allow you to publish a `/cmd_vel` topic that the robot can use to orient itself toward the speech.
```
roslaunch sound_tracking sound_tracking.launch
```

## Repository
### config
The config folder contains base_control.yaml and spatial_filter.yaml. They are used to configure the way the base will turn, with the polarity, max speed, hysteresis before moving, tolerance for reaching target ([base_control.yaml](https://github.com/introlab/odas_ros/blob/redesign_modular/sound_tracking/config/base_control.yaml)), the type of filter and the angles of the pitch ([spatial_filter.yaml](https://github.com/introlab/odas_ros/blob/redesign_modular/sound_tracking/config/spatial_filter.yaml))

### include
The include folder contains the different .h files used by the sound_tracking_node. 

### launch
The launch folder contains the launch files to use the sound_tracking_node. The current [sound_tracking.launch](https://github.com/introlab/odas_ros/blob/redesign_modular/sound_tracking/launch/sound_tracking.launch) only launches the sound_tracking_node but uncommenting line 11 will allow to launch directly the odas.launch file.

### soundcard
This folder contains a .sh file to setup a sound card.

### src
The src folder contains the sound_tracking_node used to locate sources and orient the robot.
