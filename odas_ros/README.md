# odas_ros

## Get started
1. Make sure you have the right [configuration file](https://github.com/introlab/odas_ros/tree/redesign_modular/odas_ros/odas_ros/odas/config) selected for your hardware setup.
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
### launch
This is where the ros launch file is located.

### odas
This folder contains the code from the odas librairy.
  - Hardware configuration of the microphones
  - All the code from the [original librairy](https://github.com/introlab/odas)

### scripts
This folder contains the code that converts the data from odas to ros messages.

### src
This folder contains the code to run odas core node.

