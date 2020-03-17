# odas_ros
Is basically [ODAS](https://github.com/introlab/odas/wiki) with some adaptation (transfering json socket to ros messages) to make it work trought ROS.

## Get started
### Odas
1. Make sure you have the right [configuration file](https://github.com/introlab/odas_ros/tree/redesign_modular/odas_ros/odas_ros/odas/config) selected for your hardware setup. To do this you can change the line in [odas.launch](https://github.com/introlab/odas_ros/blob/redesign_modular/odas_ros/odas_ros/launch/odas.launch) with your own configuration file
```
<param name="config_file" value="$(find odas_ros)/odas/config/securbot_turtle_ros.cfg" />
```
2. Start ROS
```
roslaunch odas_ros odas.launch
```
Or,
```
roslaunch odas_ros odas.launch config_file:="securbot_pioneer_ros.cfg"
```
if you want to launch it with a configuration file different than the default one set up in the previous step.

3. With `rostopic list` you should be able to see this:
```
/rosout
/rosout_agg
/sounds/tracked_sources
```

### Information about the energy of potentials
It may be interesting to use the energy of the potential detected by the microphone array that is already calculated by odas. To use this information, you simply have to use the launch file odas_E.launch, that starts a modified version of server.py to take the energy information from odas to make it available in ROS.

1. Modify the .cfg file you are using in the ssl section and in potential to format the information as a json socket on the 9002 port. As in this example:
```
potential: {

        format = "json";

        interface: {
            type = "socket";
            ip = "127.0.0.1";
            port = 9002;
        };

```
2. Choose your .cfg file in the odas_E.launch and then simply launch the file
```
roslaunch odas_ros odas_E.launch config_file:="securbot_pioneer_ros_E.cfg"
```

### 16SoundUSB
You can use different sound cards with odas_ros but if you are using [16SoundUSB](https://github.com/introlab/16SoundsUSB) here is how you set it up:
1. Connect the microphones to the board and the board to your computer (Jetson, Pi, PC, etc.)
2. Open [Audacity](https://www.audacityteam.org/) and select the 16SoundUSB in the device menu.
3. Select 16 channels and start recording.
4. One by one, gently scratch each microphone and raise the volume on the little potentiometre to the maximum.
5. It should look like this:
![micTest](res/audacity_micTest.png)

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

