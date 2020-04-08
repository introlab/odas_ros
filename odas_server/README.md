# odas_server
Is basically [ODAS](https://github.com/introlab/odas/wiki) with some adaptation (transfering json socket to ros messages) to make it work trought ROS.

## Get started
### Odas
1. Make sure you have the right [configuration file](https://github.com/introlab/odas_ros/tree/redesign_modular/odas_ros/odas_ros/odas/config) selected for your hardware setup. To do this you can change the line in [odas.launch](https://github.com/introlab/odas_ros/blob/redesign_modular/odas_ros/odas_ros/launch/odas.launch) with your own configuration file
```
<param name="config_file" value="$(find odas_ros)/odas/config/securbot_turtle_ros.cfg" />
```
2. Start ROS (see launch file)
```
roslaunch odas_server odas_xxxxx.launch
```
Or,
```
roslaunch odas_server odas_xxxxx.launch config_file:="name_of_the_config_file.cfg"
```
if you want to launch it with a configuration file different than the default one set up in the previous step.

3. With `rostopic list` you should be able to see something like this:
```
/rosout
/rosout_agg
/sounds/tracked_sources
```
Depending on the launch file you chose to run.

### Information about SSL and SST together (or SSS and SST together)
It may be interesting to use the energy of the potential detected by the microphone array that is already calculated by odas. To use this information, you simply have to use the launch file odas_It_sync.launch (or the odas_sst_sss.launch), that starts a modified version of server.py to take the energy information from odas to make it available in ROS (or the separated audio from odas in the sss/sst case). 

1. Modify the .cfg file you are using in the ssl (or in the sss) section and in potential to format the information as a json socket on the 9002 port (or the 10000). As in this example:
```
potential: {

        format = "json";

        interface: {
            type = "socket";
            ip = "127.0.0.1";
            port = 9002;
        };

```
2. Choose your .cfg file in the odas_It_sync.launch (or in the odas_sst_sss.launch) and then simply launch the file
```
roslaunch odas_server odas_lt_sync.launch config_file:="securbot_pioneer_ros_E.cfg"
```
For single utilisation of Sound Source Localization (SSL), Sound Source Tracking (SST) or Separated Sound Source (SSS) launch files with servers are provided. Specific .cfg file are necessary.

### 16SoundUSB
You can use different sound cards with odas_ros but if you are using [16SoundUSB](https://github.com/introlab/16SoundsUSB) here is how you set it up:
1. Connect the microphones to the board and the board to your computer (Jetson, Pi, PC, etc.)
2. Open [Audacity](https://www.audacityteam.org/) and select the 16SoundUSB in the device menu.
3. Select 16 channels and start recording.
4. One by one, gently scratch each microphone and raise the volume on the little potentiometre to the maximum.
5. It should look like this:
![micTest](res/audacity_micTest.png)


## Files descriptions

### launch
This is where the ROS launch files are located to run the different servers with the odas_core:
 - odas_core.launch:    Odas core node without server
 - odas_It_sync.launch: Odas core with tracked_sources and potential_sources
 - odas_ssl.launch:     Odas core with potential_sources
 - odas_sss.launch:     Odas core with separated_sources
 - odas_sss_sst.launch: Odas core with tracked_sources and separated_sources
 - odas_sst.launch:     Odas core with tracked_sources
 - odas_st.launch:      Odas core with tracked_sources oldversion

### odas
This folder contains the programs of the odas library.
  - Hardware configuration of the microphones
  - All the code from the [original librairy](https://github.com/introlab/odas)

### scripts
This folder contains the programs that converts the data from odas to ROS messages. There is one server_xxxxx.py for each launch file.

### src
This folder contains the programs to run odas_core_node.

