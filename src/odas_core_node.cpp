#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>

//From odas demo client
extern "C" 
{
    #include <odas.h>
    #include <parameters.h>
    #include <configs.h>
    #include <objects.h>
    #include <threads.h>
    #include <profiler.h>
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odas_core_node");
    ros::NodeHandle privateNodeHandle("~");

    std::string configFile;
    privateNodeHandle.getParam("configuration_path", configFile);

    ROS_INFO("Using configuration file = %s", configFile.c_str());

    // +------------------------------------------------------+
    // | Multiple threads                                     |
    // +------------------------------------------------------+  



    // +----------------------------------------------------------+
    // | Variables                                                |
    // +----------------------------------------------------------+  


    // +------------------------------------------------------+
    // | Objects                                              |
    // +------------------------------------------------------+   
    objects * objs = NULL;
    aobjects * aobjs = NULL;

    // +------------------------------------------------------+
    // | Configurations                                       |
    // +------------------------------------------------------+        
    configs * cfgs = NULL;   


    // +--------------------------------------------------+
    // | Configure                                        |
    // +--------------------------------------------------+ 
    ROS_INFO("| + Initializing configurations...... ");
    cfgs = configs_construct(configFile.c_str());



    // +--------------------------------------------------+
    // | Construct                                        |
    // +--------------------------------------------------+ 
    ROS_INFO("| + Initializing objects............. ");
    aobjs = aobjects_construct(cfgs);    



    // +--------------------------------------------------+
    // | Launch threads                                   |
    // +--------------------------------------------------+  
    ROS_INFO("| + Launch threads................... ");
    threads_multiple_start(aobjs);


    ROS_INFO("| + ROS SPINNING................... ");
            
    //Start ros loop
    ros::spin();

    // +--------------------------------------------------+
    // | Wait                                             |
    // +--------------------------------------------------+
    ROS_INFO("| + Threads join.................. "); 
    threads_multiple_join(aobjs);



    // +--------------------------------------------------+
    // | Free memory                                      |
    // +--------------------------------------------------+ 
    ROS_INFO("| + Free memory...................... "); 


    aobjects_destroy(aobjs);
    configs_destroy(cfgs);
    ROS_INFO("Done!");


    return 0;
}
