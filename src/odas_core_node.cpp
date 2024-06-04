#include <rclcpp/rclcpp.hpp>

#include <string>

// From odas demo client
extern "C"
{
#include <odas.h>
#include <parameters.h>
#include <configs.h>
#include <objects.h>
#include <threads.h>
#include <profiler.h>
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("odas_core_node");

    std::string configFile = node->declare_parameter("configuration_path", "");

    RCLCPP_INFO(node->get_logger(), "Using configuration file = %s", configFile.c_str());

    // +------------------------------------------------------+
    // | Multiple threads                                     |
    // +------------------------------------------------------+


    // +----------------------------------------------------------+
    // | Variables                                                |
    // +----------------------------------------------------------+


    // +------------------------------------------------------+
    // | Objects                                              |
    // +------------------------------------------------------+
    aobjects* aobjs = NULL;

    // +------------------------------------------------------+
    // | Configurations                                       |
    // +------------------------------------------------------+
    configs* cfgs = NULL;


    // +--------------------------------------------------+
    // | Configure                                        |
    // +--------------------------------------------------+
    RCLCPP_INFO(node->get_logger(), "| + Initializing configurations...... ");
    cfgs = configs_construct(configFile.c_str());


    // +--------------------------------------------------+
    // | Construct                                        |
    // +--------------------------------------------------+
    RCLCPP_INFO(node->get_logger(), "| + Initializing objects............. ");
    aobjs = aobjects_construct(cfgs);


    // +--------------------------------------------------+
    // | Launch threads                                   |
    // +--------------------------------------------------+
    RCLCPP_INFO(node->get_logger(), "| + Launch threads................... ");
    threads_multiple_start(aobjs);


    RCLCPP_INFO(node->get_logger(), "| + ROS SPINNING................... ");

    // Start ros loop
    rclcpp::spin(node);

    // +--------------------------------------------------+
    // | Wait                                             |
    // +--------------------------------------------------+
    RCLCPP_INFO(node->get_logger(), "| + Threads join.................. ");
    threads_multiple_join(aobjs);


    // +--------------------------------------------------+
    // | Free memory                                      |
    // +--------------------------------------------------+
    RCLCPP_INFO(node->get_logger(), "| + Free memory...................... ");


    aobjects_destroy(aobjs);
    configs_destroy(cfgs);
    RCLCPP_INFO(node->get_logger(), "Done!");

    rclcpp::shutdown();

    return 0;
}
