#include <ros/ros.h>
#include <vigir_ihmc_footstep_integration/ihmc_footstep_server.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ihmc_footstep_node");

    // Load config
    ros::NodeHandle config_node("ihmc_footstep_config");
    std::string name = "ihmc_footstep_server";
    if (!config_node.getParam("name", name)) {
        ROS_WARN_STREAM("Couldn't find param 'name' in namespace: " << config_node.getNamespace() << ". Using default: " << name << ".");
    }
    std::string name_space;
    if (!config_node.getParam("namespace", name_space)) {
        ROS_WARN_STREAM("Couldn't find param 'namespace' in namespace: " << config_node.getNamespace() << ". Using default: " << name_space << ".");
    }
    ros::NodeHandle node(name_space);
    ihmc_integration::IHMCFootstepServer server(node, name);
    if (!server.loadConfig(config_node)) {
        ROS_WARN_STREAM("Loading config from " << config_node.getNamespace() << " failed. Using default.");
    }
    server.start();
    ROS_INFO_STREAM(name << " started. Listening for actions on: " << server.getNamespace() << ".");
    ros::spin();

    return 0;
}
