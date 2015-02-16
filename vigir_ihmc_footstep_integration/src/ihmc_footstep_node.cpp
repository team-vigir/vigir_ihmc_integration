#include <ros/ros.h>
#include <vigir_ihmc_footstep_integration/ihmc_footstep_server.h>

int main(int argc, char **argv) {
  std::string name("ihmc_footstep");
  ros::init(argc, argv, name +"_node");
  ros::NodeHandle node;
  ihmc_integration::IHMCFootstepServer server(node, name+"_server");
  ros::NodeHandle config_node(server.getNamespace() + "/ihmc_footstep_config");
  if (!server.loadConfig(config_node)) {
      ROS_WARN_STREAM("Loading config from " << config_node.getNamespace() << " failed. Using default.");
  }
  server.start();
  ROS_INFO_STREAM(name << "_server started. Listening for actions.");
  ros::spin();

  return 0;
}
