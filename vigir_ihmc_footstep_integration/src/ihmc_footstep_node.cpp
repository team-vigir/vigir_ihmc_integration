#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "ihmc_footstep_node");
  ros::spin();

  return 0;
}
