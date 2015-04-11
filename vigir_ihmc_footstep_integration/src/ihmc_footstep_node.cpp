#include <ros/ros.h>
#include <vigir_ihmc_footstep_integration/ihmc_footstep_server.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ihmc_footstep_node");

    // Load config
    ros::NodeHandle nh;

    std::string execute_topic = "ihmc_footstep_server";
    if (!nh.getParam("ihmc_footstep_config/execute_topic", execute_topic))
        ROS_WARN_STREAM("Couldn't find param 'execute_topic'. Using default: " << execute_topic << ".");

    std::string footstep_planner_ns = "/vigir/footstep_planning";
    if (!nh.getParam("ihmc_footstep_config/footstep_planner_ns", footstep_planner_ns))
        ROS_WARN_STREAM("Couldn't find param 'footstep_planner_ns'. Using default: " << footstep_planner_ns << ".");

    ihmc_integration::IHMCFootstepServer server(nh, execute_topic, footstep_planner_ns);

    server.start();
    ros::spin();

    return 0;
}
