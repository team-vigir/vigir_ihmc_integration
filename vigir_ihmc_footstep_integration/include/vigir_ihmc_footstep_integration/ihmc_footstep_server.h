#ifndef IHMC_FOOTSTEP_SERVER_H
#define IHMC_FOOTSTEP_SERVER_H

#include <ros/ros.h>
#include <vigir_footstep_planning_msgs/ExecuteStepPlanAction.h>
#include <ihmc_msgs/FootstepDataListMessage.h>
#include <ihmc_msgs/FootstepStatusMessage.h>
#include <actionlib/server/simple_action_server.h>

namespace ihmc_integration {

class IHMCFootstepServer {
public:
    IHMCFootstepServer(const ros::NodeHandle& node, const std::string& server_name);
    bool loadConfig(const ros::NodeHandle& config_node);
    void start();
private:
    actionlib::SimpleActionServer<vigir_footstep_planning_msgs::ExecuteStepPlanAction> server_;
    void goalCB(const vigir_footstep_planning_msgs::ExecuteStepPlanGoalConstPtr& goal_ptr);
    bool stepPlanToIHCMMsg(const vigir_footstep_planning_msgs::StepPlan& step_plan, ihmc_msgs::FootstepDataListMessage& ihmc_msg);
    void stepToIHCMMsg(const vigir_footstep_planning_msgs::Step& step, ihmc_msgs::FootstepDataMessage& foot_data);

    void statusCB(const ihmc_msgs::FootstepStatusMessageConstPtr& status_ptr);

    ros::NodeHandle node_;
    ros::Publisher foot_pose_pub_;
    ros::Subscriber status_sub_;

    unsigned int current_step_index_;
    unsigned int target_step_index_;

    // Config
    double swing_time_;
    double transfer_time_;
    int traj_waypoint_gen_method_;
    double timeout_factor_;
    std::string ihmc_pub_topic_;
    std::string ihmc_status_topic_;
};

}

/*
 * TODO:
 * 0. Add timeout if controller doesn't respond & remove blocking while statement
 * 1. Stop current step plan (by receiving an empty step plan)
 * 2. Stitch two plans
 * 3. Add constants IHMC message
 */



#endif
