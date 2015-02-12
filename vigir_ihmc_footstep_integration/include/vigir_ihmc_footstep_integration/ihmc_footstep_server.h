#ifndef IHMC_FOOTSTEP_SERVER_H
#define IHMC_FOOTSTEP_SERVER_H

#include <ros/ros.h>
#include <vigir_footstep_planning_msgs/ExecuteStepPlanAction.h>
#include <ihmc_msgs/FootstepDataListMessage.h>
#include <actionlib/server/simple_action_server.h>

namespace ihmc_integration {

class IHMCFootstepServer {
public:
    IHMCFootstepServer(const ros::NodeHandle& node, const std::string& server_name);
    void start();
private:
    actionlib::SimpleActionServer<vigir_footstep_planning_msgs::ExecuteStepPlanAction> server_;
    void goalCB(const vigir_footstep_planning_msgs::ExecuteStepPlanGoalConstPtr& goal_ptr);
    bool stepPlanToIHCMMsg(const vigir_footstep_planning_msgs::StepPlan& step_plan, ihmc_msgs::FootstepDataListMessage& ihmc_msg);
    void stepToIHCMMsg(const vigir_footstep_planning_msgs::Step& step, ihmc_msgs::FootstepDataMessage& foot_data);

    ros::NodeHandle node_;
    ros::Publisher foot_pose_pub_;

    unsigned int current_step_index_;
    unsigned int target_step_index_;
};

}



#endif
