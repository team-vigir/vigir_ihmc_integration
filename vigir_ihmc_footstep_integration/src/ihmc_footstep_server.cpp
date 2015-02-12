#include <vigir_ihmc_footstep_integration/ihmc_footstep_server.h>

namespace ihmc_integration {

IHMCFootstepServer::IHMCFootstepServer(const ros::NodeHandle &node, const std::string &server_name)
  : server_(node, server_name,boost::bind(&IHMCFootstepServer::goalCB, this, _1),  false),
    node_(node),
    current_step_index_(0),
    target_step_index_(0){
}

void IHMCFootstepServer::start() {
    server_.start();
    foot_pose_pub_ = node_.advertise<ihmc_msgs::FootstepDataListMessage>("/atlas/inputs/ihmc_msgs/FootstepDataListMessage", 1000, false);
}

void IHMCFootstepServer::goalCB(const vigir_footstep_planning_msgs::ExecuteStepPlanGoalConstPtr &goal_ptr) {
    ROS_INFO_STREAM("Received new goal !!");
    ihmc_msgs::FootstepDataListMessage ihmc_msg;
    if (stepPlanToIHCMMsg(goal_ptr->step_plan, ihmc_msg)) {
        foot_pose_pub_.publish(ihmc_msg);
        server_.setSucceeded();
        ROS_INFO_STREAM("Successfully sent step plan.");
    } else {
        ROS_ERROR_STREAM("Step plan was aborted. Size needs to be at least 1.");
        server_.setAborted();
    }
}

bool IHMCFootstepServer::stepPlanToIHCMMsg(const vigir_footstep_planning_msgs::StepPlan& step_plan, ihmc_msgs::FootstepDataListMessage& ihmc_msg) {
    if (step_plan.steps.size() < 1) { // Need at least 1 step
        return false;
    }
    ihmc_msg.footstepDataList.resize(step_plan.steps.size());
    ihmc_msg.swingTime = 1.5;
    ihmc_msg.transferTime = 1.5;
    ihmc_msg.trajectoryWaypointGenerationMethod = 0; // 0=Default, 1=BY_BOX, 2=STEP_ON_OR_OFF, 3=NO_STEP, 4=LOW_HEIGHT
//    ihmc_msg.trajectoryBoxData --- only needed if generation is 'BY BOX' id=1
    for (unsigned int i = 1; i < step_plan.steps.size(); i++) {
        ihmc_msgs::FootstepDataMessage foot_data;
        stepToIHCMMsg(step_plan.steps[i], foot_data);
        ihmc_msg.footstepDataList[i] = foot_data;
    }
    target_step_index_ = step_plan.steps[step_plan.steps.size()-1].step_index;
    return true;
}

void IHMCFootstepServer::stepToIHCMMsg(const vigir_footstep_planning_msgs::Step& step, ihmc_msgs::FootstepDataMessage& foot_data) {
    foot_data.robotSide = step.foot.foot_index;
    foot_data.location.x = step.foot.pose.position.x;
    foot_data.location.y = step.foot.pose.position.y;
    foot_data.location.z = step.foot.pose.position.z;
    foot_data.orientation = step.foot.pose.orientation;
}

}
