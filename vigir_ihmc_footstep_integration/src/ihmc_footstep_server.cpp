#include <vigir_ihmc_footstep_integration/ihmc_footstep_server.h>

namespace ihmc_integration {

IHMCFootstepServer::IHMCFootstepServer(const ros::NodeHandle &node, const std::string &server_name)
  : server_(node, server_name,boost::bind(&IHMCFootstepServer::goalCB, this, _1),  false),
    node_(node),
    current_step_index_(0),
    target_step_index_(0){
}

bool IHMCFootstepServer::loadConfig(const ros::NodeHandle& config_node) {
    bool success = true;
    success = success && config_node.getParam("swing_time", swing_time_);
    success = success && config_node.getParam("transfer_time", transfer_time_);
    success = success && config_node.getParam("traj_waypoint_gen_method", traj_waypoint_gen_method_);
    success = success && config_node.getParam("timeout_factor", timeout_factor_);
    success = success && config_node.getParam("ihmc_pub_topic", ihmc_pub_topic_);
    success = success && config_node.getParam("ihmc_status_topic", ihmc_status_topic_);
    return success;
}

void IHMCFootstepServer::start() {
    server_.start();
    foot_pose_pub_ = node_.advertise<ihmc_msgs::FootstepDataListMessage>("/atlas/inputs/ihmc_msgs/FootstepDataListMessage", 1000, false);
    status_sub_ = node_.subscribe("/atlas/outputs/ihmc_msgs/FootstepStatusMessage", 1000, &IHMCFootstepServer::statusCB, this);
}

void IHMCFootstepServer::goalCB(const vigir_footstep_planning_msgs::ExecuteStepPlanGoalConstPtr &goal_ptr) {
    ROS_INFO_STREAM("Received new goal !!");
    ihmc_msgs::FootstepDataListMessage ihmc_msg;
    if (stepPlanToIHCMMsg(goal_ptr->step_plan, ihmc_msg)) {
        foot_pose_pub_.publish(ihmc_msg);
        ROS_INFO_STREAM("Successfully sent step plan.");

        bool success = false;
        ros::Rate loop_rate(100);
        while (!success)  {
            if (current_step_index_ == target_step_index_) {
                success = true;
                ROS_INFO_STREAM("Finished current step plan.");
            }
            vigir_footstep_planning_msgs::ExecuteStepPlanFeedback feedback;
            feedback.current_step_index = current_step_index_;
            feedback.stepping_status = vigir_footstep_planning_msgs::FootstepExecutionStatus::EXECUTING_STEP_PLAN;
            server_.publishFeedback(feedback);
            ros::spinOnce();
            loop_rate.sleep();
        }

        vigir_footstep_planning_msgs::ExecuteStepPlanResult result;
        result.status.status = vigir_footstep_planning_msgs::FootstepExecutionStatus::REACHED_GOAL;
        result.status.header.stamp = ros::Time::now();
        server_.setSucceeded();
    } else {
        ROS_ERROR_STREAM("Step plan was aborted. Size needs to be at least 1.");
        vigir_footstep_planning_msgs::ExecuteStepPlanResult result;
        result.status.status = vigir_footstep_planning_msgs::FootstepExecutionStatus::ERR_EMPTY_PLAN;
        result.status.header.stamp = ros::Time::now();
        server_.setAborted(result);
    }
}

bool IHMCFootstepServer::stepPlanToIHCMMsg(const vigir_footstep_planning_msgs::StepPlan& step_plan, ihmc_msgs::FootstepDataListMessage& ihmc_msg) {
    if (step_plan.steps.size() < 1) { // Need at least 1 step
        return false;
    }
    ihmc_msg.footstepDataList.resize(step_plan.steps.size()-1);
    ihmc_msg.swingTime = 1.5;
    ihmc_msg.transferTime = 1.5;
    ihmc_msg.trajectoryWaypointGenerationMethod = 0; // 0=Default, 1=BY_BOX, 2=STEP_ON_OR_OFF, 3=NO_STEP, 4=LOW_HEIGHT
//    ihmc_msg.trajectoryBoxData --- only needed if generation is 'BY BOX' id=1
    for (unsigned int i = 1; i < step_plan.steps.size(); i++) {
        ihmc_msgs::FootstepDataMessage foot_data;
        stepToIHCMMsg(step_plan.steps[i], foot_data);
        ihmc_msg.footstepDataList[i-1] = foot_data;
    }
    current_step_index_ = 0;
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

void IHMCFootstepServer::statusCB(const ihmc_msgs::FootstepStatusMessageConstPtr& status_ptr) {
    if (status_ptr->status == 1) {
        current_step_index_++;
        ROS_INFO_STREAM("Current step index: " << current_step_index_);
    }
}

}
