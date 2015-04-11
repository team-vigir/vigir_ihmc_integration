#include <vigir_ihmc_footstep_integration/ihmc_footstep_server.h>

#include <math.h>

namespace ihmc_integration {

IHMCFootstepServer::IHMCFootstepServer(const ros::NodeHandle& nh, const std::string& execute_topic, const std::string& footstep_planner_ns)
  : server_(nh, execute_topic, boost::bind(&IHMCFootstepServer::goalCB, this, _1), boost::bind(&IHMCFootstepServer::preemptCB, this, _1), false)
  , node_(nh)
  , current_step_index_(0)
  , target_step_index_(0)
  , swing_time_(1.5)
  , transfer_time_(1.5)
  , traj_waypoint_gen_method_(0)
  , timeout_factor_(1.2)
  , ihmc_pub_topic_("/atlas/inputs/ihmc_msgs/FootstepDataListMessage")
  , ihmc_status_topic_("/atlas/outputs/ihmc_msgs/FootstepStatusMessage")
  , ihmc_stop_topic_("/atlas/inputs/ihmc_msgs/PauseCommandMessage")
{
  if (!loadConfig(nh))
    ROS_ERROR("[IHMCFootstepServer] Couldn't load config!");

  ros::NodeHandle footstep_planner_nh(footstep_planner_ns);
  foot_pose_transformer_.reset(new vigir_footstep_planning::FootPoseTransformer(footstep_planner_nh));
}

bool IHMCFootstepServer::loadConfig(const ros::NodeHandle& nh) {
    bool success = true;
    success = success && nh.getParam("swing_time", swing_time_);
    success = success && nh.getParam("transfer_time", transfer_time_);
    success = success && nh.getParam("traj_waypoint_gen_method", traj_waypoint_gen_method_);
    success = success && nh.getParam("timeout_factor", timeout_factor_);
    success = success && nh.getParam("ihmc_pub_topic", ihmc_pub_topic_);
    success = success && nh.getParam("ihmc_status_topic", ihmc_status_topic_);
    success = success && nh.getParam("ihmc_stop_topic", ihmc_stop_topic_);
    return success;
}

void IHMCFootstepServer::start() {
    // publish topics
    foot_pose_pub_ = node_.advertise<ihmc_msgs::FootstepDataListMessage>(ihmc_pub_topic_, 1000, false);
    stop_pub_ = node_.advertise<ihmc_msgs::PauseCommandMessage>(ihmc_stop_topic_, 1000, false);
    status_sub_ = node_.subscribe(ihmc_status_topic_, 1000, &IHMCFootstepServer::statusCB, this);

    server_.start();

    ROS_INFO_STREAM("[IHMCFootstepServer] Server started.");
}

bool IHMCFootstepServer::goalIsActive() {
 return current_goal_.isValid() && current_goal_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE;
}

void IHMCFootstepServer::goalCB(FootstepServer::GoalHandle goal_handle) {
    ROS_INFO_STREAM("[IHMCFootstepServer] New goal received.");
    if (!goalIsActive()) {
        goal_handle.setAccepted();
        current_goal_ = goal_handle;
        step_list_.clear();
        current_step_index_ = 0;
        sendStepPlan(current_goal_.getGoal()->step_plan);
    } else {
        //preemptWithoutStop("Append new goal");
        setPreempted("Merging with new goal.");
        int32_t ns = swing_time_ * 1000000000;
        ros::Duration d = ros::Duration(0, ns);
        d.sleep();
        goal_handle.setAccepted();
        current_goal_ = goal_handle;
        sendStepPlan(current_goal_.getGoal()->step_plan);
    }
}

void IHMCFootstepServer::preemptCB(FootstepServer::GoalHandle goal_handle) {
    setPreempted("Stopped by user.");
}

void IHMCFootstepServer::sendStepPlan(const vigir_footstep_planning_msgs::StepPlan &step_plan) {
    // handle stitching
    if (step_plan.steps[step_plan.steps.size()-1].step_index+1 > step_list_.size()) {
        step_list_.resize(step_plan.steps[step_plan.steps.size()-1].step_index+1);
        target_step_index_ = step_list_.size() -1;
    }

    // copy new steps
    for (std::vector<vigir_footstep_planning_msgs::Step>::const_iterator itr = step_plan.steps.begin(); itr != step_plan.steps.end(); itr++) {
      vigir_footstep_planning_msgs::Step step = *itr;
      foot_pose_transformer_->transformToPlannerFrame(step); // transform step to sole frame
      step_list_[step.step_index] = step;
    }

    // send plan to IHMC controller
    ihmc_msgs::FootstepDataListMessage ihmc_msg;
    if (stepListToIHMCMsg(ihmc_msg)) {
        // Publish plan
        foot_pose_pub_.publish(ihmc_msg);
    } else {
        setAborted("Step plan empty/invalid.");
    }
}

bool IHMCFootstepServer::stepListToIHMCMsg(ihmc_msgs::FootstepDataListMessage& ihmc_msg) {
    if (step_list_.empty()) { // Need at least 1 step
        return false;
    }

    ihmc_msg.footstep_data_list.resize(step_list_.size()-1-current_step_index_);
    ihmc_msg.swing_time = swing_time_;
    ihmc_msg.transfer_time = transfer_time_;
    //ihmc_msg.trajectoryWaypointGenerationMethod = traj_waypoint_gen_method_; // 0=Default, 1=BY_BOX, 2=STEP_ON_OR_OFF, 3=NO_STEP, 4=LOW_HEIGHT
//    ihmc_msg.trajectoryBoxData --- only needed if generation is 'BY BOX' id=1

    for (unsigned int i = current_step_index_+1; i < step_list_.size(); i++) {
        ihmc_msgs::FootstepDataMessage foot_data;
        stepToIHMCMsg(step_list_[i], foot_data);
        ihmc_msg.footstep_data_list[i-current_step_index_-1] = foot_data;
    }

    ROS_INFO_STREAM("[IHMCFootstepServer] Sent step plan from " << current_step_index_+1 << " to " << target_step_index_ << ".");
    printStepPlan(current_step_index_+1, step_list_.size());
    return true;
}

void IHMCFootstepServer::stepToIHMCMsg(const vigir_footstep_planning_msgs::Step& step, ihmc_msgs::FootstepDataMessage& foot_data) {
    foot_data.robot_side = step.foot.foot_index;
    foot_data.location.x = step.foot.pose.position.x;
    foot_data.location.y = step.foot.pose.position.y;
    foot_data.location.z = step.foot.pose.position.z;
    foot_data.orientation = step.foot.pose.orientation;
}

void IHMCFootstepServer::printStepPlan(unsigned int from, unsigned int to) {
    std::stringstream stream;
    stream << "Positions: " << std::endl;
    for (unsigned int i = from; i < to; i++) {
        stream << i << ":\t" << step_list_[i].foot.pose.position.x << std::endl;
    }
    ROS_INFO_STREAM("[IHMCFootstepServer] Step plan:\n" << stream.str());
}

void IHMCFootstepServer::statusCB(const ihmc_msgs::FootstepStatusMessageConstPtr& status_ptr) {
    if (status_ptr->status == 1) {
        current_step_index_++;
       if (goalIsActive()) { // Check if goal was pre-empted.
            // Check if target number of steps is reached
            if (current_step_index_ == target_step_index_) {
                setSucceeded("Reached goal pose.");
            } else {
                sendFeedback();
            }
       }
    }
}

void IHMCFootstepServer::sendFeedback() {
    ROS_INFO_STREAM("[IHMCFootstepServer] Feedback: Current step index: " << current_step_index_ << "/" << target_step_index_);
    vigir_footstep_planning_msgs::ExecuteStepPlanFeedback feedback;
    /// TODO
    feedback.last_performed_step_index = current_step_index_;
    //feedback.currently_executing_step_index = TODO;
    feedback.first_changeable_step_index = current_step_index_+1;
    feedback.stepping_status = vigir_footstep_planning_msgs::FootstepExecutionStatus::EXECUTING_STEP_PLAN;
    current_goal_.publishFeedback(feedback);
}

void IHMCFootstepServer::setSucceeded(std::string msg) {
    ROS_INFO_STREAM("[IHMCFootstepServer] Step plan succeeded.");
    vigir_footstep_planning_msgs::ExecuteStepPlanResult result;
    result.status.status = vigir_footstep_planning_msgs::FootstepExecutionStatus::REACHED_GOAL;
    result.status.header.stamp = ros::Time::now();
    current_goal_.setSucceeded(result, msg);
}

void IHMCFootstepServer::setAborted(std::string msg) {
    ROS_ERROR_STREAM("[IHMCFootstepServer] Step plan was aborted.");
    vigir_footstep_planning_msgs::ExecuteStepPlanResult result;
    result.status.status = vigir_footstep_planning_msgs::FootstepExecutionStatus::ERR_EMPTY_PLAN;
    result.status.header.stamp = ros::Time::now();
    current_goal_.setAborted(result, msg);
}

void IHMCFootstepServer::preemptWithoutStop(std::string msg) {
    ROS_INFO_STREAM("[IHMCFootstepServer] Preempt without stop.");
    vigir_footstep_planning_msgs::ExecuteStepPlanResult result;
    result.status.status = vigir_footstep_planning_msgs::FootstepExecutionStatus::PREMATURE_HALT;
    result.status.header.stamp = ros::Time::now();
    current_goal_.setCanceled(result, msg);
}

void IHMCFootstepServer::setPreempted(std::string msg) {
    ROS_INFO_STREAM("[IHMCFootstepServer] Step plan was cancelled.");
    sendStopMsg();
    vigir_footstep_planning_msgs::ExecuteStepPlanResult result;
    result.status.status = vigir_footstep_planning_msgs::FootstepExecutionStatus::PREMATURE_HALT;
    result.status.header.stamp = ros::Time::now();
    current_goal_.setCanceled(result, msg);
}

void IHMCFootstepServer::sendStopMsg() {
    ihmc_msgs::PauseCommandMessage pause_msg;
    pause_msg.pause = true;
    stop_pub_.publish(pause_msg);
}

}
