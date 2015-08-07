/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Stefan Kohlbrecher,
 *  TU Darmstadt ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of TU Darmstadt, Team ViGIR, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <vigir_ihmc_control_integration/ihmc_arm_action_server.h>
#include <vigir_ihmc_control_integration/permutation_helper.h>

#include <math.h>

namespace ihmc_integration {

IhmcArmActionServer::IhmcArmActionServer(const ros::NodeHandle& nh)
  : node_(nh)
{
  if (!node_.getParam("joints", joint_names_)){
    ROS_WARN("No joint names found on param server, assuming this controller is just placeholder.");
  }

  ihmc_command_publisher_ = node_.advertise<trajectory_msgs::JointTrajectory>("/ihmc_ros/atlas/control/arm_joint_trajectory2",
                                                                              10,
                                                                              false);

  server_.reset(new FollowJointTrajectoryActionServer(node_,
                                                      "follow_joint_trajectory",
                                                      boost::bind(&IhmcArmActionServer::goalCB, this, _1),
                                                      boost::bind(&IhmcArmActionServer::preemptCB, this, _1),
                                                      false));

  command_sub_ = node_.subscribe("command", 5, &IhmcArmActionServer::commandCb, this );
}

bool IhmcArmActionServer::loadConfig(const ros::NodeHandle& nh) {

}

void IhmcArmActionServer::start() {


  server_->start();
}

void IhmcArmActionServer::reorderAndSend(const trajectory_msgs::JointTrajectory& msg)
{
  std::vector<unsigned int> permutation_vec = permutation::permutation(msg.joint_names, joint_names_);

  trajectory_msgs::JointTrajectory reordered_traj;
  permutation::reorderTrajectoryEntries(msg, permutation_vec, reordered_traj);
  reordered_traj.joint_names = joint_names_;

  ihmc_command_publisher_.publish(reordered_traj);
}

void IhmcArmActionServer::commandCb(const trajectory_msgs::JointTrajectoryConstPtr msg)
{
  reorderAndSend(*msg);
}

bool IhmcArmActionServer::goalIsActive() {
  return current_goal_.isValid() && current_goal_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE;
}

void IhmcArmActionServer::goalCB(FollowJointTrajectoryActionServer::GoalHandle goal_handle) {
  ROS_INFO_STREAM("[IhmcArmActionServer] New goal received.");

  if (!goalIsActive()) {
    goal_handle.setAccepted();
    current_goal_ = goal_handle;
    reorderAndSend(goal_handle.getGoal()->trajectory);

  } else {
    setPreempted("Preempted prior trajectory");
    goal_handle.setAccepted();
    current_goal_ = goal_handle;
    reorderAndSend(goal_handle.getGoal()->trajectory);
  }
}

void IhmcArmActionServer::preemptCB(FollowJointTrajectoryActionServer::GoalHandle goal_handle) {
  setPreempted("Stopped by user.");
}

void IhmcArmActionServer::statusCB(const ihmc_msgs::FootstepStatusMessageConstPtr& status_ptr) {

}

void IhmcArmActionServer::sendFeedback() {
}

void IhmcArmActionServer::setSucceeded(std::string msg) {
  ROS_INFO_STREAM("[IhmcArmActionServer] Trajectory execution succeeded.");
  current_goal_.setSucceeded();
}

void IhmcArmActionServer::setAborted(std::string msg) {
  ROS_ERROR_STREAM("[IhmcArmActionServer] Trajectory was aborted.");
  current_goal_.setAborted();
}

void IhmcArmActionServer::setPreempted(std::string msg) {
  ROS_INFO_STREAM("[IhmcArmActionServer] Trajectory was preempted.");
  current_goal_.setCanceled();
}

}
