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

#ifndef IHMC_ARM_ACTION_SERVER_H
#define IHMC_ARM_ACTION_SERVER_H

/* ROS */
#include <ros/ros.h>

/* ROS standard trajectory msgs */
#include <control_msgs/FollowJointTrajectoryAction.h>

/* IHMC */
#include <ihmc_msgs/FootstepDataListMessage.h>
#include <ihmc_msgs/FootstepStatusMessage.h>
#include <ihmc_msgs/PauseCommandMessage.h>

/* Actionlib */
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/action_server.h>

namespace ihmc_integration {

typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> FollowJointTrajectoryActionServer;

class IhmcArmActionServer {
public:
  IhmcArmActionServer(const ros::NodeHandle& node);

  bool loadConfig(const ros::NodeHandle& config_node);
  void start();

private:
  void reorderAndSend(const trajectory_msgs::JointTrajectory& msg);

  void commandCb(const trajectory_msgs::JointTrajectoryConstPtr msg);

  bool goalIsActive();
  void goalCB(FollowJointTrajectoryActionServer::GoalHandle goal_handle);
  void preemptCB(FollowJointTrajectoryActionServer::GoalHandle goal_handle);

  void sendFeedback();
  void setSucceeded(std::string msg = "");
  void setAborted(std::string msg = "");
  void preemptWithoutStop(std::string msg = "");
  void setPreempted(std::string msg = "");

  void sendStopMsg();

  void statusCB(const ihmc_msgs::FootstepStatusMessageConstPtr& status_ptr);

  FollowJointTrajectoryActionServer::GoalHandle current_goal_;

  ros::NodeHandle node_;

  boost::shared_ptr<FollowJointTrajectoryActionServer> server_;
  ros::Subscriber command_sub_;
  ros::Publisher ihmc_command_publisher_;

  std::vector<std::string> joint_names_;

};

}

#endif
