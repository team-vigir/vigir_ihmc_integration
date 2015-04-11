//=================================================================================================
// Copyright (c) 2015, Martin Oehler, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef IHMC_FOOTSTEP_SERVER_H
#define IHMC_FOOTSTEP_SERVER_H

/* ROS */
#include <ros/ros.h>

/* ViGIR */
#include <vigir_footstep_planning_msgs/ExecuteStepPlanAction.h>
#include <vigir_foot_pose_transformer/foot_pose_transformer.h>

/* IHMC */
#include <ihmc_msgs/FootstepDataListMessage.h>
#include <ihmc_msgs/FootstepStatusMessage.h>
#include <ihmc_msgs/PauseCommandMessage.h>

/* Actionlib */
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/action_server.h>



namespace ihmc_integration {

typedef actionlib::ActionServer<vigir_footstep_planning_msgs::ExecuteStepPlanAction> FootstepServer;

class IHMCFootstepServer {
public:
    IHMCFootstepServer(const ros::NodeHandle& node, const std::string& execute_topic, const std::string& footstep_planner_ns);

    bool loadConfig(const ros::NodeHandle& config_node);
    void start();

private:
    bool goalIsActive();
    void goalCB(FootstepServer::GoalHandle goal_handle);
    void preemptCB(FootstepServer::GoalHandle goal_handle);

    void sendStepPlan(const vigir_footstep_planning_msgs::StepPlan& step_plan);
    void sendFeedback();
    void setSucceeded(std::string msg = "");
    void setAborted(std::string msg = "");
    void preemptWithoutStop(std::string msg = "");
    void setPreempted(std::string msg = "");

    void printStepPlan(unsigned int from, unsigned int to);

    bool stepListToIHMCMsg(ihmc_msgs::FootstepDataListMessage& ihmc_msg);
    void stepToIHMCMsg(const vigir_footstep_planning_msgs::Step& step, ihmc_msgs::FootstepDataMessage& foot_data);
    void sendStopMsg();

    void statusCB(const ihmc_msgs::FootstepStatusMessageConstPtr& status_ptr);

    FootstepServer server_;
    FootstepServer::GoalHandle current_goal_;
    std::vector<vigir_footstep_planning_msgs::Step> step_list_;

    // local instance of foot pose transformer
    vigir_footstep_planning::FootPoseTransformer::Ptr foot_pose_transformer_;

    ros::NodeHandle node_;

    ros::Publisher foot_pose_pub_;
    ros::Publisher stop_pub_;
    ros::Subscriber status_sub_;

    unsigned int current_step_index_;
    unsigned int target_step_index_;

    // Config
    double swing_time_;
    double transfer_time_;
    int traj_waypoint_gen_method_;
    double timeout_factor_;
    std::string ihmc_pub_topic_;
    std::string ihmc_stop_topic_;
    std::string ihmc_status_topic_;
};

}

/*
 * TODO:
 * 0. Add timeout if controller doesn't respond
 * 2. Stitch two plans
 */



#endif
