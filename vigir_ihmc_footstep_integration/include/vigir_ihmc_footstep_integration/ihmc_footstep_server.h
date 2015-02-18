#ifndef IHMC_FOOTSTEP_SERVER_H
#define IHMC_FOOTSTEP_SERVER_H

/* ROS */
#include <ros/ros.h>

/* ViGIR */
#include <vigir_footstep_planning_msgs/ExecuteStepPlanAction.h>

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
    IHMCFootstepServer(const ros::NodeHandle& node, const std::string& server_name);
    std::string getNamespace();
    bool loadConfig(const ros::NodeHandle& config_node);
    void start();
private:
    bool goalIsActive();
    void executeCB(const vigir_footstep_planning_msgs::ExecuteStepPlanGoalConstPtr& goal_ptr);
    void goalCB(FootstepServer::GoalHandle goal_handle);
    void preemptCB(FootstepServer::GoalHandle goal_handle);

    void sendStepPlan(const vigir_footstep_planning_msgs::StepPlan& step_plan);
    void sendFeedback();
    void setSucceeded(std::string msg = "");
    void setAborted(std::string msg = "");
    void preemptWithoutStop(std::string msg = "");
    void setPreempted(std::string msg = "");

    bool stepListToIHMCMsg(ihmc_msgs::FootstepDataListMessage& ihmc_msg);
    void stepToIHMCMsg(const vigir_footstep_planning_msgs::Step& step, ihmc_msgs::FootstepDataMessage& foot_data);
    void sendStopMsg();
    void sendStepList();

    void statusCB(const ihmc_msgs::FootstepStatusMessageConstPtr& status_ptr);

    FootstepServer server_;
    FootstepServer::GoalHandle current_goal_;
    std::vector<vigir_footstep_planning_msgs::Step> step_list_;

    ros::NodeHandle node_;
    std::string name_;

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
