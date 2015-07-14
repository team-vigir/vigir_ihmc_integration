/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Martin Oehler, Alexander Stumpf,
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

#include <ros/ros.h>
#include <vigir_ihmc_footstep_integration/ihmc_footstep_server.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ihmc_footstep_node");

    // Load config
    ros::NodeHandle nh;

    std::string execute_topic = "ihmc_footstep_server";
    if (!nh.getParam("execute_topic", execute_topic))
        ROS_WARN_STREAM("Couldn't find param 'execute_topic'. Using default: " << execute_topic << ".");

    std::string footstep_planner_ns = "/vigir/footstep_planning";
    if (!nh.getParam("footstep_planner_ns", footstep_planner_ns))
        ROS_WARN_STREAM("Couldn't find param 'footstep_planner_ns'. Using default: " << footstep_planner_ns << ".");

    ihmc_integration::IHMCFootstepServer server(nh, execute_topic, footstep_planner_ns);

    server.start();
    ros::spin();

    return 0;
}
