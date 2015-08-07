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

// Below largely taken from JointTrajectoryController
// Original author Adolfo Rodriguez Tsouroukdissian

#ifndef PERMUTATION_HELPER_H
#define PERMUTATION_HELPER_H


#include <trajectory_msgs/JointTrajectory.h>

namespace permutation{

template <class T>
inline std::vector<unsigned int> permutation(const T& t1, const T& t2)
{
  typedef unsigned int SizeType;

  // Arguments must have the same size
  if (t1.size() != t2.size()) {return std::vector<SizeType>();}

  std::vector<SizeType> permutation_vector(t1.size()); // Return value
  for (typename T::const_iterator t1_it = t1.begin(); t1_it != t1.end(); ++t1_it)
  {
    typename T::const_iterator t2_it = std::find(t2.begin(), t2.end(), *t1_it);
    if (t2.end() == t2_it) {return std::vector<SizeType>();}
    else
    {
      const SizeType t1_dist = std::distance(t1.begin(), t1_it);
      const SizeType t2_dist = std::distance(t2.begin(), t2_it);
      permutation_vector[t1_dist] = t2_dist;
    }
  }
  return permutation_vector;
}

bool reorderTrajectoryEntries(const trajectory_msgs::JointTrajectory& in,
                              const std::vector<unsigned int>& permutation,
                              trajectory_msgs::JointTrajectory& out)
{
  size_t num_traj_points = in.points.size();
  size_t num_joints = in.joint_names.size();

  //Copy for simplicity so everything is sized correctly;
  out = in;

  for (size_t j = 0; j < num_traj_points; ++j){

    const trajectory_msgs::JointTrajectoryPoint& in_point = in.points[j];
    trajectory_msgs::JointTrajectoryPoint&       out_point = out.points[j];

    for (unsigned int i = 0; i < num_joints; ++i)
    {
      // Apply permutation only if it was specified, otherwise preserve original message order
      const unsigned int id = permutation.empty() ? i : permutation[i];

      if (!in_point.positions.empty())     {out_point.positions[i]     = in_point.positions[id];}
      if (!in_point.velocities.empty())    {out_point.velocities[i]     = in_point.velocities[id];}
      if (!in_point.accelerations.empty()) {out_point.accelerations[i] = in_point.accelerations[id];}
    }
  }

  if (out.points.size() > 1)
  {
    if (out.points[0].time_from_start.toSec() < 0.0001){
      //out.points[0].time_from_start = ros::Duration(0.001);
      out.points.erase(out.points.begin());
    }
  }
}

}

#endif
