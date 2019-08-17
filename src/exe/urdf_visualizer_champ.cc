/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <ros/init.h>

#include <xpp_champ/inverse_kinematics_champ.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_states/joints.h>
#include <xpp_states/endeffector_mappings.h>

#include <xpp_vis/cartesian_joint_converter.h>
#include <xpp_vis/urdf_visualizer.h>

using namespace xpp;
using namespace quad;

int main(int argc, char *argv[])
{
  ::ros::init(argc, argv, "champ_urdf_visualizer");

  const std::string joint_desired_champ = "xpp/joint_champ_des";

  auto champ_ik = std::make_shared<InverseKinematicsChamp>();
  CartesianJointConverter inv_kin_converter(champ_ik,
					    xpp_msgs::robot_state_desired,
					    joint_desired_champ);

//   urdf joint names
  int n_ee = champ_ik->GetEECount();
  int n_j  = ChamplegJointCount;

  std::vector<UrdfVisualizer::URDFName> joint_names(n_ee*n_j);
  joint_names.at(n_j*LF + HAA) = "lf_hip_joint";
  joint_names.at(n_j*LF + HFE) = "lf_upper_leg_joint";
  joint_names.at(n_j*LF + KFE) = "lf_lower_leg_joint";
  joint_names.at(n_j*RF + HAA) = "rf_hip_joint";
  joint_names.at(n_j*RF + HFE) = "rf_upper_leg_joint";
  joint_names.at(n_j*RF + KFE) = "rf_lower_leg_joint";
  joint_names.at(n_j*LH + HAA) = "lh_hip_joint";
  joint_names.at(n_j*LH + HFE) = "lh_upper_leg_joint";
  joint_names.at(n_j*LH + KFE) = "lh_lower_leg_joint";
  joint_names.at(n_j*RH + HAA) = "rh_hip_joint";
  joint_names.at(n_j*RH + HFE) = "rh_upper_leg_joint";
  joint_names.at(n_j*RH + KFE) = "rh_lower_leg_joint";

  std::string urdf = "champ_rviz_urdf_robot_description";
  UrdfVisualizer champ_desired(urdf, joint_names, "base_link", "world",
			     joint_desired_champ, "champ_des");

  ::ros::spin();

  return 1;
}

