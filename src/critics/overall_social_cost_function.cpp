// Copyright (c) 2022 SRL -Service Robotics Lab, Pablo de Olavide University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mpc_base/critics/overall_social_cost_function.hpp"

namespace mpc_base
{

SocialOverallCost::SocialOverallCost(double work_weight,double angle_weight, double proxemics_weight,
                  double path_follow_weight, double path_align_weight, const Eigen::Matrix<double, 2, 1> final_point,
                  const Eigen::Matrix<double, 2, 1> point,
                 const AgentsStates& agents_init, long unsigned int agent_index, const geometry_msgs::msg::Pose& robot_init,
                  unsigned int current_position, double time_step, unsigned int control_horizon,
                 unsigned int block_length, bool found_people, bool use_social_work_cost, bool use_angle_cost, bool use_proxemics_cost,
                bool use_path_follow_cost, bool use_path_align_cost)
  : work_weight_(work_weight)
  , angle_weight_(angle_weight)
  , proxemics_weight_(proxemics_weight)
  , path_follow_weight_(path_follow_weight)  // Default value, can be set later if needed
  , path_align_weight_(path_align_weight)  // Default value, can be set later if needed
  , final_point_(final_point)
  , point_(point)
  , agents_init_(agents_init)
  , agent_index_(agent_index)
  , robot_init_(robot_init)
  , current_position_(current_position)
  , time_step_(time_step)
  , control_horizon_(control_horizon)
  , block_length_(block_length)
  , found_people_(found_people)
  , use_social_work_cost_(use_social_work_cost)
  , use_angle_cost_(use_angle_cost)
  , use_proxemics_cost_(use_proxemics_cost)
  , use_path_follow_cost_(use_path_follow_cost)
  , use_path_align_cost_(use_path_align_cost)  // Default value, can be set later if needed
{
  original_agents_.resize(6, agents_init.size());
  for (size_t j = 0; j < agents_init.size(); j++)
  {
    original_agents_.col(j) = agents_init[j];
  }
  agents_init_ = agents_init;

  sfm_lambda_ = 2.0;
  sfm_gamma_ = 0.35;
  sfm_nPrime_ = 3.0;
  sfm_n_ = 2.0;
  sfm_relaxationTime_ = 0.5;
  sfm_forceFactorSocial_ = 2.1;
  alpha_ = 3.0;
  d0_ = 0.5;
  safe_distance_squared_ = 4.0;  // Safe distance squared to avoid division by zero in proxemics cost
}

}  // namespace mpc_base