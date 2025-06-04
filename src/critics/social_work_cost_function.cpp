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

#include "nav2_social_mpc_controller/critics/social_work_cost_function.hpp"

namespace nav2_social_mpc_controller
{

SocialWorkCost::SocialWorkCost(double weight, const AgentsStates& agents_init, const AgentsStates& agents_zero,
                               const geometry_msgs::msg::Pose& robot_init,
                               const geometry_msgs::msg::Twist& robot_init_vel, const double counter,
                               unsigned int current_position, double time_step, unsigned int control_horizon,
                               unsigned int block_length)
  : weight_(weight)
  , robot_init_(robot_init)
  , robot_init_vel_(robot_init_vel)
  , counter_(counter)
  , current_position_(current_position)
  , time_step_(time_step)
  , control_horizon_(control_horizon)
  , block_length_(block_length)
{
  for (unsigned int j = 0; j < agents_init.size(); j++)
  {
    if (agents_zero[j][4] > 0.05)
    {
      original_agents_.col(j) << agents_init[j][0], agents_init[j][1], agents_init[j][2], agents_init[j][3],
          agents_init[j][4], agents_init[j][5];
      //} else {
      original_agents_zero_.col(j) << agents_zero[j][0], agents_zero[j][1], agents_zero[j][2], agents_zero[j][3],
          agents_zero[j][4], agents_zero[j][5];
    }
    else
    {
      original_agents_.col(j) << agents_zero[j][0], agents_zero[j][1], agents_zero[j][2], agents_zero[j][3],
          agents_zero[j][4], agents_zero[j][5];
      original_agents_zero_.col(j) << agents_zero[j][0], agents_zero[j][1], agents_zero[j][2], agents_zero[j][3],
          agents_zero[j][4], agents_zero[j][5];
    }
  }

  time_step_ = time_step;
  block_length_ = block_length;
  control_horizon_ = control_horizon;
  current_position_ = current_position;
  robot_init_ = robot_init;
  robot_init_vel_ = robot_init_vel;
  counter_ = counter;

  sfm_lambda_ = 2.0;
  sfm_gamma_ = 0.35;
  sfm_nPrime_ = 3.0;
  sfm_n_ = 2.0;
  sfm_relaxationTime_ = 0.5;
  sfm_forceFactorSocial_ = 2.1;
}

}  // namespace nav2_social_mpc_controller
