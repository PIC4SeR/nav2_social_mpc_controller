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

#include "mpc_enlarged_state/critics/proxemics_cost_function.hpp"

namespace mpc_enlarged_state
{

ProxemicsCost::ProxemicsCost(double weight, const AgentsStates& agents_init, const geometry_msgs::msg::Pose& robot_init,
                             const double counter, unsigned int current_position, double time_step,
                             unsigned int control_horizon, unsigned int block_length)
  : weight_(weight)
  , agents_init_(agents_init)
  , robot_init_(robot_init)
  , counter_(counter)
  , current_position_(current_position)
  , time_step_(time_step)
  , control_horizon_(control_horizon)
  , block_length_(block_length)
  , parameter_block_count_(block_length > 0 ? (control_horizon - 1) / block_length + 1 : 0)
  , has_agent_parameters_(false)
  , agent_count_(static_cast<unsigned int>(agents_init.size()))
{
  alpha_ = 3.0;  // Scaling factor for the proxemics cost
  d0_ = 0.5;     // Minimum distance for proxemics cost
}

ProxemicsCost::ProxemicsCost(double weight, const AgentsStates& agents_init,
                             const geometry_msgs::msg::Pose& robot_init, const double counter,
                             unsigned int current_position, double time_step, unsigned int control_horizon,
                             unsigned int block_length, unsigned int parameter_block_count,
                             bool has_agent_parameters, unsigned int agent_count)
  : ProxemicsCost(weight, agents_init, robot_init, counter, current_position, time_step, control_horizon, block_length)
{
  parameter_block_count_ = parameter_block_count;
  has_agent_parameters_ = has_agent_parameters;
  agent_count_ = agent_count;
}

}  // namespace mpc_enlarged_state
