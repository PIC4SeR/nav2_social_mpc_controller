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

#include "mpc_enlarged_state/critics/agent_orca_dynamics_cost_function.hpp"

#include <algorithm>
#include <cmath>

namespace mpc_enlarged_state
{

AgentOrcaDynamicsCost::AgentOrcaDynamicsCost(double weight, double max_accel, const AgentsStates& agents_init,
                                             const geometry_msgs::msg::Pose& robot_init,
                                             unsigned int current_position, double time_step,
                                             unsigned int control_horizon, unsigned int block_length,
                                             unsigned int parameter_block_count, bool has_agent_parameters,
                                             unsigned int agent_count, const std::vector<double>& cooperation,
                                             const OrcaPredictionParams& orca)
  : sqrt_weight_(std::sqrt(std::max(0.0, weight)))
  , max_accel_(max_accel)
  , agents_init_(agents_init)
  , robot_init_(robot_init)
  , current_position_(current_position)
  , time_step_(time_step)
  , control_horizon_(control_horizon)
  , block_length_(block_length)
  , parameter_block_count_(parameter_block_count)
  , has_agent_parameters_(has_agent_parameters)
  , agent_count_(agent_count)
  , reference_velocities_(kAgentVelocityParamStride * agent_count, 0.0)
  , active_agents_(agent_count, false)
  , cooperation_(agent_count, 1.0)
  , orca_time_horizon_(orca.time_horizon)
  , orca_relaxation_time_(orca.relaxation_time)
  , orca_smoothing_(orca.smoothing)
  , agent_radius_(orca.agent_radius)
  , robot_radius_(orca.robot_radius)
{
  // Agents beyond what the caller supplied keep the 1.0 default, so a short (or
  // empty) cooperation vector degrades to standard fully-reciprocal ORCA.
  const unsigned int scored_agents =
      std::min(agent_count_, static_cast<unsigned int>(cooperation.size()));
  for (unsigned int agent_idx = 0; agent_idx < scored_agents; ++agent_idx)
  {
    cooperation_[agent_idx] = cooperation[agent_idx];
  }

  const unsigned int tracked_agents =
      std::min(agent_count_, static_cast<unsigned int>(agents_init_.size()));
  for (unsigned int agent_idx = 0; agent_idx < tracked_agents; ++agent_idx)
  {
    const auto& agent = agents_init_[agent_idx];
    if (agent[kStateTime] == -1.0)
    {
      continue;
    }

    const unsigned int idx = kAgentVelocityParamStride * agent_idx;
    reference_velocities_[idx + kAgentVxParam] =
        agent[kStateLinearVelocity] * std::cos(agent[kStateYaw]);
    reference_velocities_[idx + kAgentVyParam] =
        agent[kStateLinearVelocity] * std::sin(agent[kStateYaw]);
    active_agents_[agent_idx] = true;
  }
}

}  // namespace mpc_enlarged_state
