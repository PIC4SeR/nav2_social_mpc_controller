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

#include "mpc_enlarged_state/critics/agent_velocity_reference_cost_function.hpp"

#include <algorithm>
#include <cmath>

namespace mpc_enlarged_state
{

AgentVelocityReferenceCost::AgentVelocityReferenceCost(double weight, const AgentsStates& agents_init,
                                                       unsigned int agent_count)
  : sqrt_weight_(std::sqrt(std::max(0.0, weight)))
  , agent_count_(agent_count)
  , reference_velocities_(kAgentVelocityParamStride * agent_count, 0.0)
  , active_agents_(agent_count, false)
{
  const unsigned int tracked_agents =
      std::min(agent_count_, static_cast<unsigned int>(agents_init.size()));
  for (unsigned int agent_idx = 0; agent_idx < tracked_agents; ++agent_idx)
  {
    const auto& agent = agents_init[agent_idx];
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
