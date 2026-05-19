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

#ifndef MPC_ENLARGED_STATE__AGENT_VELOCITY_REFERENCE_COST_FUNCTION_HPP_
#define MPC_ENLARGED_STATE__AGENT_VELOCITY_REFERENCE_COST_FUNCTION_HPP_

#include <vector>

#include "ceres/ceres.h"
#include "mpc_enlarged_state/tools/type_definitions.hpp"

namespace mpc_enlarged_state
{

class AgentVelocityReferenceCost
{
public:
  using AgentVelocityReferenceCostFunction = ceres::DynamicAutoDiffCostFunction<AgentVelocityReferenceCost>;

  AgentVelocityReferenceCost(double weight, const AgentsStates& agents_init, unsigned int agent_count);

  inline static AgentVelocityReferenceCostFunction* Create(double weight, const AgentsStates& agents_init,
                                                           unsigned int agent_count)
  {
    return new AgentVelocityReferenceCostFunction(
        new AgentVelocityReferenceCost(weight, agents_init, agent_count));
  }

  template <typename T>
  bool operator()(T const* const* parameters, T* residuals) const
  {
    const unsigned int residual_count = 2 * agent_count_;
    for (unsigned int idx = 0; idx < residual_count; ++idx)
    {
      residuals[idx] = T(0.0);
    }

    const T* const agent_block = parameters[0];
    for (unsigned int agent_idx = 0; agent_idx < agent_count_; ++agent_idx)
    {
      if (!active_agents_[agent_idx])
      {
        continue;
      }
      const unsigned int idx = 2 * agent_idx;
      residuals[idx] = T(sqrt_weight_) * (agent_block[idx] - T(reference_velocities_[idx]));
      residuals[idx + 1] = T(sqrt_weight_) * (agent_block[idx + 1] - T(reference_velocities_[idx + 1]));
    }

    return true;
  }

private:
  double sqrt_weight_;
  unsigned int agent_count_;
  std::vector<double> reference_velocities_;
  std::vector<bool> active_agents_;
};

}  // namespace mpc_enlarged_state

#endif  // MPC_ENLARGED_STATE__AGENT_VELOCITY_REFERENCE_COST_FUNCTION_HPP_
