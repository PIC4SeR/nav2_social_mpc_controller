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

#ifndef MPC_ENLARGED_STATE__AGENT_OBSTACLE_COST_FUNCTION_HPP_
#define MPC_ENLARGED_STATE__AGENT_OBSTACLE_COST_FUNCTION_HPP_

#include <algorithm>
#include <memory>
#include <vector>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"
#include "geometry_msgs/msg/pose.hpp"
#include "mpc_enlarged_state/tools/type_definitions.hpp"
#include "mpc_enlarged_state/update_state.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace mpc_enlarged_state
{

class AgentObstacleCost
{
public:
  using AgentObstacleCostFunction = ceres::DynamicAutoDiffCostFunction<AgentObstacleCost>;

  AgentObstacleCost(double weight, const nav2_costmap_2d::Costmap2D* costmap,
                    const std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>& costmap_interpolator,
                    const AgentsStates& agents_init, const geometry_msgs::msg::Pose& robot_init,
                    unsigned int current_position, double time_step, unsigned int control_horizon,
                    unsigned int block_length, unsigned int parameter_block_count, bool has_agent_parameters,
                    unsigned int agent_count);

  inline static AgentObstacleCostFunction* Create(
      double weight, const nav2_costmap_2d::Costmap2D* costmap,
      const std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>& costmap_interpolator,
      const AgentsStates& agents_init, const geometry_msgs::msg::Pose& robot_init, unsigned int current_position,
      double time_step, unsigned int control_horizon, unsigned int block_length, unsigned int parameter_block_count,
      bool has_agent_parameters, unsigned int agent_count)
  {
    return new AgentObstacleCostFunction(
        new AgentObstacleCost(weight, costmap, costmap_interpolator, agents_init, robot_init, current_position,
                              time_step, control_horizon, block_length, parameter_block_count, has_agent_parameters,
                              agent_count));
  }

  template <typename T>
  bool operator()(T const* const* parameters, T* residuals) const
  {
    for (unsigned int agent_idx = 0; agent_idx < agent_count_; ++agent_idx)
    {
      residuals[agent_idx] = T(0.0);
    }

    if (!has_agent_parameters_ || parameter_block_count_ == 0)
    {
      return true;
    }

    auto [robot_x, robot_y, robot_theta, agents] =
        computeEnlargedState(robot_init_, agents_init_, parameters, parameter_block_count_, has_agent_parameters_,
                             agent_count_, time_step_, current_position_, control_horizon_, block_length_);
    (void)robot_x;
    (void)robot_y;
    (void)robot_theta;

    const unsigned int tracked_agents =
        std::min(agent_count_, static_cast<unsigned int>(active_agents_.size()));
    for (unsigned int agent_idx = 0; agent_idx < tracked_agents; ++agent_idx)
    {
      if (!active_agents_[agent_idx])
      {
        continue;
      }

      const Eigen::Index col = static_cast<Eigen::Index>(agent_idx);
      Eigen::Matrix<T, 2, 1> agent_position(agents(kStateX, col), agents(kStateY, col));
      Eigen::Matrix<T, 2, 1> costmap_position =
          (agent_position - costmap_origin_.template cast<T>()) / T(costmap_resolution_);

      T value;
      costmap_interpolator_->Evaluate(costmap_position[kY], costmap_position[kX], &value);
      residuals[agent_idx] = T(sqrt_weight_) * value / T(255.0);
    }

    return true;
  }

private:
  double sqrt_weight_;
  AgentsStates agents_init_;
  geometry_msgs::msg::Pose robot_init_;
  unsigned int current_position_;
  double time_step_;
  unsigned int control_horizon_;
  unsigned int block_length_;
  unsigned int parameter_block_count_;
  bool has_agent_parameters_;
  unsigned int agent_count_;
  Eigen::Vector2d costmap_origin_;
  double costmap_resolution_;
  std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>> costmap_interpolator_;
  std::vector<bool> active_agents_;
};

}  // namespace mpc_enlarged_state

#endif  // MPC_ENLARGED_STATE__AGENT_OBSTACLE_COST_FUNCTION_HPP_
