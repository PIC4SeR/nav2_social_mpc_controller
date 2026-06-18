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

#ifndef MPC_ENLARGED_STATE__CROSSING_COST_FUNCTION_HPP_
#define MPC_ENLARGED_STATE__CROSSING_COST_FUNCTION_HPP_

#include <mpc_enlarged_state/update_state.hpp>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "geometry_msgs/msg/pose.hpp"
#include "glog/logging.h"
#include "mpc_enlarged_state/tools/type_definitions.hpp"

namespace mpc_enlarged_state
{

class CrossingCost
{
public:
  using CrossingCostFunction = ceres::DynamicAutoDiffCostFunction<CrossingCost>;

  CrossingCost(double weight, double bearing_weight, const AgentsStates& agents_init,
               const geometry_msgs::msg::Pose& robot_init, unsigned int current_position, double time_step,
               unsigned int control_horizon, unsigned int block_length, unsigned int parameter_block_count,
               bool has_agent_parameters, unsigned int agent_count);

  inline static CrossingCostFunction* Create(double weight, double bearing_weight, const AgentsStates& agents_init,
                                             const geometry_msgs::msg::Pose& robot_init,
                                             unsigned int current_position, double time_step,
                                             unsigned int control_horizon, unsigned int block_length,
                                             unsigned int parameter_block_count, bool has_agent_parameters,
                                             unsigned int agent_count)
  {
    return new CrossingCostFunction(new CrossingCost(weight, bearing_weight, agents_init, robot_init, current_position,
                                                     time_step, control_horizon, block_length, parameter_block_count,
                                                     has_agent_parameters, agent_count));
  }

  template <typename T>
  bool operator()(T const* const* parameters, T* residuals) const
  {
    auto [new_position_x, new_position_y, new_position_orientation, agents] =
        computeEnlargedState(robot_init_, agents_init_, parameters, parameter_block_count_, has_agent_parameters_,
                             agent_count_, time_step_, current_position_, control_horizon_, block_length_);

    unsigned int block_idx = current_position_ < control_horizon_ ? current_position_ / block_length_ :
                                                                 (control_horizon_ - 1) / block_length_;
    block_idx = std::min(block_idx, parameter_block_count_ - 1);
    T robot_v = parameters[block_idx][kRobotLinearVelocityParam];
    T robot_omega = parameters[block_idx][kRobotAngularVelocityParam];

    int closest_index = -1;
    T closest_distance_squared = T(9999.0);
    for (Eigen::Index i = 0; i < agents.cols(); i++)
    {
      if (agents(kStateTime, i) == T(-1.0))
      {
        continue;
      }
      T dx = agents(kStateX, i) - new_position_x;
      T dy = agents(kStateY, i) - new_position_y;
      T distance_squared = dx * dx + dy * dy;
      if (distance_squared < closest_distance_squared && agents(kStateLinearVelocity, i) > T(0.05))
      {
        closest_distance_squared = distance_squared;
        closest_index = static_cast<int>(i);
      }
    }

    if (closest_index < 0 || closest_distance_squared > safe_distance_squared_)
    {
      residuals[0] = T(0.0);
      return true;
    }

    T agent_heading = agents(kStateYaw, closest_index);
    T heading_diff = new_position_orientation - agent_heading;
    T sin_diff = ceres::sin(heading_diff);
    T crossing_intensity = sin_diff * sin_diff;
    T speed_component = robot_v * crossing_intensity;

    T agent_heading_dx = ceres::cos(agent_heading);
    T agent_heading_dy = ceres::sin(agent_heading);
    T to_agent_x = agents(kStateX, closest_index) - new_position_x;
    T to_agent_y = agents(kStateY, closest_index) - new_position_y;
    T cross = to_agent_x * agent_heading_dy - to_agent_y * agent_heading_dx;

    T steer_scale = T(3.0);
    T k = T(5.0);
    T steer_penalty = ceres::log(T(1.0) + ceres::exp(k * cross * robot_omega * steer_scale)) / k;
    T steer_component = steer_penalty * crossing_intensity;
    T dist_decay = ceres::exp(-closest_distance_squared / T(safe_distance_squared_));

    residuals[0] = T(weight_) * dist_decay * (speed_component + T(bearing_weight_) * steer_component);
    return true;
  }

private:
  double weight_;
  double bearing_weight_;
  AgentsStates agents_init_;
  geometry_msgs::msg::Pose robot_init_;
  unsigned int current_position_;
  double time_step_;
  unsigned int control_horizon_;
  unsigned int block_length_;
  unsigned int parameter_block_count_;
  bool has_agent_parameters_;
  unsigned int agent_count_;
  double safe_distance_squared_;
};

}  // namespace mpc_enlarged_state

#endif  // MPC_ENLARGED_STATE__CROSSING_COST_FUNCTION_HPP_
