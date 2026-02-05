// Copyright (c) 2026 SRL -Service Robotics Lab, Pablo de Olavide University
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

#ifndef MPC_SFM_MOTION_MODEL__GOAL_PROXIMITY_COST_FUNCTION_HPP_
#define MPC_SFM_MOTION_MODEL__GOAL_PROXIMITY_COST_FUNCTION_HPP_

#include <algorithm>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "geometry_msgs/msg/pose.hpp"
#include "glog/logging.h"

#include "mpc_sfm_motion_model/update_state.hpp"

namespace mpc_sfm_motion_model
{

/**
 * @brief Cost that exponentially attracts the robot toward the final global goal
 *        when it is within a configurable activation radius.
 */
class GoalProximityCost
{
public:
  using GoalProximityCostFunction = ceres::DynamicAutoDiffCostFunction<GoalProximityCost>;

  GoalProximityCost(double weight, double activation_radius, double decay_distance,
                    const geometry_msgs::msg::Pose& goal_pose, const geometry_msgs::msg::Pose& robot_init,
                    unsigned int current_position, double time_step, unsigned int control_horizon,
                    unsigned int block_length)
    : weight_(weight),
      activation_radius_(std::max(activation_radius, 1e-3)),
      decay_distance_(std::max(decay_distance, 1e-3)),
      goal_pose_(goal_pose),
      robot_init_(robot_init),
      current_position_(current_position),
      time_step_(time_step),
      control_horizon_(control_horizon),
      block_length_(block_length)
  {
  }

  inline static GoalProximityCostFunction* Create(double weight, double activation_radius, double decay_distance,
                                                  const geometry_msgs::msg::Pose& goal_pose,
                                                  const geometry_msgs::msg::Pose& robot_init,
                                                  unsigned int current_position, double time_step,
                                                  unsigned int control_horizon, unsigned int block_length)
  {
    return new GoalProximityCostFunction(new GoalProximityCost(weight, activation_radius, decay_distance, goal_pose,
                                                               robot_init, current_position, time_step,
                                                               control_horizon, block_length));
  }

  template <typename T>
  bool operator()(T const* const* parameters, T* residuals) const
  {
    auto [new_position_x, new_position_y, new_position_orientation] = computeUpdatedStateRedux(
        robot_init_, parameters, time_step_, current_position_, control_horizon_, block_length_);
    (void)new_position_orientation;

    T goal_x = (T)goal_pose_.position.x;
    T goal_y = (T)goal_pose_.position.y;
    T dx = goal_x - (T)new_position_x;
    T dy = goal_y - (T)new_position_y;
    T dist = ceres::sqrt(dx * dx + dy * dy);

    if (dist > (T)activation_radius_)
    {
      residuals[0] = T(0.0);
      return true;
    }

    T normalized = dist / (T)decay_distance_;
    residuals[0] = (T)weight_ * (ceres::exp(normalized) - T(1.0));
    return true;
  }

private:
  double weight_;
  double activation_radius_;
  double decay_distance_;
  geometry_msgs::msg::Pose goal_pose_;
  geometry_msgs::msg::Pose robot_init_;
  unsigned int current_position_;
  double time_step_;
  unsigned int control_horizon_;
  unsigned int block_length_;
};

}  // namespace mpc_sfm_motion_model

#endif  // MPC_SFM_MOTION_MODEL__GOAL_PROXIMITY_COST_FUNCTION_HPP_
