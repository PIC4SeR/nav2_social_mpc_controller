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

#ifndef MPC_ENLARGED_STATE__AGENT_SFM_DYNAMICS_COST_FUNCTION_HPP_
#define MPC_ENLARGED_STATE__AGENT_SFM_DYNAMICS_COST_FUNCTION_HPP_

#include <algorithm>
#include <vector>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "geometry_msgs/msg/pose.hpp"
#include "mpc_enlarged_state/tools/type_definitions.hpp"
#include "mpc_enlarged_state/update_state.hpp"

namespace mpc_enlarged_state
{

class AgentSfmDynamicsCost
{
public:
  using AgentSfmDynamicsCostFunction = ceres::DynamicAutoDiffCostFunction<AgentSfmDynamicsCost, /*Jet stride=*/16>;

  AgentSfmDynamicsCost(double weight, double max_accel, const AgentsStates& agents_init,
                       const geometry_msgs::msg::Pose& robot_init, unsigned int current_position, double time_step,
                       unsigned int control_horizon, unsigned int block_length, unsigned int parameter_block_count,
                       bool has_agent_parameters, unsigned int agent_count,
                       const std::vector<double>& cooperation, const SfmPredictionParams& sfm);

  inline static AgentSfmDynamicsCostFunction* Create(double weight, double max_accel,
                                                     const AgentsStates& agents_init,
                                                     const geometry_msgs::msg::Pose& robot_init,
                                                     unsigned int current_position, double time_step,
                                                     unsigned int control_horizon, unsigned int block_length,
                                                     unsigned int parameter_block_count, bool has_agent_parameters,
                                                     unsigned int agent_count,
                                                     const std::vector<double>& cooperation,
                                                     const SfmPredictionParams& sfm)
  {
    return new AgentSfmDynamicsCostFunction(
        new AgentSfmDynamicsCost(weight, max_accel, agents_init, robot_init, current_position, time_step,
                                 control_horizon, block_length, parameter_block_count, has_agent_parameters,
                                 agent_count, cooperation, sfm));
  }

  template <typename T>
  bool operator()(T const* const* parameters, T* residuals) const
  {
    const unsigned int residual_count = kAgentVelocityParamStride * agent_count_;
    for (unsigned int idx = 0; idx < residual_count; ++idx)
    {
      residuals[idx] = T(0.0);
    }

    if (!has_agent_parameters_ || parameter_block_count_ == 0)
    {
      return true;
    }

    auto [robot_x, robot_y, robot_theta, agents] =
        computeEnlargedState(robot_init_, agents_init_, parameters, parameter_block_count_, has_agent_parameters_,
                             agent_count_, time_step_, current_position_, control_horizon_, block_length_);

    unsigned int block_idx = 0;
    if (control_horizon_ > 0 && block_length_ > 0)
    {
      block_idx = current_position_ < control_horizon_ ? current_position_ / block_length_ :
                                                         (control_horizon_ - 1) / block_length_;
    }
    block_idx = std::min(block_idx, parameter_block_count_ - 1);

    const T* const robot_block = parameters[block_idx];
    const T* const current_agent_block = parameters[parameter_block_count_ + block_idx];
    const T* const previous_agent_block =
        block_idx > 0 ? parameters[parameter_block_count_ + block_idx - 1] : nullptr;

    Eigen::Matrix<T, 2, 1> robot_position(robot_x, robot_y);
    Eigen::Matrix<T, 2, 1> robot_velocity(robot_block[kRobotLinearVelocityParam] * ceres::cos(robot_theta),
                                          robot_block[kRobotLinearVelocityParam] * ceres::sin(robot_theta));
    const T transition_dt =
        block_idx > 0 ? T(time_step_) * T(block_length_) : T(time_step_);

    for (unsigned int agent_idx = 0; agent_idx < agent_count_; ++agent_idx)
    {
      if (!active_agents_[agent_idx])
      {
        continue;
      }

      const unsigned int idx = kAgentVelocityParamStride * agent_idx;
      const Eigen::Index col = static_cast<Eigen::Index>(agent_idx);
      Eigen::Matrix<T, 2, 1> agent_position(agents(kStateX, col), agents(kStateY, col));
      Eigen::Matrix<T, 2, 1> current_velocity(current_agent_block[idx + kAgentVxParam],
                                              current_agent_block[idx + kAgentVyParam]);
      Eigen::Matrix<T, 2, 1> previous_velocity(
          previous_agent_block != nullptr ? previous_agent_block[idx + kAgentVxParam] :
                                            T(reference_velocities_[idx + kAgentVxParam]),
          previous_agent_block != nullptr ? previous_agent_block[idx + kAgentVyParam] :
                                            T(reference_velocities_[idx + kAgentVyParam]));

      Eigen::Matrix<T, 2, 1> optimized_accel = (current_velocity - previous_velocity) / transition_dt;
      Eigen::Matrix<T, 2, 1> sfm_accel =
          computeSocialAcceleration(agent_position, current_velocity, robot_position, robot_velocity, agents,
                                    current_agent_block, agent_idx);

      Eigen::Matrix<T, 2, 1> reference_velocity(T(reference_velocities_[idx + kAgentVxParam]),
                                                T(reference_velocities_[idx + kAgentVyParam]));
      sfm_accel += (reference_velocity - current_velocity) / T(sfm_relaxation_time_);
      clampAcceleration(sfm_accel);

      residuals[idx + kAgentVxParam] = T(sqrt_weight_) * (optimized_accel[kX] - sfm_accel[kX]);
      residuals[idx + kAgentVyParam] = T(sqrt_weight_) * (optimized_accel[kY] - sfm_accel[kY]);
    }

    return true;
  }

private:
  template <typename T>
  Eigen::Matrix<T, 2, 1> computeSocialAcceleration(
      const Eigen::Matrix<T, 2, 1>& agent_position, const Eigen::Matrix<T, 2, 1>& agent_velocity,
      const Eigen::Matrix<T, 2, 1>& robot_position, const Eigen::Matrix<T, 2, 1>& robot_velocity,
      const Eigen::Matrix<T, kStateSize, Eigen::Dynamic>& agents, const T* const current_agent_block,
      unsigned int agent_idx) const
  {
    // How much this particular person is predicted to yield to the robot: 1.0 keeps
    // the classic assumption that they socially avoid us like any other pedestrian,
    // 0.0 predicts them holding their course as if the robot were not there. Only
    // the robot's force is scaled -- people still avoid each other regardless.
    Eigen::Matrix<T, 2, 1> acceleration = T(cooperation_[agent_idx]) *
                                          computePairwiseSocialForce(agent_position, agent_velocity,
                                                                      robot_position, robot_velocity);
    for (unsigned int other_idx = 0; other_idx < agent_count_; ++other_idx)
    {
      if (other_idx == agent_idx || !active_agents_[other_idx])
      {
        continue;
      }
      const unsigned int idx = kAgentVelocityParamStride * other_idx;
      const Eigen::Index col = static_cast<Eigen::Index>(other_idx);
      Eigen::Matrix<T, 2, 1> other_position(agents(kStateX, col), agents(kStateY, col));
      Eigen::Matrix<T, 2, 1> other_velocity(current_agent_block[idx + kAgentVxParam],
                                            current_agent_block[idx + kAgentVyParam]);
      acceleration += computePairwiseSocialForce(agent_position, agent_velocity, other_position, other_velocity);
    }

    return acceleration;
  }

  template <typename T>
  Eigen::Matrix<T, 2, 1> computePairwiseSocialForce(const Eigen::Matrix<T, 2, 1>& me_position,
                                                    const Eigen::Matrix<T, 2, 1>& me_velocity,
                                                    const Eigen::Matrix<T, 2, 1>& other_position,
                                                    const Eigen::Matrix<T, 2, 1>& other_velocity) const
  {
    const T eps = T(1e-6);
    Eigen::Matrix<T, 2, 1> diff = me_position - other_position;
    T distance = ceres::sqrt(diff.squaredNorm() + eps * eps);
    Eigen::Matrix<T, 2, 1> diff_direction = diff / distance;

    Eigen::Matrix<T, 2, 1> velocity_diff = me_velocity - other_velocity;
    Eigen::Matrix<T, 2, 1> interaction_vector = T(sfm_lambda_) * velocity_diff + diff_direction;
    T interaction_length = ceres::sqrt(interaction_vector.squaredNorm() + eps * eps);
    Eigen::Matrix<T, 2, 1> interaction_direction = interaction_vector / interaction_length;

    T theta = ceres::atan2(ceres::sin(ceres::atan2(diff_direction[kY], diff_direction[kX]) -
                                      ceres::atan2(interaction_direction[kY], interaction_direction[kX])),
                           ceres::cos(ceres::atan2(diff_direction[kY], diff_direction[kX]) -
                                      ceres::atan2(interaction_direction[kY], interaction_direction[kX])));
    T b = T(sfm_gamma_) * interaction_length + eps;
    T velocity_term = T(sfm_n_prime_) * b * theta;
    T force_velocity_amount = ceres::exp(-distance / b - velocity_term * velocity_term);
    T sign = theta > T(0.0) ? T(1.0) : T(-1.0);
    T angle_term = T(sfm_n_) * b * theta;
    T force_angle_amount = sign * ceres::exp(-distance / b - angle_term * angle_term);

    Eigen::Matrix<T, 2, 1> force_velocity = force_velocity_amount * interaction_direction;
    Eigen::Matrix<T, 2, 1> left_normal(-interaction_direction[kY], interaction_direction[kX]);
    Eigen::Matrix<T, 2, 1> force_angle = force_angle_amount * left_normal;
    return T(sfm_force_factor_social_) * (force_velocity + force_angle);
  }

  template <typename T>
  void clampAcceleration(Eigen::Matrix<T, 2, 1>& acceleration) const
  {
    if (max_accel_ <= 0.0)
    {
      return;
    }

    const T norm = ceres::sqrt(acceleration.squaredNorm() + T(1e-12));
    if (norm > T(max_accel_))
    {
      acceleration *= T(max_accel_) / norm;
    }
  }

  double sqrt_weight_;
  double max_accel_;
  AgentsStates agents_init_;
  geometry_msgs::msg::Pose robot_init_;
  unsigned int current_position_;
  double time_step_;
  unsigned int control_horizon_;
  unsigned int block_length_;
  unsigned int parameter_block_count_;
  bool has_agent_parameters_;
  unsigned int agent_count_;
  std::vector<double> reference_velocities_;
  std::vector<bool> active_agents_;
  std::vector<double> cooperation_;
  double sfm_lambda_;
  double sfm_gamma_;
  double sfm_n_prime_;
  double sfm_n_;
  double sfm_relaxation_time_;
  double sfm_force_factor_social_;
};

}  // namespace mpc_enlarged_state

#endif  // MPC_ENLARGED_STATE__AGENT_SFM_DYNAMICS_COST_FUNCTION_HPP_
