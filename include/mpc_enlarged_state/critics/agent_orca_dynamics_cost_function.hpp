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

#ifndef MPC_ENLARGED_STATE__AGENT_ORCA_DYNAMICS_COST_FUNCTION_HPP_
#define MPC_ENLARGED_STATE__AGENT_ORCA_DYNAMICS_COST_FUNCTION_HPP_

#include <algorithm>
#include <vector>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "geometry_msgs/msg/pose.hpp"
#include "mpc_enlarged_state/tools/type_definitions.hpp"
#include "mpc_enlarged_state/update_state.hpp"

namespace mpc_enlarged_state
{

// Soft, autodiff-friendly ORCA-style dynamics critic.
//
// Mirrors the residual shape of AgentSfmDynamicsCost so it can be swapped in
// place. Per active agent we emit two residuals matching the optimized
// acceleration against an ORCA-derived target acceleration:
//
//   residual = sqrt(weight) * ( (v_k - v_{k-1}) / Δt - a_target )
//
// The target velocity is the preferred velocity (reference) plus, for every
// neighbor (robot and other agents), a smoothed reciprocal correction that
// pushes the relative velocity out of the τ-horizon cutoff disk of the
// velocity obstacle:
//
//   w        = v_rel - p_rel / τ
//   violation = R_comb / τ - ||w||           // > 0 inside the danger disk
//   correction = 0.5 * smooth_max(violation, 0) * w / ||w||
//
// smooth_max is a quadratic-softplus so the residual stays differentiable when
// neighbors enter/leave the danger disk. ORCA leg projections (collisions
// outside τ but inside the velocity-obstacle wedge) are intentionally omitted;
// they add branchy projections that hurt autodiff stability. The cutoff-disk
// term alone is the conservative core of ORCA and behaves smoothly.
class AgentOrcaDynamicsCost
{
public:
  using AgentOrcaDynamicsCostFunction = ceres::DynamicAutoDiffCostFunction<AgentOrcaDynamicsCost, /*Jet stride=*/16>;

  AgentOrcaDynamicsCost(double weight, double max_accel, const AgentsStates& agents_init,
                        const geometry_msgs::msg::Pose& robot_init, unsigned int current_position, double time_step,
                        unsigned int control_horizon, unsigned int block_length, unsigned int parameter_block_count,
                        bool has_agent_parameters, unsigned int agent_count,
                        const std::vector<double>& cooperation, const OrcaPredictionParams& orca);

  inline static AgentOrcaDynamicsCostFunction* Create(double weight, double max_accel,
                                                      const AgentsStates& agents_init,
                                                      const geometry_msgs::msg::Pose& robot_init,
                                                      unsigned int current_position, double time_step,
                                                      unsigned int control_horizon, unsigned int block_length,
                                                      unsigned int parameter_block_count, bool has_agent_parameters,
                                                      unsigned int agent_count,
                                                      const std::vector<double>& cooperation,
                                                      const OrcaPredictionParams& orca)
  {
    return new AgentOrcaDynamicsCostFunction(
        new AgentOrcaDynamicsCost(weight, max_accel, agents_init, robot_init, current_position, time_step,
                                  control_horizon, block_length, parameter_block_count, has_agent_parameters,
                                  agent_count, cooperation, orca));
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

      Eigen::Matrix<T, 2, 1> reference_velocity(T(reference_velocities_[idx + kAgentVxParam]),
                                                T(reference_velocities_[idx + kAgentVyParam]));
      Eigen::Matrix<T, 2, 1> velocity_correction =
          accumulateOrcaCorrections(agent_position, current_velocity, robot_position, robot_velocity, agents,
                                    current_agent_block, agent_idx);

      Eigen::Matrix<T, 2, 1> target_velocity = reference_velocity + velocity_correction;
      Eigen::Matrix<T, 2, 1> target_accel = (target_velocity - current_velocity) / T(orca_relaxation_time_);
      clampAcceleration(target_accel);

      residuals[idx + kAgentVxParam] = T(sqrt_weight_) * (optimized_accel[kX] - target_accel[kX]);
      residuals[idx + kAgentVyParam] = T(sqrt_weight_) * (optimized_accel[kY] - target_accel[kY]);
    }

    return true;
  }

private:
  template <typename T>
  Eigen::Matrix<T, 2, 1> accumulateOrcaCorrections(
      const Eigen::Matrix<T, 2, 1>& agent_position, const Eigen::Matrix<T, 2, 1>& agent_velocity,
      const Eigen::Matrix<T, 2, 1>& robot_position, const Eigen::Matrix<T, 2, 1>& robot_velocity,
      const Eigen::Matrix<T, kStateSize, Eigen::Dynamic>& agents, const T* const current_agent_block,
      unsigned int agent_idx) const
  {
    // Scale the reciprocal share this person is assumed to take for avoiding the
    // robot: 1.0 leaves ORCA's standard half-and-half split, 0.0 predicts them
    // taking no responsibility at all and holding course, which leaves the whole
    // avoidance to the robot. Shares between people are always reciprocal.
    Eigen::Matrix<T, 2, 1> correction =
        T(cooperation_[agent_idx]) *
        pairwiseOrcaCorrection(agent_position, agent_velocity, robot_position, robot_velocity, T(robot_radius_));

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
      correction +=
          pairwiseOrcaCorrection(agent_position, agent_velocity, other_position, other_velocity, T(agent_radius_));
    }

    return correction;
  }

  // Smoothed cutoff-disk ORCA correction for one neighbor. Returns the velocity
  // adjustment "me" should apply (reciprocal half of the full correction).
  template <typename T>
  Eigen::Matrix<T, 2, 1> pairwiseOrcaCorrection(const Eigen::Matrix<T, 2, 1>& me_position,
                                                const Eigen::Matrix<T, 2, 1>& me_velocity,
                                                const Eigen::Matrix<T, 2, 1>& other_position,
                                                const Eigen::Matrix<T, 2, 1>& other_velocity,
                                                const T& other_radius) const
  {
    const T eps = T(1e-6);
    const T inv_tau = T(1.0) / T(orca_time_horizon_);
    const T combined_radius = T(agent_radius_) + other_radius;

    Eigen::Matrix<T, 2, 1> relative_position = other_position - me_position;
    Eigen::Matrix<T, 2, 1> relative_velocity = me_velocity - other_velocity;
    Eigen::Matrix<T, 2, 1> cutoff_center = relative_position * inv_tau;
    Eigen::Matrix<T, 2, 1> w = relative_velocity - cutoff_center;

    T w_norm = ceres::sqrt(w.squaredNorm() + eps * eps);
    Eigen::Matrix<T, 2, 1> unit_w = w / w_norm;
    T required_radius = combined_radius * inv_tau;
    T violation = required_radius - w_norm;  // > 0 means inside τ-disk

    // Smooth max(violation, 0) via quadratic softplus.
    T smooth_eps = T(orca_smoothing_);
    T smooth_violation = T(0.5) * (violation + ceres::sqrt(violation * violation + smooth_eps * smooth_eps));

    return T(0.5) * smooth_violation * unit_w;
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
  double orca_time_horizon_;
  double orca_relaxation_time_;
  double orca_smoothing_;
  double agent_radius_;
  double robot_radius_;
};

}  // namespace mpc_enlarged_state

#endif  // MPC_ENLARGED_STATE__AGENT_ORCA_DYNAMICS_COST_FUNCTION_HPP_
