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

#ifndef MPC_BASE__PROXEMICS_COST_FUNCTION_HPP_
#define MPC_BASE__PROXEMICS_COST_FUNCTION_HPP_

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "glog/logging.h"
#include "mpc_base/update_state.hpp"
#include "mpc_base/tools/type_definitions.hpp"
#include <cmath>

namespace mpc_base
{

class ProxemicsCost
{
  /**
   * @class ProxemicsCost
   * @brief Functor for computing the proxemics cost based on the robot's interaction with agents.
   *
   * This class implements a cost function that evaluates the proxemics cost based on the robot's trajectory
   * with respect to other agents in the environment.
   */
public:
  using ProxemicsCostFunction = ceres::DynamicAutoDiffCostFunction<ProxemicsCost>;

  ProxemicsCost(double weight, const AgentsStates& agents_init, const geometry_msgs::msg::Pose& robot_init,
                const double counter, unsigned int current_position, double time_step, unsigned int control_horizon,
                unsigned int block_length);

  /**
   * @brief Creates a Ceres cost function for the ProxemicsCost.
   *
   * This function is a factory method that constructs an instance of the
   * ProxemicsCostFunction, which is a Ceres cost function
   * for computing the social work cost based on the robot's interaction with agents.
   *
   * @param weight The weight of the cost function.
   * @param agents_init A vector of AgentStatus representing the initial states of the agents.
   * @param robot_init The initial pose of the robot.
   * @param robot_init_vel The initial velocity of the robot.
   * @param counter A counter value used in the computation.
   * @param current_position The current position index in the control sequence.
   * @param time_step The time step for the state update.
   * @param control_horizon The total number of time steps in the MPC.
   * @param block_length The length of the parameter block for the MPC.
   * @return A pointer to the created ProxemicsFunctionType instance.
   */

  inline static ProxemicsCostFunction* Create(double weight, const AgentsStates& agents_init,
                                              const geometry_msgs::msg::Pose& robot_init, const double counter,
                                              unsigned int current_position, double time_step,
                                              unsigned int control_horizon, unsigned int block_length)
  {
    return new ProxemicsCostFunction(new ProxemicsCost(weight, agents_init, robot_init, counter, current_position,
                                                       time_step, control_horizon, block_length));
  }

  /**
   * @brief operator() computes the residual for the social work cost function.
   *
   * This function computes the updated state of the robot based on the input parameters and evaluates
   * the social forces acting on the robot and agents. It returns the residuals for optimization.
   *
   * @param parameters A pointer to an array of control parameters (e.g., velocities).
   * @param residual A pointer to store the computed residual value.
   * @return true if the computation was successful, false otherwise.
   */
  template <typename T>
  bool operator()(T const* const* parameters, T* residual) const
  {
    if (!std::isfinite(weight_))
    {
      residual[0] = T(0.0);
      return true;
    }

    // Compute robot social work
    Eigen::Matrix<T, 6, 3> agents = original_agents_.template cast<T>();  // Convert original agents to type T
    Eigen::Matrix<T, 6, 1> robot;

    auto [new_position_x, new_position_y, new_position_orientation] =
        getCachedUpdatedState(robot_init_, parameters, time_step_, current_position_, control_horizon_, block_length_);

    if (!ceres::IsFinite(new_position_x) || !ceres::IsFinite(new_position_y) ||
        !ceres::IsFinite(new_position_orientation))
    {
      residual[0] = T(0.0);
      return true;
    }
    //auto [new_position_x, new_position_y, new_position_orientation, agents] =
    //    computeSFMState(robot_init_, agents_, parameters, time_step_, current_position_, control_horizon_,
    //                    block_length_);  // Update robot state
    robot(0, 0) = (T)new_position_x;                                                               // x
    robot(1, 0) = (T)new_position_y;                                                               // y
    robot(2, 0) = (T)new_position_orientation;                                                     // yaw
    robot(3, 0) = (T)counter_;                                                                     // t
    if (current_position_ < control_horizon_)
    {
      const unsigned int block_index = current_position_ / block_length_;
      T lv = parameters[block_index][0];
      T av = parameters[block_index][1];
      if (!ceres::IsFinite(lv) || !ceres::IsFinite(av))
      {
        residual[0] = T(0.0);
        return true;
      }
      robot(4, 0) = lv;  // lv
      robot(5, 0) = av;  // av
    }
    else
    {
      const unsigned int block_index = (control_horizon_ - 1) / block_length_;
      T lv = parameters[block_index][0];
      T av = parameters[block_index][1];
      if (!ceres::IsFinite(lv) || !ceres::IsFinite(av))
      {
        residual[0] = T(0.0);
        return true;
      }
      robot(4, 0) = lv;  // lv
      robot(5, 0) = av;  // av
    }

    T proxemics_cost = computeProxemics(robot, agents);  // Compute proxemics cost on robot
    if (!ceres::IsFinite(proxemics_cost))
    {
      residual[0] = T(0.0);
      return true;
    }

    residual[0] = (T)weight_ * proxemics_cost;  // Scale the proxemics cost by the weight
    if (!ceres::IsFinite(residual[0]))
    {
      residual[0] = T(0.0);
      return true;
    }
    return true;
  }

  /**
   * @brief This function computes the social force acting on the robot based on its position, initial position,
   * and the positions of other agents.
   * It calculates the interaction between the robot and other agents, taking into account their velocities and
   * the social force parameters.
   *
   * @tparam T
   * @param me the current state of the robot, including position, orientation, and velocity
   * @param me_initial the initial state of the robot, including position, orientation, and velocity
   * @param agents the states of other agents in the environment, including their positions, orientations, and
   * velocities
   * @return T the computed proxemics cost
   */
  template <typename T>
  T computeProxemics(const Eigen::Matrix<T, 6, 1>& me, const Eigen::Matrix<T, 6, 3>& agents) const
  {
    if (!ceres::IsFinite(me[0]) || !ceres::IsFinite(me[1]) || !ceres::IsFinite(me[2]) || !ceres::IsFinite(me[4]))
    {
      return T(0.0);
    }

    T min_distance = T(1e6);  // Start with a large distance
    Eigen::Matrix<T, 2, 1> mePos(me[0], me[1]);        // Extract the position of the robot

    bool found_valid_agent = false;

    for (unsigned int i = 0; i < agents.cols(); i++)
    {
      if (agents(3, i) == (T)-1.0)
      {
        continue;
      }

      if (!ceres::IsFinite(agents(0, i)) || !ceres::IsFinite(agents(1, i)) || !ceres::IsFinite(agents(2, i)) ||
          !ceres::IsFinite(agents(4, i)))
      {
        continue;
      }

      Eigen::Matrix<T, 2, 1> aPos(agents(0, i), agents(1, i));
      Eigen::Matrix<T, 2, 1> diff = mePos - aPos;
      T squared_distance = diff.squaredNorm();

      if (!ceres::IsFinite(squared_distance))
      {
        continue;
      }

      squared_distance = ceres::fmax(squared_distance, T(1e-6));
      min_distance = ceres::fmin(min_distance, squared_distance);
      found_valid_agent = true;
    }

    if (!found_valid_agent)
    {
      return T(0.0);
    }

    T denom = T(d0_) * T(d0_);
    denom = ceres::fmax(denom, T(1e-6));

    T proxemics_cost = T(alpha_) * ceres::exp(-min_distance / denom);
    if (!ceres::IsFinite(proxemics_cost))
    {
      return T(0.0);
    }

    return proxemics_cost;
  }

private:
  double weight_;
  Eigen::Matrix<double, 6, 3> original_agents_;
  geometry_msgs::msg::Pose robot_init_;
  double counter_;
  unsigned int current_position_;
  double time_step_;
  unsigned int control_horizon_;
  unsigned int block_length_;
  double d0_;     // Minimum distance for proxemics cost
  double alpha_;  // Scaling factor for the proxemics cost
};

}  // namespace mpc_base

#endif