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

#ifndef MPC_ENLARGED_STATE__PROXEMICS_COST_FUNCTION_HPP_
#define MPC_ENLARGED_STATE__PROXEMICS_COST_FUNCTION_HPP_

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "glog/logging.h"
#include "mpc_enlarged_state/update_state.hpp"
#include "mpc_enlarged_state/tools/type_definitions.hpp"

namespace mpc_enlarged_state
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
  using ProxemicsCostFunction = ceres::DynamicAutoDiffCostFunction<ProxemicsCost, /*Jet stride=*/16>;

  ProxemicsCost(double weight, const AgentsStates& agents_init, const geometry_msgs::msg::Pose& robot_init,
                const double counter, unsigned int current_position, double time_step, unsigned int control_horizon,
                unsigned int block_length);
  ProxemicsCost(double weight, const AgentsStates& agents_init, const geometry_msgs::msg::Pose& robot_init,
                const double counter, unsigned int current_position, double time_step, unsigned int control_horizon,
                unsigned int block_length, unsigned int parameter_block_count, bool has_agent_parameters,
                unsigned int agent_count);

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

  inline static ProxemicsCostFunction* Create(double weight, const AgentsStates& agents_init,
                                              const geometry_msgs::msg::Pose& robot_init, const double counter,
                                              unsigned int current_position, double time_step,
                                              unsigned int control_horizon, unsigned int block_length,
                                              unsigned int parameter_block_count, bool has_agent_parameters,
                                              unsigned int agent_count)
  {
    return new ProxemicsCostFunction(new ProxemicsCost(weight, agents_init, robot_init, counter, current_position,
                                                       time_step, control_horizon, block_length,
                                                       parameter_block_count, has_agent_parameters, agent_count));
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
    Eigen::Matrix<T, kStateSize, 1> robot;
    auto [new_position_x, new_position_y, new_position_orientation, agents] =
        computeEnlargedState(robot_init_, agents_init_, parameters, parameter_block_count_, has_agent_parameters_,
                             agent_count_, time_step_, current_position_, control_horizon_, block_length_);
    robot(kStateX, 0) = (T)new_position_x;                                                         // x
    robot(kStateY, 0) = (T)new_position_y;                                                         // y
    robot(kStateYaw, 0) = (T)new_position_orientation;                                             // yaw
    robot(kStateTime, 0) = (T)counter_;                                                            // t
    unsigned int block_idx = current_position_ < control_horizon_ ? current_position_ / block_length_ :
                                                                 (control_horizon_ - 1) / block_length_;
    block_idx = std::min(block_idx, parameter_block_count_ - 1);
    robot(kStateLinearVelocity, 0) = parameters[block_idx][kRobotLinearVelocityParam];   // lv
    robot(kStateAngularVelocity, 0) = parameters[block_idx][kRobotAngularVelocityParam];  // av

    T proxemics_cost = computeProxemics(robot, agents);  // Compute proxemics cost on robot
    residual[0] = (T)weight_ * proxemics_cost;           // Scale the proxemics cost by the weight
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
  T computeProxemics(const Eigen::Matrix<T, kStateSize, 1>& me,
                     const Eigen::Matrix<T, kStateSize, Eigen::Dynamic>& agents) const
  {
    T min_distance((T)std::numeric_limits<T>::max());  // Initialize minimum distance to a large value
    Eigen::Matrix<T, 2, 1> mePos(me[kStateX], me[kStateY]);  // Extract the position of the robot
    Eigen::Matrix<T, 2, 1> meVel(me[kStateLinearVelocity] * ceres::cos(me[kStateYaw]),
                                 me[kStateLinearVelocity] * ceres::sin(me[kStateYaw]));  // Extract velocity

    for (Eigen::Index i = 0; i < agents.cols(); i++)  // Iterate through each agent
    {
      if (agents(kStateTime, i) == (T)-1.0)  // Skip agents that are invalid (e.g., not present)
        continue;

      Eigen::Matrix<T, 2, 1> aPos(agents(kStateX, i), agents(kStateY, i));  // Extract the position of the agent
      Eigen::Matrix<T, 2, 1> diff =
          mePos - aPos;                         // Calculate the difference in position between the robot and the agent
      T squared_distance = diff.squaredNorm();  // Calculate the squared distance between the robot and the agent
      if (squared_distance < 1e-6)              // If the squared distance is too small, set a fixed direction
      {
        diff = Eigen::Matrix<T, 2, 1>((T)1e-6, (T)0.0);  // Use a fixed small direction
      }
      min_distance = std::min(min_distance, squared_distance);  // Update the minimum distance
    }
    T proxemics_cost =
        (T)alpha_ * ceres::exp(-min_distance / ((T)d0_ * (T)d0_));  // Exponential decay based on distance
    return proxemics_cost;
  }

private:
  double weight_;
  AgentsStates agents_init_;
  geometry_msgs::msg::Pose robot_init_;
  double counter_;
  unsigned int current_position_;
  double time_step_;
  unsigned int control_horizon_;
  unsigned int block_length_;
  unsigned int parameter_block_count_;
  bool has_agent_parameters_;
  unsigned int agent_count_;
  double d0_;     // Minimum distance for proxemics cost
  double alpha_;  // Scaling factor for the proxemics cost
};

}  // namespace mpc_enlarged_state

#endif
