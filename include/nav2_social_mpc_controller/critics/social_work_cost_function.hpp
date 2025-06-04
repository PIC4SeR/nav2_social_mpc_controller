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

#ifndef NAV2_SOCIAL_MPC_CONTROLLER__SOCIAL_WORK_FUNCTION_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__SOCIAL_WORK_FUNCTION_HPP_

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "glog/logging.h"
#include "nav2_social_mpc_controller/update_state.hpp"
#include "nav2_social_mpc_controller/tools/type_definitions.hpp"

namespace nav2_social_mpc_controller
{

template <typename T>
/**
 * @brief Wraps an angle to the range [-pi, pi].
 *
 * This function ensures that the angle is within the range of -pi to pi,
 * which is useful for angle normalization in trigonometric calculations.
 *
 * @param angle The angle in radians to be wrapped.
 * @return The wrapped angle in radians.
 */
inline T wrapToPi(T angle)
{
  while (angle > T(M_PI))
    angle -= T(2.0 * M_PI);
  while (angle <= T(-M_PI))
    angle += T(2.0 * M_PI);
  return angle;
}

class SocialWorkCost
{
  /**
   * @class SocialWorkCost
   * @brief Functor for computing the social work cost based on the robot's interaction with agents.
   *
   * This class implements a cost function that evaluates the social work done by the robot
   * in relation to other agents in the environment. It computes the social forces acting on
   * the robot and the agents, and calculates the residuals based on these forces.
   */
public:
  using SocialWorkCostFunction = ceres::DynamicAutoDiffCostFunction<SocialWorkCost>;

  SocialWorkCost(double weight, const AgentsStates& agents_init, const AgentsStates& agents_zero,
                 const geometry_msgs::msg::Pose& robot_init, const geometry_msgs::msg::Twist& robot_init_vel,
                 const double counter, unsigned int current_position, double time_step, unsigned int control_horizon,
                 unsigned int block_length);

  /**
   * @brief Creates a Ceres cost function for the SocialWorkCost.
   *
   * This function is a factory method that constructs an instance of the
   * SocialWorkFunctionType, which is a Ceres cost function
   * for computing the social work cost based on the robot's interaction with agents.
   *
   * @param weight The weight of the cost function.
   * @param agents_init A vector of AgentStatus representing the initial states of the agents.
   * @param agents_zero A vector of AgentStatus representing the zero states of the agents.
   * @param robot_init The initial pose of the robot.
   * @param robot_init_vel The initial velocity of the robot.
   * @param counter A counter value used in the computation.
   * @param current_position The current position index in the control sequence.
   * @param time_step The time step for the state update.
   * @param control_horizon The total number of time steps in the MPC.
   * @param block_length The length of the parameter block for the MPC.
   * @return A pointer to the created SocialWorkFunctionType instance.
   */

  inline static SocialWorkCostFunction* Create(double weight, const AgentsStates& agents_init,
                                               const AgentsStates& agents_zero,
                                               const geometry_msgs::msg::Pose& robot_init,
                                               const geometry_msgs::msg::Twist& robot_init_vel, const double counter,
                                               unsigned int current_position, double time_step,
                                               unsigned int control_horizon, unsigned int block_length)
  {
    return new SocialWorkCostFunction(new SocialWorkCost(weight, agents_init, agents_zero, robot_init, robot_init_vel,
                                                         counter, current_position, time_step, control_horizon,
                                                         block_length));
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
    // Compute robot social work
    Eigen::Matrix<T, 6, 3> agents = original_agents_.template cast<T>();
    Eigen::Matrix<T, 6, 1> robot;
    Eigen::Matrix<T, 6, 1> robot_initial;
    auto [new_position_x, new_position_y, new_position_orientation] = computeUpdatedStateRedux(
        robot_init_, parameters, time_step_, current_position_, control_horizon_, block_length_);
    robot(0, 0) = (T)new_position_x;            // x
    robot(1, 0) = (T)new_position_y;            // y
    robot(2, 0) = (T)new_position_orientation;  // yaw
    robot(3, 0) = (T)counter_;                  // t
    if (current_position_ < control_horizon_)
    {
      robot(4, 0) = parameters[current_position_ / block_length_][0];  // lv
      robot(5, 0) = parameters[current_position_ / block_length_][1];  // av
    }
    else
    {
      robot(4, 0) = parameters[(control_horizon_ - 1) / block_length_][0];  // lv
      robot(5, 0) = parameters[(control_horizon_ - 1) / block_length_][1];  // av
    }
    robot_initial(0, 0) = (T)robot_init_.position.x;                // x
    robot_initial(1, 0) = (T)robot_init_.position.y;                // y
    robot_initial(2, 0) = (T)tf2::getYaw(robot_init_.orientation);  // yaw
    robot_initial(3, 0) = (T)counter_;                              // t
    robot_initial(4, 0) = (T)robot_init_vel_.linear.x;              // av
    robot_initial(5, 0) = (T)robot_init_vel_.angular.z;             // lv

    Eigen::Matrix<T, 2, 1> robot_sf = computeSocialForce(robot, robot_initial, agents);
    T wr = (T)robot_sf.squaredNorm();

    // compute agents' social work provoked by the robot
    T wp = (T)0.0;
    Eigen::Matrix<T, 6, 3> robot_agent;
    Eigen::Matrix<T, 6, 3> robot_agent_initial;
    robot_agent.col(0) << robot;
    robot_agent_initial.col(0) << robot_initial;
    // we invalidate the other two agent
    // by setting t to -1

    robot_agent.col(1) << (T)0.0, (T)0.0, (T)0.0, (T)-1.0, (T)0.0, (T)0.0;
    robot_agent.col(2) << (T)0.0, (T)0.0, (T)0.0, (T)-1.0, (T)0.0, (T)0.0;
    robot_agent_initial.col(1) << (T)0.0, (T)0.0, (T)0.0, (T)-1.0, (T)0.0, (T)0.0;
    robot_agent_initial.col(2) << (T)0.0, (T)0.0, (T)0.0, (T)-1.0, (T)0.0, (T)0.0;
    for (unsigned int i = 0; i < original_agents_.cols(); i++)
    {
      Eigen::Matrix<T, 6, 1> ag;
      Eigen::Matrix<T, 6, 1> ag_initial;
      ag_initial.col(0) << original_agents_zero_.col(i).template cast<T>();
      ag.col(0) << original_agents_.col(i).template cast<T>();
      Eigen::Matrix<T, 2, 1> agent_sf = computeSocialForce(ag, ag_initial, robot_agent);
      wp += (T)agent_sf.squaredNorm();
    }
    T total_social_force_magnitude_sq = wr + wp + (T)1e-6;  // Avoid division by zero
    const T k_force_magnitude_penalty = T(8.0);  // Tune this: higher for more aggressive penalty on small forces

    // sum the social works and multiply by the weight
    residual[0] = (T)weight_ * (ceres::exp(-k_force_magnitude_penalty * total_social_force_magnitude_sq));

    return true;
  }

  template <typename T>
  Eigen::Matrix<T, 2, 1> computeSocialForce(const Eigen::Matrix<T, 6, 1>& me, const Eigen::Matrix<T, 6, 1>& me_initial,
                                            const Eigen::Matrix<T, 6, 3>& agents) const
  {
    Eigen::Matrix<T, 2, 1> meSocialforce((T)0.0, (T)0.0);
    Eigen::Matrix<T, 2, 1> mePos(me[0], me[1]);
    Eigen::Matrix<T, 2, 1> meInitialPos(me_initial[0], me_initial[1]);
    Eigen::Matrix<T, 2, 1> meVel(me[4] * ceres::cos(me[2]), me[4] * ceres::sin(me[2]));
    Eigen::Matrix<T, 2, 1> meInitialVel(me_initial[4] * ceres::cos(me_initial[2]),
                                        me_initial[4] * ceres::sin(me_initial[2]));

    for (unsigned int i = 0; i < agents.cols(); i++)
    {
      if (agents(3, i) == (T)-1.0)
        continue;

      Eigen::Matrix<T, 2, 1> aPos(agents(0, i), agents(1, i));
      Eigen::Matrix<T, 2, 1> diff = mePos - aPos;
      Eigen::Matrix<T, 2, 1> initial_diff = meInitialPos - aPos;
      Eigen::Matrix<T, 2, 1> diffDirection = diff.normalized();
      Eigen::Matrix<T, 2, 1> initial_diffDirection = initial_diff.normalized();
      Eigen::Matrix<T, 2, 1> aVel(agents(4, i) * ceres::cos(agents(2, i)), agents(4, i) * ceres::sin(agents(2, i)));
      Eigen::Matrix<T, 2, 1> velDiff = meVel - aVel;
      Eigen::Matrix<T, 2, 1> initial_velDiff = meInitialVel - aVel;
      Eigen::Matrix<T, 2, 1> interactionVector = (T)sfm_lambda_ * velDiff + diffDirection;
      Eigen::Matrix<T, 2, 1> initial_interactionVector = (T)sfm_lambda_ * initial_velDiff + initial_diffDirection;

      T interactionLength = interactionVector.norm();
      T initial_interactionLength = initial_interactionVector.norm();
      Eigen::Matrix<T, 2, 1> interactionDirection = interactionVector / interactionLength;
      Eigen::Matrix<T, 2, 1> initial_interactionDirection = initial_interactionVector / initial_interactionLength;

      T theta = wrapToPi(ceres::atan2(diffDirection[1], diffDirection[0]) -
                         ceres::atan2(interactionDirection[1], interactionDirection[0]));
      T theta_initial = wrapToPi(ceres::atan2(initial_diffDirection[1], initial_diffDirection[0]) -
                                 ceres::atan2(initial_interactionDirection[1], initial_interactionDirection[0]));

      T B = (T)sfm_gamma_ * interactionLength;
      T forceVelocityAmount =
          -(T)ceres::exp(-(T)diff.norm() / B - ((T)sfm_nPrime_ * B * theta) * ((T)sfm_nPrime_ * B * theta));

      T sign_initial = (T)0;
      if (theta_initial > (T)0)
        sign_initial = (T)1;
      else if (theta_initial < (T)0)
        sign_initial = (T)-1;

      T sign = (T)0;
      if (theta > (T)0)
        sign = (T)1;
      else if (theta < (T)0)
        sign = (T)-1;

      T forceAngleAmount =
          -sign_initial * ceres::exp(-(T)diff.norm() / B - ((T)sfm_n_ * B * theta) * ((T)sfm_n_ * B * theta)) +
          (T)0.0 * sign;

      Eigen::Matrix<T, 2, 1> forceVelocity = forceVelocityAmount * interactionDirection;
      Eigen::Matrix<T, 2, 1> leftNormalVector(-interactionDirection[1], interactionDirection[0]);
      Eigen::Matrix<T, 2, 1> forceAngle = forceAngleAmount * leftNormalVector;

      meSocialforce += (T)sfm_forceFactorSocial_ * (forceVelocity + forceAngle);
    }

    return meSocialforce;
  }

private:
  double weight_;
  Eigen::Matrix<double, 6, 3> original_agents_;
  Eigen::Matrix<double, 6, 3> original_agents_zero_;
  geometry_msgs::msg::Pose robot_init_;
  geometry_msgs::msg::Twist robot_init_vel_;
  double counter_;
  unsigned int current_position_;
  double time_step_;
  unsigned int control_horizon_;
  unsigned int block_length_;
  double sfm_lambda_;
  double sfm_gamma_;
  double sfm_nPrime_;
  double sfm_n_;
  double sfm_relaxationTime_;
  double sfm_forceFactorSocial_;
};

}  // namespace nav2_social_mpc_controller

#endif