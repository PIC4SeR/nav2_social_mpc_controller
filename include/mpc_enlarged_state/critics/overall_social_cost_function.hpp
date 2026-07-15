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

#ifndef MPC_ENLARGED_STATE__OVERALL_SOCIAL_COST_FUNCTION_HPP_
#define MPC_ENLARGED_STATE__OVERALL_SOCIAL_COST_FUNCTION_HPP_

#include <algorithm>
#include <limits>
#include "Eigen/Core"
#include "ceres/ceres.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "glog/logging.h"
#include "mpc_enlarged_state/update_state.hpp"
#include "mpc_enlarged_state/critics/social_work_cost_function.hpp"
#include "mpc_enlarged_state/tools/type_definitions.hpp"

namespace mpc_enlarged_state
{

class SocialOverallCost
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
  using SocialOverallCostFunction = ceres::DynamicAutoDiffCostFunction<SocialOverallCost, /*Jet stride=*/16>;

  SocialOverallCost(double work_weight,double angle_weight, double proxemics_weight, 
                double path_follow_weight, double path_align_weight, const Eigen::Matrix<double,2,1> final_point,
                const Eigen::Matrix<double,2,1> point,
                const AgentsStates& agents_init, long unsigned int agent_index, const geometry_msgs::msg::Pose& robot_init,
                  unsigned int current_position, double time_step, unsigned int control_horizon,
                 unsigned int block_length, unsigned int parameter_block_count, bool has_agent_parameters,
                 bool found_people, bool use_social_work_cost, bool use_angle_cost, 
                 bool use_proxemics_cost, bool use_path_follow_cost, bool use_path_align_cost);

  /**
   * @brief Creates a Ceres cost function for the SocialWorkCost.
   *
   * This function is a factory method that constructs an instance of the
   * SocialWorkFunctionType, which is a Ceres cost function
   * for computing the social work cost based on the robot's interaction with agents.
   *
   * @param weight The weight of the cost function.
   * @param agents_init A vector of AgentStatus representing the initial states of the agents.
   * @param robot_init The initial pose of the robot.
   * @param counter A counter value used in the computation.
   * @param current_position The current position index in the control sequence.
   * @param time_step The time step for the state update.
   * @param control_horizon The total number of time steps in the MPC.
   * @param block_length The length of the parameter block for the MPC.
   * @return A pointer to the created SocialWorkFunctionType instance.
   */

  inline static SocialOverallCostFunction* Create(double work_weight,double angle_weight, double proxemics_weight,
                double path_follow_weight, double path_align_weight, const Eigen::Matrix<double, 2, 1> final_point,
                const Eigen::Matrix<double, 2, 1> point,
                 const AgentsStates& agents_init, long unsigned int agent_index, const geometry_msgs::msg::Pose& robot_init,
                 unsigned int current_position, double time_step, unsigned int control_horizon,
                 unsigned int block_length, unsigned int parameter_block_count, bool has_agent_parameters,
                 bool found_people, bool use_social_work_cost, bool use_angle_cost, bool use_proxemics_cost,
                bool use_path_follow_cost, bool use_path_align_cost)
  {
    return new SocialOverallCostFunction(new SocialOverallCost(work_weight, angle_weight, proxemics_weight, 
                                                          path_follow_weight, path_align_weight, final_point,
                                                          point,
                                                          agents_init, agent_index,robot_init, current_position,
                                                         time_step, control_horizon, block_length, parameter_block_count,
                                                         has_agent_parameters, found_people, use_social_work_cost, 
                                                         use_angle_cost, use_proxemics_cost,
                                                        use_path_follow_cost, use_path_align_cost));
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
  bool operator()(T const* const* parameters, T* residuals) const
  {
    // Compute robot social work
    Eigen::Matrix<T, kStateSize, Eigen::Dynamic> agents = original_agents_.template cast<T>();
    Eigen::Matrix<T, kStateSize, 1> robot;
    auto [new_position_x, new_position_y, new_position_orientation, agent_x, agent_y, agent_theta] = computeAgentandRobotState(
        robot_init_, agents_init_, parameters, parameter_block_count_, has_agent_parameters_, agent_index_, time_step_,
        current_position_, control_horizon_, block_length_);
    robot(kStateX, 0) = (T)new_position_x;                // x
    robot(kStateY, 0) = (T)new_position_y;                // y
    robot(kStateYaw, 0) = (T)new_position_orientation;    // yaw
    robot(kStateTime, 0) = (T)current_position_*time_step_;  // t
    unsigned int block_idx = (current_position_ < control_horizon_) ?
        (current_position_ / block_length_) :
        ((control_horizon_ - 1) / block_length_);
    block_idx = std::min(block_idx, parameter_block_count_ - 1);
    robot(kStateLinearVelocity, 0) = parameters[block_idx][kRobotLinearVelocityParam];   // lv
    robot(kStateAngularVelocity, 0) = parameters[block_idx][kRobotAngularVelocityParam];  // av
  const size_t total_agents = static_cast<size_t>(agents.cols());
  const size_t capped_agents = std::min(static_cast<size_t>(agent_index_), total_agents);
  const bool has_valid_agents =
      std::any_of(agents_init_.begin(), agents_init_.end(),
                  [](const AgentStatus& st) { return st[kStateTime] != -1.0; }) &&
      capped_agents > 0;
  const bool social_terms_active = found_people_ && has_valid_agents;

    if (social_terms_active) {
      for (size_t k = 0; k < capped_agents; ++k) {
        const Eigen::Index agent_col = static_cast<Eigen::Index>(k);
        if (agents(kStateTime, agent_col) == (T)-1.0)
        {
          continue;
        }

        agents(kStateX, agent_col) = agent_x[k];
        agents(kStateY, agent_col) = agent_y[k];
        agents(kStateYaw, agent_col) = agent_theta[k];
        agents(kStateTime, agent_col) = (T)current_position_ * time_step_;

        unsigned block_idx = (current_position_ < control_horizon_) ?
            (current_position_ / block_length_) :
            ((control_horizon_ - 1) / block_length_);
        block_idx = std::min(block_idx, parameter_block_count_ - 1);
        unsigned param_offset = static_cast<unsigned>(kAgentVelocityParamStride * k);
        const T* agent_block = has_agent_parameters_ ? parameters[parameter_block_count_ + block_idx] : nullptr;
        T vx = agent_block ? agent_block[param_offset + kAgentVxParam] : T(0.0);
        T vy = agent_block ? agent_block[param_offset + kAgentVyParam] : T(0.0);
        agents(kStateLinearVelocity, agent_col) = ceres::sqrt(vx * vx + vy * vy);
        agents(kStateAngularVelocity, agent_col) = T(0.0);
      }
    }
    residuals[kSocialWorkResidual] = (T)0.0;
    residuals[kSocialAngleResidual] = (T)0.0;
    residuals[kProxemicsResidual] = (T)0.0;
    residuals[kPathFollowResidual] = (T)0.0;
    residuals[kPathAlignResidual] = (T)0.0;

  if (social_terms_active && use_social_work_cost_) {
    residuals[kSocialWorkResidual] += usingSocialWork(robot, agents);
    }
    if (social_terms_active && use_angle_cost_) {
    residuals[kSocialAngleResidual] += usingAngle(robot, robot_init_, agents);
    }
    if (social_terms_active && use_proxemics_cost_) {
    residuals[kProxemicsResidual] += usingProxemics(robot, agents);
    }
    if (use_path_follow_cost_ == true){
      Eigen::Matrix<T, 2, 1> p((T)new_position_x, (T)new_position_y);
      Eigen::Matrix<T, 2, 1> p_target((T)final_point_[kX], (T)final_point_[kY]);
      residuals[kPathFollowResidual] =
          T(path_follow_weight_) * (p - p_target).squaredNorm() * (p - p_target).squaredNorm();
    }
    if (use_path_align_cost_ == true) {
      Eigen::Matrix<T, 2, 1> p((T)new_position_x, (T)new_position_y);
      Eigen::Matrix<T, 2, 1> p_target((T)point_[kX], (T)point_[kY]);
      residuals[kPathAlignResidual] =
          T(path_align_weight_) * (p - p_target).squaredNorm() * (p - p_target).squaredNorm();
    }
    return true;
  }

  /**
   * @brief This function computes the social force acting on the robot based on its position,
   * and the positions of other agents.
   * It calculates the interaction between the robot and other agents, taking into account their velocities and
   * the social force parameters.
   *
   * @tparam T
   * @param me the current state of the robot, including position, orientation, and velocity
   * @param agents the states of other agents in the environment, including their positions, orientations, and
   * velocities
   * @return Eigen::Matrix<T, 2, 1> the computed social force acting on the robot
   */
  template <typename T>
  T usingSocialWork(const Eigen::Matrix<T, kStateSize, 1> robot,
                    const Eigen::Matrix<T, kStateSize, Eigen::Dynamic>& agents) const
  {
    if (agents.cols() == 0)
    {
      return (T)0.0;
    }

    Eigen::Matrix<T, 2, 1> robot_sf = computeSocialForce(robot, agents);  // Compute social force on robot
    T wr = (T)robot_sf.squaredNorm();  // Compute the squared norm of the social force on the robot

    // compute agents' social work provoked by the robot
    T wp = (T)0.0;
    Eigen::Matrix<T, kStateSize, Eigen::Dynamic> robot_agent(kStateSize, 1);
    robot_agent.col(0) = robot;  // Robot reference for social force on agents
    for (Eigen::Index i = 0; i < agents.cols(); i++)              // Iterate through each agent
    {
      Eigen::Matrix<T, kStateSize, 1> ag = agents.col(i);
      Eigen::Matrix<T, 2, 1> agent_sf = computeSocialForce(ag, robot_agent);  // Compute social force on agent
      wp += (T)agent_sf.squaredNorm();  // Accumulate the squared norm of the social force on the agent
    }
    T total_social_force_magnitude_sq = wr + wp + (T)1e-6;  // Avoid division by zero

    return (T)total_social_force_magnitude_sq * (T)work_weight_;  // Return the total social force magnitude scaled by weight
  }
  template <typename T>
  T usingProxemics(const Eigen::Matrix<T, kStateSize, 1> robot,
                   const Eigen::Matrix<T, kStateSize, Eigen::Dynamic>& agents) const
  {

    if (agents.cols() == 0)
    {
      return (T)0.0;
    }

    T proxemics_cost = computeProxemics(robot, agents);  // Compute proxemics cost on robot
    return (T)proxemics_weight_ * proxemics_cost;           // Scale the proxemics cost by the weight
  }
  template <typename T>
  T usingAngle(const Eigen::Matrix<T, kStateSize, 1> robot,const geometry_msgs::msg::Pose robot_init_,
               const Eigen::Matrix<T, kStateSize, Eigen::Dynamic>& agents)
   const
  {
    if (agents.cols() == 0)
    {
      return T(0.0);
    }

    int closest_index = -1;
    T closest_distance_squared = T(9999.0);
  for (Eigen::Index i = 0; i < agents.cols(); i++)
    {
      T dx = agents(kStateX,i) - T(robot_init_.position.x);
      T dy = agents(kStateY,i) - T(robot_init_.position.y);
      T distance_squared = dx * dx + dy * dy;
      if (distance_squared < closest_distance_squared && agents(kStateLinearVelocity,i) > T(0.05))
      {
        closest_distance_squared = distance_squared;
        closest_index = i;
      }
    }
    if (closest_index < 0 || closest_distance_squared > safe_distance_squared_)
    {
      return T(0.0);  // If no valid agent is found, return zero cost
    }
  Eigen::Matrix<T, kStateSize, 1> agent = agents.col(closest_index);
    // Compute angles.
    T agent_angle_initial =
        ceres::atan2(T(agent[kStateY] - robot_init_.position.y), T(agent[kStateX] - robot_init_.position.x));
    T robot_yaw = T(tf2::getYaw(robot_init_.orientation));
    // Difference between agent's heading and the robot's initial orientation.
    T agent_heading_diff =
        ceres::atan2(ceres::sin(T(agent[kStateYaw]) - robot_yaw), ceres::cos(T(agent[kStateYaw]) - robot_yaw));
    // Helper to wrap angles into [-pi, pi].
    auto wrapAngle = [](const T& angle) -> T { return ceres::atan2(ceres::sin(angle), ceres::cos(angle)); };

    // --- Define angular thresholds and offsets ---
    // Thresholds.
    const T kThreshold = T(M_PI / 6.0);
    const T kUpperThreshold = T(5 * M_PI / 6.0);
    const T steering_right = -T(M_PI / 6.0);
    const T steering_left = T(M_PI / 6.0);
    T angular_diff;

    if (agent_heading_diff <= -kUpperThreshold || agent_heading_diff >= kThreshold)
    {
      if (wrapAngle(agent_angle_initial - robot_yaw) < 0.0)
      {
        return T(0.0);
      }
      else
      {
        angular_diff = wrapAngle(robot(kStateYaw,0) - (robot_yaw + steering_right));
      }
    }

    else
    {
      if (wrapAngle(agent_angle_initial - robot_yaw) > 0.0)
      {
        return T(0.0);
      }
      else
      {
        angular_diff = wrapAngle(robot(kStateYaw,0)- (robot_yaw + steering_left));
      }
    }
    T cost = angular_diff * angular_diff;
    return (T)angle_weight_ * cost;
  }
  template <typename T>
  Eigen::Matrix<T, 2, 1> computeSocialForce(const Eigen::Matrix<T, kStateSize, 1>& me,
                                            const Eigen::Matrix<T, kStateSize, Eigen::Dynamic>& agents) const
  {
    Eigen::Matrix<T, 2, 1> meSocialforce((T)0.0, (T)0.0);  // Initialize the social force vector
    Eigen::Matrix<T, 2, 1> mePos(me[kStateX], me[kStateY]);  // Extract the position of the robot
    Eigen::Matrix<T, 2, 1> meVel(me[kStateLinearVelocity] * ceres::cos(me[kStateYaw]),
                                 me[kStateLinearVelocity] * ceres::sin(me[kStateYaw]));  // Extract velocity

  for (Eigen::Index i = 0; i < agents.cols(); i++)  // Iterate through each agent
    {
      if (agents(kStateTime, i) == (T)-1.0)  // Skip agents that are invalid (e.g., not present)
        continue;

      Eigen::Matrix<T, 2, 1> aPos(agents(kStateX, i), agents(kStateY, i));  // Extract the position of the agent
      Eigen::Matrix<T, 2, 1> diff =
          mePos - aPos;           // Calculate the difference in position between the robot and the agent
      if (diff.norm() < (T)1e-6)  // If the robot and agent are at the same position
      {
        diff = Eigen::Matrix<T, 2, 1>((T)1e-6, (T)0.0);  // Use a fixed small direction
      }
      Eigen::Matrix<T, 2, 1> diffDirection = diff.normalized();  // Normalize the difference vector

  T agent_speed = agents(kStateLinearVelocity, i);
  Eigen::Matrix<T, 2, 1> aVel(agent_speed * ceres::cos(agents(kStateYaw, i)),
              agent_speed * ceres::sin(agents(kStateYaw, i)));  // Extract the velocity of the agent
      Eigen::Matrix<T, 2, 1> velDiff =
          meVel - aVel;  // Calculate the difference in velocity between the robot and the agent
      Eigen::Matrix<T, 2, 1> interactionVector =
          (T)sfm_lambda_ * velDiff + diffDirection;  // Calculate the interaction vector

      T interactionLength = interactionVector.norm();  // Calculate the length of the interaction vector
      Eigen::Matrix<T, 2, 1> interactionDirection =
          interactionVector / interactionLength;  // Normalize the interaction vector

      T theta = wrapToPi(ceres::atan2(diffDirection[kY], diffDirection[kX]) -
                         ceres::atan2(interactionDirection[kY],
                                      interactionDirection[kX]));  // Calculate the angle between the difference
                                                                  // direction and the interaction direction

      T B = (T)sfm_gamma_ *
            interactionLength;  // Calculate the social force parameter B based on the interaction length and gamma
      T forceVelocityAmount = (T)ceres::exp(
          -(T)diff.norm() / B -
          ((T)sfm_nPrime_ * B * theta) * ((T)sfm_nPrime_ * B * theta));  // Calculate the force velocity amount based on
                                                                         // the difference in position and the angle

      T sign = (theta > (T)0) ? (T)1 : (T)-1;  // Determine the sign of theta

      T forceAngleAmount =
          sign * ceres::exp(-(T)diff.norm() / B -
                             ((T)sfm_n_ * B * theta) *
                                 ((T)sfm_n_ * B * theta));  // Calculate the force angle amount based on the difference
                                                            // in position, the angle, and the sign of the initial angle

      Eigen::Matrix<T, 2, 1> forceVelocity =
          forceVelocityAmount * interactionDirection;  // Calculate the force velocity vector
      Eigen::Matrix<T, 2, 1> leftNormalVector(-interactionDirection[kY],
                                              interactionDirection[kX]);        // Calculate the left normal vector
      Eigen::Matrix<T, 2, 1> forceAngle = forceAngleAmount * leftNormalVector;  // Calculate the force angle vector

      meSocialforce += (T)sfm_forceFactorSocial_ * (forceVelocity + forceAngle);  // Accumulate the social force
    }

    return meSocialforce;
  }
  template <typename T>
  T computeProxemics(const Eigen::Matrix<T, kStateSize, 1>& me,
                     const Eigen::Matrix<T, kStateSize, Eigen::Dynamic>& agents) const
  {
    T min_distance(1000.0);  // Initialize minimum distance to a large value
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
      //std::cout << "squared_distance: " << squared_distance << std::endl;
      if (squared_distance < 1e-6)              // If the squared distance is too small, set a fixed direction
      {
        diff = Eigen::Matrix<T, 2, 1>((T)1e-6, (T)0.0);  // Use a fixed small direction
      }
      min_distance = std::min(min_distance, squared_distance);  // Update the minimum distance
    }
    T proxemics_cost =
        (T)alpha_ * ceres::exp(-min_distance / ((T)d0_ * (T)d0_));  // Exponential decay based on distance
    //std::cout << "min_distance: " << min_distance << std::endl;
    return proxemics_cost;
  }

private:
  double work_weight_;
  double angle_weight_;
  double proxemics_weight_;
  double path_follow_weight_;
  double path_align_weight_;
  Eigen::Matrix<double, kStateSize, Eigen::Dynamic> original_agents_;
  Eigen::Matrix<double, 2, 1> final_point_;  // Target point for
  Eigen::Matrix<double, 2, 1> point_;        // Point for path alignment
  AgentsStates agents_init_;  // Initial states of the agents
  long unsigned int agent_index_;  // Index of the agent being considered
  geometry_msgs::msg::Pose robot_init_;
  unsigned int current_position_;
  double time_step_;
  unsigned int control_horizon_;
  unsigned int block_length_;
  unsigned int parameter_block_count_;
  bool has_agent_parameters_;
  double sfm_lambda_;
  double sfm_gamma_;
  double sfm_nPrime_;
  double sfm_n_;
  double sfm_relaxationTime_;
  double sfm_forceFactorSocial_;
  double d0_;
  double alpha_;
  double safe_distance_squared_;
  bool found_people_;
  bool use_social_work_cost_;
  bool use_angle_cost_;
  bool use_proxemics_cost_;
  bool use_path_follow_cost_;
  bool use_path_align_cost_;  // Default value, can be set later if needed
  
};

}  // namespace mpc_enlarged_state

#endif
