#ifndef MPC_ENLARGED_STATE__UPDATE_STATE_HPP_
#define MPC_ENLARGED_STATE__UPDATE_STATE_HPP_

#include <Eigen/Core>
#include <algorithm>
#include <vector>

#include "ceres/ceres.h"
#include "ceres/cost_function.h"
#include "ceres/cubic_interpolation.h"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "mpc_enlarged_state/tools/type_definitions.hpp"

namespace mpc_enlarged_state
{

/**
 * @brief Computes the updated state of a robot by integrating control inputs over time.
 *
 * This function updates the robot's state by iteratively applying the control inputs to be optimized defined
 * in a 2D array of parameters. For each time step up to the specified index, the function integrates
 * the linear and angular velocities to update the robot's (x, y) position and orientation (theta).
 * If the current time step i exceeds the control horizon, it repeatedly applies the final available
 * control input.
 *
 * @param T The numerical type used for state calculations (e.g., float, double).
 * @param pose_0 The initial pose of the robot, including its position (x, y, z) and orientation.
 * @param parameters A 2D array of control inputs, where each sub-array contains two elements:
 *                   the linear velocity (vx) and angular velocity (wz) for a control block.
 * @param dt The time step duration.
 * @param i The current time step index.
 * @param control_horizon The total number of defined time steps for the control inputs.
 * @param block_size The number of time steps in each block of control inputs.
 * @return A std::tuple containing the updated x coordinate, y coordinate, and orientation (theta).
 */
template <typename T>
std::tuple<T, T, T> computeUpdatedStateRedux(const geometry_msgs::msg::Pose& pose_0, T const* const* parameters,
                                             double dt, unsigned int i, unsigned int control_horizon,
                                             unsigned int block_size)
{
  T x = T(pose_0.position.x);
  T y = T(pose_0.position.y);
  T theta = T(tf2::getYaw(pose_0.orientation)); 
  // Sum the contributions of the control inputs for the first i steps.
  for (unsigned int j = 0; j <= i; j++)
  {
    if (j < control_horizon)
    {
      unsigned int block_index = j / block_size;
      x += parameters[block_index][0] * ceres::cos(theta) * dt;
      y += parameters[block_index][0] * ceres::sin(theta) * dt;
      theta += parameters[block_index][1] * dt;
    }
    else
    {
      x += parameters[(control_horizon - 1) / block_size][0] * ceres::cos(theta) * dt;
      y += parameters[(control_horizon - 1) / block_size][0] * ceres::sin(theta) * dt;
      theta += parameters[(control_horizon - 1) / block_size][1] * dt;
    }
  }
  return std::make_tuple(x, y, theta);


}
template <typename T>
std::tuple<T,T,T,std::vector<T>,std::vector<T>,std::vector<T>> computeAgentandRobotState(const geometry_msgs::msg::Pose& pose_0,const AgentsStates& agents, T const* const* parameters,
                                             long unsigned int num_agents, double dt, unsigned int i, unsigned int control_horizon,
                                             unsigned int block_size)
{
  T x = T(pose_0.position.x);
  T y = T(pose_0.position.y);
  T theta = T(tf2::getYaw(pose_0.orientation));
  const auto tracked_agents = std::min(static_cast<size_t>(num_agents), agents.size());
  std::vector<T> agent_x(tracked_agents, T(0.0));
  std::vector<T> agent_y(tracked_agents, T(0.0));
  std::vector<T> agent_theta(tracked_agents, T(0.0));
  for (size_t idx = 0; idx < tracked_agents; ++idx)
  {
    agent_x[idx] = T(agents[idx](0, 0));
    agent_y[idx] = T(agents[idx](1, 0));
    agent_theta[idx] = T(agents[idx](2, 0));
  }

  // Sum the contributions of the control inputs for the first i steps.
  for (unsigned int j = 0; j <= i; j++)
  {
    if (j < control_horizon)
    {
      unsigned int block_index = j / block_size;
      x += parameters[block_index][0] * ceres::cos(theta) * dt;
      y += parameters[block_index][0] * ceres::sin(theta) * dt;
      theta += parameters[block_index][1] * dt;
      // Check the number of agents and update their states accordingly
    
  for (size_t k = 0; k < tracked_agents; ++k) {
      // param index for agent k:
      unsigned idx = 2 + 2*k;
      T vx = parameters[block_index][idx];
      T vy = parameters[block_index][idx + 1];

      agent_x[k] += vx * dt;
      agent_y[k] += vy * dt;
      agent_theta[k] = ceres::atan2(vy, vx);
      }
    }
    else
    {
      x += parameters[(control_horizon - 1) / block_size][0] * ceres::cos(theta) * dt;
      y += parameters[(control_horizon - 1) / block_size][0] * ceres::sin(theta) * dt;
      theta += parameters[(control_horizon - 1) / block_size][1] * dt;
  for (size_t k = 0; k < tracked_agents; ++k) {
      // param index for agent k:
      unsigned idx = 2 + 2*k;
  T vx = parameters[(control_horizon-1)/block_size][idx];
  T vy = parameters[(control_horizon-1)/block_size][idx + 1];

  agent_x[k]     += vx * dt;
  agent_y[k]     += vy * dt;
  agent_theta[k]  = ceres::atan2(vy, vx);
      }
    }
  }
  return std::make_tuple(x, y, theta, agent_x, agent_y, agent_theta);
  

}
}  // namespace mpc_enlarged_state

#endif  // MPC_HPP