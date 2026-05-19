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
std::tuple<T, T, T, std::vector<T>, std::vector<T>, std::vector<T>> computeAgentandRobotState(
    const geometry_msgs::msg::Pose& pose_0, const AgentsStates& agents, T const* const* parameters,
    unsigned int robot_block_count, bool has_agent_blocks, long unsigned int num_agents, double dt, unsigned int i,
    unsigned int control_horizon, unsigned int block_size)
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
  const T* const* robot_blocks = parameters;
  const T* const* agent_blocks = has_agent_blocks ? parameters + robot_block_count : nullptr;
  const unsigned int max_robot_index = robot_block_count > 0 ? robot_block_count - 1 : 0;
  const unsigned int agent_stride = static_cast<unsigned int>(2 * num_agents);

  for (unsigned int j = 0; j <= i; j++)
  {
    unsigned int block_index;
    if (j < control_horizon)
    {
      block_index = j / block_size;
    }
    else
    {
      block_index = (control_horizon - 1) / block_size;
    }
    block_index = std::min(block_index, max_robot_index);

    const T* robot_block = robot_blocks[block_index];
    x += robot_block[0] * ceres::cos(theta) * dt;
    y += robot_block[0] * ceres::sin(theta) * dt;
    theta += robot_block[1] * dt;

    if (agent_blocks != nullptr && agent_stride > 0)
    {
      const T* agent_block = agent_blocks[block_index];
      for (size_t k = 0; k < tracked_agents; ++k)
      {
        unsigned int idx = static_cast<unsigned int>(2 * k);
        T vx = agent_block[idx];
        T vy = agent_block[idx + 1];
        agent_x[k] += vx * dt;
        agent_y[k] += vy * dt;
        agent_theta[k] = ceres::atan2(vy, vx);
      }
    }
  }
  return std::make_tuple(x, y, theta, agent_x, agent_y, agent_theta);
  

}

template <typename T>
Eigen::Matrix<T, 6, Eigen::Dynamic> agentsToMatrix(const AgentsStates& agents)
{
  Eigen::Matrix<T, 6, Eigen::Dynamic> matrix(6, static_cast<Eigen::Index>(agents.size()));
  for (size_t idx = 0; idx < agents.size(); ++idx)
  {
    matrix.col(static_cast<Eigen::Index>(idx)) = agents[idx].template cast<T>();
  }
  return matrix;
}

template <typename T>
std::tuple<T, T, T, Eigen::Matrix<T, 6, Eigen::Dynamic>> computeEnlargedState(
    const geometry_msgs::msg::Pose& pose_0, const AgentsStates& agents, T const* const* parameters,
    unsigned int robot_block_count, bool has_agent_blocks, unsigned int num_agents, double dt, unsigned int i,
    unsigned int control_horizon, unsigned int block_size)
{
  auto agents_matrix = agentsToMatrix<T>(agents);
  if (robot_block_count == 0)
  {
    auto [x, y, theta] = computeUpdatedStateRedux(pose_0, parameters, dt, i, control_horizon, block_size);
    return std::make_tuple(x, y, theta, agents_matrix);
  }

  auto [x, y, theta, agent_x, agent_y, agent_theta] =
      computeAgentandRobotState(pose_0, agents, parameters, robot_block_count, has_agent_blocks, num_agents, dt, i,
                                control_horizon, block_size);

  const size_t tracked_agents = std::min(static_cast<size_t>(num_agents), agents.size());
  unsigned int block_idx = 0;
  if (control_horizon > 0 && block_size > 0)
  {
    block_idx = i < control_horizon ? i / block_size : (control_horizon - 1) / block_size;
  }
  block_idx = std::min(block_idx, robot_block_count - 1);
  const T* agent_block = has_agent_blocks ? parameters[robot_block_count + block_idx] : nullptr;

  for (size_t k = 0; k < tracked_agents; ++k)
  {
    const Eigen::Index col = static_cast<Eigen::Index>(k);
    if (agents_matrix(3, col) == T(-1.0))
    {
      continue;
    }
    agents_matrix(0, col) = agent_x[k];
    agents_matrix(1, col) = agent_y[k];
    agents_matrix(2, col) = agent_theta[k];
    agents_matrix(3, col) = T(i) * T(dt);
    if (agent_block != nullptr)
    {
      const unsigned int idx = static_cast<unsigned int>(2 * k);
      const T vx = agent_block[idx];
      const T vy = agent_block[idx + 1];
      agents_matrix(4, col) = ceres::sqrt(vx * vx + vy * vy);
      agents_matrix(5, col) = T(0.0);
    }
  }

  return std::make_tuple(x, y, theta, agents_matrix);
}
}  // namespace mpc_enlarged_state

#endif  // MPC_HPP
