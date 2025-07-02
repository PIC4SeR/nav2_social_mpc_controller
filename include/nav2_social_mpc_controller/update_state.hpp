#ifndef NAV2_SOCIAL_MPC_CONTROLLER__UPDATE_STATE_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__UPDATE_STATE_HPP_

#include <Eigen/Core>
#include <vector>

#include "ceres/ceres.h"
#include "ceres/cost_function.h"
#include "ceres/cubic_interpolation.h"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_social_mpc_controller/tools/type_definitions.hpp"

namespace nav2_social_mpc_controller
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
  //T agent_x_1 = T(agent(0,0));
  //T agent_y_1 = T(agent(1,0));
  //T agent_theta_1 = T(agent(2,0));
  //T agent_x_2 = T(agent(0,1));
  //T agent_y_2 = T(agent(1,1));
  //T agent_theta_2 = T(agent(2,1));
  //T agent_x_3 = T(agent(0,2));
  //T agent_y_3 = T(agent(1,2));
  //T agent_theta_3 = T(agent(2,2));
  std::array<T,3> agent_x     = { T(agents[0](0,0)), T(agents[1](0,0)), T(agents[2](0,0)) };
  std::array<T,3> agent_y     = { T(agents[0](1,0)), T(agents[1](1,0)), T(agents[2](1,0)) };
  std::array<T,3> agent_theta = { T(agents[0](2,0)), T(agents[1](2,0)), T(agents[2](2,0)) };
  //T agent_theta = T(agents[0](2,0));
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
    
      for (unsigned k = 0; k < num_agents; ++k) { 
      // param index for agent k:
      unsigned idx = 2 + 2*k;
      T vx = parameters[block_index][idx];
      T vy = parameters[block_index][idx + 1];

      agent_x[k] += vx * dt;
      agent_y[k] += vy * dt;
      agent_theta[k] = ceres::atan2(vy, vx);
      }
    }
      //  agent_x_1 += parameters[block_index][2] * ceres::cos(agent_theta_1) * dt;
      //  agent_y_1 += parameters[block_index][2] * ceres::sin(agent_theta_1) * dt;
      //  agent_theta_1 += parameters[block_index][3] * dt;
      //}
      //if (agents.cols() > 1) {
      //  agent_x_2 += parameters[block_index][4] * ceres::cos(agent_theta_2) * dt;
      //  agent_y_2 += parameters[block_index][4] * ceres::sin(agent_theta_2) * dt;
      //  agent_theta_2 += parameters[block_index][5] * dt;
      //}
      //if (agents.cols() > 2) {
      //  agent_x_3 += parameters[block_index][6] * ceres::cos(agent_theta_3) * dt;
      //  agent_y_3 += parameters[block_index][6] * ceres::sin(agent_theta_3) * dt;
      //  agent_theta_3 += parameters[block_index][7] * dt;
      //}
      // Update agent_theta based on agent's velocity along x and y
      //agent_theta = ceres::atan2(parameters[block_index][3], parameters[block_index][2]);
    else
    {
      x += parameters[(control_horizon - 1) / block_size][0] * ceres::cos(theta) * dt;
      y += parameters[(control_horizon - 1) / block_size][0] * ceres::sin(theta) * dt;
      theta += parameters[(control_horizon - 1) / block_size][1] * dt;
      for (unsigned k = 0; k < num_agents; ++k) { 
      // param index for agent k:
      unsigned idx = 2 + 2*k;
      T speed   = parameters[(control_horizon-1)/block_size][idx];
      T omega   = parameters[(control_horizon-1)/block_size][idx + 1];

      agent_x[k]     += speed * ceres::cos(agent_theta[k]) * dt;
      agent_y[k]     += speed * ceres::sin(agent_theta[k]) * dt;
      agent_theta[k] += omega * dt;
      }
      //agent_x += parameters[(control_horizon - 1) / block_size][2] * ceres::cos(agent_theta) * dt;
      //agent_y += parameters[(control_horizon - 1) / block_size][2] * ceres::sin(agent_theta) * dt;
      //agent_theta += parameters[(control_horizon - 1) / block_size][3] * dt;
      //agent_theta = ceres::atan2(parameters[(control_horizon - 1) / block_size][3], parameters[(control_horizon - 1) / block_size][2]);
    }
  }
  std::vector<T> x_(agent_x.begin(), agent_x.end());
  std::vector<T> y_(agent_y.begin(), agent_y.end());
  std::vector<T> theta_(agent_theta.begin(), agent_theta.end());

  return std::make_tuple(x, y, theta, x_, y_, theta_);
  

}
}  // namespace nav2_social_mpc_controller

#endif  // MPC_HPP