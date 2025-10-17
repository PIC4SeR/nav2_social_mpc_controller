#ifndef MPC_BASE__UPDATE_STATE_HPP_
#define MPC_BASE__UPDATE_STATE_HPP_

#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "ceres/ceres.h"
#include "ceres/cost_function.h"
#include "ceres/cubic_interpolation.h"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "mpc_base/tools/type_definitions.hpp"


namespace mpc_base
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
  inline T wrapinsideToPi(T angle)
  {
    while (angle > T(M_PI))
      angle -= T(2.0 * M_PI);
    while (angle <= T(-M_PI))
      angle += T(2.0 * M_PI);
    return angle;
  }
template <typename T>
  Eigen::Matrix<T, 2, 1> computeinsideSocialForce(const Eigen::Matrix<T, 6, 1>& me,
                                            const Eigen::Matrix<T, 6, 3>& agents)
  {
    T sfm_lambda_ = T(2.0);
    T sfm_gamma_ = T(0.35);
    T sfm_nPrime_ = T(3.0);
    T sfm_n_ = T(2.0);
    //T sfm_relaxationTime_ = T(0.5);
    T sfm_forceFactorSocial_ = T(2.1);
    Eigen::Matrix<T, 2, 1> meSocialforce((T)0.0, (T)0.0);  // Initialize the social force vector
    Eigen::Matrix<T, 2, 1> mePos(me[0], me[1]);            // Extract the position of the robot
    Eigen::Matrix<T, 2, 1> meVel(me[4] * ceres::cos(me[2]),
                                 me[4] * ceres::sin(me[2]));  // Extract the velocity of the robot

    for (unsigned int i = 0; i < agents.cols(); i++)  // Iterate through each agent
    {
      if (agents(3, i) == (T)-1.0)  // Skip agents that are invalid (e.g., not present)
        continue;

      Eigen::Matrix<T, 2, 1> aPos(agents(0, i), agents(1, i));  // Extract the position of the agent
      Eigen::Matrix<T, 2, 1> diff =
          mePos - aPos;           // Calculate the difference in position between the robot and the agent
      if (diff.norm() < (T)1e-6)  // If the robot and agent are at the same position
      {
        diff = Eigen::Matrix<T, 2, 1>((T)1e-6, (T)0.0);  // Use a fixed small direction
      }
      Eigen::Matrix<T, 2, 1> diffDirection = diff.normalized();  // Normalize the difference vector

      Eigen::Matrix<T, 2, 1> aVel(agents(4, i) * ceres::cos(agents(2, i)),
                                  agents(4, i) * ceres::sin(agents(2, i)));  // Extract the velocity of the agent
      Eigen::Matrix<T, 2, 1> velDiff =
          meVel - aVel;  // Calculate the difference in velocity between the robot and the agent
      Eigen::Matrix<T, 2, 1> interactionVector =
          (T)sfm_lambda_ * velDiff + diffDirection;  // Calculate the interaction vector

      T interactionLength = interactionVector.norm();  // Calculate the length of the interaction vector
      Eigen::Matrix<T, 2, 1> interactionDirection =
          interactionVector / interactionLength;  // Normalize the interaction vector

      T theta = wrapinsideToPi(ceres::atan2(diffDirection[1], diffDirection[0]) -
                         ceres::atan2(interactionDirection[1],
                                      interactionDirection[0]));  // Calculate the angle between the difference
                                                                  // direction and the interaction direction

      T B = (T)sfm_gamma_ *
            interactionLength;  // Calculate the social force parameter B based on the interaction length and gamma
      T forceVelocityAmount = -(T)ceres::exp(
          -(T)diff.norm() / B -
          ((T)sfm_nPrime_ * B * theta) * ((T)sfm_nPrime_ * B * theta));  // Calculate the force velocity amount based on
                                                                         // the difference in position and the angle

      T sign = (theta > (T)0) ? (T)1 : (T)-1;  // Determine the sign of theta

      T forceAngleAmount =
          -sign * ceres::exp(-(T)diff.norm() / B -
                             ((T)sfm_n_ * B * theta) *
                                 ((T)sfm_n_ * B * theta));  // Calculate the force angle amount based on the difference
                                                            // in position, the angle, and the sign of the initial angle

      Eigen::Matrix<T, 2, 1> forceVelocity =
          forceVelocityAmount * interactionDirection;  // Calculate the force velocity vector
      Eigen::Matrix<T, 2, 1> leftNormalVector(-interactionDirection[1],
                                              interactionDirection[0]);         // Calculate the left normal vector
      Eigen::Matrix<T, 2, 1> forceAngle = forceAngleAmount * leftNormalVector;  // Calculate the force angle vector

      meSocialforce += (T)sfm_forceFactorSocial_ * (forceVelocity + forceAngle);  // Accumulate the social force
    }

    return meSocialforce;
  }
template <typename T>
Eigen::Matrix<T, 2, 1> computeinsideDesiredForce(const Eigen::Matrix<T, 6, 1>& me){
  T sfm_relaxationTime_ = T(0.5);
  Eigen::Matrix<T, 2, 1> meDesiredforce((T)0.0, (T)0.0); 
  T vx = me[4] * ceres::cos(me[2]);  // Extract the x component of the velocity
  T vy = me[4] * ceres::sin(me[2]);  //
  meDesiredforce(0,0) = -vx * sfm_relaxationTime_;  // Calculate the desired force in the x direction
  meDesiredforce(1,0) = -vy * sfm_relaxationTime_;  // Calculate the desired force in the y direction
  return meDesiredforce;

}
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

namespace detail
{

struct PoseConfig
{
  double x;
  double y;
  double z;
  double qx;
  double qy;
  double qz;
  double qw;

  static PoseConfig FromPose(const geometry_msgs::msg::Pose& pose)
  {
    return PoseConfig{pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w};
  }

  bool operator==(const PoseConfig& other) const
  {
    return x == other.x && y == other.y && z == other.z && qx == other.qx && qy == other.qy && qz == other.qz &&
           qw == other.qw;
  }
};

struct CacheKey
{
  const void* first_block;
  unsigned int blocks_used;

  bool operator==(const CacheKey& other) const
  {
    return first_block == other.first_block && blocks_used == other.blocks_used;
  }
};

struct CacheKeyHash
{
  std::size_t operator()(const CacheKey& key) const noexcept
  {
    std::size_t seed = std::hash<const void*>{}(key.first_block);
    seed ^= (std::hash<unsigned int>{}(key.blocks_used) << 1U);
    return seed;
  }
};

template <typename T>
struct UpdatedStateCacheEntry
{
  PoseConfig pose_config{};
  double dt{0.0};
  unsigned int control_horizon{0};
  unsigned int block_size{0};
  std::vector<T> flat_parameters;
  std::unordered_map<unsigned int, std::tuple<T, T, T>> states;
};

template <typename T>
class UpdatedStateCache
{
public:
  static std::tuple<T, T, T> Get(const geometry_msgs::msg::Pose& pose_0, T const* const* parameters, double dt,
                                 unsigned int step, unsigned int control_horizon, unsigned int block_size)
  {
    if (block_size == 0)
    {
      return std::make_tuple(T(pose_0.position.x), T(pose_0.position.y), T(tf2::getYaw(pose_0.orientation)));
    }

    const unsigned int blocks_used = ComputeBlocksUsed(step, control_horizon, block_size);
    const CacheKey key{static_cast<const void*>(parameters[0]), blocks_used};
    auto& entry = cache_[key];

    const PoseConfig current_pose = PoseConfig::FromPose(pose_0);
    if (entry.flat_parameters.empty() || entry.block_size != block_size || entry.control_horizon != control_horizon ||
        entry.dt != dt || !(entry.pose_config == current_pose))
    {
      entry.states.clear();
      entry.flat_parameters.clear();
      entry.pose_config = current_pose;
      entry.dt = dt;
      entry.block_size = block_size;
      entry.control_horizon = control_horizon;
    }

    const std::vector<T> current_flat = Flatten(parameters, blocks_used);
    if (ParametersChanged(entry.flat_parameters, current_flat))
    {
      entry.states.clear();
      entry.flat_parameters = current_flat;
    }

    auto cached_state = entry.states.find(step);
    if (cached_state != entry.states.end())
    {
      return cached_state->second;
    }

    auto computed = computeUpdatedStateRedux(pose_0, parameters, dt, step, control_horizon, block_size);
    entry.states.emplace(step, computed);
    return computed;
  }

private:
  static unsigned int ComputeBlocksUsed(unsigned int step, unsigned int control_horizon, unsigned int block_size)
  {
    if (block_size == 0)
    {
      return 1U;
    }

    if (control_horizon == 0)
    {
      return 1U;
    }

    const unsigned int last_horizon_step = control_horizon > 0 ? control_horizon - 1 : 0;
    const unsigned int relevant_step = std::min(step, last_horizon_step);
    return (relevant_step / block_size) + 1U;
  }

  static std::vector<T> Flatten(T const* const* parameters, unsigned int blocks_used)
  {
    std::vector<T> values;
    values.reserve(blocks_used * 2U);
    for (unsigned int block = 0; block < blocks_used; ++block)
    {
      values.push_back(parameters[block][0]);
      values.push_back(parameters[block][1]);
    }
    return values;
  }

  static bool ParametersChanged(const std::vector<T>& previous, const std::vector<T>& current)
  {
    if (previous.size() != current.size())
    {
      return true;
    }
    for (std::size_t idx = 0; idx < current.size(); ++idx)
    {
      if (previous[idx] != current[idx])
      {
        return true;
      }
    }
    return false;
  }

  static thread_local std::unordered_map<CacheKey, UpdatedStateCacheEntry<T>, CacheKeyHash> cache_;
};

template <typename T>
thread_local std::unordered_map<CacheKey, UpdatedStateCacheEntry<T>, CacheKeyHash> UpdatedStateCache<T>::cache_{};

}  // namespace detail

template <typename T>
std::tuple<T, T, T> getCachedUpdatedState(const geometry_msgs::msg::Pose& pose_0, T const* const* parameters,
                                          double dt, unsigned int step, unsigned int control_horizon,
                                          unsigned int block_size)
{
  return detail::UpdatedStateCache<T>::Get(pose_0, parameters, dt, step, control_horizon, block_size);
}

template <typename T>
std::tuple<T, T, T, Eigen::Matrix<T,6,3>> computeSFMState(const geometry_msgs::msg::Pose& pose_0,const Eigen::Matrix<T,6,3> agents_zero, T const* const* parameters,
                                             double dt, unsigned int i, unsigned int control_horizon,
                                             unsigned int block_size)
{
  T x = T(pose_0.position.x);
  T y = T(pose_0.position.y);
  T theta = T(tf2::getYaw(pose_0.orientation));
  std::vector<T> vx = { agents_zero(4,0)*ceres::cos(agents_zero(2,0)), agents_zero(4,1)*ceres::cos(agents_zero(2,1)),
     agents_zero(4,2)*ceres::cos(agents_zero(4,2)) };
  std::vector<T> vy = { agents_zero(4,0)*ceres::sin(agents_zero(2,0)), agents_zero(4,1)*ceres::sin(agents_zero(2,1)),
     agents_zero(4,2)*ceres::sin(agents_zero(2,2)) };
  Eigen::Matrix<T, 6, 3> agents_updating = agents_zero;

  // Sum the contributions of the control inputs for the first i steps.
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
    x += parameters[block_index][0] * ceres::cos(theta) * dt;
    y += parameters[block_index][0] * ceres::sin(theta) * dt;
    theta += parameters[block_index][1] * dt;
    Eigen::Matrix<T, 6, 1> robot;
    robot(0, 0) = x;  // x
    robot(1, 0) = y;  // y
    robot(2, 0) = theta;  // yaw
    robot(3, 0) = T(0);  // t
    robot(4, 0) = parameters[block_index][0];  // vx
    robot(5, 0) = parameters[block_index][1];  // vy
    Eigen::Matrix<T, 6, 3> robot_agent;
    robot_agent.col(0) << robot;  // Set the first column to the robot's current state
    // we invalidate the other two agent
    // by setting t to -1

    robot_agent.col(1) << (T)0.0, (T)0.0, (T)0.0, (T)-1.0, (T)0.0, (T)0.0;  // Set the second column to an invalid state
    robot_agent.col(2) << (T)0.0, (T)0.0, (T)0.0, (T)-1.0, (T)0.0, (T)0.0;  // Set the third column to an invalid state
    
    for (unsigned int i = 0; i < agents_updating.cols(); i++)              // Iterate through each agent
    {
      if (agents_updating(3, i) == (T)-1.0)  // Skip agents that are invalid (e.g., not present)
        continue;
      Eigen::Matrix<T, 6, 1> ag;                                // Create a matrix to hold the agent's state
      ag.col(0) << agents_updating.col(i); // Set the current state of the agent
      Eigen::Matrix<T, 2, 1> agent_sf = computeinsideSocialForce(ag, robot_agent);  // Compute social force on agent
      Eigen::Matrix<T, 2, 1> agent_df = computeinsideDesiredForce(ag); // Normalize the social force vector
      Eigen::Matrix<T, 2, 1> agent_gf = agent_sf + agent_df; // Normalize the social force vector
      vx[i] += agent_gf(0)*dt;
      vy[i] += agent_gf(1)*dt;
      T init_yaw = ag(2, 0);
      ag(0, 0) += vx[i] * dt; // Update the agent's x position based on the social force
      ag(1, 0) += vy[i] * dt; // Update the agent's y position based on the social force
      ag(2, 0)  = ceres::atan2(vy[i], vx[i]); // Update the agent's orientation based on the new position
      ag(3, 0) = T(0); // Reset time to 0 for the agent
      ag(4, 0) = ceres::sqrt(vx[i]*vx[i] + vy[i]*vy[i]); // Update the agent's linear velocity (norm of velocity vector)
      ag(5, 0) = (ceres::atan2(vy[i], vx[i])- init_yaw) / dt; // Update the agent's angular velocity based on the new orientation
      agents_updating.col(i) = ag; // Update the agent's state in the matrix
    }
  }

  return std::make_tuple(x, y, theta, agents_updating);
}  // namespace mpc_base



}

#endif  // MPC_HPP