#ifndef MPC_ENLARGED_STATE__TYPE_DEFINITIONS_HPP_
#define MPC_ENLARGED_STATE__TYPE_DEFINITIONS_HPP_

#include <Eigen/Core>   
#include <ceres/ceres.h>
#include <vector>

namespace mpc_enlarged_state
{

inline constexpr int kX = 0;
inline constexpr int kY = 1;
inline constexpr int kPositionParameterBlockSize = 2;

inline constexpr int kStateX = 0;
inline constexpr int kStateY = 1;
inline constexpr int kStateYaw = 2;
inline constexpr int kStateTime = 3;
inline constexpr int kStateLinearVelocity = 4;
inline constexpr int kStateAngularVelocity = 5;
inline constexpr int kStateSize = 6;

inline constexpr int kRobotLinearVelocityParam = 0;
inline constexpr int kRobotAngularVelocityParam = 1;
inline constexpr int kRobotParameterBlockSize = 2;
inline constexpr int kPoseAndVelocityParameterBlockSize = 4;

inline constexpr int kScalarParam = 0;
inline constexpr int kScalarParameterBlockSize = 1;

inline constexpr int kHeadingTimeParam = 0;
inline constexpr int kHeadingYawParam = 1;
inline constexpr int kHeadingParameterBlockSize = 2;

inline constexpr int kAgentVelocityParamStride = 2;
inline constexpr int kAgentVxParam = 0;
inline constexpr int kAgentVyParam = 1;

inline constexpr int kCurrentVelocityBlock = 0;
inline constexpr int kPreviousVelocityBlock = 1;

inline constexpr int kSocialWorkResidual = 0;
inline constexpr int kSocialAngleResidual = 1;
inline constexpr int kProxemicsResidual = 2;
inline constexpr int kPathFollowResidual = 3;
inline constexpr int kPathAlignResidual = 4;

using AgentStatus = Eigen::Matrix<double, kStateSize, 1>;  // x, y, yaw, timestamp, lv, av
using AgentsStates = std::vector<AgentStatus>;             // vector of agent status (different agents at the same time)
using AgentTrajectory = std::vector<AgentStatus>;          // vector of agent status (for a single agent trajectory)
using AgentsTrajectories = std::vector<AgentsStates>;      // vector of agent states (trajectories for all agents)f

}  // namespace mpc_enlarged_state

using AgentStatus = mpc_enlarged_state::AgentStatus;
using AgentsStates = mpc_enlarged_state::AgentsStates;
using AgentTrajectory = mpc_enlarged_state::AgentTrajectory;
using AgentsTrajectories = mpc_enlarged_state::AgentsTrajectories;

#endif  // MPC_ENLARGED_STATE__TYPE_DEFINITIONS_HPP_
