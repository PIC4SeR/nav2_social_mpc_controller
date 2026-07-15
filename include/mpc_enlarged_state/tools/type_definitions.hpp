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

// Social Force Model parameters used to PREDICT how humans move (AgentSfmDynamicsCost).
//
// The defaults deliberately mirror the HuNav crowd we are predicting, so the predictor
// and the simulator run the same social force model:
//   - lambda/gamma/n/n_prime/relaxation_time are lightsfm's defaults, and HuNav's agent
//     yamls do not override them, so these already match the simulated crowd.
//   - force_factor_social is 5.0, NOT lightsfm's 2.1 default: HuNav sets it per agent from
//     behavior.social_force_factor (see AgentManager::updateAgents), and the crowded_env
//     agent yamls all use 5.0. Predicting at 2.1 under-estimates how hard humans repel.
// Retune these only to model a crowd that differs from the one in the agent yaml.
struct SfmPredictionParams
{
  double lambda = 2.0;
  double gamma = 0.35;
  double n = 2.0;
  double n_prime = 3.0;
  double relaxation_time = 0.5;
  double force_factor_social = 5.0;
};

// ORCA parameters used to PREDICT how humans move (AgentOrcaDynamicsCost).
//
// Defaults mirror the simulated crowd, same principle as SfmPredictionParams:
//   - time_horizon 5.0 matches hunav::OrcaParams::time_horizon (NOT the 2.0 this critic
//     used to hardcode, which made the predictor 2.5x less far-sighted than the crowd).
//   - agent_radius 0.4 matches the `radius:` in the HuNav agent yamls (was 0.35).
//   - robot_radius 0.35 matches HuNavPlugin's robotAgent.radius.
// relaxation_time and smoothing have no HuNav counterpart: they belong to this
// differentiable ORCA approximation (smoothing is the softplus width that keeps the
// residual autodiff-friendly), so they are tuning knobs, not model-matching ones.
struct OrcaPredictionParams
{
  double time_horizon = 5.0;
  double relaxation_time = 0.5;
  double smoothing = 0.05;
  double agent_radius = 0.4;
  double robot_radius = 0.35;
};

}  // namespace mpc_enlarged_state

using OrcaPredictionParams = mpc_enlarged_state::OrcaPredictionParams;
using SfmPredictionParams = mpc_enlarged_state::SfmPredictionParams;
using AgentStatus = mpc_enlarged_state::AgentStatus;
using AgentsStates = mpc_enlarged_state::AgentsStates;
using AgentTrajectory = mpc_enlarged_state::AgentTrajectory;
using AgentsTrajectories = mpc_enlarged_state::AgentsTrajectories;

#endif  // MPC_ENLARGED_STATE__TYPE_DEFINITIONS_HPP_
