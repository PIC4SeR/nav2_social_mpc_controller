#ifndef MPC_ENLARGED_STATE__TYPE_DEFINITIONS_HPP_
#define MPC_ENLARGED_STATE__TYPE_DEFINITIONS_HPP_

#include <Eigen/Core>   
#include <ceres/ceres.h>

typedef Eigen::Matrix<double, 6, 1> AgentStatus;       // x, y, yaw, timestamp, lv, av
typedef std::vector<AgentStatus> AgentsStates;         // vector of agent status (different agents at the same time)
typedef std::vector<AgentStatus> AgentTrajectory;      // vector of agent status (for a single agent trajectory)
typedef std::vector<AgentsStates> AgentsTrajectories;  // vector of agent states (trajectories for all agents)f

#endif  // MPC_ENLARGED_STATE__TYPE_DEFINITIONS_HPP_