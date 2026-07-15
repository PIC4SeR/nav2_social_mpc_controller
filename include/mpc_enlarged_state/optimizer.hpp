// Copyright (c) 2022 SRL -Service Robotics Lab, Pablo de Olavide University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// ributed under the License is ributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MPC_ENLARGED_STATE__OPTIMIZER_HPP_
#define MPC_ENLARGED_STATE__OPTIMIZER_HPP_

#include <math.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "ceres/cost_function.h"
#include "ceres/cubic_interpolation.h"
#include "nav2_util/node_utils.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// cost functions
#include "mpc_enlarged_state/critics/agent_angle_cost_function.hpp"
#include "mpc_enlarged_state/critics/agent_obstacle_cost_function.hpp"
#include "mpc_enlarged_state/critics/agent_orca_dynamics_cost_function.hpp"
#include "mpc_enlarged_state/critics/agent_sfm_dynamics_cost_function.hpp"
#include "mpc_enlarged_state/critics/agent_velocity_reference_cost_function.hpp"
#include "mpc_enlarged_state/critics/angle_cost_function.hpp"
#include "mpc_enlarged_state/critics/crossing_cost_function.hpp"
#include "mpc_enlarged_state/critics/curvature_cost_function.hpp"
#include "mpc_enlarged_state/critics/distance_cost_function.hpp"
#include "mpc_enlarged_state/critics/goal_align_cost_function.hpp"
#include "mpc_enlarged_state/critics/goal_proximity_cost_function.hpp"
#include "mpc_enlarged_state/critics/obstacle_cost_function.hpp"
#include "mpc_enlarged_state/critics/social_work_cost_function.hpp"
#include "mpc_enlarged_state/critics/velocity_cost_function.hpp"
#include "mpc_enlarged_state/critics/velocity_feasibility_cost_function.hpp"
#include "mpc_enlarged_state/critics/proxemics_cost_function.hpp"
#include "mpc_enlarged_state/update_state.hpp"

#include "mpc_enlarged_state/sfm.hpp"
#include "mpc_enlarged_state/trajectory_memory.hpp"
#include "obstacle_distance_msgs/msg/obstacle_distance.hpp"
#include "people_msgs/msg/people.hpp"
#include "mpc_enlarged_state/tools/type_definitions.hpp"

namespace mpc_enlarged_state
{

struct OptimizerParams
{
  OptimizerParams()
  {
  }

  /**
   * @brief Get params from ROS parameter
   * @param node Ptr to node
   * @param name Name of plugin
   */
  void get(rclcpp_lifecycle::LifecycleNode* node, const std::string& name);
  const std::map<std::string, ceres::LinearSolverType> solver_types = {
    { "DENSE_SCHUR", ceres::DENSE_SCHUR },
    { "SPARSE_SCHUR", ceres::SPARSE_SCHUR },
    { "DENSE_NORMAL_CHOLESKY", ceres::DENSE_NORMAL_CHOLESKY },
    { "DENSE_QR", ceres::DENSE_QR },
    { "SPARSE_NORMAL_CHOLESKY", ceres::SPARSE_NORMAL_CHOLESKY }
  };

  std::string linear_solver_type;

  double param_tol;     // Ceres default: 1e-8
  double fn_tol;        // Ceres default: 1e-6
  double gradient_tol;  // Ceres default: 1e-10
  double socialwork_w_;
  double distance_w_;
  double velocity_w_;
  double angle_w_;
  double agent_angle_w_;
  double velocity_alignment_w_;
  double crossing_w_;
  double crossing_bearing_w_;
  double velocity_feasibility_w_;
  double agent_velocity_reference_w_;
  double agent_sfm_dynamics_w_;
  std::string agent_dynamics_model_;  // "sfm", "orca" or "cv"
  double agent_obstacle_w_;
  double goal_align_w_;
  double obstacle_w_;
  double proxemics_w_;
  double goal_proximity_w_;
  double goal_proximity_activation_radius_;
  double goal_proximity_decay_distance_;
  bool use_adaptive_velocity_cost;
  double adaptive_velocity_distance_;
  double adaptive_velocity_min_scale_;
  bool use_social_work_cost;
  bool use_social_angle_cost;
  bool use_social_crossing_cost;
  bool use_social_proxemics_cost;
  bool use_social_path_follow_cost;
  bool use_social_path_align_cost;
  float current_path_w;
  float current_cmds_w;
  float max_time;
  int discretization_;
  int control_horizon_;
  int parameter_block_length_;
  int max_agents;
  bool debug;
  int max_iterations;
  double max_linear_vel;
  double min_linear_vel;
  double max_angular_vel;
  double min_angular_vel;
  double desired_linear_vel;
  double agent_velocity_bound;
  double agent_max_accel;
  double agent_track_timeout;
  double agent_coast_decay_time;
  double agent_association_radius;
  int agent_speed_window;
  double human_cooperation_factor;
  SfmPredictionParams sfm;
  OrcaPredictionParams orca;
  double background_agent_weight;
  double stationary_agent_velocity_bound;
  double stationary_agent_speed_threshold;
};

/**
 * @brief Optimizer class for the social MPC controller
 */
class Optimizer
{
public:
  // x, y
  struct position
  {
    double params[kPositionParameterBlockSize];
  };

  // x, y, lv, av
  struct posandvel
  {
    double params[kPoseAndVelocityParameterBlockSize];
  };

  // lv, av
  struct vel
  {
    double params[kRobotParameterBlockSize];
  };

  struct agent_velocity
  {
    double params[kAgentVelocityParamStride];  
  };

  struct optimizing_velocities
  {
    double params[kPoseAndVelocityParameterBlockSize];
  };
  struct dynamic_optimizing_velocities
  {
    vel robot;
    std::vector<double> agents;

    void set_num_agents(size_t num_agents)
    {
      agents.assign(kAgentVelocityParamStride * num_agents, 0.0);
    }

    double* robot_data()
    {
      return robot.params;
    }

    const double* robot_data() const
    {
      return robot.params;
    }

    double* agents_data()
    {
      return agents.empty() ? nullptr : agents.data();
    }

    const double* agents_data() const
    {
      return agents.empty() ? nullptr : agents.data();
    }

    unsigned int agent_block_size() const
    {
      return static_cast<unsigned int>(agents.size());
    }
  };
  // t, yaw
  struct heading
  {
    double params[kHeadingParameterBlockSize];
  };
  struct linear_velocity
  {
    double params[kScalarParameterBlockSize];
  };
  struct angular_velocity
  {
    double params[kScalarParameterBlockSize];
  };
  Optimizer();

  /**
   * @brief Destrructor for
   * mpc_enlarged_state::MPCEnlargedState
   */
  ~Optimizer();

  /**
   * @brief Initialization of the optimizer
   * @param params OptimizerParam struct
   */
  void initialize(const OptimizerParams params);

  /**
   * @brief Optimize the path using the social MPC controller
   * @param path The path to optimize
   * @param people_proj Projected people positions
   * @param costmap Costmap for obstacle avoidance
   * @param obstacles Obstacle distances
   * @param cmds Commands to execute
   * @param people People detected in the environment
   * @param speed Current robot speed
   * @param time_step Time step for discretization
   * @return true if optimization succeeded
   * @return false if optimization failed
   */
  bool optimize(nav_msgs::msg::Path& path, AgentsTrajectories& people_proj, const nav2_costmap_2d::Costmap2D* costmap,
                std::vector<geometry_msgs::msg::TwistStamped>& cmds, const people_msgs::msg::People& people,
                const geometry_msgs::msg::Twist& speed, const float time_step,
                const geometry_msgs::msg::PoseStamped& goal_pose);

private:
  /**
   * @brief Convert people messages to agent status
   * @param people People messages
   * @param robot_pose Current robot pose, used to rank agents by interaction relevance
   * @param speed Current robot speed, used to rank agents by interaction relevance
   * @return Vector of agent statuses, most relevant first, truncated to max_agents
   */
  AgentsStates people_to_status(const people_msgs::msg::People& people, double time_step,
                                const geometry_msgs::msg::Pose& robot_pose, const geometry_msgs::msg::Twist& speed);

  /**
   * @brief Relevance score of an agent: predicted closest distance to the robot over the
   * MPC horizon under constant velocity. Lower is more relevant.
   */
  double agent_relevance(const AgentStatus& agent, const geometry_msgs::msg::Pose& robot_pose,
                         const geometry_msgs::msg::Twist& speed) const;

  /**
   * @brief Format path and commands for optimization
   * @param path Current path
   * @param previous_path Previous path
   * @param cmds Current commands
   * @param previous_cmds Previous commands
   * @param speed Current robot speed
   * @param current_path_w Weight for current path
   * @param current_cmds_w Weight for current commands
   * @param maxtime Maximum time horizon
   * @param timestep Time step
   * @return Vector of agent statuses
   */
  AgentTrajectory format_to_optimize(nav_msgs::msg::Path& path, const nav_msgs::msg::Path& previous_path,
                                     const std::vector<geometry_msgs::msg::TwistStamped>& cmds,
                                     const std::vector<geometry_msgs::msg::TwistStamped>& previous_cmds,
                                     const geometry_msgs::msg::Twist& speed, const float current_path_w,
                                     const float current_cmds_w, const float maxtime, const float timestep);

  struct TrackedAgent
  {
    AgentStatus state;
    double last_seen_time{0.0};
    double last_update_time{0.0};
    // Recent observed speeds; the track reports their max, so a person who briefly
    // reads as slow (occlusion, a noisy detection) is still predicted at the pace
    // they have actually been walking.
    std::deque<double> speed_window;
    // How much this person is predicted to yield to the robot (see
    // AgentSfmDynamicsCost). Per-agent rather than global because a crowd is a
    // mixture: some people negotiate, some walk straight through you.
    double cooperation{1.0};
  };

  // Hook for revising a track's cooperation from what we have actually observed it
  // do (did it deviate when the robot closed in?). Currently a no-op: every track
  // keeps the human_cooperation_factor prior it was born with.
  void update_cooperation(TrackedAgent& track, const geometry_msgs::msg::Pose& robot_pose);

  bool debug_;
  unsigned int control_horizon_;
  unsigned int parameter_block_length_;
  float max_time;
  double obstacle_w_;
  double velocity_feasibility_w_;
  double agent_velocity_reference_w_;
  double agent_sfm_dynamics_w_;
  std::string agent_dynamics_model_;
  double agent_obstacle_w_;
  double agent_angle_w_;
  double velocity_alignment_w_;
  double crossing_w_;
  double crossing_bearing_w_;
  double angle_w_;
  double distance_w_;
  double socialwork_w_;
  double goal_align_w_;
  double velocity_w_;
  double curvature_w_;
  double proxemics_w_;
  double curvature_angle_min_;
  double goal_proximity_w_;
  double goal_proximity_activation_radius_;
  double goal_proximity_decay_distance_;
  bool use_adaptive_velocity_cost_;
  double adaptive_velocity_distance_;
  double adaptive_velocity_min_scale_;
  bool use_social_work_cost_{true};
  bool use_social_angle_cost_{true};
  bool use_social_crossing_cost_{false};
  bool use_social_proxemics_cost_{true};
  bool use_social_path_follow_cost_{true};
  bool use_social_path_align_cost_{true};
  float current_path_w;
  float current_cmds_w;
  double max_linear_vel_;
  double min_linear_vel_;
  double max_angular_vel_;
  double min_angular_vel_;
  double desired_linear_vel_;
  double agent_velocity_bound_;
  double agent_max_accel_;
  double agent_track_timeout_;
  double agent_coast_decay_time_;
  double agent_association_radius_;
  size_t agent_speed_window_;
  double human_cooperation_factor_;
  // Per-agent cooperation, aligned with the AgentsStates returned by people_to_status.
  std::vector<double> agent_cooperation_;
  SfmPredictionParams sfm_params_;
  OrcaPredictionParams orca_params_;
  // The humans that max_agents truncated away. They are NOT part of the enlarged state --
  // they carry no decision variables and are never co-optimized -- but they are still
  // constant-velocity extrapolated and fed to a proxemics residual over the robot blocks,
  // so a crowd bigger than max_agents does not go completely unseen. Cheap: adds residuals,
  // not parameters.
  AgentsStates background_agents_;
  double background_agent_w_;
  double stationary_agent_velocity_bound_;
  double stationary_agent_speed_threshold_;
  ceres::Solver::Options options_;
  std::shared_ptr<ceres::Grid2D<u_char>> costmap_grid_;
  std::string frame_;
  rclcpp::Time path_time_;
  size_t max_agents_;
  std::vector<TrackedAgent> tracked_agents_;
  bool have_tracking_time_{false};
  double last_tracking_time_{0.0};
};

}  // namespace mpc_enlarged_state

#endif  // MPC_ENLARGED_STATE__OPTIMIZER_HPP_
