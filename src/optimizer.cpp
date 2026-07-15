#include "mpc_enlarged_state/optimizer.hpp"

#include <algorithm>
#include <limits>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tf2/utils.h"

namespace mpc_enlarged_state
{


void add_robot_parameter_blocks(ceres::DynamicCostFunction* cost_function, const unsigned int active_blocks)
{
  for (unsigned int j = 0; j < active_blocks; ++j)
  {
    cost_function->AddParameterBlock(kRobotParameterBlockSize);
  }
}

void add_enlarged_parameter_blocks(ceres::DynamicCostFunction* cost_function, const unsigned int active_blocks,
                                   const bool has_agent_parameters, const unsigned int num_agents)
{
  add_robot_parameter_blocks(cost_function, active_blocks);
  if (has_agent_parameters)
  {
    const unsigned int agent_block_dim = static_cast<unsigned int>(kAgentVelocityParamStride * num_agents);
    for (unsigned int j = 0; j < active_blocks; ++j)
    {
      cost_function->AddParameterBlock(agent_block_dim);
    }
  }
}


/**
 * @brief OptimizerParams class constructor
 * @param node LifecycleNode to get parameters from
 * @param name Name of the optimizer parameters
 */

void OptimizerParams::get(rclcpp_lifecycle::LifecycleNode* node, const std::string& name)
{
  // Get the different parameters from the node, defined in the config file
  // if not defined, set to default values

  std::string trajectorizer = name + std::string(".trajectorizer.");
  std::string local_name = name + std::string(".optimizer.");
  std::string weights = local_name + std::string("weights.");
  std::string overall_cost = local_name + std::string("critics.");

  // Optimizer params
  nav2_util::declare_parameter_if_not_declared(
      node, local_name + "linear_solver_type",
      rclcpp::ParameterValue("SPARSE_NORMAL_CHOLESKY"));  // SPARSE_NORMAL_CHOLESKY

  node->get_parameter(local_name + "linear_solver_type", linear_solver_type);
  if (solver_types.find(linear_solver_type) == solver_types.end())
  {
    std::stringstream valid_types_str;
    for (auto type = solver_types.begin(); type != solver_types.end(); type++)
    {
      if (type != solver_types.begin())
      {
        valid_types_str << ", ";
      }
      valid_types_str << type->first;
    }
    RCLCPP_ERROR(rclcpp::get_logger("optimizer"), "Invalid linear_solver_type. Valid values are %s",
                 valid_types_str.str().c_str());
    throw std::runtime_error("Invalid parameter: linear_solver_type");
  }
  nav2_util::declare_parameter_if_not_declared(node, local_name + "param_tol", rclcpp::ParameterValue(1e-15));
  node->get_parameter(local_name + "param_tol", param_tol);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "fn_tol", rclcpp::ParameterValue(1e-7));
  node->get_parameter(local_name + "fn_tol", fn_tol);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "gradient_tol", rclcpp::ParameterValue(1e-10));
  node->get_parameter(local_name + "gradient_tol", gradient_tol);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "max_iterations", rclcpp::ParameterValue(100));
  node->get_parameter(local_name + "max_iterations", max_iterations);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "debug_optimizer", rclcpp::ParameterValue(false));
  node->get_parameter(local_name + "debug_optimizer", debug);

  nav2_util::declare_parameter_if_not_declared(node, weights + "distance_weight", rclcpp::ParameterValue(3.0));
  node->get_parameter(weights + "distance_weight", distance_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "social_weight", rclcpp::ParameterValue(1.0));
  node->get_parameter(weights + "social_weight", socialwork_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "velocity_weight", rclcpp::ParameterValue(0.5));
  node->get_parameter(weights + "velocity_weight", velocity_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "angle_weight", rclcpp::ParameterValue(0.0));
  node->get_parameter(weights + "angle_weight", angle_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "agent_angle_weight", rclcpp::ParameterValue(0.5));
  node->get_parameter(weights + "agent_angle_weight", agent_angle_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "velocity_alignment_weight", rclcpp::ParameterValue(0.5));
  node->get_parameter(weights + "velocity_alignment_weight", velocity_alignment_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "crossing_weight", rclcpp::ParameterValue(0.0));
  node->get_parameter(weights + "crossing_weight", crossing_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "crossing_bearing_weight", rclcpp::ParameterValue(2.0));
  node->get_parameter(weights + "crossing_bearing_weight", crossing_bearing_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "proxemics_weight", rclcpp::ParameterValue(90.0));
  node->get_parameter(weights + "proxemics_weight", proxemics_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "velocity_feasibility_weight",
                                               rclcpp::ParameterValue(0.5));
  node->get_parameter(weights + "velocity_feasibility_weight", velocity_feasibility_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "agent_velocity_reference_weight",
                                               rclcpp::ParameterValue(10.0));
  node->get_parameter(weights + "agent_velocity_reference_weight", agent_velocity_reference_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "agent_sfm_dynamics_weight",
                                               rclcpp::ParameterValue(0.5));
  node->get_parameter(weights + "agent_sfm_dynamics_weight", agent_sfm_dynamics_w_);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "agent_dynamics_model",
                                               rclcpp::ParameterValue("sfm"));
  node->get_parameter(local_name + "agent_dynamics_model", agent_dynamics_model_);
  if (agent_dynamics_model_ != "sfm" && agent_dynamics_model_ != "orca" && agent_dynamics_model_ != "cv")
  {
    RCLCPP_ERROR(rclcpp::get_logger("optimizer"),
                 "Invalid agent_dynamics_model '%s'. Valid values are sfm, orca, cv",
                 agent_dynamics_model_.c_str());
    throw std::runtime_error("Invalid parameter: agent_dynamics_model");
  }
  nav2_util::declare_parameter_if_not_declared(node, weights + "agent_obstacle_weight", rclcpp::ParameterValue(1.0));
  node->get_parameter(weights + "agent_obstacle_weight", agent_obstacle_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "obstacle_weight", rclcpp::ParameterValue(0.0));
  node->get_parameter(weights + "obstacle_weight", obstacle_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "goal_align_weight", rclcpp::ParameterValue(0.0));
  node->get_parameter(weights + "goal_align_weight", goal_align_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "goal_proximity_weight", rclcpp::ParameterValue(0.0));
  node->get_parameter(weights + "goal_proximity_weight", goal_proximity_w_);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "use_adaptive_velocity_cost", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(node, local_name + "adaptive_velocity_distance", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, local_name + "adaptive_velocity_min_scale", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(node, overall_cost + "enable_social_work", rclcpp::ParameterValue(true));
  node->get_parameter(overall_cost + "enable_social_work", use_social_work_cost);
  nav2_util::declare_parameter_if_not_declared(node, overall_cost + "enable_angle", rclcpp::ParameterValue(true));
  node->get_parameter(overall_cost + "enable_angle", use_social_angle_cost);
  nav2_util::declare_parameter_if_not_declared(node, overall_cost + "enable_crossing", rclcpp::ParameterValue(false));
  node->get_parameter(overall_cost + "enable_crossing", use_social_crossing_cost);
  nav2_util::declare_parameter_if_not_declared(node, overall_cost + "enable_proxemics", rclcpp::ParameterValue(true));
  node->get_parameter(overall_cost + "enable_proxemics", use_social_proxemics_cost);
  nav2_util::declare_parameter_if_not_declared(node, overall_cost + "enable_path_follow", rclcpp::ParameterValue(true));
  node->get_parameter(overall_cost + "enable_path_follow", use_social_path_follow_cost);
  nav2_util::declare_parameter_if_not_declared(node, overall_cost + "enable_path_align", rclcpp::ParameterValue(true));
  node->get_parameter(overall_cost + "enable_path_align", use_social_path_align_cost);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "control_horizon", rclcpp::ParameterValue(5));
  node->get_parameter(local_name + "control_horizon", control_horizon_);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "parameter_block_length", rclcpp::ParameterValue(5));
  node->get_parameter(local_name + "parameter_block_length", parameter_block_length_);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "current_path_weight", rclcpp::ParameterValue(1.0));
  node->get_parameter(local_name + "current_path_weight", current_path_w);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "current_cmds_weight", rclcpp::ParameterValue(1.0));
  node->get_parameter(local_name + "current_cmds_weight", current_cmds_w);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "max_agents", rclcpp::ParameterValue(0));
  node->get_parameter(local_name + "max_agents", max_agents);
  node->get_parameter(trajectorizer + "max_time", max_time);

  nav2_util::declare_parameter_if_not_declared(node, name + ".max_linear_vel", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(node, name + ".min_linear_vel", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".max_angular_vel", rclcpp::ParameterValue(1.4));
  nav2_util::declare_parameter_if_not_declared(node, trajectorizer + "desired_linear_vel", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(node, local_name + "agent_velocity_bound", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, local_name + "agent_max_accel", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, local_name + "agent_track_timeout", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, local_name + "agent_coast_decay_time",
                                               rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(node, local_name + "agent_association_radius",
                                               rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(node, local_name + "agent_speed_window",
                                               rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(node, local_name + "human_cooperation_factor",
                                               rclcpp::ParameterValue(1.0));
  // Weight of the proxemics penalty applied to the humans that max_agents truncated away.
  // They cost no decision variables (constant-velocity constants, robot blocks only), so this
  // buys awareness of a crowd larger than max_agents almost for free. 0.0 disables it, which
  // restores the previous behaviour of being blind to everyone past the cap.
  nav2_util::declare_parameter_if_not_declared(node, weights + "background_agent_weight",
                                               rclcpp::ParameterValue(0.0));
  node->get_parameter(weights + "background_agent_weight", background_agent_weight);

  // SFM parameters used to PREDICT the humans. Defaults mirror the simulated crowd
  // (lightsfm defaults, with force_factor_social at HuNav's behavior.social_force_factor
  // of 5.0 rather than lightsfm's 2.1) -- see SfmPredictionParams.
  const std::string sfm_ns = local_name + "sfm.";
  nav2_util::declare_parameter_if_not_declared(node, sfm_ns + "lambda", rclcpp::ParameterValue(sfm.lambda));
  node->get_parameter(sfm_ns + "lambda", sfm.lambda);
  nav2_util::declare_parameter_if_not_declared(node, sfm_ns + "gamma", rclcpp::ParameterValue(sfm.gamma));
  node->get_parameter(sfm_ns + "gamma", sfm.gamma);
  nav2_util::declare_parameter_if_not_declared(node, sfm_ns + "n", rclcpp::ParameterValue(sfm.n));
  node->get_parameter(sfm_ns + "n", sfm.n);
  nav2_util::declare_parameter_if_not_declared(node, sfm_ns + "n_prime", rclcpp::ParameterValue(sfm.n_prime));
  node->get_parameter(sfm_ns + "n_prime", sfm.n_prime);
  nav2_util::declare_parameter_if_not_declared(node, sfm_ns + "relaxation_time",
                                               rclcpp::ParameterValue(sfm.relaxation_time));
  node->get_parameter(sfm_ns + "relaxation_time", sfm.relaxation_time);
  nav2_util::declare_parameter_if_not_declared(node, sfm_ns + "force_factor_social",
                                               rclcpp::ParameterValue(sfm.force_factor_social));
  node->get_parameter(sfm_ns + "force_factor_social", sfm.force_factor_social);

  // ORCA parameters used to PREDICT the humans. Defaults mirror the simulated crowd
  // (hunav::OrcaParams time_horizon, the agent yaml radius, HuNavPlugin's robot radius)
  // -- see OrcaPredictionParams.
  const std::string orca_ns = local_name + "orca.";
  nav2_util::declare_parameter_if_not_declared(node, orca_ns + "time_horizon",
                                               rclcpp::ParameterValue(orca.time_horizon));
  node->get_parameter(orca_ns + "time_horizon", orca.time_horizon);
  nav2_util::declare_parameter_if_not_declared(node, orca_ns + "relaxation_time",
                                               rclcpp::ParameterValue(orca.relaxation_time));
  node->get_parameter(orca_ns + "relaxation_time", orca.relaxation_time);
  nav2_util::declare_parameter_if_not_declared(node, orca_ns + "smoothing", rclcpp::ParameterValue(orca.smoothing));
  node->get_parameter(orca_ns + "smoothing", orca.smoothing);
  nav2_util::declare_parameter_if_not_declared(node, orca_ns + "agent_radius",
                                               rclcpp::ParameterValue(orca.agent_radius));
  node->get_parameter(orca_ns + "agent_radius", orca.agent_radius);
  nav2_util::declare_parameter_if_not_declared(node, orca_ns + "robot_radius",
                                               rclcpp::ParameterValue(orca.robot_radius));
  node->get_parameter(orca_ns + "robot_radius", orca.robot_radius);
  nav2_util::declare_parameter_if_not_declared(node, local_name + "stationary_agent_velocity_bound",
                                               rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node, local_name + "stationary_agent_speed_threshold",
                                               rclcpp::ParameterValue(0.01));
                                               
  nav2_util::declare_parameter_if_not_declared(node, local_name + "goal_proximity_activation_radius",
                                               rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(node, local_name + "goal_proximity_decay_distance",
                                               rclcpp::ParameterValue(0.25));
  node->get_parameter(name + ".max_linear_vel", max_linear_vel);
  node->get_parameter(name + ".min_linear_vel", min_linear_vel);
  node->get_parameter(name + ".max_angular_vel", max_angular_vel);
  node->get_parameter(trajectorizer + "desired_linear_vel", desired_linear_vel);
  node->get_parameter(local_name + "agent_velocity_bound", agent_velocity_bound);
  node->get_parameter(local_name + "agent_max_accel", agent_max_accel);
  node->get_parameter(local_name + "agent_track_timeout", agent_track_timeout);
  node->get_parameter(local_name + "agent_coast_decay_time", agent_coast_decay_time);
  node->get_parameter(local_name + "agent_association_radius", agent_association_radius);
  node->get_parameter(local_name + "agent_speed_window", agent_speed_window);
  node->get_parameter(local_name + "human_cooperation_factor", human_cooperation_factor);
  node->get_parameter(local_name + "stationary_agent_velocity_bound", stationary_agent_velocity_bound);
  node->get_parameter(local_name + "stationary_agent_speed_threshold", stationary_agent_speed_threshold);
  node->get_parameter(local_name + "goal_proximity_activation_radius", goal_proximity_activation_radius_);
  node->get_parameter(local_name + "goal_proximity_decay_distance", goal_proximity_decay_distance_);
  node->get_parameter(local_name + "use_adaptive_velocity_cost", use_adaptive_velocity_cost);
  node->get_parameter(local_name + "adaptive_velocity_distance", adaptive_velocity_distance_);
  node->get_parameter(local_name + "adaptive_velocity_min_scale", adaptive_velocity_min_scale_);
}
// constructor and destructor for Optimizer
Optimizer::Optimizer()
  : max_agents_(0)
{
}
Optimizer::~Optimizer()
{
}

/**
 * @brief Initialization of the smoother
 * @param params OptimizerParam struct
 */
void Optimizer::initialize(const OptimizerParams params)
{
  // Initialize the optimizer with the parameters, getting the values from the
  // OptimizerParams struct
  debug_ = params.debug;
  obstacle_w_ = params.obstacle_w_;
  goal_align_w_ = params.goal_align_w_;
  velocity_feasibility_w_ = params.velocity_feasibility_w_;
  agent_velocity_reference_w_ = params.agent_velocity_reference_w_;
  agent_sfm_dynamics_w_ = params.agent_sfm_dynamics_w_;
  agent_dynamics_model_ = params.agent_dynamics_model_;
  agent_obstacle_w_ = params.agent_obstacle_w_;
  socialwork_w_ = params.socialwork_w_;
  distance_w_ = params.distance_w_;
  velocity_w_ = params.velocity_w_;
  angle_w_ = params.angle_w_;
  agent_angle_w_ = params.agent_angle_w_;
  velocity_alignment_w_ = params.velocity_alignment_w_;
  crossing_w_ = params.crossing_w_;
  crossing_bearing_w_ = params.crossing_bearing_w_;
  proxemics_w_ = params.proxemics_w_;
  goal_proximity_w_ = params.goal_proximity_w_;
  use_social_work_cost_ = params.use_social_work_cost;
  use_social_angle_cost_ = params.use_social_angle_cost;
  use_social_crossing_cost_ = params.use_social_crossing_cost;
  use_social_proxemics_cost_ = params.use_social_proxemics_cost;
  use_social_path_follow_cost_ = params.use_social_path_follow_cost;
  use_social_path_align_cost_ = params.use_social_path_align_cost;
  control_horizon_ = params.control_horizon_;
  parameter_block_length_ = params.parameter_block_length_;
  max_time = params.max_time;
  current_path_w = params.current_path_w;
  current_cmds_w = params.current_cmds_w;
  max_agents_ = params.max_agents > 0 ? static_cast<size_t>(params.max_agents) : 0;
  options_.linear_solver_type = params.solver_types.at(params.linear_solver_type);
  options_.max_num_iterations = params.max_iterations;
  options_.function_tolerance = params.fn_tol;
  options_.gradient_tolerance = params.gradient_tol;
  options_.parameter_tolerance = params.param_tol;
  max_linear_vel_ = params.max_linear_vel;
  min_linear_vel_ = params.min_linear_vel;
  max_angular_vel_ = params.max_angular_vel;
  desired_linear_vel_ = params.desired_linear_vel;
  goal_proximity_activation_radius_ = params.goal_proximity_activation_radius_;
  goal_proximity_decay_distance_ = params.goal_proximity_decay_distance_;
  use_adaptive_velocity_cost_ = params.use_adaptive_velocity_cost;
  adaptive_velocity_distance_ = params.adaptive_velocity_distance_;
  adaptive_velocity_min_scale_ = params.adaptive_velocity_min_scale_;
  agent_velocity_bound_ = params.agent_velocity_bound;
  agent_max_accel_ = params.agent_max_accel;
  agent_track_timeout_ = params.agent_track_timeout;
  agent_coast_decay_time_ = params.agent_coast_decay_time;
  agent_association_radius_ = params.agent_association_radius;
  agent_speed_window_ = static_cast<size_t>(std::max(1, params.agent_speed_window));
  human_cooperation_factor_ = std::clamp(params.human_cooperation_factor, 0.0, 1.0);
  sfm_params_ = params.sfm;
  orca_params_ = params.orca;
  background_agent_w_ = params.background_agent_weight;
  stationary_agent_velocity_bound_ = params.stationary_agent_velocity_bound;
  stationary_agent_speed_threshold_ = params.stationary_agent_speed_threshold;
  if (debug_)
  {
    options_.minimizer_progress_to_stdout = true;
    options_.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
  }
  else
  {
    options_.logging_type = ceres::SILENT;
  }
  options_.max_solver_time_in_seconds = params.max_time;
}

/**
 * @brief main optimization function, where the trajectory is optimized around
 * a starting trajectory, given a set of people and a costmap.
 *
 * @param path starting trajectory to optimize
 * @param people_proj projected people in the field of view of the robot (filled by optimizer)
 * @param costmap costmap to use for the optimization
 * @param obstacles obstacles surrounding the robot, to fill the obstacles points for agents
 * @param cmds starting commands to optimize, which would result in the given path
 * @param people people that fall in the field of view of the robot, which will be projected
 * @param speed initial speed of the robot
 * @param time_step length of the time step for the forward projection of the trajectory
 * @return true if the optimization was successful, false otherwise
 */
bool Optimizer::optimize(nav_msgs::msg::Path& path, AgentsTrajectories& people_proj,
                         const nav2_costmap_2d::Costmap2D* costmap,
                         std::vector<geometry_msgs::msg::TwistStamped>& cmds, const people_msgs::msg::People& people,
                         const geometry_msgs::msg::Twist& speed, const float time_step,
                         const geometry_msgs::msg::PoseStamped& goal_pose)
{
  // Path has always at least 2 points
  if (path.poses.size() < 2)
  {
    RCLCPP_WARN(rclcpp::get_logger("optimizer"), "Path has less than 2 points, cannot optimize");
    return false;
  }
  frame_ = path.header.frame_id;
  path_time_ = rclcpp::Time(path.header.stamp);
  const geometry_msgs::msg::Pose goal_pose_in = goal_pose.pose;

  // Transform people detections into tracked agent states. Lost agents coast for a short timeout.
  AgentsStates init_people = people_to_status(people, time_step, path.poses[0].pose, speed);

  // Create costmap grid
  costmap_grid_ = std::make_shared<ceres::Grid2D<u_char>>(costmap->getCharMap(), 0, costmap->getSizeInCellsY(), 0,
                                                          costmap->getSizeInCellsX());
  // create the bi-cubic interpolator for the costmap, to be used in the obstacles critic
  auto costmap_interpolator = std::make_shared<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>(*costmap_grid_);
  // this should keep the previous optimized path and commands
  // in order to use them as a starting point for the optimization
  // i do not know if this works as expected, but it should
  auto& memory = TrajectoryMemory::getInstance();

  nav_msgs::msg::Path previous_path;
  std::vector<geometry_msgs::msg::TwistStamped> previous_cmds;
  {
    std::lock_guard<std::mutex> lock(memory.mtx);
    if (memory.previous_path.poses.empty())
    {
      memory.previous_path = path;
      memory.previous_cmds = cmds;
    }
    previous_path = memory.previous_path;
    previous_cmds = memory.previous_cmds;
  }

  // use the projected path to make it into the a parametrized format
  AgentsStates optim_status = format_to_optimize(path, previous_path, cmds, previous_cmds, speed, current_path_w,
                                                 current_cmds_w, max_time, time_step);
  people_proj.push_back(init_people);

  const size_t num_agents_size = init_people.size();
  const unsigned int num_agents = static_cast<unsigned int>(num_agents_size);
  const bool has_agent_parameters = num_agents > 0;

  AgentsStates fallback_agents;
  if (num_agents_size > 0)
  {
    fallback_agents.resize(num_agents_size);
    for (size_t k = 0; k < num_agents_size; ++k)
    {
      fallback_agents[k] = AgentStatus::Zero();
      fallback_agents[k][kStateTime] = -1.0;  // mark as invalid by default
    }
  }

  const AgentsStates& people_states_for_cost = (!people_proj.empty() ? people_proj.front() : fallback_agents);

  // Social-comfort costs are only meaningful for moving agents. Stationary agents
  // are already penalised geometrically via costmap inflation and agent_obstacle_weight;
  // adding proxemics/social-work on top causes a deadlock in narrow passages.
  AgentsStates social_people_states = people_states_for_cost;
  for (auto& agent : social_people_states)
  {
    if (agent[kStateTime] != -1.0 && agent[kStateLinearVelocity] < stationary_agent_speed_threshold_)
      agent[kStateTime] = -1.0;
  }

  std::vector<dynamic_optimizing_velocities> variables_to_optimize;
  variables_to_optimize.reserve(optim_status.size());
  for (unsigned int j = 0; j < optim_status.size(); ++j)
  {
    dynamic_optimizing_velocities doa;
    doa.set_num_agents(num_agents);
    doa.robot.params[kRobotLinearVelocityParam] = optim_status[j][kStateLinearVelocity];
    doa.robot.params[kRobotAngularVelocityParam] = optim_status[j][kStateAngularVelocity];
    for (unsigned int k = 0; k < num_agents; ++k)
    {
      const auto& agent_state = (people_proj.empty() || people_proj[0].size() <= k)
                                    ? fallback_agents[k]
                                    : people_proj[0][k];
      const unsigned int agent_param_idx = kAgentVelocityParamStride * k;
      doa.agents[agent_param_idx + kAgentVxParam] =
          agent_state[kStateLinearVelocity] * ceres::cos(agent_state[kStateYaw]);
      doa.agents[agent_param_idx + kAgentVyParam] =
          agent_state[kStateLinearVelocity] * ceres::sin(agent_state[kStateYaw]);
    }
    variables_to_optimize.push_back(doa);
  }

  // get different parameters from the initial status
  // and create the evolving poses, positions, headings and velocities
  std::vector<geometry_msgs::msg::PoseStamped> evolving_poses;
  std::vector<position> optim_positions;
  std::vector<heading> optim_headings;
  std::vector<vel> optim_velocities;
  std::vector<linear_velocity> optim_linear_velocities;
  std::vector<angular_velocity> optim_angular_velocities;

  for (auto a : optim_status)
  {
    position p;
    p.params[kX] = a[kStateX];  // x
    p.params[kY] = a[kStateY];  // y
    vel v;
    v.params[kRobotLinearVelocityParam] = a[kStateLinearVelocity];    // lv
    v.params[kRobotAngularVelocityParam] = a[kStateAngularVelocity];  // av
    linear_velocity lv;
    lv.params[kScalarParam] = a[kStateLinearVelocity];  // lv
    angular_velocity av;
    av.params[kScalarParam] = a[kStateAngularVelocity];  // av
    heading h;
    h.params[kHeadingTimeParam] = a[kStateTime];  // t
    h.params[kHeadingYawParam] = a[kStateYaw];    // yaw
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = a[kStateX];
    pose.pose.position.y = a[kStateY];
    pose.pose.position.z = 0.0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, a[kStateYaw]);  // yaw
    pose.pose.orientation = tf2::toMsg(quaternion);
    optim_positions.push_back(p);
    optim_velocities.push_back(v);
    optim_headings.push_back(h);
    optim_linear_velocities.push_back(lv);
    optim_angular_velocities.push_back(av);
    evolving_poses.push_back(pose);
  }
  Eigen::Matrix<double, 2, 1> final_trajectorized_point(
      optim_positions[optim_status.size() - 1].params[kX], optim_positions[optim_status.size() - 1].params[kY]);

  optim_velocities.pop_back();

  // setting ceres variables
  ceres::Problem problem;
  ceres::Solver::Summary summary;

  std::vector<double*> robot_parameter_blocks;
  std::vector<double*> agent_parameter_blocks;
  unsigned int control_horizon = std::min(control_horizon_, static_cast<unsigned int>(optim_velocities.size()));
  unsigned int block_length = std::min(parameter_block_length_, control_horizon);
  if (control_horizon == 0 || block_length == 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("optimizer"), "Control horizon or parameter block length is zero");
    return false;
  }
  for (unsigned int i = 0; i < optim_velocities.size(); i++)  // i is the index of the current time step
  {
    unsigned int block_used = i / block_length;

    // add the velocities to optimize
    if (i < control_horizon &&
        (robot_parameter_blocks.empty() ||
         robot_parameter_blocks.back() != variables_to_optimize[block_used].robot_data()))
    {
      robot_parameter_blocks.push_back(variables_to_optimize[block_used].robot_data());
      if (num_agents > 0)
      {
        agent_parameter_blocks.push_back(variables_to_optimize[block_used].agents_data());
      }
    }
    const unsigned int active_blocks = static_cast<unsigned int>(robot_parameter_blocks.size());
    const bool found_people = has_agent_parameters;
    std::vector<double*> social_parameter_blocks(robot_parameter_blocks.begin(), robot_parameter_blocks.end());
    if (has_agent_parameters)
    {
      social_parameter_blocks.insert(social_parameter_blocks.end(), agent_parameter_blocks.begin(),
                                     agent_parameter_blocks.begin() + active_blocks);
    }
    Eigen::Matrix<double, 2, 1> point(optim_positions[i + 1].params[kX], optim_positions[i + 1].params[kY]);
    const double counter_step = static_cast<double>(i) * time_step;
    if (found_people)
    {
      if (use_social_work_cost_)
      {
        auto* social_work_function_f =
            SocialWorkCost::Create(socialwork_w_, social_people_states, evolving_poses[0].pose, counter_step, i,
                                   time_step, control_horizon, block_length, active_blocks, has_agent_parameters,
                                   num_agents);
        add_enlarged_parameter_blocks(social_work_function_f, active_blocks, has_agent_parameters, num_agents);
        social_work_function_f->SetNumResiduals(1);
        problem.AddResidualBlock(social_work_function_f, NULL, social_parameter_blocks);
      }
      if (use_social_angle_cost_)
      {
        auto* agent_angle_function_f =
            AgentAngleCost::Create(agent_angle_w_, velocity_alignment_w_, social_people_states,
                                   evolving_poses[0].pose, i, time_step, control_horizon, block_length, active_blocks,
                                   has_agent_parameters, num_agents);
        add_enlarged_parameter_blocks(agent_angle_function_f, active_blocks, has_agent_parameters, num_agents);
        agent_angle_function_f->SetNumResiduals(1);
        problem.AddResidualBlock(agent_angle_function_f, NULL, social_parameter_blocks);
      }
      if (use_social_crossing_cost_)
      {
        auto* crossing_function_f =
            CrossingCost::Create(crossing_w_, crossing_bearing_w_, social_people_states, evolving_poses[0].pose, i,
                                 time_step, control_horizon, block_length, active_blocks, has_agent_parameters,
                                 num_agents);
        add_enlarged_parameter_blocks(crossing_function_f, active_blocks, has_agent_parameters, num_agents);
        crossing_function_f->SetNumResiduals(1);
        problem.AddResidualBlock(crossing_function_f, NULL, social_parameter_blocks);
      }
      if (use_social_proxemics_cost_)
      {
        auto* proxemics_function_f =
            ProxemicsCost::Create(proxemics_w_, people_states_for_cost, evolving_poses[0].pose, counter_step, i,
                                  time_step, control_horizon, block_length, active_blocks, has_agent_parameters,
                                  num_agents);
        add_enlarged_parameter_blocks(proxemics_function_f, active_blocks, has_agent_parameters, num_agents);
        proxemics_function_f->SetNumResiduals(1);
        problem.AddResidualBlock(proxemics_function_f, NULL, social_parameter_blocks);
      }
      if (agent_obstacle_w_ > 0.0)
      {
        auto* agent_obstacle_function_f =
            AgentObstacleCost::Create(agent_obstacle_w_, costmap, costmap_interpolator, people_states_for_cost,
                                      evolving_poses[0].pose, i, time_step, control_horizon, block_length,
                                      active_blocks, has_agent_parameters, num_agents);
        add_enlarged_parameter_blocks(agent_obstacle_function_f, active_blocks, has_agent_parameters, num_agents);
        agent_obstacle_function_f->SetNumResiduals(num_agents);
        problem.AddResidualBlock(agent_obstacle_function_f, NULL, social_parameter_blocks);
      }
      // "cv" adds no dynamics critic: the per-block velocity-reference cost already
      // pins agent velocities to the observed ones (constant-velocity model).
      if (agent_sfm_dynamics_w_ > 0.0 && i < control_horizon && i % block_length == 0 &&
          agent_dynamics_model_ != "cv")
      {
        ceres::DynamicCostFunction* agent_dynamics_function_f;
        if (agent_dynamics_model_ == "orca")
        {
          agent_dynamics_function_f =
              AgentOrcaDynamicsCost::Create(agent_sfm_dynamics_w_, agent_max_accel_, people_states_for_cost,
                                            evolving_poses[0].pose, i, time_step, control_horizon, block_length,
                                            active_blocks, has_agent_parameters, num_agents, agent_cooperation_,
                                            orca_params_);
        }
        else
        {
          agent_dynamics_function_f =
              AgentSfmDynamicsCost::Create(agent_sfm_dynamics_w_, agent_max_accel_, people_states_for_cost,
                                           evolving_poses[0].pose, i, time_step, control_horizon, block_length,
                                           active_blocks, has_agent_parameters, num_agents, agent_cooperation_,
                                           sfm_params_);
        }
        add_enlarged_parameter_blocks(agent_dynamics_function_f, active_blocks, has_agent_parameters, num_agents);
        agent_dynamics_function_f->SetNumResiduals(kAgentVelocityParamStride * num_agents);
        problem.AddResidualBlock(agent_dynamics_function_f, NULL, social_parameter_blocks);
      }
    }
    if (use_social_path_follow_cost_)
    {
      auto* path_follow_cost_function_f = DistanceCost::Create(distance_w_, final_trajectorized_point,
                                                               evolving_poses[0].pose, i, time_step, control_horizon,
                                                               block_length);
      add_robot_parameter_blocks(path_follow_cost_function_f, active_blocks);
      path_follow_cost_function_f->SetNumResiduals(1);
      problem.AddResidualBlock(path_follow_cost_function_f, NULL, robot_parameter_blocks);
    }
    if (use_social_path_align_cost_)
    {
      auto* path_align_cost_function_f =
          DistanceCost::Create(angle_w_, point, evolving_poses[0].pose, i, time_step, control_horizon, block_length);
      add_robot_parameter_blocks(path_align_cost_function_f, active_blocks);
      path_align_cost_function_f->SetNumResiduals(1);
      problem.AddResidualBlock(path_align_cost_function_f, NULL, robot_parameter_blocks);
    }
    double velocity_weight = velocity_w_;

    auto* velocity_function_f =
        VelocityCost::Create(velocity_weight, desired_linear_vel_, i, control_horizon, block_length);
        
    Eigen::Matrix<double, 2, 1> final_heading(optim_headings.back().params[kHeadingTimeParam],
                                              optim_headings.back().params[kHeadingYawParam]);
    auto* goal_align_cost_function_f = GoalAlignCost::Create(goal_align_w_, final_heading, evolving_poses[0].pose, i,
                                                             time_step, control_horizon, block_length);
    add_robot_parameter_blocks(velocity_function_f, active_blocks);
    add_robot_parameter_blocks(goal_align_cost_function_f, active_blocks);

    velocity_function_f->SetNumResiduals(1);
    goal_align_cost_function_f->SetNumResiduals(1);

    problem.AddResidualBlock(velocity_function_f, NULL, robot_parameter_blocks);
    problem.AddResidualBlock(goal_align_cost_function_f, NULL, robot_parameter_blocks);

    // Humans beyond max_agents: constant-velocity extrapolated to this step and fed to the
    // robot-blocks-only ProxemicsCost. They are constants here, so they add one residual and
    // ZERO decision variables -- the enlarged state (and therefore the solve cost that
    // max_agents is capping) is untouched. Non-reactive by construction, which is exactly
    // right for `cv` humans and a sane short-horizon approximation for the rest.
    if (background_agent_w_ > 0.0 && !background_agents_.empty())
    {
      const double t = static_cast<double>(i) * time_step;
      AgentsStates projected = background_agents_;
      for (auto& a : projected)
      {
        a[kStateX] += a[kStateLinearVelocity] * std::cos(a[kStateYaw]) * t;
        a[kStateY] += a[kStateLinearVelocity] * std::sin(a[kStateYaw]) * t;
      }
      auto* background_cost_function_f = ProxemicsCost::Create(
          background_agent_w_, projected, evolving_poses[0].pose, counter_step, i, time_step, control_horizon,
          block_length);
      add_robot_parameter_blocks(background_cost_function_f, active_blocks);
      background_cost_function_f->SetNumResiduals(1);
      problem.AddResidualBlock(background_cost_function_f, NULL, robot_parameter_blocks);
    }

    // add the obstacle cost function, which is used to avoid obstacles
    // the obstacle cost function is used to avoid obstacles, it takes the costmap and the interpolator as parameters
    auto* obs_cost_function_f = ObstacleCost::Create(obstacle_w_, costmap, costmap_interpolator, evolving_poses[0].pose,
                                                     i, time_step, control_horizon, block_length);
    add_robot_parameter_blocks(obs_cost_function_f, active_blocks);
    obs_cost_function_f->SetNumResiduals(1);
    problem.AddResidualBlock(obs_cost_function_f, NULL, robot_parameter_blocks);
    if (goal_proximity_w_ > 0.0)
    {
      auto* goal_proximity_cost_function_f =
          GoalProximityCost::Create(goal_proximity_w_, goal_proximity_activation_radius_,
                                    goal_proximity_decay_distance_, goal_pose_in, evolving_poses[0].pose, i, time_step,
                                    control_horizon, block_length);
      add_robot_parameter_blocks(goal_proximity_cost_function_f, active_blocks);
      goal_proximity_cost_function_f->SetNumResiduals(1);
      problem.AddResidualBlock(goal_proximity_cost_function_f, NULL, robot_parameter_blocks);
    }
  }

  const unsigned int block_count =
      block_length > 0 ? static_cast<unsigned int>(robot_parameter_blocks.size()) : 0;
  if (has_agent_parameters && agent_velocity_reference_w_ > 0.0)
  {
    const unsigned int agent_block_dim = static_cast<unsigned int>(kAgentVelocityParamStride * num_agents);
    for (unsigned int i = 0; i < block_count; ++i)
    {
      auto* agent_velocity_reference_cost_function_f =
          AgentVelocityReferenceCost::Create(agent_velocity_reference_w_, people_states_for_cost, num_agents);
      agent_velocity_reference_cost_function_f->AddParameterBlock(agent_block_dim);
      agent_velocity_reference_cost_function_f->SetNumResiduals(agent_block_dim);
      problem.AddResidualBlock(agent_velocity_reference_cost_function_f, NULL, agent_parameter_blocks[i]);
    }
  }
  for (unsigned int i = 1; i < block_count; i++)
  {
    auto* velocity_feasibility_cost_function_f =
        VelocityFeasibilityCost::Create(velocity_feasibility_w_, i, block_count);
    problem.AddResidualBlock(velocity_feasibility_cost_function_f, NULL, robot_parameter_blocks[i],
                             robot_parameter_blocks[i - 1]);
    if (has_agent_parameters)
    {
      auto* agent_velocity_feasibility_cost_function_f =
          AgentVelocityFeasibilityCost::Create(velocity_feasibility_w_, i, block_count, num_agents);
      const unsigned int agent_block_dim = static_cast<unsigned int>(kAgentVelocityParamStride * num_agents);
      agent_velocity_feasibility_cost_function_f->AddParameterBlock(agent_block_dim);
      agent_velocity_feasibility_cost_function_f->AddParameterBlock(agent_block_dim);
      agent_velocity_feasibility_cost_function_f->SetNumResiduals(num_agents);
      problem.AddResidualBlock(agent_velocity_feasibility_cost_function_f, NULL, agent_parameter_blocks[i],
                               agent_parameter_blocks[i - 1]);
    }
  }
  for (unsigned int i = 0; i < block_count; i++)
  {
    auto* robot_block = robot_parameter_blocks[i];
    problem.SetParameterLowerBound(robot_block, kRobotLinearVelocityParam, min_linear_vel_);
    problem.SetParameterUpperBound(robot_block, kRobotLinearVelocityParam, max_linear_vel_);
    problem.SetParameterLowerBound(robot_block, kRobotAngularVelocityParam, -max_angular_vel_);
    problem.SetParameterUpperBound(robot_block, kRobotAngularVelocityParam, max_angular_vel_);

    if (has_agent_parameters)
    {
      auto* agent_block = agent_parameter_blocks[i];
      for (unsigned int j = 0; j < num_agents; j++)
      {
        const auto& agent_state = (people_proj.empty() || people_proj[0].size() <= j)
                                      ? fallback_agents[j]
                                      : people_proj[0][j];
        const bool agent_is_stationary = agent_state[kStateLinearVelocity] < stationary_agent_speed_threshold_;
        const double bound = agent_is_stationary ? stationary_agent_velocity_bound_ : agent_velocity_bound_;
        if (agent_is_stationary)
        {
          RCLCPP_WARN_STREAM(rclcpp::get_logger("optimizer"),
                             "Agent " << j << " detected as stationary ("
                                      << agent_state[kStateLinearVelocity] << " m/s), constraining velocity params to +-"
                                      << stationary_agent_velocity_bound_);
        }
        const unsigned int idx = kAgentVelocityParamStride * j;
        problem.SetParameterLowerBound(agent_block, idx + kAgentVxParam, -bound);
        problem.SetParameterUpperBound(agent_block, idx + kAgentVxParam, bound);
        problem.SetParameterLowerBound(agent_block, idx + kAgentVyParam, -bound);
        problem.SetParameterUpperBound(agent_block, idx + kAgentVyParam, bound);
      }
    }
  }
  ceres::Solve(options_, &problem, &summary);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("optimizer"), "Brief report: " << summary.BriefReport() << std::endl);

  if (debug_)
  {
    // One line per solve: everything needed to decide whether the control rate is limited by
    // the linear solver, by the iteration cap, or by the size of the enlarged state.
    // hit_iter_cap=1 means max_iterations bound the solve (pure wasted time if it had converged).
    const bool hit_iter_cap = summary.iterations.size() >= static_cast<size_t>(options_.max_num_iterations);
    // Autodiff tax = per-call Jacobian-eval cost / per-call residual-eval cost. Jacobian evals
    // run once per iteration; residual (cost-only) evals run more often in the line search, so
    // both times must be normalised by their own call counts before dividing. This ratio is the
    // ceiling analytic Jacobians could remove: tax->~1.5-2x instead of whatever this prints.
    const double jac_per_call = summary.num_jacobian_evaluations > 0 ?
        summary.jacobian_evaluation_time_in_seconds / summary.num_jacobian_evaluations : 0.0;
    const double res_per_call = summary.num_residual_evaluations > 0 ?
        summary.residual_evaluation_time_in_seconds / summary.num_residual_evaluations : 0.0;
    RCLCPP_INFO(rclcpp::get_logger("optimizer"),
                "[solve] %.1f ms (max %.1f Hz) | solver=%s | iters=%zu hit_iter_cap=%d | params=%d residuals=%d | "
                "agents=%u background=%zu | linsolve=%.1f ms jacobian=%.1f ms residual=%.1f ms | "
                "jac_evals=%d res_evals=%d autodiff_tax=%.1fx",
                summary.total_time_in_seconds * 1e3,
                summary.total_time_in_seconds > 0.0 ? 1.0 / summary.total_time_in_seconds : 0.0,
                ceres::LinearSolverTypeToString(options_.linear_solver_type), summary.iterations.size(),
                hit_iter_cap ? 1 : 0, problem.NumParameters(), problem.NumResiduals(), num_agents,
                background_agents_.size(), summary.linear_solver_time_in_seconds * 1e3,
                summary.jacobian_evaluation_time_in_seconds * 1e3,
                summary.residual_evaluation_time_in_seconds * 1e3, summary.num_jacobian_evaluations,
                summary.num_residual_evaluations, res_per_call > 0.0 ? jac_per_call / res_per_call : 0.0);
  }

  if (!summary.IsSolutionUsable())
  {
    RCLCPP_ERROR(rclcpp::get_logger("optimizer"), "Optimization failed!!!");
    return false;
  }

  for (unsigned int i = control_horizon / block_length; i < variables_to_optimize.size(); i++)
  {
    variables_to_optimize[i].robot.params[kRobotLinearVelocityParam] =
        variables_to_optimize[(control_horizon - 1) / block_length].robot.params[kRobotLinearVelocityParam];
    variables_to_optimize[i].robot.params[kRobotAngularVelocityParam] =
        variables_to_optimize[(control_horizon - 1) / block_length].robot.params[kRobotAngularVelocityParam];
  }
  std::vector<vel> saving_velocities;
  for (unsigned int i = 0; i < control_horizon; i++)
  {
    vel v;
    unsigned int block_idx = i / (block_length);
    v.params[kRobotLinearVelocityParam] =
        variables_to_optimize[block_idx].robot.params[kRobotLinearVelocityParam];
    v.params[kRobotAngularVelocityParam] =
        variables_to_optimize[block_idx].robot.params[kRobotAngularVelocityParam];
    saving_velocities.push_back(v);
  }
  for (unsigned int i = control_horizon; i < (variables_to_optimize.size() + 1); i++)
  {
    vel v;
    unsigned int block_idx = (i - 1);
    v.params[kRobotLinearVelocityParam] =
        variables_to_optimize[block_idx].robot.params[kRobotLinearVelocityParam];
    v.params[kRobotAngularVelocityParam] =
        variables_to_optimize[block_idx].robot.params[kRobotAngularVelocityParam];
    saving_velocities.push_back(v);
  }
  cmds.resize(saving_velocities.size());
  for (unsigned int i = 0; i < saving_velocities.size(); i++)
  {
    cmds[i].header = path.header;
    cmds[i].twist.linear.x = saving_velocities[i].params[kRobotLinearVelocityParam];
    cmds[i].twist.linear.y = 0.0;
    cmds[i].twist.angular.z = saving_velocities[i].params[kRobotAngularVelocityParam];
  }
  people_proj.clear();
  if (has_agent_parameters && block_count > 0)
  {
    // project the agents' states forward in time using the optimized velocities, 
    // to get the projected agents' states for each time step
    AgentsStates projected_agents = init_people;
    people_proj.push_back(projected_agents);
    for (unsigned int step = 0; step < saving_velocities.size(); ++step)
    {
      const unsigned int block_idx =
          step < control_horizon ? std::min(step / block_length, block_count - 1) : block_count - 1;
      const auto& agent_block = variables_to_optimize[block_idx].agents;
      for (unsigned int agent_idx = 0; agent_idx < num_agents; ++agent_idx)
      {
        auto& agent_state = projected_agents[agent_idx];
        if (agent_state[kStateTime] == -1.0)
        {
          continue;
        }
        const unsigned int param_idx = kAgentVelocityParamStride * agent_idx;
        const double vx = agent_block[param_idx + kAgentVxParam];
        const double vy = agent_block[param_idx + kAgentVyParam];
        const double previous_yaw = agent_state[kStateYaw];
        const double new_yaw = std::hypot(vx, vy) > 1e-6 ? std::atan2(vy, vx) : previous_yaw;
        const double yaw_delta = std::atan2(std::sin(new_yaw - previous_yaw), std::cos(new_yaw - previous_yaw));
        agent_state[kStateX] += vx * time_step;
        agent_state[kStateY] += vy * time_step;
        agent_state[kStateYaw] = new_yaw;
        agent_state[kStateTime] = static_cast<double>(step + 1) * time_step;
        agent_state[kStateLinearVelocity] = std::hypot(vx, vy);
        agent_state[kStateAngularVelocity] = yaw_delta / time_step;
      }
      people_proj.push_back(projected_agents);
    }
  }

  path.poses.clear();
  geometry_msgs::msg::PoseStamped pose_old;
  geometry_msgs::msg::PoseStamped previous_pose_old;
  pose_old.header = path.header;
  pose_old.pose.position.x = evolving_poses[0].pose.position.x;
  pose_old.pose.position.y = evolving_poses[0].pose.position.y;
  pose_old.pose.orientation = evolving_poses[0].pose.orientation;
  previous_pose_old.pose.position.x = evolving_poses[0].pose.position.x;
  previous_pose_old.pose.position.y = evolving_poses[0].pose.position.y;
  previous_pose_old.pose.orientation = evolving_poses[0].pose.orientation;

  for (auto vel : saving_velocities)
  {
    pose_old.pose.position.x = previous_pose_old.pose.position.x +
                               vel.params[kRobotLinearVelocityParam] *
                                   cos(tf2::getYaw(previous_pose_old.pose.orientation)) * time_step;
    pose_old.pose.position.y = previous_pose_old.pose.position.y +
                               vel.params[kRobotLinearVelocityParam] *
                                   sin(tf2::getYaw(previous_pose_old.pose.orientation)) * time_step;
    tf2::Quaternion uao;
    uao.setRPY(0, 0, tf2::getYaw(previous_pose_old.pose.orientation) +
                         vel.params[kRobotAngularVelocityParam] * time_step);
    pose_old.pose.orientation = tf2::toMsg(uao);

    previous_pose_old.pose.position.x = pose_old.pose.position.x;
    previous_pose_old.pose.position.y = pose_old.pose.position.y;
    previous_pose_old.pose.orientation = pose_old.pose.orientation;

    path.poses.push_back(pose_old);
  }

  {
    std::lock_guard<std::mutex> lock(memory.mtx);
    memory.previous_path = path;
    memory.previous_cmds = cmds;
  }

  return true;
}

double Optimizer::agent_relevance(const AgentStatus& agent, const geometry_msgs::msg::Pose& robot_pose,
                                  const geometry_msgs::msg::Twist& speed) const
{
  // Relative state of the agent with respect to the robot.
  const double px = agent[kStateX] - robot_pose.position.x;
  const double py = agent[kStateY] - robot_pose.position.y;

  const double robot_yaw = tf2::getYaw(robot_pose.orientation);
  const double vx = agent[kStateLinearVelocity] * std::cos(agent[kStateYaw]) - speed.linear.x * std::cos(robot_yaw);
  const double vy = agent[kStateLinearVelocity] * std::sin(agent[kStateYaw]) - speed.linear.x * std::sin(robot_yaw);

  // Closest point of approach under constant velocity, clamped to the MPC horizon: an agent that
  // is far but closing fast outranks a closer one that is walking away or already behind the robot.
  const double speed_sq = vx * vx + vy * vy;
  double t_cpa = 0.0;
  if (speed_sq > 1e-9)
  {
    t_cpa = std::clamp(-(px * vx + py * vy) / speed_sq, 0.0, static_cast<double>(max_time));
  }

  const double dx = px + vx * t_cpa;
  const double dy = py + vy * t_cpa;
  return std::sqrt(dx * dx + dy * dy);
}

void Optimizer::update_cooperation(TrackedAgent& track, const geometry_msgs::msg::Pose& robot_pose)
{
  // ponytail: constant prior, no online estimation. The track already carries the
  // history needed to infer this (did the person deviate as the robot closed in?);
  // upgrade to a per-agent posterior when a fixed prior provably is not enough.
  (void)track;
  (void)robot_pose;
}

AgentsStates Optimizer::people_to_status(const people_msgs::msg::People& people, double time_step,
                                         const geometry_msgs::msg::Pose& robot_pose,
                                         const geometry_msgs::msg::Twist& speed)
{
  const size_t cap = max_agents_ > 0 ? max_agents_ : std::numeric_limits<size_t>::max();
  double current_time = path_time_.seconds();
  if (current_time <= 0.0)
  {
    current_time = have_tracking_time_ ? last_tracking_time_ + time_step : 0.0;
  }

  for (auto& track : tracked_agents_)
  {
    const double dt = std::max(0.0, current_time - track.last_update_time);
    if (dt <= 0.0)
    {
      continue;
    }

    const double vx = track.state[kStateLinearVelocity] * std::cos(track.state[kStateYaw]);
    const double vy = track.state[kStateLinearVelocity] * std::sin(track.state[kStateYaw]);
    track.state[kStateX] += vx * dt;
    track.state[kStateY] += vy * dt;

    const double age = std::max(0.0, current_time - track.last_seen_time);
    if (age > 0.0 && agent_coast_decay_time_ > 1e-6)
    {
      track.state[kStateLinearVelocity] *= std::exp(-dt / agent_coast_decay_time_);
    }

    track.state[kStateTime] = age;
    track.last_update_time = current_time;
  }

  AgentsStates detections;
  detections.reserve(people.people.size());
  for (const auto& p : people.people)
  {
    double yaw = atan2(p.velocity.y, p.velocity.x);
    double lv = sqrt(p.velocity.x * p.velocity.x + p.velocity.y * p.velocity.y);
    AgentStatus st;
    if (lv < 1e-6)
    {
      lv = 1e-6;
    }
    st << (double)p.position.x, (double)p.position.y, yaw, 0.0, lv, (double)p.velocity.z;
    detections.push_back(st);
  }

  std::vector<bool> matched_tracks(tracked_agents_.size(), false);
  const double association_radius_sq = agent_association_radius_ * agent_association_radius_;
  for (const auto& detection : detections)
  {
    size_t best_track = tracked_agents_.size();
    double best_distance_sq = association_radius_sq;
    for (size_t track_idx = 0; track_idx < tracked_agents_.size(); ++track_idx)
    {
      if (matched_tracks[track_idx])
      {
        continue;
      }

      const double dx = tracked_agents_[track_idx].state[kStateX] - detection[kStateX];
      const double dy = tracked_agents_[track_idx].state[kStateY] - detection[kStateY];
      const double distance_sq = dx * dx + dy * dy;
      if (distance_sq <= best_distance_sq)
      {
        best_distance_sq = distance_sq;
        best_track = track_idx;
      }
    }

    // Take the fastest speed the person has shown over the window rather than the
    // latest sample: an under-read speed makes the SFM prediction lag behind where
    // they will actually be, which is the direction that ends in a collision.
    auto apply_speed_window = [this](TrackedAgent& track, const AgentStatus& detection) {
      track.state = detection;
      track.speed_window.push_back(detection[kStateLinearVelocity]);
      while (track.speed_window.size() > agent_speed_window_)
      {
        track.speed_window.pop_front();
      }
      track.state[kStateLinearVelocity] =
          *std::max_element(track.speed_window.begin(), track.speed_window.end());
    };

    if (best_track < tracked_agents_.size())
    {
      apply_speed_window(tracked_agents_[best_track], detection);
      tracked_agents_[best_track].last_seen_time = current_time;
      tracked_agents_[best_track].last_update_time = current_time;
      matched_tracks[best_track] = true;
    }
    else
    {
      TrackedAgent track;
      apply_speed_window(track, detection);
      track.cooperation = human_cooperation_factor_;
      track.last_seen_time = current_time;
      track.last_update_time = current_time;
      tracked_agents_.push_back(track);
      matched_tracks.push_back(true);
    }
  }

  tracked_agents_.erase(
      std::remove_if(tracked_agents_.begin(), tracked_agents_.end(),
                     [this, current_time](const TrackedAgent& track) {
                       return current_time - track.last_seen_time > agent_track_timeout_;
                     }),
      tracked_agents_.end());

  for (auto& track : tracked_agents_)
  {
    track.state[kStateTime] = std::max(0.0, current_time - track.last_seen_time);
  }

  // Rank the tracks by interaction relevance so that, when max_agents truncates the enlarged
  // state, the agents that survive are the ones the robot actually has to negotiate with.
  std::sort(tracked_agents_.begin(), tracked_agents_.end(),
            [this, &robot_pose, &speed](const TrackedAgent& a, const TrackedAgent& b) {
              return agent_relevance(a.state, robot_pose, speed) < agent_relevance(b.state, robot_pose, speed);
            });

  AgentsStates people_status;
  const size_t kept = std::min(cap, tracked_agents_.size());
  people_status.reserve(kept);
  agent_cooperation_.clear();
  agent_cooperation_.reserve(kept);
  for (size_t i = 0; i < kept; ++i)
  {
    update_cooperation(tracked_agents_[i], robot_pose);
    people_status.push_back(tracked_agents_[i].state);
    agent_cooperation_.push_back(tracked_agents_[i].cooperation);
  }

  // Everyone the cap truncated away. They stay out of the enlarged state (no decision
  // variables, never co-optimized) but are still handed to a proxemics residual so the robot
  // is not blind to them -- see background_agent_w_.
  background_agents_.clear();
  background_agents_.reserve(tracked_agents_.size() - kept);
  for (size_t i = kept; i < tracked_agents_.size(); ++i)
  {
    background_agents_.push_back(tracked_agents_[i].state);
  }

  last_tracking_time_ = current_time;
  have_tracking_time_ = true;
  return people_status;
}

AgentTrajectory Optimizer::format_to_optimize(nav_msgs::msg::Path& path, const nav_msgs::msg::Path& previous_path,
                                              const std::vector<geometry_msgs::msg::TwistStamped>& cmds,
                                              const std::vector<geometry_msgs::msg::TwistStamped>& previous_cmds,
                                              const geometry_msgs::msg::Twist& speed, const float current_path_w,
                                              const float current_cmds_w, const float maxtime, const float timestep)
{
  // we check the timestep and the path size in order to cut the path
  // to a maximum duration.
  int maxsize = (int)round(maxtime / timestep);
  if ((int)path.poses.size() > maxsize)
  {
    std::vector<geometry_msgs::msg::PoseStamped> p(path.poses.begin(), (path.poses.begin() + (maxsize - 1)));
    path.poses = p;
  }

  AgentTrajectory robot_status;
  for (unsigned int i = 0; i < path.poses.size(); i++)
  {
    //  Robot
    //  x, y, yaw, t, lv, av
    AgentStatus r;
    if (!previous_path.poses.empty() && i < previous_path.poses.size())
    {
      // blend current and previous poses for smoother transition
      geometry_msgs::msg::Pose smoothed;
      smoothed.position.x = current_path_w * path.poses[i].pose.position.x +
                            (1.0 - current_path_w) * previous_path.poses[i].pose.position.x;
      smoothed.position.y = current_path_w * path.poses[i].pose.position.y +
                            (1.0 - current_path_w) * previous_path.poses[i].pose.position.y;
      double yaw_current = tf2::getYaw(path.poses[i].pose.orientation);
      double yaw_prev = tf2::getYaw(previous_path.poses[i].pose.orientation);
      // Use angle-aware interpolation to avoid wrap-around errors near ±π
      double yaw_diff = std::atan2(std::sin(yaw_current - yaw_prev), std::cos(yaw_current - yaw_prev));
      double smoothed_yaw = yaw_prev + static_cast<double>(current_path_w) * yaw_diff;
      tf2::Quaternion q;
      q.setRPY(0, 0, smoothed_yaw);
      smoothed.orientation = tf2::toMsg(q);
      // update the current pose with the smoothed pose
      path.poses[i].pose = smoothed;
    }
    r(kStateX, 0) = path.poses[i].pose.position.x;
    r(kStateY, 0) = path.poses[i].pose.position.y;
    r(kStateYaw, 0) = tf2::getYaw(path.poses[i].pose.orientation);
    r(kStateTime, 0) = i * timestep;

    if (i == 0)
    {
      // Robot vel
      r(kStateLinearVelocity, 0) = speed.linear.x;
      r(kStateAngularVelocity, 0) = speed.angular.z;
    }
    else if (i - 1 < cmds.size())
    {
      const double lv_curr = cmds[i - 1].twist.linear.x;
      const double av_curr = cmds[i - 1].twist.angular.z;
      const double lv_prev = (i - 1 < previous_cmds.size()) ? previous_cmds[i - 1].twist.linear.x : lv_curr;
      const double av_prev = (i - 1 < previous_cmds.size()) ? previous_cmds[i - 1].twist.angular.z : av_curr;
      r(kStateLinearVelocity, 0) = current_cmds_w * lv_curr + (1.0 - current_cmds_w) * lv_prev;
      r(kStateAngularVelocity, 0) = current_cmds_w * av_curr + (1.0 - current_cmds_w) * av_prev;
    }
    else
    {
      // Beyond available commands: hold last known velocity
      r(kStateLinearVelocity, 0) = robot_status.empty() ? 0.0 : robot_status.back()(kStateLinearVelocity, 0);
      r(kStateAngularVelocity, 0) = robot_status.empty() ? 0.0 : robot_status.back()(kStateAngularVelocity, 0);
    }
    robot_status.push_back(r);
  }
  return robot_status;
}


}  // namespace mpc_enlarged_state
