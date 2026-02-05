#include "mpc_enlarged_state/optimizer.hpp"

#include <algorithm>
#include <limits>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tf2/utils.h"

namespace mpc_enlarged_state
{

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
  nav2_util::declare_parameter_if_not_declared(node, weights + "proxemics_weight", rclcpp::ParameterValue(90.0));
  node->get_parameter(weights + "proxemics_weight", proxemics_w_);
  nav2_util::declare_parameter_if_not_declared(node, weights + "velocity_feasibility_weight",
                                               rclcpp::ParameterValue(0.5));
  node->get_parameter(weights + "velocity_feasibility_weight", velocity_feasibility_w_);
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
  socialwork_w_ = params.socialwork_w_;
  distance_w_ = params.distance_w_;
  velocity_w_ = params.velocity_w_;
  angle_w_ = params.angle_w_;
  agent_angle_w_ = params.agent_angle_w_;
  proxemics_w_ = params.proxemics_w_;
  goal_proximity_w_ = params.goal_proximity_w_;
  use_social_work_cost_ = params.use_social_work_cost;
  use_social_angle_cost_ = params.use_social_angle_cost;
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
  // transfrom people to agent status factor
  AgentsStates init_people = people_to_status(people);

  // Path has always at least 2 points
  if (path.poses.size() < 2)
  {
    RCLCPP_WARN(rclcpp::get_logger("optimizer"), "Path has less than 2 points, cannot optimize");
    return false;
  }
  frame_ = path.header.frame_id;
  path_time_ = rclcpp::Time(path.header.stamp);
  const geometry_msgs::msg::Pose goal_pose_in = goal_pose.pose;

  // Create costmap grid
  costmap_grid_ = std::make_shared<ceres::Grid2D<u_char>>(costmap->getCharMap(), 0, costmap->getSizeInCellsY(), 0,
                                                          costmap->getSizeInCellsX());
  // create the bi-cubic interpolator for the costmap, to be used in the obstacles critic
  auto costmap_interpolator = std::make_shared<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>(*costmap_grid_);
  // this should keep the previous optimized path and commands
  // in order to use them as a starting point for the optimization
  // i do not know if this works as expected, but it should
  auto& memory = TrajectoryMemory::getInstance();

  // if no previous path is set, set the current path and commands as the previous ones
  if (memory.previous_path.poses.size() == 0)
  {
    memory.previous_path = path;
    memory.previous_cmds = cmds;
  }

  nav_msgs::msg::Path previous_path = memory.previous_path;
  std::vector<geometry_msgs::msg::TwistStamped> previous_cmds = memory.previous_cmds;

  // use the projected path to make it into the a parametrized format
  AgentsStates optim_status = format_to_optimize(path, previous_path, cmds, previous_cmds, speed, current_path_w,
                                                 current_cmds_w, max_time, time_step);
  people_proj.push_back(init_people);

  std::vector<agent_velocity> agent_uno_velocities;
  long unsigned int closest_agent_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  if (!people_proj.empty() && !people_proj[0].empty()) {
    double robot_x = optim_status[0][0];
    double robot_y = optim_status[0][1];
    for (size_t i = 0; i < people_proj[0].size(); ++i) {
      double dx = people_proj[0][i][0] - robot_x;
      double dy = people_proj[0][i][1] - robot_y;
      double dist = std::hypot(dx, dy);
      if (dist < min_dist) {
        min_dist = dist;
        closest_agent_idx = static_cast<long unsigned int>(i);
      }
    }
  }
  const size_t num_agents_size = init_people.size();
  const unsigned int num_agents = static_cast<unsigned int>(num_agents_size);

  AgentsStates fallback_agents;
  if (num_agents_size > 0)
  {
    fallback_agents.resize(num_agents_size);
    for (size_t k = 0; k < num_agents_size; ++k)
    {
      fallback_agents[k] = AgentStatus::Zero();
      fallback_agents[k][3] = -1.0;  // mark as invalid by default
    }
  }

  const AgentsStates& people_states_for_cost = (!people_proj.empty() ? people_proj.front() : fallback_agents);

  std::vector<dynamic_optimizing_velocities> variables_to_optimize;
  for (unsigned int j= 0; j < optim_status.size(); ++j)
  {
    dynamic_optimizing_velocities doa;
    doa.set_num_agents(static_cast<int>(num_agents));
    doa.params[0] = optim_status[j][4];
    doa.params[1] = optim_status[j][5];
     // fill the 2 coordinates for each of the num_agents
    for (unsigned int k = 0; k < num_agents; ++k) {
      const auto& agent_state = (people_proj.empty() || people_proj[0].size() <= k)
            ? fallback_agents[k]
            : people_proj[0][k];
      doa.params[2 + 2*k]     = agent_state[4]*ceres::cos(agent_state[2]);  // velocity x of agent k
      doa.params[2 + 2*k + 1] = agent_state[4]*ceres::sin(agent_state[2]);  // velocity y of agent k
    }
    variables_to_optimize.push_back(doa);
  }

  // Extract the closest agent's state at each time step
  AgentsStates tentativo;
  for (const auto& timestep : people_proj) {
    if (timestep.size() > closest_agent_idx) {
      tentativo.push_back(timestep[closest_agent_idx]);
    }
  }
  // Extract the first agent's state at each time step
  if (!tentativo.empty()){
    for (auto x : tentativo){
      agent_velocity av;
      av.params[0] = x[4];  // lv
      av.params[1] = x[5];  // av
      agent_uno_velocities.push_back(av);
    }
  } else {
    for (auto x : optim_status)
    {
      agent_velocity av;
      av.params[0] = x[0]*0.0;  // lv
      av.params[1] = x[0]*0.0;  // av
      agent_uno_velocities.push_back(av);
    }
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
    p.params[0] = a[0];  // x
    p.params[1] = a[1];  // y
    vel v;
    v.params[0] = a[4];  // lv
    v.params[1] = a[5];  // av
    linear_velocity lv;
    lv.params[0] = a[4];  // lv
    angular_velocity av;
    av.params[0] = a[5];  // av
    heading h;
    h.params[0] = a[3];  // t
    h.params[1] = a[2];  // yaw
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = a[0];
    pose.pose.position.y = a[1];
    pose.pose.position.z = 0.0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, a[2]);  // yaw
    pose.pose.orientation = tf2::toMsg(quaternion);
    optim_positions.push_back(p);
    optim_velocities.push_back(v);
    optim_headings.push_back(h);
    optim_linear_velocities.push_back(lv);
    optim_angular_velocities.push_back(av);
    evolving_poses.push_back(pose);
  }
  Eigen::Matrix<double, 2, 1> final_trajectorized_point(optim_positions[optim_status.size() - 1].params[0],
                                                        optim_positions[optim_status.size() - 1].params[1]);

  optim_velocities.pop_back();

  // setting ceres variables
  ceres::Problem problem;
  ceres::Solver::Summary summary;

  // set the value of control horizon, if size of velociteies is smaller than control horizon, set it to the size of
  // velocities also set the block length, if it is larger than the control horizon, set it to the control horizon
  //double counter = 0.0;
  std::vector<double*> parameter_blocks;
  unsigned int control_horizon = std::min(control_horizon_, static_cast<unsigned int>(optim_velocities.size()));
  unsigned int block_length = std::min(parameter_block_length_, control_horizon);
  for (unsigned int i = 0; i < optim_velocities.size(); i++)  // i is the index of the current time step
  {
    
    //counter = counter + 1.0;
    unsigned int block_used = i / block_length;

    // add the velocities to optimize
    if (i < control_horizon &&
        (parameter_blocks.empty() || parameter_blocks.back() != variables_to_optimize[block_used].params.data()))
    {
      parameter_blocks.push_back(variables_to_optimize[block_used].params.data());
    }
    bool found_people = false;
    if (num_agents > 0){
      found_people = true;
    }
    Eigen::Matrix<double, 2, 1> point(optim_positions[i + 1].params[0], optim_positions[i + 1].params[1]);
    auto* overall_social_cost_function_f =
  SocialOverallCost::Create(socialwork_w_, agent_angle_w_, proxemics_w_,distance_w_, angle_w_, final_trajectorized_point, point, people_states_for_cost, num_agents, evolving_poses[0].pose,
                                           i, time_step, control_horizon, block_length,found_people, use_social_work_cost_,
                                           use_social_angle_cost_, use_social_proxemics_cost_, use_social_path_follow_cost_,
                                           use_social_path_align_cost_);
    unsigned int b = 2 + 2*(unsigned int)num_agents;
    if (i < control_horizon)
    {
      for (unsigned int j = 0; j <= i / block_length; j++)
      {
        overall_social_cost_function_f->AddParameterBlock(b);  // Each velocity block has 2 params (v, ω)
      }
    }
    else
    {
      for (unsigned int j = 0; j <= (control_horizon - 1) / block_length; j++)
      {
        overall_social_cost_function_f->AddParameterBlock(b);  // Each velocity block has 2 params (v, ω)
      }
    }
    unsigned int a = 5;
    overall_social_cost_function_f->SetNumResiduals(a);
    problem.AddResidualBlock(overall_social_cost_function_f, NULL, parameter_blocks);
    double velocity_weight = velocity_w_;
    if (use_adaptive_velocity_cost_)
    {
      double dx_goal = goal_pose_in.position.x - optim_positions[i + 1].params[0];
      double dy_goal = goal_pose_in.position.y - optim_positions[i + 1].params[1];
      double dist_to_goal = std::hypot(dx_goal, dy_goal);
      double distance_scale = adaptive_velocity_distance_ > 1e-6
                                  ? std::clamp(dist_to_goal / adaptive_velocity_distance_, 0.0, 1.0)
                                  : 1.0;
      double min_scale = std::clamp(adaptive_velocity_min_scale_, 0.0, 1.0);
      double adaptive_scale = min_scale + (1.0 - min_scale) * distance_scale;
      velocity_weight *= adaptive_scale;
    }
    auto* velocity_function_f =
        VelocityCost::Create(velocity_weight, desired_linear_vel_, i, control_horizon, block_length);
        
    Eigen::Matrix<double, 2, 1> final_heading(optim_headings.back().params[0], optim_headings.back().params[1]);
    auto* goal_align_cost_function_f = GoalAlignCost::Create(goal_align_w_, final_heading, evolving_poses[0].pose, i,
                                                             time_step, control_horizon, block_length);
    if (i < control_horizon)
    {
      for (unsigned int j = 0; j <= i / block_length; j++)
      {
        // Each velocity block has 2 params (v, ω)
        velocity_function_f->AddParameterBlock(b);
        goal_align_cost_function_f->AddParameterBlock(2);
      }
    }
    else
    {
      for (unsigned int j = 0; j <= (control_horizon - 1) / block_length; j++)
      {
        // Each velocity block has 2 params (v, ω)
        velocity_function_f->AddParameterBlock(b);
        goal_align_cost_function_f->AddParameterBlock(2);
      }
    }

    velocity_function_f->SetNumResiduals(1);
    goal_align_cost_function_f->SetNumResiduals(1);

    problem.AddResidualBlock(velocity_function_f, NULL, parameter_blocks);
    problem.AddResidualBlock(goal_align_cost_function_f, NULL, parameter_blocks);

    // add the obstacle cost function, which is used to avoid obstacles
    // the obstacle cost function is used to avoid obstacles, it takes the costmap and the interpolator as parameters
    auto* obs_cost_function_f = ObstacleCost::Create(obstacle_w_, costmap, costmap_interpolator, evolving_poses[0].pose,
                                                     i, time_step, control_horizon, block_length);
    if (i < control_horizon)
    {
      for (unsigned int j = 0; j <= i / block_length; j++)
      {
        obs_cost_function_f->AddParameterBlock(b);
      }
    }
    else
    {
      for (unsigned int j = 0; j <= (control_horizon - 1) / block_length; j++)
      {
        obs_cost_function_f->AddParameterBlock(b);
      }
    }
    obs_cost_function_f->SetNumResiduals(1);
    problem.AddResidualBlock(obs_cost_function_f, NULL, parameter_blocks);
    if (goal_proximity_w_ > 0.0)
    {
      auto* goal_proximity_cost_function_f =
          GoalProximityCost::Create(goal_proximity_w_, goal_proximity_activation_radius_,
                                    goal_proximity_decay_distance_, goal_pose_in, evolving_poses[0].pose, i, time_step,
                                    control_horizon, block_length);
      if (i < control_horizon)
      {
        for (unsigned int j = 0; j <= i / block_length; j++)
        {
          goal_proximity_cost_function_f->AddParameterBlock(b);
        }
      }
      else
      {
        for (unsigned int j = 0; j <= (control_horizon - 1) / block_length; j++)
        {
          goal_proximity_cost_function_f->AddParameterBlock(b);
        }
      }
      goal_proximity_cost_function_f->SetNumResiduals(1);
      problem.AddResidualBlock(goal_proximity_cost_function_f, NULL, parameter_blocks);
    }
    if (i != 0 && i < control_horizon / block_length)
    {
      auto* velocity_feasibility_cost_function_f =
          VelocityFeasibilityCost::Create(velocity_feasibility_w_, i, control_horizon);
      problem.AddResidualBlock(velocity_feasibility_cost_function_f, NULL, optim_velocities[i].params,
                               optim_velocities[i - 1].params);
    }
  }

  for (unsigned int i = 0; i < control_horizon / block_length; i++)
  {
    problem.SetParameterLowerBound(variables_to_optimize[i].params.data(), 0, min_linear_vel_);   // lower bound for linear velocity
    problem.SetParameterUpperBound(variables_to_optimize[i].params.data(), 0, max_linear_vel_);   // upper bound for linear velocity
    problem.SetParameterLowerBound(variables_to_optimize[i].params.data(), 1, -max_angular_vel_);  // lower bound for angular velocity
    problem.SetParameterUpperBound(variables_to_optimize[i].params.data(), 1, max_angular_vel_);   // upper bound for angular velocity
    for (unsigned int j = 0; j < num_agents; j++) {
      const auto& agent_state = (people_proj.empty() || people_proj[0].size() <= j)
                                    ? fallback_agents[j]
                                    : people_proj[0][j];
      const bool agent_is_stationary = agent_state[4] < stationary_agent_speed_threshold_;
      const double bound = agent_is_stationary ? stationary_agent_velocity_bound_ : agent_velocity_bound_;
      if (agent_is_stationary)
      {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("optimizer"),
                           "Agent " << j << " detected as stationary ("
                                    << agent_state[4] << " m/s), constraining velocity params to +-"
                                    << stationary_agent_velocity_bound_);
      }
      problem.SetParameterLowerBound(variables_to_optimize[i].params.data(), 2 + 2 * j, -bound);
      problem.SetParameterUpperBound(variables_to_optimize[i].params.data(), 2 + 2 * j, bound);
      problem.SetParameterLowerBound(variables_to_optimize[i].params.data(), 2 + 2 * j + 1, -bound);
      problem.SetParameterUpperBound(variables_to_optimize[i].params.data(), 2 + 2 * j + 1, bound);
    }
  }
  ceres::Solve(options_, &problem, &summary);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("optimizer"), "Brief report: " << summary.BriefReport() << std::endl);

  if (!summary.IsSolutionUsable())
  {
    RCLCPP_ERROR(rclcpp::get_logger("optimizer"), "Optimization failed!!!");
    return false;
  }

  for (unsigned int i = control_horizon / block_length; i < variables_to_optimize.size(); i++)
  {
    variables_to_optimize[i].params[0] = variables_to_optimize[(control_horizon - 1) / block_length].params[0];
    variables_to_optimize[i].params[1] = variables_to_optimize[(control_horizon - 1) / block_length].params[1];
  }
  std::vector<vel> saving_velocities;
  for (unsigned int i = 0; i < control_horizon; i++)
  {
    vel v;
    unsigned int block_idx = i / (block_length);
    v.params[0] = variables_to_optimize[block_idx].params[0];
    v.params[1] = variables_to_optimize[block_idx].params[1];
    saving_velocities.push_back(v);
  }
  for (unsigned int i = control_horizon; i < (variables_to_optimize.size() + 1); i++)
  {
    vel v;
    unsigned int block_idx = (i - 1);
    v.params[0] = variables_to_optimize[block_idx].params[0];
    v.params[1] = variables_to_optimize[block_idx].params[1];
    saving_velocities.push_back(v);
  }
  cmds.resize(saving_velocities.size());
  for (unsigned int i = 0; i < saving_velocities.size(); i++)
  {
    cmds[i].header = path.header;
    cmds[i].twist.linear.x = saving_velocities[i].params[0];
    cmds[i].twist.linear.y = 0.0;
    cmds[i].twist.angular.z = saving_velocities[i].params[1];
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
                               vel.params[0] * cos(tf2::getYaw(previous_pose_old.pose.orientation)) * time_step;
    pose_old.pose.position.y = previous_pose_old.pose.position.y +
                               vel.params[0] * sin(tf2::getYaw(previous_pose_old.pose.orientation)) * time_step;
    tf2::Quaternion uao;
    uao.setRPY(0, 0, tf2::getYaw(previous_pose_old.pose.orientation) + vel.params[1] * time_step);
    pose_old.pose.orientation = tf2::toMsg(uao);

    previous_pose_old.pose.position.x = pose_old.pose.position.x;
    previous_pose_old.pose.position.y = pose_old.pose.position.y;
    previous_pose_old.pose.orientation = pose_old.pose.orientation;

    path.poses.push_back(pose_old);
  }

  memory.previous_path = path;
  memory.previous_cmds = cmds;

  return true;
}

AgentsStates Optimizer::people_to_status(const people_msgs::msg::People& people)
{
  AgentsStates people_status;
  const size_t cap = max_agents_ > 0 ? max_agents_ : std::numeric_limits<size_t>::max();
  // the agents status contain 5 values:
  // x, y, yaw, timestamp, lv, av
  for (auto p : people.people)
  {
    if (people_status.size() >= cap)
    {
      break;
    }
    double yaw = atan2(p.velocity.y, p.velocity.x);
    double lv = sqrt(p.velocity.x * p.velocity.x + p.velocity.y * p.velocity.y);
    AgentStatus st;
    if (lv < 1e-6)
      lv = 1e-6;
    st << (double)p.position.x, (double)p.position.y, yaw, 0.0, lv, (double)p.velocity.z;
    people_status.push_back(st);
  }
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
      double smoothed_yaw = current_path_w * yaw_current + (1.0 - current_path_w) * yaw_prev;
      tf2::Quaternion q;
      q.setRPY(0, 0, smoothed_yaw);
      smoothed.orientation = tf2::toMsg(q);
      // update the current pose with the smoothed pose
      path.poses[i].pose = smoothed;
    }
    r(0, 0) = path.poses[i].pose.position.x;
    r(1, 0) = path.poses[i].pose.position.y;
    r(2, 0) = tf2::getYaw(path.poses[i].pose.orientation);
    r(3, 0) = i * timestep;

    // t += timestep;

    if (i == 0)
    {
      // Robot vel
      r(4, 0) = speed.linear.x;
      r(5, 0) = speed.angular.z;
    }
    else
    {
      geometry_msgs::msg::TwistStamped cmd_smoothed;
      cmd_smoothed.twist.linear.x =
          current_cmds_w * cmds[i - 1].twist.linear.x + (1.0 - current_cmds_w) * previous_cmds[i - 1].twist.linear.x;
      cmd_smoothed.twist.angular.z =
          current_cmds_w * cmds[i - 1].twist.angular.z + (1.0 - current_cmds_w) * previous_cmds[i - 1].twist.angular.z;
      //  Robot vel
      r(4, 0) = cmd_smoothed.twist.linear.x;
      r(5, 0) = cmd_smoothed.twist.angular.z;
    }
    robot_status.push_back(r);
  }
  return robot_status;
}


}  // namespace mpc_enlarged_state
