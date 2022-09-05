// Copyright (c) 2022 SRL -Service Robotics Lab, Pablo de Olavide University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_SOCIAL_MPC_CONTROLLER__OPTIMIZER_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__OPTIMIZER_HPP_

#include "nav2_util/node_utils.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <limits>
#include <math.h>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"

// cost functions
#include "nav2_social_mpc_controller/curvature_cost_function.hpp"
#include "nav2_social_mpc_controller/distance_cost_function.hpp"
#include "nav2_social_mpc_controller/obstacle_cost_function.hpp"
#include "nav2_social_mpc_controller/sfm.hpp"
#include "nav2_social_mpc_controller/social_work_function.hpp"

#include "obstacle_distance_msgs/msg/obstacle_distance.hpp"
#include "people_msgs/msg/people.hpp"

// the agents status contain 6 values:
// x, y, yaw, timestamp, lv, av
// typedef Eigen::Matrix<double, 6, 1> AgentStatus;

namespace nav2_social_mpc_controller {

struct OptimizerParams {
  OptimizerParams()
      : debug(false), max_iterations(50), param_tol(1e-8), fn_tol(1e-6),
        gradient_tol(1e-10) {}

  /**
   * @brief Get params from ROS parameter
   * @param node Ptr to node
   * @param name Name of plugin
   */
  void get(rclcpp_lifecycle::LifecycleNode *node, const std::string &name) {
    std::string local_name = name + std::string(".optimizer.");

    // Optimizer params
    nav2_util::declare_parameter_if_not_declared(
        node, local_name + "linear_solver_type",
        rclcpp::ParameterValue(
            "SPARSE_NORMAL_CHOLESKY")); // SPARSE_NORMAL_CHOLESKY //DENSE_QR
    node->get_parameter(local_name + "linear_solver_type", linear_solver_type);
    if (solver_types.find(linear_solver_type) == solver_types.end()) {
      std::stringstream valid_types_str;
      for (auto type = solver_types.begin(); type != solver_types.end();
           type++) {
        if (type != solver_types.begin()) {
          valid_types_str << ", ";
        }
        valid_types_str << type->first;
      }
      RCLCPP_ERROR(rclcpp::get_logger("optimizer"),
                   "Invalid linear_solver_type. Valid values are %s",
                   valid_types_str.str().c_str());
      throw std::runtime_error("Invalid parameter: linear_solver_type");
    }
    nav2_util::declare_parameter_if_not_declared(node, local_name + "param_tol",
                                                 rclcpp::ParameterValue(1e-15));
    node->get_parameter(local_name + "param_tol", param_tol);
    nav2_util::declare_parameter_if_not_declared(node, local_name + "fn_tol",
                                                 rclcpp::ParameterValue(1e-7));
    node->get_parameter(local_name + "fn_tol", fn_tol);
    nav2_util::declare_parameter_if_not_declared(
        node, local_name + "gradient_tol", rclcpp::ParameterValue(1e-10));
    node->get_parameter(local_name + "gradient_tol", gradient_tol);
    nav2_util::declare_parameter_if_not_declared(
        node, local_name + "max_iterations", rclcpp::ParameterValue(100));
    node->get_parameter(local_name + "max_iterations", max_iterations);
    nav2_util::declare_parameter_if_not_declared(
        node, local_name + "debug_optimizer", rclcpp::ParameterValue(false));
    node->get_parameter(local_name + "debug_optimizer", debug);

    nav2_util::declare_parameter_if_not_declared(
        node, local_name + "max_time_allowed", rclcpp::ParameterValue(2.0));
    node->get_parameter(local_name + "max_time_allowed", max_time);
  }

  const std::map<std::string, ceres::LinearSolverType> solver_types = {
      {"DENSE_QR", ceres::DENSE_QR},
      {"SPARSE_NORMAL_CHOLESKY", ceres::SPARSE_NORMAL_CHOLESKY}};

  bool debug;
  std::string linear_solver_type;
  int max_iterations; // Ceres default: 50

  double max_time;

  double param_tol;    // Ceres default: 1e-8
  double fn_tol;       // Ceres default: 1e-6
  double gradient_tol; // Ceres default: 1e-10
};

/**
 * @brief
 *
 */
class Optimizer {
public:
  // x, y
  struct position {
    double params[2];
  };

  // x, y, lv, av
  struct posandvel {
    double params[4];
  };

  // lv, av
  struct vel {
    double params[2];
  };

  // t, yaw
  struct heading {
    double params[2];
  };

  /**
   * @brief Construct a new Optimizer object
   *
   */
  Optimizer() {}

  /**
   * @brief Destroy the Optimizer object
   *
   */
  ~Optimizer() {}

  /**
   * @brief Initialization of the smoother
   * @param params OptimizerParam struct
   */
  void initialize(const OptimizerParams params) {
    debug_ = params.debug;

    options_.linear_solver_type =
        params.solver_types.at(params.linear_solver_type);

    options_.max_num_iterations = params.max_iterations;

    options_.function_tolerance = params.fn_tol;
    options_.gradient_tolerance = params.gradient_tol;
    options_.parameter_tolerance = params.param_tol;

    if (debug_) {
      options_.minimizer_progress_to_stdout = true;
      options_.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
    } else {
      options_.logging_type = ceres::SILENT;
    }
    options_.max_solver_time_in_seconds = params.max_time;
  }

  /**
   * @brief
   *
   * @param path
   * @param cmds
   * @param people
   * @param speed
   * @param time_step
   * @return true
   * @return false
   */
  bool optimize(nav_msgs::msg::Path &path,
                std::vector<std::vector<AgentStatus>> &people_proj,
                const nav2_costmap_2d::Costmap2D *costmap,
                const obstacle_distance_msgs::msg::ObstacleDistance &obstacles,
                const std::vector<geometry_msgs::msg::TwistStamped> &cmds,
                const people_msgs::msg::People &people,
                const geometry_msgs::msg::Twist &speed, const float time_step) {

    // Path has always at least 2 points
    if (path.poses.size() < 2) {
      // throw std::runtime_error("Optimizer: Path must have at least 2
      // points");
      return false;
    }

    frame_ = path.header.frame_id;
    distance_w_ = 0.3; // 0.2;
    // 1.0-low effect
    // 0.01-no effect;
    // 10.0-too much;
    // 5.0-not good effect;
    socialwork_w_ = 3.0;
    obstacle_w_ = 0.1; // 3.0;
    curvature_w_ = 100.0;
    curvature_angle_min_ = M_PI / 15.0;

    // Create costmap grid
    costmap_grid_ = std::make_shared<ceres::Grid2D<u_char>>(
        costmap->getCharMap(), 0, costmap->getSizeInCellsY(), 0,
        costmap->getSizeInCellsX());
    auto costmap_interpolator =
        std::make_shared<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>(
            *costmap_grid_);

    // Grid2D(const T* data,
    //      const int row_begin,
    //      const int row_end,
    //      const int col_begin,
    //      const int col_end)
    // obsdist_grid_ = std::make_shared<ceres::Grid2D<float>>(
    //     &obstacles.distances, 0, obstacles.info.height, 0,
    //     obstacles.info.width);
    // auto obsdist_interpolator =
    //     std::make_shared<ceres::BiCubicInterpolator<ceres::Grid2D<float>>>(
    //         *obsdist_grid_);

    std::vector<AgentStatus> init_people = people_to_status(people);

    // Transform the initial solution path to a format suitable for the
    // optimizer
    std::vector<AgentStatus> initial_status =
        format_to_optimize(path, cmds, speed, time_step);

    // compute the future people position according to the SFM for each point of
    // the robot trajectory
    // WATCH OUT! people, robot and obstacles must be in the same coordinate
    // frame!!!
    std::vector<std::vector<AgentStatus>> projected_people =
        project_people(init_people, initial_status, obstacles, time_step);
    people_proj = projected_people;

    std::vector<position> ini_positions;
    std::vector<heading> ini_headings;
    std::vector<vel> ini_velocities;
    // std::vector<posandvel> ini_posandvels;
    for (auto a : initial_status) {
      position p;
      p.params[0] = a[0]; // x
      p.params[1] = a[1]; // y
      vel v;
      v.params[0] = a[4]; // lv
      v.params[1] = a[5]; // av
      heading h;
      h.params[0] = a[3]; // t
      h.params[1] = a[2]; // yaw
      ini_positions.push_back(p);
      ini_velocities.push_back(v);
      ini_headings.push_back(h);
    }
    // printf("Path length: %i\n", (int)initial_status.size());
    std::vector<AgentStatus> optim_status = initial_status;
    std::vector<position> optim_positions = ini_positions;

    // goal
    // position g;
    // g.params[0] = initial_status.back()[0];
    // g.params[1] = initial_status.back()[1];

    // build the problem
    ceres::Problem problem;
    // ceres::LossFunction *loss_function = NULL;

    // para cada punto del path inicial, añadir una costFunction (uno de los
    // parámetros es ese punto) Después en el addResidualBlock(), añadir la cost
    // function así como el punto actual, su anterior y su posterior.

    // Set up a cost function per point into the path
    for (unsigned int i = 0; i < optim_status.size(); i++) {

      // --------------------------------------------
      //   Obstacle cost
      // --------------------------------------------
      // ObstacleCostFunction *obs_cost_function =
      //     new ObstacleCostFunction(obstacle_w_, costmap,
      //     costmap_interpolator);
      ceres::CostFunction *obs_cost_function =
          new AutoDiffCostFunction<ObstacleCostFunction, 1, 2>(
              new ObstacleCostFunction(obstacle_w_, costmap,
                                       costmap_interpolator));
      problem.AddResidualBlock(obs_cost_function, NULL,
                               optim_positions[i].params);
      // problem.AddResidualBlock(cost_function->AutoDiff(), loss_function,
      //                         optim_status[i].data());

      // --------------------------------------------
      //   Social work
      // --------------------------------------------
      ceres::CostFunction *social_work_function =
          new AutoDiffCostFunction<SocialWorkFunction, 1, 2, 2, 2>(
              new SocialWorkFunction(socialwork_w_, projected_people[i]));
      problem.AddResidualBlock(
          social_work_function, NULL, optim_positions[i].params,
          ini_headings[i].params, ini_velocities[i].params);

      // --------------------------------------------
      //   Distance cost
      // --------------------------------------------
      Eigen::Matrix<double, 2, 1> point(ini_positions[i].params[0],
                                        ini_positions[i].params[1]);
      ceres::CostFunction *distance_cost_function =
          new AutoDiffCostFunction<DistanceCostFunction, 1, 2>(
              new DistanceCostFunction(distance_w_, point));
      problem.AddResidualBlock(distance_cost_function, NULL,
                               optim_positions[i].params);

      // --------------------------------------------
      //   curvature cost
      // --------------------------------------------
      if (i < optim_status.size() - 2) {
        CostFunction *curvature_cost_function =
            new AutoDiffCostFunction<CurvatureCostFunction, 1, 2, 2, 2>(
                new CurvatureCostFunction(curvature_w_, curvature_angle_min_));
        problem.AddResidualBlock(
            curvature_cost_function, NULL, optim_positions[i].params,
            optim_positions[i + 1].params, optim_positions[i + 2].params);
      }
    }

    // first and last points are constant
    // problem.SetParameterBlockConstant(optim_positions.front().data());
    problem.SetParameterBlockConstant(optim_positions.front().params);
    problem.SetParameterBlockConstant(optim_positions.back().params);

    // solve the problem
    ceres::Solver::Summary summary;
    // printf("Before calling Solve!!!!");
    ceres::Solve(options_, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    if (!summary.IsSolutionUsable()) {
      // printf("Optimization failed!!!\n");
      return false;
    }

    // Get the solution from optim_status
    path.poses.clear();
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    // for (auto point : optim_status) {
    //   pose.pose.position.x = point(0, 0);
    //   pose.pose.position.y = point(1, 0);
    //   tf2::Quaternion myQuaternion;
    //   myQuaternion.setRPY(0, 0, point(2, 0));
    //   pose.pose.orientation = tf2::toMsg(myQuaternion);
    //   path.poses.push_back(pose);
    // }
    // Get the solution from optim_positions
    for (auto point : optim_positions) {
      pose.pose.position.x = point.params[0];
      pose.pose.position.y = point.params[1];
      // tf2::Quaternion myQuaternion;
      // myQuaternion.setRPY(0, 0, point(2, 0));
      // pose.pose.orientation = tf2::toMsg(myQuaternion);
      path.poses.push_back(pose);
    }
    // printf("Optimized Path length: %i\n", (int)path.poses.size());
    return true;

    // std::vector<Eigen::Vector3d> path_optim;
    //  std::vector<bool> optimized;
    // if (buildProblem(path, costmap, params, problem, path_optim, optimized))
    // {
    // if (buildProblem(path, problem, path_optim, optimized)) {
    //   // solve the problem
    //   ceres::Solver::Summary summary;
    //   ceres::Solve(options_, &problem, &summary);
    //   if (debug_) {
    //     RCLCPP_INFO(rclcpp::get_logger("smoother_server"), "%s",
    //                 summary.FullReport().c_str());
    //   }
    //   if (!summary.IsSolutionUsable() ||
    //       summary.initial_cost - summary.final_cost < 0.0) {
    //     return false;
    //   }
    // } else {
    //   RCLCPP_INFO(rclcpp::get_logger("smoother_server"),
    //               "Path too short to optimize");
    // }

    // upsampleAndPopulate(path_optim, optimized, start_dir, end_dir, params,
    //                    path);
  }

private:
  std::vector<AgentStatus>
  people_to_status(const people_msgs::msg::People &people) {
    std::vector<AgentStatus> people_status;
    // the agents status contain 5 values:
    // x, y, yaw, timestamp, lv, av
    for (auto p : people.people) {
      double yaw = atan2(p.velocity.y, p.velocity.x);
      double lv =
          sqrt(p.velocity.x * p.velocity.x + p.velocity.y * p.velocity.y);
      AgentStatus st;
      st << (double)p.position.x, (double)p.position.y, yaw, 0.0, lv,
          (double)p.velocity.z;
      people_status.push_back(st);
    }
    // we add agents if needed
    while ((int)people_status.size() < 3) {
      AgentStatus st;
      // we fill with invalid agent: time=-1
      st << 0.0, 0.0, 0.0, -1.0, 0.0, 0.0;
      people_status.push_back(st);
    }
    // we remove agents if needed
    while ((int)people_status.size() > 3) {
      people_status.pop_back();
    }

    return people_status;
  }

  std::vector<AgentStatus>
  format_to_optimize(nav_msgs::msg::Path &path,
                     const std::vector<geometry_msgs::msg::TwistStamped> &cmds,
                     const geometry_msgs::msg::Twist &speed,
                     const float timestep) {

    // we check the timestep and the path size in order to cut the path
    // to a maximum duration.
    float maxtime = 5.0;
    // t = size * timestep
    int maxsize = (int)round(maxtime / timestep);
    if ((int)path.poses.size() > maxsize) {
      std::vector<geometry_msgs::msg::PoseStamped> p(
          path.poses.begin(), (path.poses.begin() + (maxsize - 1)));
      path.poses = p;
    }

    // double t = 0.0;

    std::vector<AgentStatus> robot_status;
    for (unsigned int i = 0; i < path.poses.size(); i++) {

      // Robot
      // x, y, yaw, t, lv, av
      AgentStatus r;
      r(0, 0) = path.poses[i].pose.position.x;
      r(1, 0) = path.poses[i].pose.position.y;
      r(2, 0) = tf2::getYaw(path.poses[i].pose.orientation);
      r(3, 0) = i * timestep;

      // t += timestep;

      if (i == 0) {
        // Robot vel
        r(4, 0) = speed.linear.x;
        r(5, 0) = speed.angular.z;
      } else {
        // Robot vel
        r(4, 0) = cmds[i - 1].twist.linear.x;
        r(5, 0) = cmds[i - 1].twist.angular.z;
      }
      robot_status.push_back(r);
    }
    return robot_status;
  }

  // we project the people state for each time step of the robot path
  std::vector<std::vector<AgentStatus>>
  project_people(const std::vector<AgentStatus> &init_people,
                 const std::vector<AgentStatus> &robot_path,
                 const obstacle_distance_msgs::msg::ObstacleDistance &od,
                 const float &timestep) {

    double naive_goal_time = 3.0; // secs
    // double people_desired_vel = 1.0;
    std::vector<std::vector<AgentStatus>> people_traj;
    people_traj.push_back(init_people);

    // I NEED TO ADD THE CLOSER OBSTACLE POSITION TO EACH AGENT
    // FOR EACH STEP. THAT OBSTACLE POSITION MUST BE IN THE
    // SAME COORDINATE FRAME THAT THE AGENT POSITION.

    std::vector<sfm_controller::Agent> agents;

    // transform people to sfm agents
    for (unsigned int i = 0; i < init_people.size(); i++) {

      // if person not valid, skip it
      if (init_people[i][3] == -1)
        continue;

      sfm_controller::Agent a;
      a.id = i + 1;
      a.position << init_people[i][0], init_people[i][1];
      a.yaw = init_people[i][2];
      a.linearVelocity = init_people[i][4];
      a.angularVelocity = init_people[i][5];
      // vx = linearVelocity * cos(yaw), vy = linearVelocity * sin(yaw)
      a.velocity << a.linearVelocity * cos(a.yaw),
          a.linearVelocity * sin(a.yaw);
      a.desiredVelocity = a.linearVelocity; // people_desired_vel; // could be
                                            // computed somehow???
      a.radius = 0.35;
      // compute goal with the Constant Velocity Model
      sfm_controller::Goal g;
      g.radius = 0.25;
      Eigen::Vector2d gpos = a.position + naive_goal_time * a.velocity;
      g.center = gpos;
      a.goals.push_back(g);
      // Fill the obstacles
      // std::vector<utils::Vector2d> obstacles1;
      // std::vector<utils::Vector2d> obstacles2;
      a.obstacles2.push_back(computeObstacle(a.position, od));
      agents.push_back(a);
    }

    // compute for each robot state of the path
    for (unsigned int i = 0; i < robot_path.size() - 1; i++) {

      // robot as sfm agent
      sfm_controller::Agent sfmrobot;
      sfmrobot.desiredVelocity = 0.5;
      sfmrobot.radius = 0.4;
      sfmrobot.id = 0;
      sfmrobot.position << robot_path[i][0], robot_path[i][1];
      sfmrobot.yaw = robot_path[i][2];
      sfmrobot.linearVelocity = robot_path[i][4];
      sfmrobot.angularVelocity = robot_path[i][5];
      // vx = linearVelocity * cos(yaw), vy = linearVelocity * sin(yaw)
      sfmrobot.velocity << sfmrobot.linearVelocity * cos(sfmrobot.yaw),
          sfmrobot.linearVelocity * sin(sfmrobot.yaw);
      sfm_controller::Goal g;
      g.radius = 0.25;
      Eigen::Vector2d gpos(robot_path.back()[0], robot_path.back()[1]);
      g.center = gpos;
      sfmrobot.goals.push_back(g);

      // add the robot to the agents
      agents.push_back(sfmrobot);

      // Compute Social Forces
      sfm_controller::SFM.computeForces(agents);
      // Project the people movement according to the SFM
      sfm_controller::SFM.updatePosition(agents, timestep);

      // remove the robot (last agent)
      agents.pop_back();

      // update agents obstacles
      for (unsigned int j = 0; j < agents.size(); j++) {
        agents[j].obstacles2.clear();
        agents[j].obstacles2.push_back(computeObstacle(agents[j].position, od));
      }

      // Take the people agents
      std::vector<AgentStatus> humans;
      for (auto p : agents) {
        AgentStatus as;
        as(0, 0) = p.position[0];
        as(1, 0) = p.position[1];
        as(2, 0) = p.yaw;
        as(3, 0) = (i + 1) * timestep;
        as(4, 0) = p.linearVelocity;
        as(5, 0) = p.angularVelocity;
        humans.push_back(as);
      }
      // fill with empty agents if needed
      while (humans.size() < init_people.size()) {
        AgentStatus ag;
        ag.setZero();
        ag(3, 0) = -1.0;
        humans.push_back(ag);
      }
      people_traj.push_back(humans);
    }
    return people_traj;
  }

  Eigen::Vector2d
  computeObstacle(const Eigen::Vector2d &apos,
                  const obstacle_distance_msgs::msg::ObstacleDistance &od) {

    // if the distancegrid frame is different from the path frame
    // we must transform the agent position to the general frame
    // if (ob.header.frame_id != frame_) {
    // TODO: think about how we could do the transformation here
    // }

    // map point (person) to cell in the distance grid
    unsigned int xcell = (unsigned int)floor(
        (apos[0] - od.info.origin.position.x) / od.info.resolution);
    unsigned int ycell = (unsigned int)floor(
        (apos[1] - od.info.origin.position.y) / od.info.resolution);
    // cell to index of the array
    if (xcell >= (unsigned int)od.info.width) {
      xcell = (unsigned int)(od.info.width - 1);
    }
    if (ycell >= (unsigned int)od.info.height) {
      ycell = (unsigned int)(od.info.height - 1);
    }
    unsigned int index = xcell + ycell * od.info.width;

    float dist = od.distances[index]; // not used
    unsigned int ob_idx = od.indexes[index];

    // index to cell
    if (ob_idx >= (od.info.width * od.info.height)) {
      ob_idx = (od.info.width * od.info.height) - 1;
    }
    // const div_t result = div(ob_idx, (int)od.info.width);
    // xcol = result.rem;
    // yrow = result.quot;
    ycell = floor(ob_idx / od.info.width);
    xcell = ob_idx % od.info.width;

    // cell to world point (obstacle)
    float x = xcell * od.info.resolution + od.info.origin.position.x;
    float y = ycell * od.info.resolution + od.info.origin.position.y;
    Eigen::Vector2d obstacle(x, y);

    // vector between person and obstacle
    Eigen::Vector2d diff = apos - obstacle;

    std::cout << "a.x: " << apos[0] << " a.y: " << apos[1] << std::endl;
    std::cout << "obs dist: " << dist << " position x: " << x << " y: " << y
              << std::endl;
    return diff;
  }

  /**
   * @brief Build problem method
   * @param path Reference to path
   * @param costmap Pointer to costmap
   * @param params Smoother parameters
   * @param problem Output problem to solve
   * @param path_optim Output path on which the problem will be solved
   * @param optimized False for points skipped by downsampling
   * @return If there is a problem to solve
   */
  //   bool buildProblem(const std::vector<Eigen::Vector3d> &path,
  //                     // const nav2_costmap_2d::Costmap2D *costmap,
  //                     // const SmootherParams &params,
  //                     ceres::Problem &problem,
  //                     std::vector<Eigen::Vector3d> &path_optim,
  //                     std::vector<bool> &optimized) {
  //     // Create costmap grid
  //     // costmap_grid_ = std::make_shared<ceres::Grid2D<u_char>>(
  //     //     costmap->getCharMap(), 0, costmap->getSizeInCellsY(), 0,
  //     //     costmap->getSizeInCellsX());
  //     // auto costmap_interpolator =
  //     //
  //     std::make_shared<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>(
  //     //         *costmap_grid_);

  //     // Create residual blocks
  //     // const double cusp_half_length = params.cusp_zone_length / 2;
  //     ceres::LossFunction *loss_function = NULL;
  //     path_optim = path;
  //     optimized = std::vector<bool>(path.size());
  //     optimized[0] = true;
  //     // int prelast_i = -1;
  //     // int last_i = 0;
  //     // double last_direction = path_optim[0][2];
  //     // bool last_was_cusp = false;
  //     // bool last_is_reversing = false;
  //     // std::deque<std::pair<double, SmootherCostFunction *>>
  //     // potential_cusp_funcs; double last_segment_len = EPSILON; double
  //     // potential_cusp_funcs_len = 0; double len_since_cusp =
  //     // std::numeric_limits<double>::infinity();

  //     // for (size_t i = 1; i < path_optim.size(); i++) {
  //     //   auto &pt = path_optim[i];
  //     //   bool is_cusp = false;
  //     //   if (i != path_optim.size() - 1) {
  //     //     is_cusp = pt[2] * last_direction < 0;
  //     //     last_direction = pt[2];

  //     //     // skip to downsample if can be skipped (no forward/reverse
  //     direction
  //     //     // change)
  //     //     if (!is_cusp && i > (params.keep_start_orientation ? 1 : 0) &&
  //     //         i < path_optim.size() - (params.keep_goal_orientation ? 2
  //     : 1) &&
  //     //         static_cast<int>(i - last_i) <
  //     params.path_downsampling_factor) {
  //     //       continue;
  //     //     }
  //     //   }

  //     //   // keep distance inequalities between poses
  //     //   // (some might have been downsampled while others might not)
  //     //   double current_segment_len =
  //     //       (path_optim[i] - path_optim[last_i]).block<2, 1>(0,
  //     0).norm();

  //     //   // forget cost functions which don't have chance to be part of a
  //     cusp
  //     //   zone potential_cusp_funcs_len += current_segment_len; while
  //     //   (!potential_cusp_funcs.empty() &&
  //     //          potential_cusp_funcs_len > cusp_half_length) {
  //     //     potential_cusp_funcs_len -=
  //     potential_cusp_funcs.front().first;
  //     //     potential_cusp_funcs.pop_front();
  //     //   }

  //     //   // update cusp zone costmap weights
  //     //   if (is_cusp) {
  //     //     double len_to_cusp = current_segment_len;
  //     //     for (int i = potential_cusp_funcs.size() - 1; i >= 0; i--) {
  //     //       auto &f = potential_cusp_funcs[i];
  //     //       double new_weight =
  //     //           params.cusp_costmap_weight *
  //     //               (1.0 - len_to_cusp / cusp_half_length) +
  //     //           params.costmap_weight * len_to_cusp / cusp_half_length;
  //     //       if (std::abs(new_weight - params.cusp_costmap_weight) <
  //     //           std::abs(f.second->getCostmapWeight() -
  //     //                    params.cusp_costmap_weight)) {
  //     //         f.second->setCostmapWeight(new_weight);
  //     //       }
  //     //       len_to_cusp += f.first;
  //     //     }
  //     //     potential_cusp_funcs_len = 0;
  //     //     potential_cusp_funcs.clear();
  //     //     len_since_cusp = 0;
  //     //   }

  //     //   // add cost function
  //     //   optimized[i] = true;
  //     //   if (prelast_i != -1) {
  //     //     double costmap_weight = params.costmap_weight;
  //     //     if (len_since_cusp <= cusp_half_length) {
  //     //       costmap_weight =
  //     //           params.cusp_costmap_weight *
  //     //               (1.0 - len_since_cusp / cusp_half_length) +
  //     //           params.costmap_weight * len_since_cusp /
  //     cusp_half_length;
  //     //     }
  //     //     SmootherCostFunction *cost_function = new
  //     SmootherCostFunction(
  //     //         path[last_i].template block<2, 1>(0, 0),
  //     //         (last_was_cusp ? -1 : 1) * last_segment_len /
  //     //         current_segment_len, last_is_reversing, costmap,
  //     //         costmap_interpolator, params, costmap_weight);
  //     //     problem.AddResidualBlock(cost_function->AutoDiff(),
  //     loss_function,
  //     //                              path_optim[last_i].data(), pt.data(),
  //     //                              path_optim[prelast_i].data());

  //     //     potential_cusp_funcs.emplace_back(current_segment_len,
  //     //     cost_function);
  //     //   }

  //     //   // shift current to last and last to pre-last
  //     //   last_was_cusp = is_cusp;
  //     //   last_is_reversing = last_direction < 0;
  //     //   prelast_i = last_i;
  //     //   last_i = i;
  //     //   len_since_cusp += current_segment_len;
  //     //   last_segment_len = std::max(EPSILON, current_segment_len);
  //     // }

  //     // int posesToOptimize =
  //     //     problem.NumParameterBlocks() - 2; // minus start and goal
  //     // if (params.keep_goal_orientation) {
  //     //   posesToOptimize -= 1; // minus goal orientation holder
  //     // }
  //     // if (params.keep_start_orientation) {
  //     //   posesToOptimize -= 1; // minus start orientation holder
  //     // }
  //     // if (posesToOptimize <= 0) {
  //     //   return false; // nothing to optimize
  //     // }
  //     // // first two and last two points are constant (to keep start and
  //     end
  //     // // direction)
  //     // problem.SetParameterBlockConstant(path_optim.front().data());
  //     // if (params.keep_start_orientation) {
  //     //   problem.SetParameterBlockConstant(path_optim[1].data());
  //     // }
  //     // if (params.keep_goal_orientation) {
  //     //   problem.SetParameterBlockConstant(
  //     //       path_optim[path_optim.size() - 2].data());
  //     // }
  //     // problem.SetParameterBlockConstant(path_optim.back().data());
  //     return true;
  //   }

  // bool buildProblem(const std::vector<Eigen::Vector3d> &path,

  bool debug_;
  double distance_w_;
  double socialwork_w_;
  double obstacle_w_;
  double curvature_w_;
  double curvature_angle_min_;
  ceres::Solver::Options options_;
  std::shared_ptr<ceres::Grid2D<u_char>> costmap_grid_;
  std::shared_ptr<ceres::Grid2D<float>> obsdist_grid_;
  std::string frame_;
};
} // namespace nav2_social_mpc_controller

#endif
