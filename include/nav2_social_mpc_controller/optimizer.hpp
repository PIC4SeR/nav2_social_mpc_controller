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
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"

namespace nav2_social_mpc_controller {

struct OptimizerParams {
  OptimizerParams()
      : debug(false), max_iterations(50), param_tol(1e-8), fn_tol(1e-6),
        gradient_tol(1e-10) {}

  /**
   * @brief Get params from ROS parameter
   * @param node_ Ptr to node
   * @param name Name of plugin
   */
  void get(rclcpp_lifecycle::LifecycleNode *node, const std::string &name) {
    std::string local_name = name + std::string(".optimizer.");

    // Optimizer params
    nav2_util::declare_parameter_if_not_declared(
        node, local_name + "linear_solver_type",
        rclcpp::ParameterValue("SPARSE_NORMAL_CHOLESKY"));
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
        node, local_name + "max_time_allowed", rclcpp::ParameterValue(0.5));
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
   * @brief optimizer method
   * @param path Reference to path
   * @param start_dir Orientation of the first pose
   * @param end_dir Orientation of the last pose
   * @param costmap Pointer to minimal costmap
   * @param params parameters weights
   * @return If smoothing was successful
   */
  bool optimize(std::vector<Eigen::Vector3d> &path) {
    // const nav2_costmap_2d::Costmap2D *costmap,
    // const SmootherParams &params) {
    // Path has always at least 2 points
    if (path.size() < 2) {
      throw std::runtime_error("Optimizer: Path must have at least 2 points");
    }

    // options_.max_solver_time_in_seconds = max_time; // params.max_time;

    ceres::Problem problem;
    std::vector<Eigen::Vector3d> path_optim;
    std::vector<bool> optimized;
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

    return true;
  }

private:
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
  //     // std::make_shared<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>(
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
  //     //         i < path_optim.size() - (params.keep_goal_orientation ? 2 :
  //     1) &&
  //     //         static_cast<int>(i - last_i) <
  //     params.path_downsampling_factor) {
  //     //       continue;
  //     //     }
  //     //   }

  //     //   // keep distance inequalities between poses
  //     //   // (some might have been downsampled while others might not)
  //     //   double current_segment_len =
  //     //       (path_optim[i] - path_optim[last_i]).block<2, 1>(0, 0).norm();

  //     //   // forget cost functions which don't have chance to be part of a
  //     cusp
  //     //   zone potential_cusp_funcs_len += current_segment_len; while
  //     //   (!potential_cusp_funcs.empty() &&
  //     //          potential_cusp_funcs_len > cusp_half_length) {
  //     //     potential_cusp_funcs_len -= potential_cusp_funcs.front().first;
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
  //     //           params.costmap_weight * len_since_cusp / cusp_half_length;
  //     //     }
  //     //     SmootherCostFunction *cost_function = new SmootherCostFunction(
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
  //     // // first two and last two points are constant (to keep start and end
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

  bool debug_;
  ceres::Solver::Options options_;
  std::shared_ptr<ceres::Grid2D<u_char>> costmap_grid_;
};
} // namespace nav2_social_mpc_controller

#endif
