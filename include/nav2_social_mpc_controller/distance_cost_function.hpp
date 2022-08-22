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

#ifndef NAV2_SOCIAL_MPC_CONTROLLER__DISTANCE_COST_FUNCTION_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__DISTANCE_COST_FUNCTION_HPP_

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "glog/logging.h"

// the agents status contain 6 values:
// x, y, yaw, timestamp, lv, av
typedef Eigen::Matrix<double, 6, 1> AgentStatus;

namespace nav2_social_mpc_controller {

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class DistanceCostFunction {

public:
  DistanceCostFunction(double weight, const Eigen::Matrix<double, 2, 1> point)
      : weight_(weight), point_(point) {}

  ceres::CostFunction *AutoDiff() {
    // the first number is the number of cost functions.
    // the following number are the length of the parameters passed in the
    // addResidualBlock function.
    return new ceres::AutoDiffCostFunction<DistanceCostFunction, 1, 2>(this);
  }

  template <typename T>
  bool operator()(const T *const state, T *residual) const {

    Eigen::Matrix<T, 2, 1> p(state[0], state[1]);
    Eigen::Matrix<T, 2, 1> p_ori((T)point_[0], (T)point_[1]);
    residual[0] =
        (T)weight_ * (p - p_ori).squaredNorm(); // objective function value

    return true;
  }

  double weight_;
  const Eigen::Matrix<double, 2, 1> point_;
  // const double point_[2];
};

} // namespace nav2_social_mpc_controller

#endif