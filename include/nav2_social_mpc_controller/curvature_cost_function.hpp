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

#ifndef NAV2_SOCIAL_MPC_CONTROLLER__CURVATURE_COST_FUNCTION_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__CURVATURE_COST_FUNCTION_HPP_

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "glog/logging.h"

// the agents status contain 6 values:
// x, y, yaw, timestamp, lv, av
// typedef Eigen::Matrix<double, 6, 1> AgentStatus;

namespace nav2_social_mpc_controller {

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class CurvatureCostFunction {

public:
  CurvatureCostFunction(double weight, double max_angle)
      : weight_(weight), max_angle_(max_angle) {}

  ceres::CostFunction *AutoDiff() {
    // the first number is the number of cost functions.
    // the following number are the length of the parameters passed in the
    // addResidualBlock function.
    return new ceres::AutoDiffCostFunction<CurvatureCostFunction, 1, 2, 2, 2>(
        this);
  }

  template <typename T>
  bool operator()(const T *const state1, const T *const state2,
                  const T *const state3, T *residual) const {

    T vector1[2] = {state2[0] - state1[0], state2[1] - state1[1]};
    T vector2[2] = {state2[0] - state3[0], state2[1] - state3[1]};

    T dot_product = (vector2[0] * vector1[0]) + (vector2[1] * vector1[1]);
    T norm_vector1 =
        sqrt((vector1[0] * vector1[0]) + (vector1[1] * vector1[1]));
    T norm_vector2 =
        sqrt((vector2[0] * vector2[0]) + (vector2[1] * vector2[1]));

    T angle = acos(dot_product / (norm_vector1 * norm_vector2));

    T bound1 = T(M_PI) - T(max_angle_);
    T bound2 = T(M_PI) + T(max_angle_);
    T bound = (bound1 + bound2) / (T)2.0;

    if ((angle < bound1) || (angle > bound2))
      residual[0] = (T)weight_ * exp(sqrt((angle - bound) * (angle - bound)));
    else
      residual[0] = T(0.0);

    return true;
  }

  double weight_;
  double max_angle_;
};

} // namespace nav2_social_mpc_controller

#endif