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

#ifndef NAV2_SOCIAL_MPC_CONTROLLER__OBSTACLE_COST_FUNCTION_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__OBSTACLE_COST_FUNCTION_HPP_

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"
#include "glog/logging.h"

#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_social_mpc_controller {

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class ObstacleCostFunction {

public:
  ObstacleCostFunction(
      double weight, const nav2_costmap_2d::Costmap2D *costmap,
      const std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>
          &costmap_interpolator)
      : weight_(weight),
        costmap_origin_(costmap->getOriginX(), costmap->getOriginY()),
        costmap_resolution_(costmap->getResolution()),
        costmap_interpolator_(costmap_interpolator) {}

  ceres::CostFunction *AutoDiff() {
    // the first number is the number of cost functions.
    // the following number are the length of the parameters passed in the
    // addResidualBlock function.
    return new ceres::AutoDiffCostFunction<ObstacleCostFunction, 1, 2>(this);
  }

  template <typename T>
  bool operator()(const T *const state, T *residual) const {

    // residual[0] = wf_ * (state1[1] - it_);
    Eigen::Matrix<T, 2, 1> p(state[0], state[1]);
    Eigen::Matrix<T, 2, 1> interp_pos =
        (p - costmap_origin_.template cast<T>()) / (T)costmap_resolution_;
    T value;
    costmap_interpolator_->Evaluate(interp_pos[1] - (T)0.5,
                                    interp_pos[0] - (T)0.5, &value);

    // objective function value
    // Should be = or += ??
    residual[0] = (T)weight_ * value * value;

    return true;
  }

  double weight_;
  Eigen::Vector2d costmap_origin_;
  double costmap_resolution_;
  std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>
      costmap_interpolator_;
};

} // namespace nav2_social_mpc_controller

#endif