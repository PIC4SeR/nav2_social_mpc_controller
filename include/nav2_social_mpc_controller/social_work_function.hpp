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

#ifndef NAV2_SOCIAL_MPC_CONTROLLER__SOCIAL_WORK_FUNCTION_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__SOCIAL_WORK_FUNCTION_HPP_

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

class SocialWorkFunction {

public:
  SocialWorkFunction(double weight, const std::vector<AgentStatus> &agents_init)
      : weight_(weight) {

    int i = 0;
    for (auto a : agents_init) {
      original_agents_.col(i) << a[0], a[1], a[2], a[3], a[4], a[5];
      i++;
    }
    // std::cout << "Constructor social cost function. x:" <<
    // original_status_[0]
    //           << " y:" << original_status_[1] << " t:" << original_status_[3]
    //           << std::endl;
    // std::cout << "Agents init:" << std::endl;
    // // x, y, yaw, timestamp, lv, av
    // std::cout << original_agents_ << std::endl;

    sfm_lambda_ = 2.0;
    sfm_gamma_ = 0.35;
    sfm_nPrime_ = 3.0;
    sfm_n_ = 2.0;
    sfm_relaxationTime_ = 0.5;
    sfm_forceFactorSocial_ = 2.1;
  }

  ceres::CostFunction *AutoDiff() {
    // the first number is the number of cost functions.
    // the following number are the length of the parameters passed in the
    // addResidualBlock function.
    return new ceres::AutoDiffCostFunction<SocialWorkFunction, 1, 6>(this);
  }

  template <typename T>
  bool operator()(const T *const state, T *residual) const {

    // Compute robot social work
    Eigen::Matrix<T, 6, 3> agents = original_agents_.template cast<T>();
    Eigen::Matrix<T, 6, 1> robot;
    // robot.col(0) << state;
    robot(0, 0) = state[0];
    robot(1, 0) = state[1];
    robot(2, 0) = state[2];
    robot(3, 0) = state[3];
    robot(4, 0) = state[4];
    robot(5, 0) = state[5];
    Eigen::Matrix<T, 2, 1> robot_sf = computeSocialForce(robot, agents);
    T wr = (T)robot_sf.squaredNorm();

    // compute agents' social work provoked by the robot
    T wp = (T)0.0;
    Eigen::Matrix<T, 6, 3> robot_agent;
    robot_agent.col(0) << robot;
    // we invalidate the other two agent
    // by setting t to -1
    robot_agent.col(1) << (T)0.0, (T)0.0, (T)0.0, (T)-1.0, (T)0.0, (T)0.0;
    robot_agent.col(2) << (T)0.0, (T)0.0, (T)0.0, (T)-1.0, (T)0.0, (T)0.0;
    for (unsigned int i = 0; i < original_agents_.cols(); i++) {
      Eigen::Matrix<T, 6, 1> ag;
      ag.col(0) << original_agents_.col(i).template cast<T>();
      Eigen::Matrix<T, 2, 1> agent_sf = computeSocialForce(ag, robot_agent);
      wp += (T)agent_sf.squaredNorm();
    }
    // sum the social works and multiply by the weight
    residual[0] = (T)weight_ * (wr + wp);
    return true;
  }

  template <typename T>
  inline Eigen::Matrix<T, 2, 1> computeSocialForce(
      const Eigen::Matrix<T, 6, 1> &me,
      const Eigen::Matrix<T, 6, 3> &agents) const { // ceres::DYNAMIC

    // AgentStatus: x, y, yaw, t, lv, av
    Eigen::Matrix<T, 2, 1> meSocialforce((T)0.0, (T)0.0);
    Eigen::Matrix<T, 2, 1> mePos(me[0], me[1]); // x,y, position vector
    // vx = linearVelocity * cos(yaw), vy = linearVelocity * sin(yaw)
    Eigen::Matrix<T, 2, 1> meVel(me[4] * (T)cos(me[2]), me[4] * (T)sin(me[2]));

    for (unsigned int i = 0; i < agents.cols(); i++) {

      // if time t is -1, agent is not valid
      if (agents(3, i) == (T)-1.0)
        continue;

      // utils::Vector2d diff = agents[i].position - me.position;
      Eigen::Matrix<T, 2, 1> aPos(agents(0, i), agents(1, i));
      Eigen::Matrix<T, 2, 1> diff = mePos - aPos;
      // utils::Vector2d diffDirection = diff.normalized();
      Eigen::Matrix<T, 2, 1> diffDirection = diff.normalized();
      // utils::Vector2d velDiff = me.velocity - agents[i].velocity;
      Eigen::Matrix<T, 2, 1> aVel(agents(4, i) * cos(agents(2, i)),
                                  agents(4, i) * sin(agents(2, i)));
      Eigen::Matrix<T, 2, 1> velDiff = meVel - aVel;
      // utils::Vector2d interactionVector =
      //    me.params.lambda * velDiff + diffDirection;
      Eigen::Matrix<T, 2, 1> interactionVector =
          (T)sfm_lambda_ * velDiff + diffDirection;
      // double interactionLength = interactionVector.norm();
      T interactionLength = interactionVector.norm();
      // utils::Vector2d interactionDirection =
      //    interactionVector / interactionLength;
      Eigen::Matrix<T, 2, 1> interactionDirection =
          interactionVector / interactionLength;
      // utils::Angle theta = interactionDirection.angleTo(diffDirection);
      T angle1 = atan2(diffDirection[1], diffDirection[0]);
      while (angle1 <= -(T)M_PI)
        angle1 += (T)2 * (T)M_PI;
      while (angle1 > (T)M_PI)
        angle1 -= (T)2 * (T)M_PI;
      T angle2 = atan2(interactionDirection[1], interactionDirection[0]);
      while (angle2 <= -(T)M_PI)
        angle2 += (T)2 * (T)M_PI;
      while (angle2 > (T)M_PI)
        angle2 -= (T)2 * (T)M_PI;
      T theta = angle1 - angle2;
      while (theta <= -(T)M_PI)
        theta += (T)2 * (T)M_PI;
      while (theta > (T)M_PI)
        theta -= (T)2 * (T)M_PI;

      // double B = me.params.gamma * interactionLength;
      T B = (T)sfm_gamma_ * interactionLength;
      // double thetaRad = theta.toRadian();
      // double forceVelocityAmount =
      //    -std::exp(-diff.norm() / B - PW(me.params.nPrime * B * thetaRad));
      T forceVelocityAmount = -(T)ceres::exp(-(T)diff.norm() / B -
                                             ((T)sfm_nPrime_ * B * theta) *
                                                 ((T)sfm_nPrime_ * B * theta));
      // double forceAngleAmount =
      //     -theta.sign() *
      //     std::exp(-diff.norm() / B - PW(me.params.n * B * thetaRad));
      T sign = (T)0;
      if (theta > (T)0)
        sign = (T)1;
      if (theta < (T)0)
        sign = (T)-1;
      T forceAngleAmount =
          -sign * ceres::exp(-(T)diff.norm() / B -
                             ((T)sfm_n_ * B * theta) * ((T)sfm_n_ * B * theta));
      // utils::Vector2d forceVelocity =
      //     forceVelocityAmount * interactionDirection;
      Eigen::Matrix<T, 2, 1> forceVelocity =
          forceVelocityAmount * interactionDirection;
      // utils::Vector2d forceAngle =
      //    forceAngleAmount * interactionDirection.leftNormalVector();
      Eigen::Matrix<T, 2, 1> leftNormalVector(-interactionDirection[1],
                                              interactionDirection[0]);
      Eigen::Matrix<T, 2, 1> forceAngle = forceAngleAmount * leftNormalVector;
      // me.forces.socialForce +=
      //    me.params.forceFactorSocial * (forceVelocity + forceAngle);
      meSocialforce += (T)sfm_forceFactorSocial_ * (forceVelocity + forceAngle);
    }
    return meSocialforce;
  }

  double weight_;
  Eigen::Matrix<double, 6, 3> original_agents_; // ceres::DYNAMIC
  // const float timestep_;
  double sfm_lambda_;
  double sfm_gamma_;
  double sfm_nPrime_;
  double sfm_n_;
  double sfm_relaxationTime_;
  double sfm_forceFactorSocial_;
};

} // namespace nav2_social_mpc_controller

#endif