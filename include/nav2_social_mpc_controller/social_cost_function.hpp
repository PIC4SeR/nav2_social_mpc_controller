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

#ifndef NAV2_SOCIAL_MPC_CONTROLLER__COST_FUNCTION_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__COST_FUNCTION_HPP_

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

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "people_msgs/msg/people.hpp"

// Social Force Model
//#include <lightsfm/sfm.hpp>

// the agents status contain 5 values:
// x, y, yaw, timestamp, lv, av
typedef Eigen::Matrix<double, 6, 1> AgentStatus;

namespace nav2_social_mpc_controller {

using std::placeholders::_1;

/**
 * @struct nav2_social_mpc_controller::SocialCostFunction
 * @brief Cost function for human-aware navigation
 */
class SocialCostFunction {
public:
  /**
   * @brief
   *
   * cost = (wv*vel_diff) + (wd*d) + (wa*angle_diff) + (wc*costmapcost) +
   * (ws*social_work)
   *
   * vel_diff = fabs(max_vel_x-vx_i)/max_vel_x (last point of the proyected
   * traj)
   * d = distance to local goal (last point of the proyected traj)
   * angle_diff = heading diff(norm) between local goal heading and point
   * headind (last point)
   * costmapcost = average of costmap costs in the traj
   * social_work = Wr + SUM(Wpi) (sum of the costs in the traj steps)
   * Wr: The social work of the robot
   * Wr = robot.forces.socialForce.norm() + robot.forces.obstacleForce.norm();
   * SUM(Wpi): the social work provoked by the robot in the other agents
   * std::vector<sfm::Agent> agent_robot;
   * agent_robot.push_back(agents[0]);
   * double wp = 0.0;
   * for (unsigned int i = 1; i < agents.size(); i++) {
   *   sfm::Agent agent = agents[i];
   *   sfm::SFM.computeForces(agent, agent_robot);
   *   wp += agent.forces.socialForce.norm();
   * }
   */

  // Should I use the costmap or a DistanceFunction???
  SocialCostFunction(
      const AgentStatus &robot_state, const nav2_costmap_2d::Costmap2D *costmap,
      const std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>
          &costmap_interpolator,
      const std::vector<AgentStatus> &agents_init, const float timestep)
      : original_status_(robot_state),
        costmap_origin_(costmap->getOriginX(), costmap->getOriginY()),
        costmap_resolution_(costmap->getResolution()),
        costmap_interpolator_(costmap_interpolator), timestep_(timestep) {

    // const int cols = agents_init.size();
    // Eigen::Matrix<double, 6, 3> m;
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

    distance_w_ = 1.0;
    socialwork_w_ = 1.0;
    obstacle_w_ = 3.0;
    sfm_lambda_ = 2.0;
    sfm_gamma_ = 0.35;
    sfm_nPrime_ = 3.0;
    sfm_n_ = 2.0;
    sfm_relaxationTime_ = 0.5;
    sfm_forceFactorSocial_ = 2.1;
  }

  // /**
  //  * @brief A constructor for
  //  * nav2_constrained_smoother::SmootherCostFunction
  //  * @param original_path Original position of the path node
  //  * @param next_to_last_length_ratio Ratio of next path segment compared to
  //  * previous. Negative if one of them represents reversing motion.
  //  * @param reversing Whether the path segment after this node represents
  //  * reversing motion.
  //  * @param costmap A costmap to get values for collision and obstacle
  //  * avoidance
  //  * @param params Optimization weights and parameters
  //  * @param costmap_weight Costmap cost weight. Can be params.costmap_weight
  //  * or params.cusp_costmap_weight
  //  */
  // SocialCostFunction(const Eigen::Vector2d &original_pos,
  //                    double next_to_last_length_ratio, bool reversing,
  //                    const nav2_costmap_2d::Costmap2D *costmap,
  //                    const std::shared_ptr<
  //                        ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>
  //                        &costmap_interpolator,
  //                    const SmootherParams &params, double costmap_weight)
  // : original_pos_(original_pos),
  //   next_to_last_length_ratio_(next_to_last_length_ratio),
  //   reversing_(reversing), params_(params), costmap_weight_(costmap_weight),
  //   costmap_origin_(costmap->getOriginX(), costmap->getOriginY()),
  //   costmap_resolution_(costmap->getResolution()),
  //   costmap_interpolator_(costmap_interpolator) {}

  ceres::CostFunction *AutoDiff() {
    // the first number is the number of cost functions.
    // the following number are the length of the parameters passed in the
    // addResidualBlock function.
    // return new ceres::AutoDiffCostFunction<SocialCostFunction, 2, 6, 6,
    // 6>(this)
    return new ceres::AutoDiffCostFunction<SocialCostFunction, 1, 6>(this);
  }

  /**
   * @brief Social cost function evaluation
   * @param pt x,y,yaw,t,lv,av of current robot status
   * @param pt_next values of next status
   * @param pt_prev values of previous status
   * @param pt_residual array of output residuals (distance, social_work)
   * @return if successful in computing values
   */
  // template <typename T>
  // bool operator()(const T *const pt, T *pt_residual) const {
  //   // Eigen::Map<const Eigen::Matrix<T, 6, 1>> xi_next(pt_next);
  //   // Eigen::Map<const Eigen::Matrix<T, 6, 1>> xi_prev(pt_prev);
  //   Eigen::Matrix<T, 6, 1> xi = Eigen::Map<const Eigen::Matrix<T, 6, 1>>(pt);
  //   // Eigen::Map<Eigen::Matrix<T, 2, 1>> residual(pt_residual);
  //   Eigen::Matrix<T, 1, 1> residual =
  //       Eigen::Map<Eigen::Matrix<T, 1, 1>>(pt_residual);
  //   residual.setZero();

  //   Eigen::Matrix<T, 6, 1> pathi_status = original_status_.template
  //   cast<T>();
  //   // pathi_status.col(0) << original_status_.template cast<T>();
  //   addDistanceResidual(distance_w_, xi, pathi_status, residual[0]);
  //   // addSocialWorkResidual(socialwork_w_, xi, residual[1]);
  //   // addObstacleResidual<T>(obstacle_w_, xi, residual[0]);

  //   return true;
  // }
  template <typename T>
  bool operator()(const T *const pt, T *pt_residual) const {

    Eigen::Matrix<T, 2, 1> p(pt[0], pt[1]);
    Eigen::Matrix<T, 2, 1> p_ori((T)original_status_[0],
                                 (T)original_status_[1]);
    pt_residual[0] = (T)distance_w_ * (p - p_ori).squaredNorm();

    return true;
  }

protected:
  /**
   * @brief Cost function derivative term for steering away changes in pose
   * @param weight Weight to apply to function
   * @param xi Point Xi for evaluation
   * @param xi_original original point Xi for evaluation
   * @param r Residual (cost) of term
   */
  template <typename T>
  inline void
  addDistanceResidual(const double &weight, const Eigen::Matrix<T, 6, 1> &xi,
                      const Eigen::Matrix<T, 6, 1> &xi_original, T &r) const {

    Eigen::Matrix<T, 2, 1> p(xi[0], xi[1]);
    Eigen::Matrix<T, 2, 1> p_ori(xi_original[0], xi_original[1]);
    r += (T)weight * (p - p_ori).squaredNorm(); // objective function value
    // std::cout << "p x:" << (double)p[0] << "y:" << (double)p[1]
    //          << "po x:" << (double)p_ori[0] << "y:" << (double)p_ori[1]
    //          << " r:" << (double)r << std::endl;
  }

  // If we are using the local costmap, the coordinates do not match.
  // check it!!
  template <typename T>
  inline void addObstacleResidual(const double &weight,
                                  const Eigen::Matrix<T, 6, 1> &xi,
                                  T &r) const {
    Eigen::Matrix<T, 2, 1> p(xi[0], xi[1]);
    Eigen::Matrix<T, 2, 1> interp_pos =
        (p - costmap_origin_.template cast<T>()) / (T)costmap_resolution_;
    T value;
    costmap_interpolator_->Evaluate(interp_pos[1] - (T)0.5,
                                    interp_pos[0] - (T)0.5, &value);
    r += (T)weight * value * value; // objective function value
  }

  template <typename T>
  inline void
  addSocialWorkResidual(const double &weight,
                        // const Eigen::Matrix<double, 6, 1> &pt_prev,
                        const Eigen::Matrix<T, 6, 1> &pt,
                        // const Eigen::Matrix<double, 6, 1> &pt_next,
                        T &r) const {

    // Compute robot social work
    Eigen::Matrix<T, 6, 3> agents = original_agents_.template cast<T>();
    Eigen::Matrix<T, 2, 1> robot_sf = computeSocialForce(pt, agents);
    T wr = (T)robot_sf.squaredNorm();

    // compute agents' social work provoked by the robot
    T wp = (T)0.0;
    Eigen::Matrix<T, 6, 3> robot_agent;
    robot_agent.col(0) << pt;
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
    r += (T)weight * (wr + wp);
  }

  // /**
  //  * @brief Cost function term for steering away from costs
  //  * @param weight Weight to apply to function
  //  * @param value Point Xi's cost'
  //  * @param params computed values to reduce overhead
  //  * @param r Residual (cost) of term
  //  */
  // template <typename T>
  // inline void
  // addCostResidual(const double &weight, const Eigen::Matrix<T, 2, 1>
  // &pt_prev,
  //                 const Eigen::Matrix<T, 2, 1> &pt,
  //                 const Eigen::Matrix<T, 2, 1> &pt_next, T &r) const {
  //   if (params_.cost_check_points.empty()) {
  //     Eigen::Matrix<T, 2, 1> interp_pos =
  //         (pt - costmap_origin_.template cast<T>()) / (T)costmap_resolution_;
  //     T value;
  //     costmap_interpolator_->Evaluate(interp_pos[1] - (T)0.5,
  //                                     interp_pos[0] - (T)0.5, &value);
  //     r += (T)weight * value * value; // objective function value
  //   } else {
  //     Eigen::Matrix<T, 2, 1> dir =
  //         tangentDir(pt_prev, pt, pt_next, next_to_last_length_ratio_ < 0);
  //     dir.normalize();
  //     if (((pt_next - pt).dot(dir) < (T)0) != reversing_) {
  //       dir = -dir;
  //     }
  //     Eigen::Matrix<T, 3, 3> transform;
  //     transform << dir[0], -dir[1], pt[0], dir[1], dir[0], pt[1], (T)0, (T)0,
  //         (T)1;
  //     for (size_t i = 0; i < params_.cost_check_points.size(); i += 3) {
  //       Eigen::Matrix<T, 3, 1> ccpt((T)params_.cost_check_points[i],
  //                                   (T)params_.cost_check_points[i + 1],
  //                                   (T)1);
  //       auto ccpt_world = (transform * ccpt).template block<2, 1>(0, 0);
  //       Eigen::Matrix<T, 2, 1> interp_pos =
  //           (ccpt_world - costmap_origin_.template cast<T>()) /
  //           (T)costmap_resolution_;
  //       T value;
  //       costmap_interpolator_->Evaluate(interp_pos[1] - (T)0.5,
  //                                       interp_pos[0] - (T)0.5, &value);

  //       r += (T)weight * (T)params_.cost_check_points[i + 2] * value * value;
  //     }
  //   }
  // }

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

  template <typename T> inline T normalizeAngle(const T &angle) {
    T a = *angle;
    while (a <= -(T)M_PI)
      a += (T)2 * (T)M_PI;
    while (a > (T)M_PI)
      a -= (T)2 * (T)M_PI;
    return a;
  }

  const Eigen::Matrix<double, 6, 1> original_status_;
  Eigen::Matrix<double, 6, 3> original_agents_; // ceres::DYNAMIC
  double distance_w_;
  double socialwork_w_;
  double obstacle_w_;
  Eigen::Vector2d costmap_origin_;
  double costmap_resolution_;
  std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>
      costmap_interpolator_;
  const float timestep_;
  double sfm_lambda_;
  double sfm_gamma_;
  double sfm_nPrime_;
  double sfm_n_;
  double sfm_relaxationTime_;
  double sfm_forceFactorSocial_;
};

} // namespace nav2_social_mpc_controller

#endif // NAV2_CONSTRAINED_SMOOTHER__SMOOTHER_COST_FUNCTION_HPP_
