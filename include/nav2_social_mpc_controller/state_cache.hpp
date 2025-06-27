#ifndef NAV2_SOCIAL_MPC_CONTROLLER__STATE_CACHE_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__STATE_CACHE_HPP_

#include "ceres/ceres.h"
#include "ceres/cost_function.h"
#include "ceres/cubic_interpolation.h"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_social_mpc_controller/tools/type_definitions.hpp"
namespace nav2_social_mpc_controller
{

//template <typename T>
//
//class StateCache {
// public:
//  StateCache(const geometry_msgs::msg::Pose& init_pose)
//  {
//    T x = T(init_pose.position.x);
//    T y = T(init_pose.position.y);
//    T theta = T(tf2::getYaw(init_pose.orientation));
//    states_.emplace_back(x, y, theta);
//  }
//
//  // Compute the next state from the last one and append it
//  void appendState(T const* control_input, double dt)
//  {
//    const auto& [x_prev, y_prev, theta_prev] = states_.back();
//    T v = control_input[0];
//    T omega = control_input[1];
//
//    T x = x_prev + v * ceres::cos(theta_prev) * T(dt);
//    T y = y_prev + v * ceres::sin(theta_prev) * T(dt);
//    T theta = theta_prev + omega * T(dt);
//
//    states_.emplace_back(x, y, theta);
//  }
//
//  const std::tuple<T, T, T>& getState(unsigned int index) const {
//    return states_.at(index);
//  }
//
//  size_t size() const { return states_.size(); }
//
// private:
//  std::vector<std::tuple<T, T, T>> states_;
//};
//forward_state initialState (const geometry_msgs::msg::Pose& init_pose)
//{
//    ceres::Jet<double, 4> x(init_pose.position.x, 0);
//    ceres::Jet<double, 4> y(init_pose.position.y, 0);
//    ceres::Jet<double, 4> theta(tf2::getYaw(init_pose.orientation), 0);
//    return std::make_tuple(x, y, theta);
//}
//forward_state nextStepComputation(const forward_state& current_state, const double* control_input, double dt)
//{
//    auto [x, y, theta] = current_state;
//    auto v = control_input[0];
//    auto omega = control_input[1];
//
//    x += v * ceres::cos(theta) * dt;
//    y += v * ceres::sin(theta) * dt;
//    theta += omega * dt;
//
//    return std::make_tuple(x, y, theta);
//}
}

#endif  // NAV2_SOCIAL_MPC_CONTROLLER__STATE_CACHE_HPP_