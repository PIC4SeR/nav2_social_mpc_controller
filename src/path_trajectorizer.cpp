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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// #include "nav2_core/exceptions.hpp"
// #include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_social_mpc_controller/path_trajectorizer.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

#include "tf2/utils.h"

using nav2_util::declare_parameter_if_not_declared;
// using nav2_util::geometry_utils::euclidean_distance;
// using namespace nav2_costmap_2d; // NOLINT

namespace nav2_social_mpc_controller {

/**
 * Find element in iterator with the minimum calculated value
 */
template <typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getCompareVal) {
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

PathTrajectorizer::PathTrajectorizer() {}
PathTrajectorizer::~PathTrajectorizer() {}

void PathTrajectorizer::configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf) {
  // std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  // std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) {

  node_ = parent;
  // auto node = node_.lock();
  // if (!node) {
  //   throw std::runtime_error("Unable to lock node!");
  // }

  // costmap_sub_ = costmap_sub;
  clock_ = node_->get_clock();
  tf_ = tf;
  plugin_name_ = name + ".trajectorizer";
  logger_ = node_->get_logger();

  declare_parameter_if_not_declared(node_, plugin_name_ + ".omnidirectional",
                                    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node_, plugin_name_ + ".desired_linear_vel",
                                    rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(node_, plugin_name_ + ".lookahead_dist",
                                    rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(node_, plugin_name_ + ".max_angular_vel",
                                    rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node_,
                                    plugin_name_ + ".transform_tolerance",
                                    rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node_, plugin_name_ + ".base_frame",
                                    rclcpp::ParameterValue("base_footprint"));
  declare_parameter_if_not_declared(node_, plugin_name_ + ".time_step",
                                    rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node_, plugin_name_ + ".max_time",
                                    rclcpp::ParameterValue(5.0));

  // declare_parameter_if_not_declared(node, plugin_name_ +
  // ".min_lookahead_dist",
  //                                   rclcpp::ParameterValue(0.3));
  // declare_parameter_if_not_declared(node, plugin_name_ +
  // ".max_lookahead_dist",
  //                                   rclcpp::ParameterValue(0.9));
  // declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_time",
  //                                   rclcpp::ParameterValue(1.5));

  node_->get_parameter(plugin_name_ + ".omnidirectional", omnidirectional_);
  node_->get_parameter(plugin_name_ + ".desired_linear_vel",
                       desired_linear_vel_);
  node_->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node_->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node_->get_parameter(plugin_name_ + ".base_frame", base_frame_);
  node_->get_parameter(plugin_name_ + ".time_step", time_step_);
  double max_time;
  node_->get_parameter(plugin_name_ + ".max_time", max_time);
  double transform_tolerance;
  node_->get_parameter(plugin_name_ + ".transform_tolerance",
                       transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

  RCLCPP_INFO(logger_, "-------------------------------------");
  RCLCPP_INFO(logger_, "Path Trajectorizer params:");
  RCLCPP_INFO(logger_, "omnidirectional: %i", (int)omnidirectional_);
  RCLCPP_INFO(logger_, "desired_linear_vel: %.2f m/s", desired_linear_vel_);
  RCLCPP_INFO(logger_, "lookahead_dist: %.2f m", lookahead_dist_);
  RCLCPP_INFO(logger_, "max_angular_vel: %.2f rad/s", max_angular_vel_);
  RCLCPP_INFO(logger_, "time_step: %.2f secs", time_step_);
  RCLCPP_INFO(logger_, "max_time: %.2f secs", max_time);
  RCLCPP_INFO(logger_, "base_frame: %s", base_frame_.c_str());
  RCLCPP_INFO(logger_, "-------------------------------------");

  max_steps_ = (int)round(max_time / time_step_);

  received_path_pub_ =
      node_->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  computed_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
      "trajectorized_global_plan", 1);
}

void PathTrajectorizer::cleanup() {
  RCLCPP_INFO(logger_,
              "Cleaning up path trajectorizer: %s of type"
              " nav2_path_trajectorizer::PathTrajectorizer",
              plugin_name_.c_str());
  received_path_pub_.reset();
  computed_path_pub_.reset();
}

void PathTrajectorizer::activate() {
  RCLCPP_INFO(logger_,
              "Activating smoother: %s of type "
              "nav2_path_trajectorizer::PathTrajectorizer",
              plugin_name_.c_str());
  received_path_pub_->on_activate();
  computed_path_pub_->on_activate();
  // Testing other options to get the robot pose
  // geometry_msgs::msg::PoseStamped global_pose;
  // nav2_util::getCurrentPose(global_pose, *tf_, "map", "base_footprint", 1.0);
  // RCLCPP_INFO(logger_, "robot pose in global frame. x: %.2f, y:%.2f",
  //             global_pose.pose.position.x, global_pose.pose.position.y);
}

void PathTrajectorizer::deactivate() {
  RCLCPP_INFO(logger_,
              "Deactivating smoother: %s of type "
              "nav2_path_trajectorizer::PathTrajectorizer",
              plugin_name_.c_str());
  received_path_pub_->on_deactivate();
  computed_path_pub_->on_deactivate();
}

bool PathTrajectorizer::trajectorize(
    nav_msgs::msg::Path &path,
    const geometry_msgs::msg::PoseStamped &path_robot_pose,
    std::vector<geometry_msgs::msg::TwistStamped> &cmds) {
  if (path.poses.size() < 2) {
    return true;
  }
  rclcpp::Time t = clock_->now(); // node_->get_clock()->now();

  nav_msgs::msg::Path new_path;
  new_path.header.frame_id = path.header.frame_id;
  new_path.header.stamp = t;

  // path_robot_pose must be in the same frame as the path
  geometry_msgs::msg::PoseStamped robot_pose = path_robot_pose;
  new_path.poses.push_back(robot_pose);

  double rx = robot_pose.pose.position.x;
  double ry = robot_pose.pose.position.y;
  double rt = tf2::getYaw(robot_pose.pose.orientation);

  // Now, do a loop:
  // 1- Find the look-ahead point on the path.
  // 2- Find the cmds to approach the point
  // 3- simulate the robot movement by applying
  // those cmds for a small time step.
  // 4- Repeat steps 1 and 2 for the new simulated robot pose until
  // reaching the end of the path

  double goal_dist = 1000.0;
  double goal_dist_threshold = 0.05;
  int steps = 0;
  while (goal_dist > goal_dist_threshold && steps < max_steps_) {
    double wpx;
    double wpy;
    double min_dist = 100.0;
    int wp_index = -1;
    double wp_dist = 1000.0;
    // --- 1 ---
    for (int i = path.poses.size() - 1; i >= 0; i--) {
      wpx = path.poses[i].pose.position.x;
      wpy = path.poses[i].pose.position.y;
      wp_dist = sqrt((rx - wpx) * (rx - wpx) + (ry - wpy) * (ry - wpy));
      // RCLCPP_INFO(logger_, "waypoint i:%i, wx:%.2f, wy:%.2f, d: %.2f", i,
      // wpx,wpy, wp_dist);
      if (wp_dist <= lookahead_dist_) {
        wp_index = i;
        // RCLCPP_INFO(logger_, "waypoint in lookAhead i: %i!!!", wp_index);
        break;
      }
      if (wp_dist < min_dist) {
        min_dist = wp_dist;
        wp_index = i;
      }
    }

    wpx = path.poses[wp_index].pose.position.x;
    wpy = path.poses[wp_index].pose.position.y;

    // --- 2 ---
    // Transform way-point into local robot frame and get desired x,y,theta
    double dx = (wpx - rx) * cos(rt) + (wpy - ry) * sin(rt);
    double dy = -(wpx - rx) * sin(rt) + (wpy - ry) * cos(rt);
    double dt = atan2(dy, dx);
    dt = normalizeAngle(dt, -M_PI, M_PI);

    double vx = 0.0;
    double vy = 0.0;
    double vt = 0.0;
    if (omnidirectional_) {
      vx = desired_linear_vel_ * cos(dt);
      vy = desired_linear_vel_ * sin(dt);
    } else {
      if (dx > 0) {
        vx = desired_linear_vel_ *
             (0.1 + exp(-fabs(dt))); // desired_linear_vel_;
        if (vx > desired_linear_vel_)
          vx = desired_linear_vel_;
        // vt = max_angular_vel_ * dt;
        auto curvature = 2.0 * dy / (dx * dx + dy * dy);
        vt = desired_linear_vel_ * curvature;
      } else {
        vx = 0.0;
        vt = max_angular_vel_;
        if (dt < 0.0)
          vt = -max_angular_vel_;
      }
    }

    // --- 3 ---
    rx = computeNewXPosition(rx, vx, vy, rt, time_step_);
    ry = computeNewYPosition(ry, vx, vy, rt, time_step_);
    rt = computeNewThetaPosition(rt, vt, time_step_);
    // store the point
    robot_pose.pose.position.x = rx;
    robot_pose.pose.position.y = ry;
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, rt);
    robot_pose.pose.orientation = tf2::toMsg(myQuaternion);
    rclcpp::Time curr_t = rclcpp::Time(robot_pose.header.stamp);
    rclcpp::Time time = curr_t + rclcpp::Duration(time_step_, 0);
    robot_pose.header.stamp = time;
    new_path.poses.push_back(robot_pose);

    // cmd vel
    geometry_msgs::msg::TwistStamped vel;
    vel.header.frame_id = base_frame_;
    vel.header.stamp = curr_t;
    vel.twist.linear.x = vx;
    vel.twist.linear.y = vy;
    vel.twist.angular.z = vt;
    cmds.push_back(vel);

    // update goal dist
    wpx = path.poses[path.poses.size() - 1].pose.position.x;
    wpy = path.poses[path.poses.size() - 1].pose.position.y;
    goal_dist = sqrt((rx - wpx) * (rx - wpx) + (ry - wpy) * (ry - wpy));
    steps++;
  }

  // RCLCPP_INFO(logger_, "Path length received: %i \nPath length computed: %i
  // \n",
  //            (int)path.poses.size(), (int)new_path.poses.size());

  // Publish the path received
  received_path_pub_->publish(path);

  // copy the new path into the path
  path.poses.clear();
  path.poses = new_path.poses;

  // publish the new path
  computed_path_pub_->publish(path);

  return true;
}

} // namespace nav2_social_mpc_controller
