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
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "mpc_enlarged_state/path_trajectorizer.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "angles/angles.h"

using nav2_util::declare_parameter_if_not_declared;

namespace mpc_enlarged_state
{

PathTrajectorizer::PathTrajectorizer()
{
}
PathTrajectorizer::~PathTrajectorizer()
{
}

void PathTrajectorizer::configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent, std::string name,
                                  std::shared_ptr<tf2_ros::Buffer> tf)
{
  auto node = parent.lock();
  clock_ = node->get_clock();
  tf_ = tf;
  plugin_name_ = name + ".trajectorizer";
  logger_ = node->get_logger();

  declare_parameter_if_not_declared(node, plugin_name_ + ".omnidirectional", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(node, plugin_name_ + ".base_frame", rclcpp::ParameterValue("base_footprint"));
  declare_parameter_if_not_declared(node, plugin_name_ + ".time_step", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_time", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_failed_trajectorizations", rclcpp::ParameterValue(50));
  declare_parameter_if_not_declared(node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".waypoint_dist_tol", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(node, name + ".max_angular_vel", rclcpp::ParameterValue(1.4));
  declare_parameter_if_not_declared(node, plugin_name_ + ".offset_extra_pose", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".extra_points_to_waypoint", rclcpp::ParameterValue(1));
  
  node->get_parameter(plugin_name_ + ".omnidirectional", omnidirectional_);
  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(name + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".base_frame", base_frame_);
  node->get_parameter(plugin_name_ + ".time_step", time_step_);
  node->get_parameter(plugin_name_ + ".max_failed_trajectorizations", max_iterations_);
  node->get_parameter(plugin_name_ + ".waypoint_dist_tol", waypoint_dist_tol_);
  node->get_parameter(plugin_name_ + ".offset_extra_pose", offset_extra_pose_);

  double max_time;
  node->get_parameter(plugin_name_ + ".max_time", max_time);

  RCLCPP_DEBUG(logger_, "-------------------------------------");
  RCLCPP_DEBUG(logger_, "Path Trajectorizer params:");
  RCLCPP_DEBUG(logger_, "omnidirectional: %i", (int)omnidirectional_);
  RCLCPP_DEBUG(logger_, "desired_linear_vel: %.2f m/s", desired_linear_vel_);
  RCLCPP_DEBUG(logger_, "lookahead_dist: %.2f m", lookahead_dist_);
  RCLCPP_DEBUG(logger_, "max_angular_vel: %.2f rad/s", max_angular_vel_);
  RCLCPP_DEBUG(logger_, "time_step: %.2f secs", time_step_);
  RCLCPP_DEBUG(logger_, "max_time: %.2f secs", max_time);
  RCLCPP_DEBUG(logger_, "base_frame: %s", base_frame_.c_str());
  RCLCPP_DEBUG(logger_, "-------------------------------------");

  max_steps_ = (int)round(max_time / time_step_);

  received_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  computed_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("trajectorized_global_plan", 1);
}

void PathTrajectorizer::cleanup()
{
  RCLCPP_INFO(logger_,
              "Cleaning up path trajectorizer: %s of type"
              " nav2_path_trajectorizer::PathTrajectorizer",
              plugin_name_.c_str());
  received_path_pub_.reset();
  computed_path_pub_.reset();
}

void PathTrajectorizer::activate()
{
  RCLCPP_INFO(logger_,
              "Activating smoother: %s of type "
              "nav2_path_trajectorizer::PathTrajectorizer",
              plugin_name_.c_str());
  received_path_pub_->on_activate();
  computed_path_pub_->on_activate();
}

void PathTrajectorizer::deactivate()
{
  RCLCPP_INFO(logger_,
              "Deactivating smoother: %s of type "
              "nav2_path_trajectorizer::PathTrajectorizer",
              plugin_name_.c_str());
  received_path_pub_->on_deactivate();
  computed_path_pub_->on_deactivate();
}

bool PathTrajectorizer::trajectorize(nav_msgs::msg::Path& path, const geometry_msgs::msg::PoseStamped& path_robot_pose,
                                     std::vector<geometry_msgs::msg::TwistStamped>& cmds)
{
  if (path.poses.empty())
  {
    RCLCPP_WARN(logger_, "Received empty path, cannot trajectorize");
    return false;
  }

  // path_robot_pose must be in the same frame as the path
  geometry_msgs::msg::PoseStamped robot_pose = path_robot_pose;

  double rx = robot_pose.pose.position.x;
  double ry = robot_pose.pose.position.y;
  double rtheta = tf2::getYaw(robot_pose.pose.orientation);
  
  if (path.poses.size() == 1)
  {
    // compute an extra pose in the path in order to allow the trajectorization
    // compute the distance and orientation of the only pose in the path with respect to the robot
    double dx = path.poses[0].pose.position.x - rx;
    double dy = path.poses[0].pose.position.y - ry;
    double dist = std::hypot(dx, dy);

    if (dist < waypoint_dist_tol_)
    {
      RCLCPP_WARN(logger_, "Path has only one pose very close to the robot (%.2f m), cannot trajectorize", dist);
      return false;
    }
    double angle = std::atan2(dy, dx);
    // compute a new pose at offset_extra_pose_ distance from the only pose in the path
    geometry_msgs::msg::PoseStamped new_pose;
    new_pose.header = path.poses[0].header;
    new_pose.pose.position.x = path.poses[0].pose.position.x + offset_extra_pose_ * std::cos(angle);
    new_pose.pose.position.y = path.poses[0].pose.position.y + offset_extra_pose_ * std::sin(angle);
    new_pose.pose.position.z = path.poses[0].pose.position.z;
    new_pose.pose.orientation = path.poses[0].pose.orientation;
    path.poses.push_back(new_pose);
    RCLCPP_DEBUG(logger_, "Path had only one pose, added an extra pose at (%.2f, %.2f)", new_pose.pose.position.x,
                  new_pose.pose.position.y);
  }

  rclcpp::Time t = clock_->now();
  nav_msgs::msg::Path new_path;
  new_path.header.frame_id = path.header.frame_id;
  new_path.header.stamp = t;

  new_path.poses.push_back(robot_pose);
  // Now, do a loop:
  // 1- Find the look-ahead point on the path.
  // 2- Find the cmds to approach the point
  // 3- simulate the robot movement by applying
  // those cmds for a small time step.
  // 4- Repeat steps 1 and 2 for the new simulated robot pose until
  // reaching the end of the path

  double goal_dist = std::numeric_limits<double>::max();
  int steps = 0;
  // for (int steps = 0; steps < max_steps_ && goal_dist > waypoint_dist_tol_; steps++)
  // goal_dist is updated inside the loop
  while (goal_dist > waypoint_dist_tol_ && steps < max_steps_) // max_steps_ to avoid infinite loops
  {
    double wpx; // lookahead point x
    double wpy; // lookahead point y
    double min_dist = std::numeric_limits<double>::max();
    int wp_index = static_cast<int>(path.poses.size() - 1); // lookahead point index
    // --- 1 ---

    for (std::size_t idx = path.poses.size(); idx-- > 0;) // iterate from end to start
    {
      const auto dist = nav2_util::geometry_utils::euclidean_distance(robot_pose.pose, path.poses[idx].pose);
      if (dist <= lookahead_dist_) // first point within lookahead distance
      {
        wp_index = static_cast<int>(idx);
        break;
      }
      if (dist < min_dist) // update closest point if no point within lookahead distance is found
      {
        min_dist = dist;
        wp_index = static_cast<int>(idx);
      }
    }

    wpx = path.poses[wp_index].pose.position.x;
    wpy = path.poses[wp_index].pose.position.y;

    // --- 2 ---
    // Transform way-point into local robot frame and get desired x,y,theta
    double dx = (wpx - rx) * cos(rtheta) + (wpy - ry) * sin(rtheta);
    double dy = -(wpx - rx) * sin(rtheta) + (wpy - ry) * cos(rtheta);
    double dtheta = atan2(dy, dx);
    dtheta = angles::normalize_angle(dtheta);  // it should be not necessary since atan2 is already normalized
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
    // todo use different models for omnidirectional and non-omnidirectional robots
    if (omnidirectional_)
    {
      vx = desired_linear_vel_ * cos(dtheta);
      vy = desired_linear_vel_ * sin(dtheta);
    }
    else  // non-omnidirectional robot (use different control laws to implement the reference trajectory)
    {

      double point_dist2 = (dx * dx + dy * dy);
      double curvature = 0.0;
      
      if (point_dist2 > 1e-3)
      {
        curvature = 2.0 * dy / point_dist2;
      }
      // Setting the velocity direction
      vx = desired_linear_vel_;
      // apply curvature to angular velocity
      wz = vx * curvature;

    }

    // --- 3 ---

    // todo: use a motion model to compute the trajectory
    rx = computeNewXPosition(rx, vx, vy, rtheta, time_step_);
    ry = computeNewYPosition(ry, vx, vy, rtheta, time_step_);
    rtheta = computeNewThetaPosition(rtheta, wz, time_step_);
    // store the point
    robot_pose.pose.position.x = rx;
    robot_pose.pose.position.y = ry;
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, rtheta);
    robot_pose.pose.orientation = tf2::toMsg(myQuaternion);
    rclcpp::Time curr_t = rclcpp::Time(robot_pose.header.stamp);
    rclcpp::Time time = curr_t + rclcpp::Duration(time_step_, 0);
    robot_pose.header.stamp = time;
    new_path.poses.push_back(robot_pose);

    geometry_msgs::msg::TwistStamped vel;
    vel.header.frame_id = base_frame_;
    vel.header.stamp = curr_t;
    vel.twist.linear.x = vx;
    vel.twist.linear.y = vy;
    vel.twist.angular.z = wz;
    cmds.push_back(vel);

    wpx = path.poses[path.poses.size() - 1].pose.position.x;
    wpy = path.poses[path.poses.size() - 1].pose.position.y;
    goal_dist = sqrt((rx - wpx) * (rx - wpx) + (ry - wpy) * (ry - wpy));
    steps++;
  }

  // Publish the path received
  received_path_pub_->publish(path);

  // copy the new path into the path
  path.poses.clear();
  path.poses = new_path.poses;

  // publish the new path
  computed_path_pub_->publish(path);

  return true;
}

}  // namespace mpc_enlarged_state
