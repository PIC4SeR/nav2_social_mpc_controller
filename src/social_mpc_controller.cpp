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

#include "nav2_core/exceptions.hpp"
#include "nav2_social_mpc_controller/social_mpc_controller.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;
using namespace nav2_costmap_2d; // NOLINT

double clamp(double value, double min, double max) {
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}

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

void SocialMPCController::configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr &node, std::string name,
    const std::shared_ptr<tf2_ros::Buffer> &tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros) {
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  double transform_tolerance = 0.1;
  // double control_frequency = 20.0;

  declare_parameter_if_not_declared(node, plugin_name_ + ".desired_linear_vel",
                                    rclcpp::ParameterValue(0.5));
  node->get_parameter(plugin_name_ + ".desired_linear_vel",
                      desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".transform_tolerance",
                      transform_tolerance);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  // control_duration_ = 1.0 / control_frequency;

  // if (inflation_cost_scaling_factor_ <= 0.0) {
  //   RCLCPP_WARN(
  //       logger_,
  //       "The value inflation_cost_scaling_factor is incorrectly set, "
  //       "it should be >0. Disabling cost regulated linear velocity
  //       scaling.");
  //   use_cost_regulated_linear_velocity_scaling_ = false;
  // }

  // Create the trajectorizer
  trajectorizer_ = std::make_unique<PathTrajectorizer>();
  trajectorizer_->configure(node, name, tf_);

  // Create the optimizer
  optimizer_ = std::make_unique<Optimizer>();
  optimizer_params_.get(node.get(), name);

  global_path_pub_ =
      node->create_publisher<nav_msgs::msg::Path>("robot_global_plan", 1);
  // carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
  //    "lookahead_point", 1);
  // carrot_arc_pub_ =
  //    node->create_publisher<nav_msgs::msg::Path>("lookahead_collision_arc",
  //    1);
}

void SocialMPCController::cleanup() {
  RCLCPP_INFO(logger_,
              "Cleaning up controller: %s of type"
              "nav2_social_mpc_controller::SocialMPCController",
              plugin_name_.c_str());
  global_path_pub_.reset();
  // carrot_pub_.reset();
  // carrot_arc_pub_.reset();
}

void SocialMPCController::activate() {
  RCLCPP_INFO(logger_,
              "Activating controller: %s of type "
              "nav2_social_mpc_controller::SocialMPCController",
              plugin_name_.c_str());
  trajectorizer_->activate();
  global_path_pub_->on_activate();
  // carrot_pub_->on_activate();
  // carrot_arc_pub_->on_activate();
}

void SocialMPCController::deactivate() {
  RCLCPP_INFO(logger_,
              "Deactivating controller: %s of type "
              "nav2_social_mpc_controller::SocialMPCController",
              plugin_name_.c_str());
  trajectorizer_->deactivate();
  global_path_pub_->on_deactivate();
  // carrot_pub_->on_deactivate();
  // carrot_arc_pub_->on_deactivate();
}

// std::unique_ptr<geometry_msgs::msg::PointStamped>
// SocialMPCController::createCarrotMsg(
//     const geometry_msgs::msg::PoseStamped &carrot_pose) {
//   auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
//   carrot_msg->header = carrot_pose.header;
//   carrot_msg->point.x = carrot_pose.pose.position.x;
//   carrot_msg->point.y = carrot_pose.pose.position.y;
//   carrot_msg->point.z = 0.01; // publish right over map to stand out
//   return carrot_msg;
// }

// double SocialMPCController::getLookAheadDistance(
//     const geometry_msgs::msg::Twist &speed) {
//   // If using velocity-scaled look ahead distances, find and clamp the dist
//   // Else, use the static look ahead distance
//   double lookahead_dist = lookahead_dist_;
//   if (use_velocity_scaled_lookahead_dist_) {
//     lookahead_dist = speed.linear.x * lookahead_time_;
//     lookahead_dist =
//         clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
//   }

//   return lookahead_dist;
// }

geometry_msgs::msg::TwistStamped SocialMPCController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &speed) {

  // Get robot pose in the same frame of the global path
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException(
        "Unable to transform robot pose into global plan's frame");
  }

  // Trajectorize the path
  nav_msgs::msg::Path traj_path = global_plan_;
  std::vector<geometry_msgs::msg::TwistStamped> cmds;
  trajectorizer_->trajectorize(traj_path, robot_pose, cmds);

  // Do the optimization stuff
  RCLCPP_INFO(logger_, "Current speed, vx: %.2f, vz: %.2f", speed.linear.x,
              speed.angular.z);

  // populate and return twist message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = cmds[0].header;
  cmd_vel.twist.linear.x = cmds[0].twist.linear.x;
  cmd_vel.twist.linear.y = cmds[0].twist.linear.y;
  cmd_vel.twist.angular.z = cmds[0].twist.angular.z;
  return cmd_vel;
}

// bool SocialMPCController::shouldRotateToPath(
//     const geometry_msgs::msg::PoseStamped &carrot_pose, double
//     &angle_to_path) {
//   // Whether we should rotate robot to rough path heading
//   angle_to_path =
//       atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
//   return use_rotate_to_heading_ &&
//          fabs(angle_to_path) > rotate_to_heading_min_angle_;
// }

// bool SocialMPCController::shouldRotateToGoalHeading(
//     const geometry_msgs::msg::PoseStamped &carrot_pose) {
//   // Whether we should rotate robot to goal heading
//   double dist_to_goal =
//       std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
//   return use_rotate_to_heading_ && dist_to_goal < goal_dist_tol_;
// }

// void SocialMPCController::rotateToHeading(
//     double &linear_vel, double &angular_vel, const double &angle_to_path,
//     const geometry_msgs::msg::Twist &curr_speed) {
//   // Rotate in place using max angular velocity / acceleration possible
//   linear_vel = 0.0;
//   const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
//   angular_vel = sign * rotate_to_heading_angular_vel_;

//   const double &dt = control_duration_;
//   const double min_feasible_angular_speed =
//       curr_speed.angular.z - max_angular_accel_ * dt;
//   const double max_feasible_angular_speed =
//       curr_speed.angular.z + max_angular_accel_ * dt;
//   angular_vel = clamp(angular_vel, min_feasible_angular_speed,
//                       max_feasible_angular_speed);
// }

// geometry_msgs::msg::PoseStamped SocialMPCController::getLookAheadPoint(
//     const double &lookahead_dist, const nav_msgs::msg::Path
//     &transformed_plan) {
//   // Find the first pose which is at a distance greater than the lookahead
//   // distance
//   auto goal_pose_it = std::find_if(
//       transformed_plan.poses.begin(), transformed_plan.poses.end(),
//       [&](const auto &ps) {
//         return hypot(ps.pose.position.x, ps.pose.position.y) >=
//         lookahead_dist;
//       });

//   // If the no pose is not far enough, take the last pose
//   if (goal_pose_it == transformed_plan.poses.end()) {
//     goal_pose_it = std::prev(transformed_plan.poses.end());
//   }

//   return *goal_pose_it;
// }

// bool SocialMPCController::isCollisionImminent(
//     const geometry_msgs::msg::PoseStamped &robot_pose, const double
//     &linear_vel, const double &angular_vel) {
//   // Note(stevemacenski): This may be a bit unusual, but the robot_pose is in
//   // odom frame and the carrot_pose is in robot base frame.

//   // check current point is OK
//   if (inCollision(robot_pose.pose.position.x, robot_pose.pose.position.y)) {
//     return true;
//   }

//   // visualization messages
//   nav_msgs::msg::Path arc_pts_msg;
//   arc_pts_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
//   arc_pts_msg.header.stamp = robot_pose.header.stamp;
//   geometry_msgs::msg::PoseStamped pose_msg;
//   pose_msg.header.frame_id = arc_pts_msg.header.frame_id;
//   pose_msg.header.stamp = arc_pts_msg.header.stamp;

//   const double projection_time = costmap_->getResolution() /
//   fabs(linear_vel);

//   geometry_msgs::msg::Pose2D curr_pose;
//   curr_pose.x = robot_pose.pose.position.x;
//   curr_pose.y = robot_pose.pose.position.y;
//   curr_pose.theta = tf2::getYaw(robot_pose.pose.orientation);

//   int i = 1;
//   while (true) {
//     // only forward simulate within time requested
//     if (i * projection_time > max_allowed_time_to_collision_) {
//       break;
//     }

//     i++;

//     // apply velocity at curr_pose over distance
//     curr_pose.x += projection_time * (linear_vel * cos(curr_pose.theta));
//     curr_pose.y += projection_time * (linear_vel * sin(curr_pose.theta));
//     curr_pose.theta += projection_time * angular_vel;

//     // store it for visualization
//     pose_msg.pose.position.x = curr_pose.x;
//     pose_msg.pose.position.y = curr_pose.y;
//     pose_msg.pose.position.z = 0.01;
//     arc_pts_msg.poses.push_back(pose_msg);

//     // check for collision at this point
//     if (inCollision(curr_pose.x, curr_pose.y)) {
//       carrot_arc_pub_->publish(arc_pts_msg);
//       return true;
//     }
//   }

//   carrot_arc_pub_->publish(arc_pts_msg);

//   return false;
// }

// bool SocialMPCController::inCollision(const double &x, const double &y) {
//   unsigned int mx, my;
//   costmap_->worldToMap(x, y, mx, my);

//   unsigned char cost = costmap_->getCost(mx, my);

//   if (costmap_ros_->getLayeredCostmap()->isTrackingUnknown()) {
//     return cost >= INSCRIBED_INFLATED_OBSTACLE && cost != NO_INFORMATION;
//   } else {
//     return cost >= INSCRIBED_INFLATED_OBSTACLE;
//   }
// }

// double SocialMPCController::costAtPose(const double &x, const double &y) {
//   unsigned int mx, my;
//   costmap_->worldToMap(x, y, mx, my);

//   unsigned char cost = costmap_->getCost(mx, my);
//   return static_cast<double>(cost);
// }

// void SocialMPCController::applyConstraints(
//     const double &dist_error, const double &lookahead_dist,
//     const double &curvature, const geometry_msgs::msg::Twist &
//     /*curr_speed*/, const double &pose_cost, double &linear_vel) {
//   double curvature_vel = linear_vel;
//   double cost_vel = linear_vel;
//   double approach_vel = linear_vel;

//   // limit the linear velocity by curvature
//   const double radius = fabs(1.0 / curvature);
//   const double &min_rad = regulated_linear_scaling_min_radius_;
//   if (use_regulated_linear_velocity_scaling_ && radius < min_rad) {
//     curvature_vel *= 1.0 - (fabs(radius - min_rad) / min_rad);
//   }

//   // limit the linear velocity by proximity to obstacles
//   if (use_cost_regulated_linear_velocity_scaling_ &&
//       pose_cost != static_cast<double>(NO_INFORMATION) &&
//       pose_cost != static_cast<double>(FREE_SPACE)) {
//     const double inscribed_radius =
//         costmap_ros_->getLayeredCostmap()->getInscribedRadius();
//     const double min_distance_to_obstacle =
//         (-1.0 / inflation_cost_scaling_factor_) *
//             std::log(pose_cost / (INSCRIBED_INFLATED_OBSTACLE - 1)) +
//         inscribed_radius;

//     if (min_distance_to_obstacle < cost_scaling_dist_) {
//       cost_vel *=
//           cost_scaling_gain_ * min_distance_to_obstacle / cost_scaling_dist_;
//     }
//   }

//   // Use the lowest of the 2 constraint heuristics, but above the minimum
//   // translational speed
//   linear_vel = std::min(cost_vel, curvature_vel);
//   linear_vel = std::max(linear_vel, regulated_linear_scaling_min_speed_);

//   // if the actual lookahead distance is shorter than requested, that means
//   // we're at the end of the path. We'll scale linear velocity by error to
//   slow
//   // to a smooth stop
//   if (use_approach_vel_scaling_ &&
//       dist_error > 2.0 * costmap_->getResolution()) {
//     double velocity_scaling = 1.0 - (dist_error / lookahead_dist);
//     double unbounded_vel = approach_vel * velocity_scaling;
//     if (unbounded_vel < min_approach_linear_velocity_) {
//       approach_vel = min_approach_linear_velocity_;
//     } else {
//       approach_vel *= velocity_scaling;
//     }

//     // Use the lowest velocity between approach and other constraints, if all
//     // overlapping
//     linear_vel = std::min(linear_vel, approach_vel);
//   }

//   // Limit linear velocities to be valid and kinematically feasible, v = v0 +
//   a
//   // * dt
//   linear_vel = clamp(linear_vel, 0.0, desired_linear_vel_);
// }

void SocialMPCController::setPlan(const nav_msgs::msg::Path &path) {
  global_plan_ = path;
}

nav_msgs::msg::Path SocialMPCController::transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped &pose) {
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException(
        "Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
  const double max_costmap_dim =
      std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
  const double max_transform_dist =
      max_costmap_dim * costmap->getResolution() / 2.0;

  // First find the closest pose on the path to the robot
  auto transformation_begin =
      min_by(global_plan_.poses.begin(), global_plan_.poses.end(),
             [&robot_pose](const geometry_msgs::msg::PoseStamped &ps) {
               return euclidean_distance(robot_pose, ps);
             });

  // Find points definitely outside of the costmap so we won't transform them.
  auto transformation_end =
      std::find_if(transformation_begin, end(global_plan_.poses),
                   [&](const auto &global_plan_pose) {
                     return euclidean_distance(robot_pose, global_plan_pose) >
                            max_transform_dist;
                   });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto &global_plan_pose) {
    geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
    stamped_pose.header.frame_id = global_plan_.header.frame_id;
    stamped_pose.header.stamp = robot_pose.header.stamp;
    stamped_pose.pose = global_plan_pose.pose;
    transformPose(costmap_ros_->getBaseFrameID(), stamped_pose,
                  transformed_pose);
    return transformed_pose;
  };

  // Transform the near part of the global plan into the robot's frame of
  // reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(transformation_begin, transformation_end,
                 std::back_inserter(transformed_plan.poses),
                 transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  global_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool SocialMPCController::transformPose(
    const std::string frame, const geometry_msgs::msg::PoseStamped &in_pose,
    geometry_msgs::msg::PoseStamped &out_pose) const {
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

} // namespace nav2_social_mpc_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_social_mpc_controller::SocialMPCController,
                       nav2_core::Controller)
