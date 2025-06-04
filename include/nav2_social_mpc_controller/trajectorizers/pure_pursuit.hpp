#pragma once

#include <vector>
#include <cmath>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace nav2_social_mpc_controller
{
class PurePursuit
{
public:
  PurePursuit(double lookahead_distance) : lookahead_distance_(lookahead_distance)
  {
  }

  void setPath(const std::vector<geometry_msgs::msg::Pose2D>& path)
  {
    path_ = path;
  }

  void setLookaheadDistance(double lookahead_distance)
  {
    lookahead_distance_ = lookahead_distance;
  }

  // Returns the target point on the path for pure pursuit
  bool getLookaheadPoint(const geometry_msgs::msg::Pose2D& current_pose, geometry_msgs::msg::Point& lookahead_point)
  {
    for (size_t i = 0; i < path_.size(); ++i)
    {
      double dx = path_[i].x - current_pose.x;
      double dy = path_[i].y - current_pose.y;
      double dist = std::hypot(dx, dy);
      if (dist >= lookahead_distance_)
      {
        lookahead_point.x = path_[i].x;
        lookahead_point.y = path_[i].y;
        lookahead_point.z = 0.0;
        return true;
      }
    }
    // If no point is found, return the last point
    if (!path_.empty())
    {
      lookahead_point.x = path_.back().x;
      lookahead_point.y = path_.back().y;
      lookahead_point.z = 0.0;
      return true;
    }
    return false;
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                           const geometry_msgs::msg::Twist& speed)
  {
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = pose.header.stamp;
    cmd_vel.header.frame_id = pose.header.frame_id;

    geometry_msgs::msg::Point lookahead_point;
    if (getLookaheadPoint(pose.pose, lookahead_point))
    {
      double dx = lookahead_point.x - pose.pose.position.x;
      double dy = lookahead_point.y - pose.pose.position.y;
      double angle_to_target = std::atan2(dy, dx);
      double current_yaw = tf2::getYaw(pose.pose.orientation);
      double angle_error = angles::shortest_angular_distance(current_yaw, angle_to_target);
      double distance_to_target_2 = std::hypot(dx, dy);

      // Proportional control for linear and angular velocities
      double linear_velocity = std::min(speed.linear.x, distance_to_target / 2.0);  // Limit linear velocity
      double curvature = 2.0 * dy /
    }

    return cmd_vel;
  }

private:
  bool shouldRotateToGoalHeading(const geometry_msgs::msg::PoseStamped& carrot_pose)
  {
    // Whether we should rotate robot to goal heading
    double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
    return use_rotate_to_heading_ && dist_to_goal < goal_dist_tol_;
  }

  void rotateToHeading(double& linear_vel, double& angular_vel, const double& angle_to_path,
                       const geometry_msgs::msg::Twist& curr_speed)
  {
    // Rotate in place using max angular velocity / acceleration possible
    linear_vel = 0.0;
    const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
    angular_vel = sign * rotate_to_heading_angular_vel_;

    const double& dt = control_duration_;
    const double min_feasible_angular_speed = curr_speed.angular.z - max_angular_accel_ * dt;
    const double max_feasible_angular_speed = curr_speed.angular.z + max_angular_accel_ * dt;
    angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
  }

  bool shouldRotateToPath(const geometry_msgs::msg::PoseStamped& carrot_pose, double& angle_to_path)
  {
    // Whether we should rotate robot to rough path heading
    angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
    return use_rotate_to_heading_ && fabs(angle_to_path) > rotate_to_heading_min_angle_;
  }

  double lookahead_distance_;
  std::vector<geometry_msgs::msg::Pose2D> path_;
  // use a motion model to compute the trajectory
  // e.g., a simple kinematic model or a more complex dynamic model
};
}  // namespace nav2_social_mpc_controller