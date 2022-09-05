#ifndef NAV2_SOCIAL_MPC_CONTROLLER__OBSTACLE_INTERFACE_H_
#define NAV2_SOCIAL_MPC_CONTROLLER__OBSTACLE_INTERFACE_H_

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "obstacle_distance_msgs/msg/obstacle_distance.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <mutex>

namespace nav2_social_mpc_controller {

class ObstacleDistInterface {
public:
  ObstacleDistInterface(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node);
  ~ObstacleDistInterface();

  obstacle_distance_msgs::msg::ObstacleDistance getDistanceTransform();

  void worldpoint2Cell(const geometry_msgs::msg::Point &p, unsigned int &i,
                       unsigned int &j);
  void cell2Worldpoint(const unsigned int &i, const unsigned int &j,
                       geometry_msgs::msg::Point &p);
  bool cell2Index(const unsigned int &i, const unsigned int &j,
                  unsigned int &index);
  bool index2Cell(const unsigned int &index, unsigned int &i, unsigned int &j);

private:
  // bool transformPointStamped(const std::string frame,
  //                            const geometry_msgs::msg::PointStamped
  //                            &in_point, geometry_msgs::msg::PointStamped
  //                            &out_point) const;
  // bool transformPoseStamped(const std::string frame,
  //                           const geometry_msgs::msg::PoseStamped &in_point,
  //                           geometry_msgs::msg::PoseStamped &out_point)
  //                           const;

  /**
   * @brief callback to process obstacle msgs
   *ret
   * @param msg ObstacleDistance msg
   */
  void obs_callback(
      const obstacle_distance_msgs::msg::ObstacleDistance::SharedPtr msg);

  rclcpp::Subscription<
      obstacle_distance_msgs::msg::ObstacleDistance>::ConstSharedPtr obs_sub_;
  obstacle_distance_msgs::msg::ObstacleDistance obs_;
  std::mutex mutex_;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};
} // namespace nav2_social_mpc_controller

#endif