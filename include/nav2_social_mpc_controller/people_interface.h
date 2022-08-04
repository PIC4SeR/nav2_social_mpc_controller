#ifndef NAV2_SOCIAL_MPC_CONTROLLER__PEOPLE_H_
#define NAV2_SOCIAL_MPC_CONTROLLER__PEOPLE_H_

#include "people_msgs/msg/people.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <mutex>

namespace nav2_social_mpc_controller {

class PeopleInterface {
public:
  PeopleInterface(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node);
  ~PeopleInterface();

  people_msgs::msg::People getPeople();

private:
  /**
   * @brief callback to process people msgs
   *ret
   * @param msg people msg
   */
  void people_callback(const people_msgs::msg::People::SharedPtr msg);

  rclcpp::Subscription<people_msgs::msg::People>::ConstSharedPtr people_sub_;
  people_msgs::msg::People people_;
  std::mutex mutex_;
};
} // namespace nav2_social_mpc_controller

#endif