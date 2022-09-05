#include "nav2_social_mpc_controller/obstacle_distance_interface.h"

namespace nav2_social_mpc_controller {

// TODO:
// add a function that takes the occupancyGrid of the costmap as argument
// and call the obstacleDistance service to get the ObstacleDistance msg

ObstacleDistInterface::ObstacleDistInterface(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr &node) {
  obs_sub_ =
      node->create_subscription<obstacle_distance_msgs::msg::ObstacleDistance>(
          "obstacle_distance", rclcpp::SystemDefaultsQoS(),
          std::bind(&ObstacleDistInterface::obs_callback, this,
                    std::placeholders::_1));

  node_ = node;
}

ObstacleDistInterface::~ObstacleDistInterface() {}

void ObstacleDistInterface::obs_callback(
    const obstacle_distance_msgs::msg::ObstacleDistance::SharedPtr msg) {
  mutex_.lock();
  obs_ = *msg;
  mutex_.unlock();
}

obstacle_distance_msgs::msg::ObstacleDistance
ObstacleDistInterface::getDistanceTransform() {
  mutex_.lock();
  obstacle_distance_msgs::msg::ObstacleDistance o = obs_;
  mutex_.unlock();
  return o;
}

void ObstacleDistInterface::worldpoint2Cell(const geometry_msgs::msg::Point &p,
                                            unsigned int &i, unsigned int &j) {
  mutex_.lock();
  // We account for no rotation of the map
  i = (unsigned int)floor((p.x - obs_.info.origin.position.x) /
                          obs_.info.resolution);
  j = (unsigned int)floor((p.y - obs_.info.origin.position.y) /
                          obs_.info.resolution);
  mutex_.unlock();
}
void ObstacleDistInterface::cell2Worldpoint(const unsigned int &i,
                                            const unsigned int &j,
                                            geometry_msgs::msg::Point &p) {
  mutex_.lock();
  p.x = i * obs_.info.resolution + obs_.info.origin.position.x;
  p.y = j * obs_.info.resolution + obs_.info.origin.position.y;
  p.z = 0.0;
  mutex_.unlock();
}
bool ObstacleDistInterface::cell2Index(const unsigned int &i,
                                       const unsigned int &j,
                                       unsigned int &index) {
  mutex_.lock();
  unsigned int x = i;
  unsigned int y = j;
  if (x >= (unsigned int)obs_.info.width) {
    x = (unsigned int)(obs_.info.width - 1);
  }
  if (y >= (unsigned int)obs_.info.height) {
    y = (unsigned int)(obs_.info.height - 1);
  }
  // Get the index in the array
  index = x + y * obs_.info.width;
  mutex_.unlock();
  return true;
}
bool ObstacleDistInterface::index2Cell(const unsigned int &index,
                                       unsigned int &i, unsigned int &j) {
  mutex_.lock();
  unsigned int idx = index;
  if (idx >= (obs_.info.width * obs_.info.height)) {
    idx = (obs_.info.width * obs_.info.height) - 1;
    // printf("Index %u out of bound[%u]!!!\n", *index, (map_info_.width *
    // map_info_.height));
    // mutex_.unlock();
    // return false;
  }
  const div_t result = div((int)idx, (int)obs_.info.width);
  // cell.x = result.rem;
  i = result.rem;
  // cell.y = result.quot;
  j = result.quot;
  mutex_.unlock();
  return true;
}

// bool ObstacleDistInterface::transformPointStamped(
//     const std::string frame, const geometry_msgs::msg::PointStamped
//     &in_point, geometry_msgs::msg::PointStamped &out_point) const {

//   try {
//     tf_->transform(in_point, out_point, frame, transform_tolerance_);

//     // what the transform function does:
//     // tf2::doTransform(in, out, lookupTransform(target_frame,
//     // tf2::getFrameId(in), tf2::getTimestamp(in), timeout));
//     // return out;

//     return true;
//   } catch (tf2::TransformException &ex) {
//     RCLCPP_ERROR(logger_, "Exception in transformPointStamped: %s",
//     ex.what());
//   }
//   return false;
// }

// bool ObstacleDistInterface::transformPoseStamped(
//     const std::string frame, const geometry_msgs::msg::PoseStamped &in_point,
//     geometry_msgs::msg::PoseStamped &out_point) const {

//   try {
//     tf_->transform(in_point, out_point, frame, transform_tolerance_);
//     return true;
//   } catch (tf2::TransformException &ex) {
//     RCLCPP_ERROR(logger_, "Exception in transformPoseStamped: %s",
//     ex.what());
//   }
//   return false;
// }

} // namespace nav2_social_mpc_controller
