#include "nav2_social_mpc_controller/obstacle_distance_interface.hpp"

namespace nav2_social_mpc_controller
{

// TODO:
// add a function that takes the occupancyGrid of the costmap as argument
// and call the obstacleDistance service to get the ObstacleDistance msg
ObstacleDistInterface::ObstacleDistInterface(rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
                                             const std::string& agents_frame_id,
                                             std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                                             tf2::Duration transform_tolerance)
{
  auto node = parent.lock();
  agents_frame_id_ = agents_frame_id;
  tf_buffer_ = tf_buffer;
  transform_tolerance_ = transform_tolerance;
}

ObstacleDistInterface::~ObstacleDistInterface()
{
}

void ObstacleDistInterface::computeObstacleDistance(nav2_costmap_2d::Costmap2DROS& costmap_ros)
{
  nav2_costmap_2d::Costmap2D* costmap = costmap_ros.getCostmap();
  int width = costmap->getSizeInCellsX();
  int height = costmap->getSizeInCellsY();
  unsigned char* data = costmap->getCharMap();
  if (data == nullptr || width == 0 || height == 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ObstacleDistInterface"), "Empty costmap data");
    return;
  }
  // Convert costmap to OpenCV image
  cv::Mat binary_map = cv::Mat::zeros(height, width, CV_8UC1);  // binary map for occupied cells
  cv::Mat dist_map = cv::Mat::zeros(binary_map.size(), CV_32FC1);
  cv::Mat labels = cv::Mat(binary_map.size(), CV_32SC1);      // 32-bit signed int single channel image for labels
  cv::Mat indexes = cv::Mat::zeros(height, width, CV_32SC1);  // 32-bit signed int single channel image for indexes
  // cv::Mat nearest_obstacle_coords = cv::Mat::zeros(height, width, CV_32SC2);  // 32-bit signed int single channel

  unsigned char map_occ_thresh = (unsigned char)obs_thresh_;  // threshold for occupied cells

  cv::Mat original_map = cv::Mat(height, width, CV_8UC1, data);

  // use thresholding to create a binary map
  cv::threshold(original_map, binary_map, map_occ_thresh, 255, cv::THRESH_BINARY);
  // Compute the distance transform
  cv::distanceTransform(binary_map, dist_map, labels, cv::DIST_L2, cv::DIST_MASK_PRECISE, cv::DIST_LABEL_PIXEL);
  // compute labels
  std::vector<cv::Point> obstacle_points;
  cv::findNonZero(binary_map, obstacle_points);  // gets coordinates where binary_map

  // is non-zero (occupied cells)
  for (size_t i = 0; i < obstacle_points.size(); ++i)
  {
    // Get the coordinates of the obstacle points
    // int y = obstacle_points[i].y;
    // int x = obstacle_points[i].x;
    // condition ( if the labels at the point is equal to i + 1, then set the
    // nearest_obstacle_coords at that point to the coordinates of the obstacle)
    // i + 1 because labels are 1-based
    cv::Mat mask = (labels == (i + 1));
    indexes.setTo(obstacle_points[i].x + obstacle_points[i].y * width,
                  mask);  // Set the indexes at the point to the index of the obstacle point

    // Set the indexes at the point to the index of the obstacle point
  }
  obs_.header.stamp = rclcpp::Clock().now();
  obs_.info.width = width;
  obs_.info.height = height;
  obs_.info.resolution = costmap->getResolution();
  obs_.info.origin.position.x = costmap->getOriginX();
  obs_.info.origin.position.y = costmap->getOriginY();
  obs_.header.frame_id = costmap_ros.getGlobalFrameID();
  obs_.distances = std::vector<float>(dist_map.begin<float>(), dist_map.end<float>());
  obs_.indexes = std::vector<int>(indexes.begin<int>(), indexes.end<int>());
}

obstacle_distance_msgs::msg::ObstacleDistance ObstacleDistInterface::getDistanceTransform()
{
  mutex_.lock();
  obstacle_distance_msgs::msg::ObstacleDistance o = obs_;
  mutex_.unlock();
  return o;
}

void ObstacleDistInterface::worldpoint2Cell(const geometry_msgs::msg::Point& p, unsigned int& i, unsigned int& j)
{
  mutex_.lock();
  // We account for no rotation of the map
  i = (unsigned int)floor((p.x - obs_.info.origin.position.x) / obs_.info.resolution);
  j = (unsigned int)floor((p.y - obs_.info.origin.position.y) / obs_.info.resolution);
  mutex_.unlock();
}

void ObstacleDistInterface::cell2Worldpoint(const unsigned int& i, const unsigned int& j, geometry_msgs::msg::Point& p)
{
  mutex_.lock();
  p.x = i * obs_.info.resolution + obs_.info.origin.position.x;
  p.y = j * obs_.info.resolution + obs_.info.origin.position.y;
  p.z = 0.0;
  mutex_.unlock();
}

bool ObstacleDistInterface::cell2Index(const unsigned int& i, const unsigned int& j, unsigned int& index)
{
  mutex_.lock();
  unsigned int x = i;
  unsigned int y = j;
  if (x >= (unsigned int)obs_.info.width)
  {
    x = (unsigned int)(obs_.info.width - 1);
  }
  if (y >= (unsigned int)obs_.info.height)
  {
    y = (unsigned int)(obs_.info.height - 1);
  }
  // Get the index in the array
  index = x + y * obs_.info.width;
  mutex_.unlock();
  return true;
}

bool ObstacleDistInterface::index2Cell(const unsigned int& index, unsigned int& i, unsigned int& j)
{
  mutex_.lock();
  unsigned int idx = index;
  if (idx >= (obs_.info.width * obs_.info.height))
  {
    idx = (obs_.info.width * obs_.info.height) - 1;
  }
  const div_t result = div((int)idx, (int)obs_.info.width);
  i = result.rem;
  j = result.quot;
  mutex_.unlock();
  return true;
}

// get the posestamped from the obstacle distance msg
geometry_msgs::msg::PoseStamped
ObstacleDistInterface::getOriginPoseStamped(const obstacle_distance_msgs::msg::ObstacleDistance& od)
{
  geometry_msgs::msg::PoseStamped origin_pose;
  origin_pose.header = od.header;
  origin_pose.pose.position = od.info.origin.position;
  origin_pose.pose.orientation = od.info.origin.orientation;
  return origin_pose;
}

bool ObstacleDistInterface::transformObstacleDistance(const std::string& out_frame_id,
                                                      obstacle_distance_msgs::msg::ObstacleDistance& od)
{
  geometry_msgs::msg::PoseStamped origin_pose = getOriginPoseStamped(od);
  geometry_msgs::msg::PoseStamped transformed_origin;

  if (!transformPose(out_frame_id, origin_pose, transformed_origin))
  {
    RCLCPP_ERROR(rclcpp::get_logger("ObstacleDistInterface"), "Failed to transform origin pose: %s",
                 origin_pose.header.frame_id.c_str());
    return false;
  }

  od.header.frame_id = out_frame_id;
  od.info.origin = transformed_origin.pose;
  return true;
}

bool ObstacleDistInterface::transformPose(const std::string frame, const geometry_msgs::msg::PoseStamped& in_pose,
                                          geometry_msgs::msg::PoseStamped& out_pose) const
{
  if (in_pose.header.frame_id == frame)
  {
    out_pose = in_pose;
    return true;
  }

  try
  {
    tf_buffer_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ObstacleDistInterface"), "Exception in transformPose: %s", ex.what());
  }
  return false;
}

}  // namespace nav2_social_mpc_controller
