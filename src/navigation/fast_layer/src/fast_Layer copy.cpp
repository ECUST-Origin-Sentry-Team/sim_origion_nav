#include "fast_layer/fast_Layer.hpp"

// #include <nav2_costmap_2d/footprint.hpp>
namespace fast_layer
{
FastLayer::FastLayer() {}
FastLayer::~FastLayer() {}
void FastLayer::onInitialize()
{
  bool track_unknown_space;
  std::string topics_string;
  declareParameter(name_ + ".enabled", rclcpp::ParameterValue(true));
  declareParameter(name_ + ".footprint_clearing_enabled", rclcpp::ParameterValue(true));
  declareParameter(name_ + ".pointcloud_topic", rclcpp::ParameterValue("pointcloud"));
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  node->get_parameter("track_unknown_space", track_unknown_space);
  node->get_parameter("pointcloud_topic", topics_string);
  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "footprint_clearing_enabled", footprint_clearing_enabled_);
  if (track_unknown_space) {
    default_value_ = nav2_costmap_2d::NO_INFORMATION;
  } else {
    default_value_ = nav2_costmap_2d::FREE_SPACE;
  }
  pointcloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topics_string, rclcpp::SensorDataQoS(),
    std::bind(&FastLayer::pointCloudCallback, this, std::placeholders::_1));
  rolling_window_ = layered_costmap_->isRolling();
  matchSize();
  was_reset_ = false;
}
void FastLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    return;
  }
  useExtraBounds(min_x, min_y, max_x, max_y);
  //...
  pcl::PointCloud<pcl::PointXYZ> cloud_temp;
  {
    std::lock_guard<std::mutex> lock(mutex_cloud);
    cloud_temp = cloud_;
  }
  for (auto & point : cloud_temp.points) {
    double wx = point.x;
    double wy = point.y;
    // double p_cost = 0.0;
    unsigned int mx, my;
    if (worldToMap(wx, wy, mx, my)) {
      unsigned int index = getIndex(mx, my);
      costmap_[index] = nav2_costmap_2d::LETHAL_OBSTACLE;
      touch(wx, wy, min_x, min_y, max_x, max_y);
    }
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

// void FastLayer::matchSize() {}

void FastLayer::reset() {}

void FastLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }
  if (footprint_clearing_enabled_) {
    setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}
void FastLayer::deactivate() {}
void FastLayer::activate() {}

void FastLayer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Process the incoming point cloud message
  std::string frame_id = msg->header.frame_id;
  if (frame_id != "map") {
    try {
      geometry_msgs::msg::TransformStamped transform_stamped =
        tf_->lookupTransform("map", frame_id, tf2::TimePointZero);
      Eigen::Matrix4f eigen_transform =
        tf2::transformToEigen(transform_stamped).matrix().cast<float>();
      pcl::PointCloud<pcl::PointXYZ> cloud_temp;
      pcl::fromROSMsg(*msg, cloud_temp);
      pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
      pcl::transformPointCloud(cloud_temp, transformed_cloud, eigen_transform);
      std::lock_guard<std::mutex> lock(mutex_cloud);
      cloud_ = transformed_cloud;

    } catch (...) {
      RCLCPP_WARN(
        logger_, "Could not transform point cloud from frame %s to frame map", frame_id.c_str());
      return;
    }
  } else {
    std::lock_guard<std::mutex> lock(mutex_cloud);
    pcl::fromROSMsg(*msg, cloud_);
  }
}
void FastLayer::updateOrigin(double new_origin_x, double new_origin_y) {}

void FastLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {
    return;
  }
  nav2_costmap_2d::transformFootprint(
    robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}
}  // namespace fast_layer
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fast_layer::FastLayer, nav2_costmap_2d::Layer)