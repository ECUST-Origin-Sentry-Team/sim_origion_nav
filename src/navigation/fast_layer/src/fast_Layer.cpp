#include "fast_layer/fast_Layer.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace fast_layer
{
FastLayer::FastLayer() = default;
FastLayer::~FastLayer() = default;

void FastLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("pointcloud_topic", rclcpp::ParameterValue("pointcloud"));
  declareParameter("pointcloud_timeout", rclcpp::ParameterValue(0.5));
  declareParameter("transform_timeout", rclcpp::ParameterValue(0.1));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string pointcloud_topic;
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".pointcloud_topic", pointcloud_topic);
  node->get_parameter(name_ + ".pointcloud_timeout", pointcloud_timeout_);
  node->get_parameter(name_ + ".transform_timeout", transform_timeout_);
  if (pointcloud_timeout_ < 0.0) {
    throw std::invalid_argument(name_ + ".pointcloud_timeout must be >= 0");
  }
  if (transform_timeout_ < 0.0) {
    throw std::invalid_argument(name_ + ".transform_timeout must be >= 0");
  }

  clock_ = node->get_clock();
  global_frame_ = layered_costmap_->getGlobalFrameID();
  rolling_window_ = layered_costmap_->isRolling();
  default_value_ = layered_costmap_->isTrackingUnknown() ?
    nav2_costmap_2d::NO_INFORMATION : nav2_costmap_2d::FREE_SPACE;
  matchSize();
  resetMaps();
  current_ = true;

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  pointcloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&FastLayer::pointCloudCallback, this, std::placeholders::_1), options);

  RCLCPP_INFO(
    logger_, "Subscribed to %s (stale timeout %.3f s, TF timeout %.3f s)",
    pointcloud_topic.c_str(), pointcloud_timeout_, transform_timeout_);
}

void FastLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  (void)robot_yaw;
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  useExtraBounds(min_x, min_y, max_x, max_y);
  if (!enabled_) {
    return;
  }

  // DOG map messages are complete snapshots, not incremental observations.
  // Rebuild this layer every cycle so an empty/new snapshot removes old cells.
  resetMaps();

  pcl::PointCloud<pcl::PointXYZ> cloud_temp;
  bool stale_cloud_cleared = false;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (cloud_received_ && pointcloud_timeout_ > 0.0) {
      const int64_t now_ns = clock_->now().nanoseconds();
      if (now_ns < last_cloud_receive_time_ns_) {
        // A simulation reset can move ROS time backwards. Keep the current
        // observation and restart its timeout window.
        last_cloud_receive_time_ns_ = now_ns;
      } else {
        const double age =
          static_cast<double>(now_ns - last_cloud_receive_time_ns_) * 1e-9;
        if (age > pointcloud_timeout_) {
          cloud_.clear();
          cloud_received_ = false;
          stale_cloud_cleared = true;
        }
      }
    }
    cloud_temp = cloud_;
  }

  if (stale_cloud_cleared) {
    RCLCPP_WARN(
      logger_, "Point cloud timed out after %.3f s; clearing cached dynamic obstacles",
      pointcloud_timeout_);
  }

  *min_x = std::min(*min_x, getOriginX());
  *min_y = std::min(*min_y, getOriginY());
  *max_x = std::max(*max_x, getOriginX() + getSizeInMetersX());
  *max_y = std::max(*max_y, getOriginY() + getSizeInMetersY());

  for (auto & point : cloud_temp.points) {
    const double wx = point.x;
    const double wy = point.y;
    unsigned int mx, my;
    if (worldToMap(wx, wy, mx, my)) {
      costmap_[getIndex(mx, my)] = nav2_costmap_2d::LETHAL_OBSTACLE;
      touch(wx, wy, min_x, min_y, max_x, max_y);
    }
  }
}

void FastLayer::reset()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud_.clear();
    cloud_received_ = false;
    last_cloud_receive_time_ns_ = 0;
  }
  resetMaps();
  addExtraBounds(
    getOriginX(), getOriginY(), getOriginX() + getSizeInMetersX(),
    getOriginY() + getSizeInMetersY());
  current_ = true;
}

void FastLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void FastLayer::deactivate() {}
void FastLayer::activate() {}

void FastLayer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ> incoming_cloud;
  pcl::fromROSMsg(*msg, incoming_cloud);

  // An empty snapshot is an explicit clearing observation. It does not need a
  // transform, which also lets a shutdown/failure publisher clear safely.
  if (incoming_cloud.empty()) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud_.clear();
    cloud_received_ = true;
    last_cloud_receive_time_ns_ = clock_->now().nanoseconds();
    return;
  }

  const std::string & frame_id = msg->header.frame_id;
  if (frame_id.empty()) {
    RCLCPP_WARN(logger_, "Ignoring non-empty point cloud with an empty frame_id");
    return;
  }

  if (frame_id != global_frame_) {
    try {
      const auto transform_stamped = tf_->lookupTransform(
        global_frame_, frame_id, rclcpp::Time(msg->header.stamp),
        tf2::durationFromSec(transform_timeout_));
      const Eigen::Matrix4f eigen_transform =
        tf2::transformToEigen(transform_stamped).matrix().cast<float>();
      pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
      pcl::transformPointCloud(incoming_cloud, transformed_cloud, eigen_transform);
      incoming_cloud.swap(transformed_cloud);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000, "Could not transform point cloud from %s to %s: %s",
        frame_id.c_str(), global_frame_.c_str(), ex.what());
      return;
    }
  }

  std::lock_guard<std::mutex> lock(cloud_mutex_);
  cloud_ = std::move(incoming_cloud);
  cloud_received_ = true;
  last_cloud_receive_time_ns_ = clock_->now().nanoseconds();
}

}  // namespace fast_layer
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fast_layer::FastLayer, nav2_costmap_2d::Layer)
