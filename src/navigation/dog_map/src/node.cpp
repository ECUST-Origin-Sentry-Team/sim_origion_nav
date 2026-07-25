#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <message_filters/subscriber.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include "dog_map/occ_map.hpp"

using namespace analy_utils;
class DogMapNode : public rclcpp::Node
{
public:
  struct ROSCallback
  {
    rclcpp::CallbackGroup::SharedPtr cloud_me_cbk_group, update_cbk_group;
    int unfinished_frame_cnt{0};
    bool received_frame{false};
    Pose pc_pose;
    pcl::PointCloud<pcl::PointXYZ> pc;
    builtin_interfaces::msg::Time pc_stamp;
    rclcpp::TimerBase::SharedPtr update_timer;
    std::mutex update_lock;
  } rc_;
  struct CostPub
  {
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cost_pc_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      ground_pc_pub;
#ifdef FIX_MAP
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fix_pc_pub;
#endif
  } cost_pub_;
  struct RobotState
  {
    Vec3f p, v, a, j;
    double yaw;
    double rcv_time;
    bool rcv{false};
    Quatf q;
  } robot_state_;

  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
  {

    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::fromROSMsg(*cloud_msg, cloud_in);
    Eigen::Matrix4f tf_mat = Eigen::Matrix4f::Identity();
    geometry_msgs::msg::TransformStamped cloud_transform;
    geometry_msgs::msg::TransformStamped base_transform;
    try {
      const rclcpp::Time cloud_stamp(cloud_msg->header.stamp);
      // The MessageFilter only invokes this callback after both transform
      // chains are available at the cloud stamp. Exact-time lookups avoid
      // silently mixing a new cloud with a newer or older robot pose.
      cloud_transform = this->tf_buffer_->lookupTransform(
        target_frame_, cloud_msg->header.frame_id, cloud_stamp);
      base_transform = this->tf_buffer_->lookupTransform(
        target_frame_, base_frame_, cloud_stamp);

      const auto & tr = cloud_transform.transform.translation;
      const auto & rr = cloud_transform.transform.rotation;
      Eigen::Quaternionf q(
        static_cast<float>(rr.w), static_cast<float>(rr.x),
        static_cast<float>(rr.y), static_cast<float>(rr.z));
      q.normalize();

      tf_mat.block<3, 3>(0, 0) = q.toRotationMatrix();
      tf_mat(0, 3) = static_cast<float>(tr.x);
      tf_mat(1, 3) = static_cast<float>(tr.y);
      tf_mat(2, 3) = static_cast<float>(tr.z);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[DOG_MAP] Cannot transform cloud from %s to %s: %s",
        cloud_msg->header.frame_id.c_str(), target_frame_.c_str(),
        ex.what());
      return;
    }

    pcl::transformPointCloud(cloud_in, cloud_in, tf_mat);
    {
      std::lock_guard<std::mutex> lock(rc_.update_lock);
      rc_.pc = std::move(cloud_in);
      rc_.pc_stamp = cloud_msg->header.stamp;
      rc_.pc_pose.first[0] = base_transform.transform.translation.x;
      rc_.pc_pose.first[1] = base_transform.transform.translation.y;
      rc_.pc_pose.first[2] = base_transform.transform.translation.z;
      const auto & base_rotation = base_transform.transform.rotation;
      rc_.pc_pose.second = Quatf(
        base_rotation.w, base_rotation.x, base_rotation.y,
        base_rotation.z);
      rc_.pc_pose.second.normalize();
      rc_.unfinished_frame_cnt++;
      rc_.received_frame = true;
    }
  }

  void updateCallback()
  {
    auto start_time = std::chrono::steady_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pc =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    Pose temp_pose;
    builtin_interfaces::msg::Time temp_stamp;
    int pending_frame_count = 0;
    bool received_frame = false;
    {
      std::lock_guard<std::mutex> lock(rc_.update_lock);
      received_frame = rc_.received_frame;
      pending_frame_count = rc_.unfinished_frame_cnt;
      if (pending_frame_count > 0) {
        temp_pc->swap(rc_.pc);
        temp_pose = rc_.pc_pose;
        temp_stamp = rc_.pc_stamp;
        rc_.unfinished_frame_cnt = 0;
      }
    }

    if (!received_frame) {
      static double last_print_t = this->get_clock()->now().seconds();
      double cur_t = this->get_clock()->now().seconds();
      if ((cur_t - last_print_t > 1.0)) {
        RCLCPP_WARN(
          this->get_logger(),
          "[DOG_MAP] No transformable point cloud received; check "
          "the cloud topic and TF timestamps");
        last_print_t = cur_t;
      }
      return;
    }
    if (pending_frame_count == 0) {
      return;
    }

    if (pending_frame_count > 1) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[DOG_MAP] Replacing %d queued clouds with the newest frame; "
        "map update is not keeping real time",
        pending_frame_count - 1);
    }

    const size_t input_point_count = temp_pc->size();
    pcl::PointCloud<pcl::PointXYZ> ground_pc;
    occ_map_->Update(temp_pc, temp_pose, &ground_pc);
    const size_t raycast_point_count = temp_pc->size();

    sensor_msgs::msg::PointCloud2 ground_msg;
    pcl::toROSMsg(ground_pc, ground_msg);
    ground_msg.header.frame_id = target_frame_;
    ground_msg.header.stamp = temp_stamp;
    cost_pub_.ground_pc_pub->publish(std::move(ground_msg));

    pcl::PointCloud<pcl::PointXYZI> occ_pc;
    const auto publication_stats = occ_map_->PubRes(occ_pc);
    occ_pc.width = occ_pc.points.size();
    occ_pc.height = 1;
    occ_pc.is_dense = true;
    const size_t published_point_count = occ_pc.points.size();
    auto msg = sensor_msgs::msg::PointCloud2();
    pcl::toROSMsg(occ_pc, msg);
    msg.header.frame_id = target_frame_;
    msg.header.stamp = temp_stamp;
    // Empty clouds are meaningful: they tell downstream costmap layers to
    // clear their previous dynamic-obstacle observation.
    cost_pub_.cost_pc_pub->publish(std::move(msg));

#ifdef FIX_MAP
    static pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_fix =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    cloud_in_fix->clear();
    cloud_in_fix->swap(occ_pc);
    pcl::PointCloud<pcl::PointXYZ>::Ptr fix_pc =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    occ_map_->GetStaticFixMap()->AdjustAndFixByRacying2d(
      temp_pose.first[0], temp_pose.first[1], cloud_in_fix, fix_pc);
    /************* */
    if (!fix_pc->empty()) {
      auto fix_msg = sensor_msgs::msg::PointCloud2();
      pcl::toROSMsg(*fix_pc, fix_msg);
      fix_msg.header.frame_id = target_frame_;
      fix_msg.header.stamp = temp_stamp;
      cost_pub_.fix_pc_pub->publish(fix_msg);
    } else {
      std::cout << " -- [DOG INFO] No fix point to publish." << std::endl;
    }
#endif
    /************* */
    auto end_time = std::chrono::steady_clock::now();
    double time_consuming =
      std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time -
      start_time)
      .count();
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "[DOG_MAP] Map updated in %.0f ms: input=%zu, raycast=%zu, "
      "occupied=%zu, filter={tracked_voxels=%zu, candidates=%zu, "
      "accepted=%zu, span_reject=%zu, hole_reject=%zu}",
      time_consuming, input_point_count, raycast_point_count,
      published_point_count, publication_stats.tracked_voxels,
      publication_stats.candidate_columns,
      publication_stats.accepted_columns, publication_stats.span_reject,
      publication_stats.hole_reject);
  }

  DogMapNode()
  : Node("dog_map_node")
  {
    this->declare_parameter<std::string>("cloud_topic", cloud_topic);
    this->declare_parameter<std::string>("map_yaml_path", map_yaml_path);
    this->declare_parameter<std::string>("target_frame", target_frame_);
    this->declare_parameter<std::string>("base_frame", base_frame_);
    this->declare_parameter<double>(
      "transform_timeout",
      transform_timeout_);
    this->declare_parameter<int>(
      "transform_queue_size",
      transform_queue_size_);
    this->declare_parameter<double>("ground_z_in_base", ground_z_in_base);
    this->declare_parameter<double>(
      "min_obstacle_height",
      min_obstacle_height);
    this->declare_parameter<double>(
      "max_obstacle_height",
      max_obstacle_height);
    this->declare_parameter<double>(
      "min_obstacle_column_height",
      min_obstacle_column_height);
    this->declare_parameter<int8_t>("LOG_OCC_HIT_mid360", 41);
    this->declare_parameter<int8_t>("LOG_OCC_FREE", -5);
    this->declare_parameter<uint8_t>("THR_OCC", 20);
    this->declare_parameter<uint8_t>("MAX_LOG_MID360", 100);
    this->declare_parameter<uint8_t>("MIN_LOG", 10);
    int8_t LOG_OCC_HIT_MID360 = 40;       // 占据增量
    int8_t LOG_OCC_FREE = -3;      // 空闲减量
    uint8_t THR_OCC = 30;          // 判定为障碍物的阈值
    uint8_t MAX_LOG_MID360 = 100;
    uint8_t MIN_LOG = 5;
    this->declare_parameter<double>("half_map_size", 10.0);
    this->declare_parameter<double>("resolution_z", resolution_z);
    this->declare_parameter<int>("frame_save", frame_save);
    this->get_parameter("LOG_OCC_HIT_mid360", LOG_OCC_HIT_MID360);
    this->get_parameter("LOG_OCC_FREE", LOG_OCC_FREE);
    this->get_parameter("THR_OCC", THR_OCC);
    this->get_parameter("MAX_LOG_MID360", MAX_LOG_MID360);
    this->get_parameter("MIN_LOG", MIN_LOG);
    std::cout << " LOG_OCC_HIT " << (int)LOG_OCC_HIT_MID360 << std::endl;
    std::cout << " LOG_OCC_FREE " << (int)LOG_OCC_FREE << std::endl;
    std::cout << " THR_OCC " << (int)THR_OCC << std::endl;
    std::cout << " MAX_LOG_MID360 " << (int)MAX_LOG_MID360 << std::endl;
    std::cout << " MIN_LOG " << (int)MIN_LOG << std::endl;
    this->get_parameter("frame_save", frame_save);
    this->get_parameter("resolution_z", resolution_z);
    this->get_parameter("cloud_topic", cloud_topic);
    this->get_parameter("map_yaml_path", map_yaml_path);
    this->get_parameter("target_frame", target_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("transform_timeout", transform_timeout_);
    this->get_parameter("transform_queue_size", transform_queue_size_);
    this->get_parameter("ground_z_in_base", ground_z_in_base);
    this->get_parameter("min_obstacle_height", min_obstacle_height);
    this->get_parameter("max_obstacle_height", max_obstacle_height);
    this->get_parameter(
      "min_obstacle_column_height",
      min_obstacle_column_height);
    this->get_parameter("half_map_size", half_map_size);

    if (transform_timeout_ <= 0.0) {
      throw std::invalid_argument("transform_timeout must be > 0");
    }
    if (transform_queue_size_ < 1) {
      throw std::invalid_argument("transform_queue_size must be >= 1");
    }
    if (min_obstacle_column_height <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "min_obstacle_column_height <= 0 disables column-noise "
        "filtering; base_link-relative ground filtering remains active");
    }

    const rclcpp::QoS qos(
      rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile());
    const auto qos_sub = rclcpp::SensorDataQoS().keep_last(1);
    occ_map_ = std::make_unique<OccMap>(
      min_obstacle_column_height, min_obstacle_height,
      max_obstacle_height, ground_z_in_base, half_map_size, resolution_z,
      frame_save, map_yaml_path);
    occ_map_->SetLOGParams(
      LOG_OCC_HIT_MID360, LOG_OCC_FREE, THR_OCC, MAX_LOG_MID360,
      MIN_LOG);
    cost_pub_.cost_pc_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "rog_map/inf_occ", qos);
    cost_pub_.ground_pc_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "rog_map/ground", qos);
#ifdef FIX_MAP
    cost_pub_.fix_pc_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "rog_map/fix",
      qos);
#endif
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ =
      std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    {

      rc_.cloud_me_cbk_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions so;
      so.callback_group = rc_.cloud_me_cbk_group;
      cloud_sub_.subscribe(
        this, cloud_topic, qos_sub.get_rmw_qos_profile(), so);
      cloud_filter_ = std::make_unique<
        tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        cloud_sub_, *tf_buffer_, target_frame_,
        static_cast<uint32_t>(transform_queue_size_),
        this->get_node_logging_interface(),
        this->get_node_clock_interface(),
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(transform_timeout_)));
      std::vector<std::string> required_frames{target_frame_};
      if (base_frame_ != target_frame_) {
        required_frames.push_back(base_frame_);
      }
      cloud_filter_->setTargetFrames(required_frames);
      cloud_filter_->registerCallback(
        std::bind(
          &DogMapNode::cloudCallback, this,
          std::placeholders::_1));

      rc_.update_cbk_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
      rc_.update_timer = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&DogMapNode::updateCallback, this),
        rc_.update_cbk_group);
    }
  }
  ~DogMapNode() override = default;

private:
  std::unique_ptr<OccMap> occ_map_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>
  cloud_filter_;
  std::string cloud_topic{"/livox/lidar/pointcloud"};
  std::string target_frame_{"map"};
  std::string base_frame_{"base_link"};
  double transform_timeout_{0.2};
  int transform_queue_size_{10};

private:
  double half_map_size{10.0};
  double ground_z_in_base{0.0};
  double min_obstacle_height{0.03};
  double max_obstacle_height{0.23};
  double min_obstacle_column_height{0.02};
  double resolution_z{0.03};
  int frame_save{10};
  std::string map_yaml_path{""};
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DogMapNode>();
  // cloud 与地图更新已经放在不同 callback group；必须使用多线程 executor
  // 才能让 TF/点云接收不被一次较重的射线更新阻塞。
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
