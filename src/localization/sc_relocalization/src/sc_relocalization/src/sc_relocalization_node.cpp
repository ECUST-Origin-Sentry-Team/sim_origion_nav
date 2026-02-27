#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <thread>
#include <queue>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include "scantext_module/Relocalization.hpp"
#include "scantext_module/Mapping.hpp"
#include "common/eigen_types.h"

using namespace std::chrono_literals;

class SCRelocalizationNode : public rclcpp::Node
{
public:
    SCRelocalizationNode()
        : Node("sc_relocalization_node")
    {
        declareParameters();

        /* ---------------- Core Init FIRST ---------------- */
        relo_core_ = std::make_shared<scantext::RelocalizationCore>();
        mapping_core_ = std::make_shared<scantext::MappingCore>();

        /* ---------------- Load Config ---------------- */
        std::string config_path;
        get_parameter("config_path", config_path);
        if (!config_path.empty())
        {
            loadConfig(config_path);
        }

        /* ---------------- Load Map ---------------- */
        std::string map_path;
        get_parameter("map_path", map_path);

        if (!mapping_core_->loadDatabase(map_path))
        {
            RCLCPP_FATAL(get_logger(), "Failed to load map: %s", map_path.c_str());
            throw std::runtime_error("Map load failed");
        }
        relo_core_->setMap(mapping_core_->getKeyFrames());

        /* ---------------- TF ---------------- */
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        /* ---------------- Subscribers ---------------- */
        sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/scan", rclcpp::SensorDataQoS(),
            std::bind(&SCRelocalizationNode::cloudCallback, this, std::placeholders::_1));

        sub_init_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&SCRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

        /* ---------------- Service ---------------- */
        srv_state_ = create_service<std_srvs::srv::Trigger>(
            "relocalization_state",
            std::bind(&SCRelocalizationNode::stateService, this,
                      std::placeholders::_1, std::placeholders::_2));

        /* ---------------- Worker Thread ---------------- */
        worker_thread_ = std::thread(&SCRelocalizationNode::processingLoop, this);

        RCLCPP_INFO(get_logger(), "SC Relocalization Node started");
    }

    ~SCRelocalizationNode()
    {
        stop_thread_.store(true);
        cv_.notify_all();
        if (worker_thread_.joinable())
            worker_thread_.join();
    }

private:
    /* ========================================================= */
    /* Parameters */
    void declareParameters()
    {
        declare_parameter<std::string>("config_path", "");
        declare_parameter<std::string>("map_path", "");
        declare_parameter<std::string>("odom_frame", "odom");
        declare_parameter<std::string>("base_frame", "aft_mapped");
    }

    /* ========================================================= */
    void loadConfig(const std::string &path)
    {
        YAML::Node yaml = YAML::LoadFile(path);
        auto node = yaml["relocalization_module"];

        scantext::RelocalizationCore::Config cfg;
        cfg.ringkey_top_k = node["ringkey_top_k"].as<int>();
        cfg.sc_top_k = node["sc_top_k"].as<int>();
        cfg.icp_top_k = node["icp_top_k"].as<int>();
        cfg.sc_dist_thresh = node["sc_dist_thresh"].as<double>();
        cfg.icp_fitness_thresh = node["icp_fitness_thresh"].as<double>();
        cfg.icp_max_corr_dist = node["icp_max_corr_dist"].as<double>();
        cfg.icp_max_iter = node["icp_max_iter"].as<int>();
        cfg.local_submap_kf_num = node["local_submap_kf_num"].as<int>();
        cfg.local_map_voxel_leaf = node["local_map_voxel_leaf"].as<double>();
        cfg.query_voxel_leaf = node["query_voxel_leaf"].as<double>();

        relo_core_->setConfig(cfg);
    }

    /* ========================================================= */
    struct Task
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        Eigen::Isometry3d odom_pose;
        builtin_interfaces::msg::Time stamp;
    };

    /* ========================================================= */
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        Eigen::Isometry3d odom_pose;
        if (!lookupOdomPose(msg->header.stamp, odom_pose))
        {
            RCLCPP_WARN(get_logger(), "Failed to lookup odom pose");
            return;
        }


        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty())
            return;

        Task task;
        task.cloud = cloud;
        task.odom_pose = odom_pose;
        task.stamp = msg->header.stamp;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (queue_.size() < 3)
            {
                queue_.push(std::move(task));
                cv_.notify_one();
            }
        }
    }
    /* ========================================================= */
    bool lookupOdomPose(const builtin_interfaces::msg::Time &stamp,
                        Eigen::Isometry3d &pose)
    {
        std::string odom, base;
        get_parameter("odom_frame", odom);
        get_parameter("base_frame", base);

        try
        {
            // 强制等待 TF（非常关键）
            if (!tf_buffer_->canTransform(
                    odom, base,
                    rclcpp::Time(stamp),
                    tf2::durationFromSec(0.2)))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "TF NOT READY: %s -> %s at time %.6f",
                    odom.c_str(), base.c_str(),
                    rclcpp::Time(stamp).seconds());
                return false;
            }

            auto tf = tf_buffer_->lookupTransform(
                odom, base,
                tf2::TimePointZero);

            pose.setIdentity();
            pose.translation() << tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z;

            pose.linear() =
                Eigen::Quaterniond(
                    tf.transform.rotation.w,
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z)
                    .toRotationMatrix();

            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(
                get_logger(),
                "TF EXCEPTION [%s -> %s]: %s",
                odom.c_str(), base.c_str(), ex.what());
            return false;
        }
    }

    /* ========================================================= */
    void processingLoop()
    {
        while (!stop_thread_.load())
        {
            Task task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [&]
                         { return stop_thread_.load() || !queue_.empty(); });
                if (stop_thread_.load())
                    return;

                task = std::move(queue_.front());
                queue_.pop();
            }

            if (relo_core_->isLocalized())
                continue;

            auto sc = sc_extractor_.makeScanContext(*task.cloud);
            auto rk = sc_extractor_.makeRingKey(sc);

            if (rk.size() != sc_extractor_.getParams().num_ring)
            {
                RCLCPP_WARN(get_logger(),
                            "RingKey dim mismatch, skip frame");
                continue;
            }

            if (relo_core_->updateFromSC(
                    sc,
                    rk,
                    task.cloud,
                    task.odom_pose,
                    rclcpp::Time(task.stamp).seconds()))
            {
                publishMapOdomTF(task.stamp);
                RCLCPP_INFO(get_logger(), "Relocalization SUCCESS");
            }
        }
    }

    /* ========================================================= */
    void publishMapOdomTF(const builtin_interfaces::msg::Time &stamp)
    {
        auto T = relo_core_->getMapOdomTransform();

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "map";
        tf.child_frame_id = "odom";

        tf.transform.translation.x = T.translation().x();
        tf.transform.translation.y = T.translation().y();
        tf.transform.translation.z = T.translation().z();

        Eigen::Quaterniond q(T.rotation());
        tf.transform.rotation.w = q.w();
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();

        static_tf_broadcaster_->sendTransform(tf);
    }

    /* ========================================================= */
    void initialPoseCallback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() << msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z;
        pose.linear() =
            Eigen::Quaterniond(msg->pose.pose.orientation.w,
                               msg->pose.pose.orientation.x,
                               msg->pose.pose.orientation.y,
                               msg->pose.pose.orientation.z)
                .toRotationMatrix();

        Mat6d cov = Mat6d::Identity();
        for (int i = 0; i < 36; ++i)
            cov(i / 6, i % 6) = msg->pose.covariance[i];

        relo_core_->setInitialPose(pose, cov);
        relo_core_->resetRelocalization();
    }

    /* ========================================================= */
    void stateService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        res->success = relo_core_->isLocalized();
        res->message = res->success ? "Localized" : "Not localized";
    }

private:
    std::shared_ptr<scantext::RelocalizationCore> relo_core_;
    std::shared_ptr<scantext::MappingCore> mapping_core_;

    scantext::ScanContext sc_extractor_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_pose_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_state_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    std::thread worker_thread_;
    std::queue<Task> queue_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic<bool> stop_thread_{false};
};

/* ========================================================= */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SCRelocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
