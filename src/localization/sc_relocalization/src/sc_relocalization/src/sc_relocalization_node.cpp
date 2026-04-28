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
#include <pcl/common/transforms.h>

#include <yaml-cpp/yaml.h>
#include <QSharedMemory>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <thread>
#include <queue>
#include <mutex>
#include <atomic>
#include <array>
#include <cmath>
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

        initSharedMemory();
        tf_timer_ = create_wall_timer(100ms, std::bind(&SCRelocalizationNode::syncAndPublishStaticTF, this));

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
        if (xyr_shared_memory_.isAttached())
            xyr_shared_memory_.detach();
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
        if (node["rebuild_index_every_n"]) cfg.rebuild_index_every_n = node["rebuild_index_every_n"].as<int>();
        if (node["gicp_num_threads"]) cfg.gicp_num_threads = node["gicp_num_threads"].as<int>();
        if (node["gicp_num_neighbors"]) cfg.gicp_num_neighbors = node["gicp_num_neighbors"].as<int>();
        if (node["use_4dof"]) cfg.use_4dof = node["use_4dof"].as<bool>();
        if (node["gravity_align"]) cfg.gravity_align = node["gravity_align"].as<bool>();
        if (node["use_cart_context"]) cfg.use_cart_context = node["use_cart_context"].as<bool>();
        if (node["cart_weight"]) cfg.cart_weight = node["cart_weight"].as<double>();
        if (node["min_candidate_separation"]) cfg.min_candidate_separation = node["min_candidate_separation"].as<double>();

        scantext::SCParams sc_cfg;
        if (node["scantext"]) {
            auto sc_node = node["scantext"];
            if (sc_node["num_ring"]) sc_cfg.num_ring = sc_node["num_ring"].as<int>();
            if (sc_node["num_sector"]) sc_cfg.num_sector = sc_node["num_sector"].as<int>();
            if (sc_node["max_radius"]) sc_cfg.max_radius = sc_node["max_radius"].as<double>();
            if (sc_node["lidar_height"]) sc_cfg.lidar_height = sc_node["lidar_height"].as<double>();
            if (sc_node["use_scpp"]) sc_cfg.use_scpp = sc_node["use_scpp"].as<bool>();
            if (sc_node["scpp_search_ratio"]) sc_cfg.scpp_search_ratio = sc_node["scpp_search_ratio"].as<double>();
            if (sc_node["cart_x_unit"]) sc_cfg.cart_x_unit = sc_node["cart_x_unit"].as<double>();
            if (sc_node["cart_y_unit"]) sc_cfg.cart_y_unit = sc_node["cart_y_unit"].as<double>();
            if (sc_node["cart_x_max"]) sc_cfg.cart_x_max = sc_node["cart_x_max"].as<double>();
            if (sc_node["cart_y_max"]) sc_cfg.cart_y_max = sc_node["cart_y_max"].as<double>();
        }

        relo_core_->setConfig(cfg);
        relo_core_->setScanContextParams(sc_cfg);
        mapping_core_->setScanContextParams(sc_cfg);
        sc_extractor_ = scantext::ScanContext(sc_cfg);
    }

    /* ========================================================= */
    struct Task
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        Eigen::Isometry3d odom_pose;
        builtin_interfaces::msg::Time stamp;
    };

    void initSharedMemory()
    {
        xyr_shared_memory_.setKey("direction");
        if (!xyr_shared_memory_.attach())
        {
            if (!xyr_shared_memory_.create(sizeof(double) * 3))
            {
                if (xyr_shared_memory_.error() == QSharedMemory::AlreadyExists)
                {
                    xyr_shared_memory_.detach();
                    xyr_shared_memory_.attach();
                }
            }
        }

        if (xyr_shared_memory_.isAttached())
        {
            xyr_shared_memory_.lock();
            auto *data = static_cast<double *>(xyr_shared_memory_.data());
            if (data)
            {
                manual_xyr_[0] = data[0];
                manual_xyr_[1] = data[1];
                manual_xyr_[2] = data[2];
            }
            xyr_shared_memory_.unlock();
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Failed to attach shared memory 'direction': %s",
                        xyr_shared_memory_.errorString().toStdString().c_str());
        }
    }

    static double yawFromRotation(const Eigen::Matrix3d &R)
    {
        return std::atan2(R(1, 0), R(0, 0));
    }

    bool readManualXYR(double &x, double &y, double &yaw)
    {
        if (!xyr_shared_memory_.isAttached())
            return false;
        xyr_shared_memory_.lock();
        auto *data = static_cast<double *>(xyr_shared_memory_.data());
        if (!data)
        {
            xyr_shared_memory_.unlock();
            return false;
        }
        x = data[0];
        y = data[1];
        yaw = data[2];
        xyr_shared_memory_.unlock();
        return true;
    }

    void writeManualXYRFromTransform(const Eigen::Isometry3d &T)
    {
        if (!xyr_shared_memory_.isAttached())
            return;
        xyr_shared_memory_.lock();
        auto *data = static_cast<double *>(xyr_shared_memory_.data());
        if (data)
        {
            data[0] = T.translation().x();
            data[1] = T.translation().y();
            data[2] = yawFromRotation(T.rotation());
            manual_xyr_[0] = data[0];
            manual_xyr_[1] = data[1];
            manual_xyr_[2] = data[2];
        }
        xyr_shared_memory_.unlock();
    }

    /* ========================================================= */
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        Eigen::Isometry3d odom_pose;
        if (!lookupOdomPose(msg->header.stamp, odom_pose))
        {
            RCLCPP_WARN(get_logger(), "Failed to lookup odom pose");
            return;
        }

        std::string odom, base;
        get_parameter("odom_frame", odom);
        get_parameter("base_frame", base);

        auto cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::fromROSMsg(*msg, *cloud_in);

        if (cloud_in->empty())
            return;

        // /livox/scan produced by Adaptive-LIO is in odom/world frame.
        // Convert to local base frame (aft_mapped) before SC + registration.
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        const std::string &frame_id = msg->header.frame_id;
        if (frame_id.empty() || frame_id == odom)
        {
            // odom_pose is T_odom_base (odom -> base). We need base-frame cloud.
            const Eigen::Isometry3d T_base_odom = odom_pose.inverse();
            pcl::transformPointCloud(*cloud_in, *cloud, T_base_odom.matrix().cast<float>());
        }
        else if (frame_id == base)
        {
            cloud = cloud_in;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Unexpected cloud frame_id='%s' (expected '%s' or '%s'), dropping frame",
                        frame_id.c_str(), odom.c_str(), base.c_str());
            return;
        }

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
                rclcpp::Time(stamp));

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
                auto T = relo_core_->getMapOdomTransform();
                writeManualXYRFromTransform(T);
                publishMapOdomStaticTF(task.stamp, T);
                RCLCPP_INFO(get_logger(), "Relocalization SUCCESS");
            }
        }
    }

    /* ========================================================= */
    void publishMapOdomStaticTF(const builtin_interfaces::msg::Time &stamp,
                                const Eigen::Isometry3d &T)
    {
        std::string odom;
        get_parameter("odom_frame", odom);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "map";
        tf.child_frame_id = odom;

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

    void syncAndPublishStaticTF()
    {
        if (!relo_core_ || !relo_core_->isLocalized())
            return;

        double x = manual_xyr_[0];
        double y = manual_xyr_[1];
        double yaw = manual_xyr_[2];
        if (readManualXYR(x, y, yaw))
        {
            manual_xyr_[0] = x;
            manual_xyr_[1] = y;
            manual_xyr_[2] = yaw;
        }

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.translation() = Eigen::Vector3d(manual_xyr_[0], manual_xyr_[1], 0.0);
        T.linear() = Eigen::AngleAxisd(manual_xyr_[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();

        const bool changed = !has_last_published_xyr_ ||
                             std::abs(manual_xyr_[0] - last_published_xyr_[0]) > 1e-6 ||
                             std::abs(manual_xyr_[1] - last_published_xyr_[1]) > 1e-6 ||
                             std::abs(manual_xyr_[2] - last_published_xyr_[2]) > 1e-6;
        if (!changed)
            return;

        const int64_t ns = now().nanoseconds();
        builtin_interfaces::msg::Time stamp;
        stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
        stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);

        publishMapOdomStaticTF(stamp, T);
        last_published_xyr_ = manual_xyr_;
        has_last_published_xyr_ = true;
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

    rclcpp::TimerBase::SharedPtr tf_timer_;

    QSharedMemory xyr_shared_memory_;
    std::array<double, 3> manual_xyr_{0.0, 0.0, 0.0};
    std::array<double, 3> last_published_xyr_{0.0, 0.0, 0.0};
    bool has_last_published_xyr_ = false;

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
