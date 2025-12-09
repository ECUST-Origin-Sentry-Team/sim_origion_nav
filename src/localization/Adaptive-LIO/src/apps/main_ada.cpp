// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <functional>
#include <memory>

// ros2 lib
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/transform_datatypes.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <random>

#include "common/utility.h"
#include "preprocess/cloud_convert/cloud_convert2.h"
#include "lio/lidarodom.h"

// 由于ROS2中没有完全对应的CustomMsg，我们暂时注释掉这部分
// #include "livox_ros_driver/msg/custom_msg.h"

nav_msgs::msg::Path laserOdoPath;

zjloc::lidarodom_m *lio;
zjloc::CloudConvert2 *convert;
double gnorm = 1.0;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_repub;

DEFINE_string(config_yaml, "./config/mapping.yaml", "配置文件");
#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "Log/" + name))

// void livox_pcl_cbk(const livox_ros_driver::msg::CustomMsg::SharedPtr msg)
// {
//     std::vector<std::vector<point3D>> cloud_vec;
//     std::vector<double> t_out;
//     zjloc::common::Timer::Evaluate([&]()
//                                    { convert->Process(msg, cloud_vec, t_out); },
//                                    "laser convert");

//     for (int i = 0; i < cloud_vec.size(); i++)
//     {
//         auto &cloud_out = cloud_vec[i];
//         double sample_size = lio->getIndex() < 20 ? 0.01 : 0.01;
//         // double sample_size = 0.01;
//         std::mt19937_64 g;
//         zjloc::common::Timer::Evaluate([&]()
//                                        { std::shuffle(cloud_out.begin(), cloud_out.end(), g);
//             subSampleFrame(cloud_out, sample_size);
//             std::shuffle(cloud_out.begin(), cloud_out.end(), g); },
//                                        "laser ds");

//         lio->pushData(cloud_out, std::make_pair(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9 + t_out[i] - t_out[0], t_out[0]));
//     }
// }

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // sensor_msgs::msg::PointCloud2::SharedPtr cloud(new sensor_msgs::msg::PointCloud2(*msg));

    std::vector<std::vector<point3D>> cloud_vec;
    std::vector<double> t_out;
    zjloc::common::Timer::Evaluate([&]()
                                   { convert->Process(msg, cloud_vec, t_out); },
                                   "laser convert");

    for (int i = 0; i < cloud_vec.size(); i++)
    {
        auto &cloud_out = cloud_vec[i];
        double sample_size = lio->getIndex() < 30 ? 0.02 : 0.1;
        // double sample_size = 0.05;
        zjloc::common::Timer::Evaluate([&]() { // boost::mt19937_64 g;
            std::mt19937_64 g;
            std::shuffle(cloud_out.begin(), cloud_out.end(), g);
            sub_sample_frame(cloud_out, sample_size);
            std::shuffle(cloud_out.begin(), cloud_out.end(), g);
        },
                                       "laser ds");

        // 在ROS2中使用的是nanoseconds().count()来获取时间戳
        lio->pushData(cloud_out, std::make_pair(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9 + t_out[i] - t_out[0], t_out[0]));
    }
}

void imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    sensor_msgs::msg::Imu::SharedPtr msg_temp(new sensor_msgs::msg::Imu(*msg));
    IMUPtr imu = std::make_shared<zjloc::IMU>(
        msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
        Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
        Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * gnorm);
    lio->pushData(imu);
    {
        msg_temp->linear_acceleration.x = msg_temp->linear_acceleration.x * gnorm;
        msg_temp->linear_acceleration.y = msg_temp->linear_acceleration.y * gnorm;
        msg_temp->linear_acceleration.z = msg_temp->linear_acceleration.z * gnorm;
        imu_repub->publish(*msg_temp);
    }
}

int main(int argc, char **argv)
{
    // 处理命令行参数，将gflags参数与ROS2参数分离
    // Debug: print initial argv for diagnosing flag parsing issues
    std::cout << "[adaptive_lio] argv before init:";
    for (int i = 0; i < argc; ++i) {
        std::cout << " '" << (argv[i] ? argv[i] : std::string("(null)")) << "'";
    }
    std::cout << std::endl;
    
    // Initialize logging and ROS2. Important: call rclcpp::init before
    // parsing Google gflags so ROS2's --ros-args (and -r) are handled by
    // rclcpp and not consumed (or rejected) by gflags.
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    rclcpp::init(argc, argv);

    // Build a filtered argv for gflags so ROS2 launch args are not seen by gflags.
    // Keep program name.
    std::vector<char *> gflags_argv;
    gflags_argv.push_back(argv[0]);
    for (int i = 1; i < argc; ++i) {
        std::string a(argv[i]);
        // Skip ROS2-specific markers and remappings
        if (a == "--ros-args") {
            continue;
        }
        if (a.rfind("--ros", 0) == 0) {
            continue;
        }
        if (a == "-r") {
            // remap flag, skip it (its argument is the remapping spec which
            // usually starts with __node:= or topic remap; skip that token too)
            continue;
        }
        if (a.rfind("__", 0) == 0) {
            // __node:=... or other remap specs
            continue;
        }
        if (a == "--params-file") {
            // skip params-file and its value (if present)
            ++i;
            continue;
        }
        // otherwise keep the arg for gflags
        gflags_argv.push_back(argv[i]);
    }

    int gflags_argc = static_cast<int>(gflags_argv.size());
    char **gflags_argv_ptr = gflags_argv.data();
    google::ParseCommandLineFlags(&gflags_argc, &gflags_argv_ptr, true);
    auto node = rclcpp::Node::make_shared("adaptive_lio_node");

    std::string config_file = std::string(ROOT_DIR) + "config/mapping_m.yaml";
    std::cout << ANSI_COLOR_GREEN << "config_file:" << config_file << ANSI_COLOR_RESET << std::endl;

    lio = new zjloc::lidarodom_m();
    if (!lio->init(config_file))
    {
        return -1;
    }

    auto pub_scan = node->create_publisher<sensor_msgs::msg::PointCloud2>("scan", 10);
    auto cloud_pub_func = std::function<bool(std::string & topic_name, zjloc::CloudPtr & cloud, double time)>(
        [&](std::string &topic_name, zjloc::CloudPtr &cloud, double time)
        {
            sensor_msgs::msg::PointCloud2::SharedPtr cloud_ptr_output(new sensor_msgs::msg::PointCloud2());
            pcl::toROSMsg(*cloud, *cloud_ptr_output);

            cloud_ptr_output->header.stamp = node->now(); // 在ROS2中使用node->now()获取当前时间
            cloud_ptr_output->header.frame_id = "map";
            if (topic_name == "laser")
                pub_scan->publish(*cloud_ptr_output);
            else
                ; // publisher_.publish(*cloud_ptr_output);
            return true;
        }

    );

    auto pubLaserOdometry = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 100);
    auto pubLaserOdometryPath = node->create_publisher<nav_msgs::msg::Path>("/odometry_path", 5);

    // 创建tf广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> br = std::unique_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster(node));

    auto pose_pub_func = std::function<bool(std::string & topic_name, SE3 & pose, double stamp)>(
        [&](std::string &topic_name, SE3 &pose, double stamp)
        {
            geometry_msgs::msg::TransformStamped transform;
            Eigen::Quaterniond q_current(pose.so3().matrix());
            
            transform.header.stamp = node->now();
            transform.transform.translation.x = pose.translation().x();
            transform.transform.translation.y = pose.translation().y();
            transform.transform.translation.z = pose.translation().z();
            transform.transform.rotation.x = q_current.x();
            transform.transform.rotation.y = q_current.y();
            transform.transform.rotation.z = q_current.z();
            transform.transform.rotation.w = q_current.w();
            
            if (topic_name == "laser")
            {
                transform.header.frame_id = "map";
                transform.child_frame_id = "base_link";
                br->sendTransform(transform);

                // publish odometry
                nav_msgs::msg::Odometry laserOdometry;
                laserOdometry.header.frame_id = "map";
                laserOdometry.child_frame_id = "base_link";
                laserOdometry.header.stamp = node->now();

                laserOdometry.pose.pose.orientation.x = q_current.x();
                laserOdometry.pose.pose.orientation.y = q_current.y();
                laserOdometry.pose.pose.orientation.z = q_current.z();
                laserOdometry.pose.pose.orientation.w = q_current.w();
                laserOdometry.pose.pose.position.x = pose.translation().x();
                laserOdometry.pose.pose.position.y = pose.translation().y();
                laserOdometry.pose.pose.position.z = pose.translation().z();
                pubLaserOdometry->publish(laserOdometry);

                //  publish path
                geometry_msgs::msg::PoseStamped laserPose;
                laserPose.header = laserOdometry.header;
                laserPose.pose = laserOdometry.pose.pose;
                laserOdoPath.header.stamp = laserOdometry.header.stamp;
                laserOdoPath.poses.push_back(laserPose);
                laserOdoPath.header.frame_id = "map";
                pubLaserOdometryPath->publish(laserOdoPath);
            }
            else if (topic_name == "world")
            {
                transform.header.frame_id = "world";
                transform.child_frame_id = "map";
                br->sendTransform(transform);
            }

            return true;
        }

    );

    auto vel_pub = node->create_publisher<std_msgs::msg::Float32>("/velocity", 1);
    auto dist_pub = node->create_publisher<std_msgs::msg::Float32>("/move_dist", 1);
    imu_repub = node->create_publisher<sensor_msgs::msg::Imu>("/repub_imu", 1);

    auto data_pub_func = std::function<bool(std::string & topic_name, double time1, double time2)>(
        [&](std::string &topic_name, double time1, double time2)
        {
            std_msgs::msg::Float32 time_rviz;

            time_rviz.data = time1;
            if (topic_name == "velocity")
                vel_pub->publish(time_rviz);
            else
                dist_pub->publish(time_rviz);

            return true;
        }

    );

    lio->setFunc(cloud_pub_func);
    lio->setFunc(pose_pub_func);
    lio->setFunc(data_pub_func);

    convert = new zjloc::CloudConvert2;
    convert->LoadFromYAML(config_file);

    lio->setCloudConvert(convert);
    std::cout << ANSI_COLOR_GREEN_BOLD << "init successful" << ANSI_COLOR_RESET << std::endl;

    auto yaml = YAML::LoadFile(config_file);
    std::string laser_topic = yaml["common"]["lid_topic"].as<std::string>();
    std::string imu_topic = yaml["common"]["imu_topic"].as<std::string>();
    gnorm = yaml["common"]["gnorm"].as<double>();

    // 创建订阅者
    // auto subLaserCloud = convert->lidar_type_ == zjloc::CloudConvert2::LidarType::AVIA
    //                         ? node->create_subscription<livox_ros_driver::msg::CustomMsg>(laser_topic, 100, livox_pcl_cbk)
    //                         : 
    auto subLaserCloud = node->create_subscription<sensor_msgs::msg::PointCloud2>(laser_topic, 100, standard_pcl_cbk);

    auto sub_imu_ori = node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 500, imuHandler);

    std::thread measurement_process(&zjloc::lidarodom_m::run, lio);

    rclcpp::spin(node);
    rclcpp::shutdown();

    zjloc::common::Timer::PrintAll();
    zjloc::common::Timer::DumpIntoFile(DEBUG_FILE_DIR("log_time.txt"));

    std::cout << ANSI_COLOR_GREEN_BOLD << " out done. " << ANSI_COLOR_RESET << std::endl;

    sleep(3);
    return 0;
}