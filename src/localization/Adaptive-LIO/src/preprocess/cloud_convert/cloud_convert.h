/*
 * @Author: chengwei zhao
 * @LastEditors: cc
 * @Data:
 */
#pragma once


// 为 ROS2 添加 livox 驱动支持（如果启用）
#ifdef LIVOX_ROS2_DRIVER
#include <livox_ros_driver2/msg/custom_msg.hpp>
#elif defined(LIVOX_ROS1_DRIVER)
// 保留对 ROS1 livox 驱动的兼容（仅在启用该宏时）
#include <livox_ros_driver/CustomMsg.h>
#endif

#include <pcl_conversions/pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "tools/point_types.h"
#include "common/cloudMap.hpp"

namespace zjloc
{
    enum class LidarType
    {
        AVIA = 1,    // 大疆的固态雷达
        VELO32,      // Velodyne 32线
        OUST64,      // ouster 64线
        ROBOSENSE16, //  速腾16线
        PANDAR,
    };

    struct CVTParam
    {
        LidarType lidar_type;
        int point_filter_num;
        double blind;

        CVTParam()
        {
            lidar_type = LidarType::AVIA;
            point_filter_num = 1;
            blind = 0.1;
        }
        CVTParam(LidarType type, int filter_num, double blind_)
            : lidar_type(type), point_filter_num(filter_num), blind(blind_) {}
    };

    /**
     * 预处理雷达点云
     *
     * 将Velodyne, ouster, avia等数据转到FullCloud
     * 该类由MessageSync类持有，负责将收到的雷达消息与IMU同步并预处理后，再交给LO/LIO算法
     */
    class CloudConvert
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CloudConvert() {}
        ~CloudConvert() {}

        /**
         * 处理livox avia 点云
         * @param msg
         * @param pcl_out
         */
    // livox (AVIA) processing
#ifdef LIVOX_ROS2_DRIVER
    void Process(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg, std::vector<point3D> &pcl_out);
#elif defined(LIVOX_ROS1_DRIVER)
    void Process(const livox_ros_driver::CustomMsg::ConstPtr &msg, std::vector<point3D> &pcl_out);
#else
    // Fallback generic signature to avoid build breakage when no livox driver is present
    void Process(const void *msg, std::vector<point3D> &pcl_out);
#endif

        /**
         * 处理sensor_msgs::PointCloud2点云
         * @param msg
         * @param pcl_out
         */
    void Process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, std::vector<point3D> &pcl_out);
        // void Process(const sensor_msgs::PointCloud2::ConstPtr &msg, FullCloudPtr &pcl_out);

        /// 从YAML中读取参数
        void LoadFromYAML(const std::string &yaml);

        void initFromConfig(const CVTParam &param);

        //  返回激光的时间
        double getTimeSpan() { return timespan_; }

        LidarType lidar_type_ = LidarType::AVIA; // 雷达类型

    private:
#ifdef LIVOX_ROS2_DRIVER
    void AviaHandler(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg);
#elif defined(LIVOX_ROS1_DRIVER)
    void AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
#else
    void AviaHandler(const void *msg);
#endif
    void Oust64Handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
    void VelodyneHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
    void RobosenseHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
    void PandarHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

        std::vector<point3D> cloud_full_, cloud_out_; //  输出点云
        CVTParam param_;
        int point_filter_num_ = 1; // 跳点
        double blind = 0.1;

        double timespan_;
    };
} // namespace zjloc