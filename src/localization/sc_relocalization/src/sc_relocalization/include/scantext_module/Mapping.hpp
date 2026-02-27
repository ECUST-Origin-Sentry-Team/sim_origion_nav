#pragma once

#include <vector>
#include <string>
#include <mutex>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

#include "scantext_module/ScanContext.hpp"

namespace scantext
{

    /**
     * @brief KeyFrame structure for the map
     */
    struct KeyFrame
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        int index;
        double time;
        Eigen::Isometry3d pose;
        ScanContext::PointCloudType::Ptr cloud;
        ScanContext::SCDescriptor descriptor;
        ScanContext::RingKey ring_key;
    };

    /**
     * @brief Mapping Core for managing KeyFrames and Map saving
     */
    class MappingCore
    {
    public:
        struct Config
        {
            bool enable_mapping = false;
            double keyframe_dist_thresh = 1.0;  // Meters
            double keyframe_angle_thresh = 0.2; // Radians
            std::string map_save_path = "/tmp/map.pcd";
            double auto_save_interval = 0.0; // Meters, 0 to disable
        };

        MappingCore();
        ~MappingCore() = default;

        void setConfig(const Config &config);
        Config getConfig() const { return config_; }

        /**
         * @brief Add a new frame to the map
         *
         * Decides if the frame should be a keyframe based on motion thresholds.
         *
         * @param cloud Point cloud (local frame)
         * @param pose Pose (world frame)
         * @param time Timestamp
         * @return true if a keyframe was added
         */
        bool addFrame(const ScanContext::PointCloudType::Ptr &cloud, const Eigen::Isometry3d &pose, double time);

        /**
         * @brief Save the global map to a PCD file
         * @param path File path
         * @return true Success
         */
        bool saveMap(const std::string &path);

        /**
         * @brief Save the database (poses + individual clouds + descriptors)
         * @param directory Directory to save to
         * @return true Success
         */
        bool saveDatabase(const std::string &directory);

        /**
         * @brief Load the database
         * @param directory Directory to load from
         * @return true Success
         */
        bool loadDatabase(const std::string &directory);

        // Get all keyframes (thread safe)
        std::vector<std::shared_ptr<KeyFrame>> getKeyFrames();

        bool addFrameWithSC(const ScanContext::PointCloudType::Ptr &cloud,
                            const Eigen::Isometry3d &pose,
                            double time,
                            const ScanContext::SCDescriptor &sc,
                            const ScanContext::RingKey &rk,
                            std::shared_ptr<KeyFrame> *out_kf = nullptr);

    private:
        Config config_;
        ScanContext scan_context_;

        std::vector<std::shared_ptr<KeyFrame>> keyframes_;
        std::mutex map_mutex_;

        Eigen::Isometry3d last_keyframe_pose_ = Eigen::Isometry3d::Identity();
        double accumulated_dist_since_save_ = 0.0;

        // Helper to check if we should create a keyframe
        bool isKeyFrame(const Eigen::Isometry3d &pose);
    };

} // namespace scantext
