#include "scantext_module/Mapping.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace scantext
{

    MappingCore::MappingCore()
    {
        last_keyframe_pose_ = Eigen::Isometry3d::Identity();
    }

    void MappingCore::setConfig(const Config &config)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        config_ = config;
    }

    bool MappingCore::addFrame(const ScanContext::PointCloudType::Ptr &cloud, const Eigen::Isometry3d &pose, double time)
    {
        if (!config_.enable_mapping)
            return false;

        if (!isKeyFrame(pose))
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(map_mutex_);

        auto kf = std::make_shared<KeyFrame>();
        kf->index = keyframes_.size();
        kf->time = time;
        kf->pose = pose;
        kf->cloud = cloud; // In a real system, might want to downsample or deep copy if the ptr is reused

        // Extract ScanContext
        kf->descriptor = scan_context_.makeScanContext(*cloud);
        kf->ring_key = scan_context_.makeRingKey(kf->descriptor);

        keyframes_.push_back(kf);

        // Update auto-save logic
        if (keyframes_.size() > 1)
        {
            double dist = (pose.translation() - last_keyframe_pose_.translation()).norm();
            accumulated_dist_since_save_ += dist;
            if (config_.auto_save_interval > 0 && accumulated_dist_since_save_ >= config_.auto_save_interval)
            {
                saveMap(config_.map_save_path); // Note: This might block, better to run in thread
                accumulated_dist_since_save_ = 0.0;
            }
        }

        last_keyframe_pose_ = pose;
        return true;
    }

    bool MappingCore::isKeyFrame(const Eigen::Isometry3d &pose)
    {
        if (keyframes_.empty())
            return true;

        Eigen::Isometry3d delta = last_keyframe_pose_.inverse() * pose;
        double dist = delta.translation().norm();
        double angle = Eigen::AngleAxisd(delta.rotation()).angle();

        return dist >= config_.keyframe_dist_thresh || angle >= config_.keyframe_angle_thresh;
    }

    bool MappingCore::saveMap(const std::string &path)
    {
        // std::lock_guard<std::mutex> lock(map_mutex_); // Be careful with long holding locks
        // We'll copy pointers quickly then process
        std::vector<std::shared_ptr<KeyFrame>> kfs_copy;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            kfs_copy = keyframes_;
        }

        if (kfs_copy.empty())
            return false;

        ScanContext::PointCloudType::Ptr global_map(new ScanContext::PointCloudType());

        for (const auto &kf : kfs_copy)
        {
            ScanContext::PointCloudType::Ptr transformed_cloud(new ScanContext::PointCloudType());
            pcl::transformPointCloud(*kf->cloud, *transformed_cloud, kf->pose.matrix());
            *global_map += *transformed_cloud;
        }

        // Voxel grid filter could be applied here to reduce size

        pcl::io::savePCDFileBinary(path, *global_map);
        std::cout << "[Mapping] Saved map with " << global_map->size() << " points to " << path << std::endl;
        return true;
    }

    std::vector<std::shared_ptr<KeyFrame>> MappingCore::getKeyFrames()
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        return keyframes_;
    }

    bool MappingCore::saveDatabase(const std::string &directory)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (keyframes_.empty())
            return false;

        if (!std::filesystem::exists(directory))
        {
            std::filesystem::create_directories(directory);
        }

        std::ofstream ofs(directory + "/poses.txt");
        if (!ofs.is_open())
            return false;

        for (const auto &kf : keyframes_)
        {
            // Save Pose
            Eigen::Quaterniond q(kf->pose.rotation());
            Eigen::Vector3d t = kf->pose.translation();
            ofs << kf->index << " " << std::fixed << kf->time << " "
                << t.x() << " " << t.y() << " " << t.z() << " "
                << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";

            // Save Cloud
            std::string pcd_path = directory + "/cloud_" + std::to_string(kf->index) + ".pcd";
            pcl::io::savePCDFileBinary(pcd_path, *kf->cloud);
        }

        ofs.close();
        std::cout << "[Mapping] Saved " << keyframes_.size() << " keyframes to " << directory << std::endl;
        return true;
    }

    bool MappingCore::loadDatabase(const std::string &directory)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        keyframes_.clear();

        std::ifstream ifs(directory + "/poses.txt");
        if (!ifs.is_open())
            return false;

        std::string line;
        while (std::getline(ifs, line))
        {
            std::stringstream ss(line);
            auto kf = std::make_shared<KeyFrame>();
            double tx, ty, tz, qx, qy, qz, qw;
            ss >> kf->index >> kf->time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

            kf->pose = Eigen::Translation3d(tx, ty, tz) * Eigen::Quaterniond(qw, qx, qy, qz);

            kf->cloud.reset(new ScanContext::PointCloudType());
            std::string pcd_path = directory + "/cloud_" + std::to_string(kf->index) + ".pcd";
            if (pcl::io::loadPCDFile(pcd_path, *kf->cloud) == -1)
            {
                std::cerr << "[Mapping] Failed to load " << pcd_path << std::endl;
                continue;
            }

            // Recompute SC
            kf->descriptor = scan_context_.makeScanContext(*kf->cloud);
            kf->ring_key = scan_context_.makeRingKey(kf->descriptor);

            keyframes_.push_back(kf);
        }

        if (!keyframes_.empty())
        {
            last_keyframe_pose_ = keyframes_.back()->pose;
        }

        std::cout << "[Mapping] Loaded " << keyframes_.size() << " keyframes from " << directory << std::endl;
        return true;
    }

    // Mapping.cpp

    bool MappingCore::addFrameWithSC(const ScanContext::PointCloudType::Ptr &cloud,
                                     const Eigen::Isometry3d &pose,
                                     double time,
                                     const ScanContext::SCDescriptor &sc,
                                     const ScanContext::RingKey &rk,
                                     std::shared_ptr<KeyFrame> *out_kf)
    {
        if (!config_.enable_mapping)
            return false;

        if (!isKeyFrame(pose))
            return false;

        std::lock_guard<std::mutex> lock(map_mutex_);

        auto kf = std::make_shared<KeyFrame>();
        kf->index = static_cast<int>(keyframes_.size());
        kf->time = time;
        kf->pose = pose;

        // 这里 cloud 可以是 downsample 后的小云（推荐：省内存）
        kf->cloud = cloud;

        // 直接使用外部算好的 SC
        kf->descriptor = sc;
        kf->ring_key = rk;

        keyframes_.push_back(kf);

        if (out_kf)
            *out_kf = kf;

        // auto-save 逻辑保持不变
        if (keyframes_.size() > 1)
        {
            double dist = (pose.translation() - last_keyframe_pose_.translation()).norm();
            accumulated_dist_since_save_ += dist;
            if (config_.auto_save_interval > 0 && accumulated_dist_since_save_ >= config_.auto_save_interval)
            {
                saveMap(config_.map_save_path);
                accumulated_dist_since_save_ = 0.0;
            }
        }

        last_keyframe_pose_ = pose;
        return true;
    }

} // namespace scantext
