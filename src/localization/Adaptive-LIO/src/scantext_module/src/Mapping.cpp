#include "scantext_module/Mapping.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>

namespace scantext
{

    MappingCore::MappingCore()
    {
        last_keyframe_pose_ = Eigen::Isometry3d::Identity();
        global_map_ = pcl::make_shared<ScanContext::PointCloudType>();
    }
    void MappingCore::initDatabaseIfNeeded()
    {
        if (db_initialized_)
            return;

        db_dir_ = config_.map_save_path + "_db";

        if (std::filesystem::exists(db_dir_))
        {
            std::filesystem::remove_all(db_dir_);
        }

        std::filesystem::create_directories(db_dir_);

        pose_ofs_.open(db_dir_ + "/poses.txt",
                       std::ios::out | std::ios::trunc);

        if (!pose_ofs_.is_open())
        {
            throw std::runtime_error("Failed to open poses.txt in " + db_dir_);
        }

        frame_index_ = 0;
        accumulated_dist_since_save_ = 0.0;
        db_initialized_ = true;
    }

    void MappingCore::setConfig(const Config &config)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        config_ = config;
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
        ScanContext::PointCloudType::Ptr map_copy;

        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if (!global_map_ || global_map_->empty())
                return false;

            map_copy = global_map_;
        }

        pcl::io::savePCDFileBinaryCompressed(path, *map_copy);

        std::cout << "[Mapping] Saved map with "
                  << map_copy->size()
                  << " points to " << path << std::endl;
        return true;
    }

    void MappingCore::saveMapAsync(const std::string &path)
    {
        // 防止重复触发
        if (saving_.exchange(true))
        {
            std::cout << "[Mapping] Save already in progress, skip request." << std::endl;
            return;
        }

        // 后台线程
        std::thread([this, path]()
                    {
        std::cout << "[Mapping] Async saving map..." << std::endl;

        bool ok1 = this->saveMap(path);
        if (ok1)
            std::cout << "[Mapping] Async map & database saved." << std::endl;
        else
            std::cout << "[Mapping] Async save failed." << std::endl;

        saving_ = false; })
            .detach();
    }
    bool MappingCore::addFrameWithSC(
        const ScanContext::PointCloudType::Ptr &cloud,
        const Eigen::Isometry3d &pose,
        double time,
        const ScanContext::SCDescriptor &sc,
        const ScanContext::RingKey &rk,
        std::shared_ptr<KeyFrame> * /* out_kf */)
    {
        if (!config_.enable_mapping)
            return false;

        if (!isKeyFrame(pose))
            return false;

        std::lock_guard<std::mutex> lock(map_mutex_);

        initDatabaseIfNeeded();

        const int idx = frame_index_++;
        std::cout << "[Mapping] Adding frame (SC external) " << idx << std::endl;

        Eigen::Quaterniond q(pose.rotation());
        Eigen::Vector3d t = pose.translation();

        pose_ofs_ << idx << " " << std::fixed << time << " "
                  << t.x() << " " << t.y() << " " << t.z() << " "
                  << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";

        const std::string cloud_path =
            db_dir_ + "/cloud_" + std::to_string(idx) + ".pcd";

        pcl::io::savePCDFileBinary(cloud_path, *cloud);

        global_map_->points.reserve(
            global_map_->points.size() + cloud->points.size());

        for (const auto &p : cloud->points)
        {
            ScanContext::PointType q_pt;
            q_pt.getVector4fMap() = pose.matrix().cast<float>() * p.getVector4fMap();
            global_map_->points.emplace_back(q_pt);
        }
        double dist = (pose.translation() - last_keyframe_pose_.translation()).norm();
        accumulated_dist_since_save_ += dist;
        std::cout << "[Mapping] Distance since last save: " << accumulated_dist_since_save_ << config_.map_save_path << std::endl;
        if (config_.auto_save_interval > 0 && accumulated_dist_since_save_ >= config_.auto_save_interval)
        {
            saveMapAsync(config_.map_save_path);
            accumulated_dist_since_save_ = 0.0;
        }
        last_keyframe_pose_ = pose;
        return true;
    }

} // namespace scantext
