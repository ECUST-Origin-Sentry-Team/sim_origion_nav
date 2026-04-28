#include "scantext_module/Mapping.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

namespace scantext
{
namespace
{
constexpr uint32_t kCacheVersion = 1U;
constexpr uint32_t kCacheMagic = 0x53434348U; // SCCH

template <typename T>
void writeBinary(std::ostream &os, const T &value)
{
    os.write(reinterpret_cast<const char *>(&value), sizeof(T));
}

template <typename T>
bool readBinary(std::istream &is, T &value)
{
    is.read(reinterpret_cast<char *>(&value), sizeof(T));
    return static_cast<bool>(is);
}

uint64_t checksumAppend(uint64_t hash, const void *data, size_t bytes)
{
    const auto *ptr = static_cast<const uint8_t *>(data);
    for (size_t i = 0; i < bytes; ++i)
    {
        hash ^= static_cast<uint64_t>(ptr[i]);
        hash *= 1099511628211ULL;
    }
    return hash;
}

uint64_t computeDescriptorChecksum(const ScanContext::SCDescriptor &sc,
                                  const ScanContext::RingKey &rk,
                                  const ScanContext::CartDescriptor &cart)
{
    uint64_t hash = 1469598103934665603ULL;
    const int sc_rows = sc.rows();
    const int sc_cols = sc.cols();
    const int rk_size = rk.size();
    const int cart_rows = cart.rows();
    const int cart_cols = cart.cols();
    hash = checksumAppend(hash, &sc_rows, sizeof(sc_rows));
    hash = checksumAppend(hash, &sc_cols, sizeof(sc_cols));
    hash = checksumAppend(hash, &rk_size, sizeof(rk_size));
    hash = checksumAppend(hash, &cart_rows, sizeof(cart_rows));
    hash = checksumAppend(hash, &cart_cols, sizeof(cart_cols));
    if (sc.size() > 0)
    {
        hash = checksumAppend(hash, sc.data(), sizeof(double) * static_cast<size_t>(sc.size()));
    }
    if (rk.size() > 0)
    {
        hash = checksumAppend(hash, rk.data(), sizeof(double) * static_cast<size_t>(rk.size()));
    }
    if (cart.size() > 0)
    {
        hash = checksumAppend(hash, cart.data(), sizeof(double) * static_cast<size_t>(cart.size()));
    }
    return hash;
}

std::string descriptorFileName(int index)
{
    return "descriptor_" + std::to_string(index) + ".bin";
}

bool writeDescriptorCache(const std::string &path, const KeyFrame &kf, const SCParams &params)
{
    const std::string tmp_path = path + ".tmp";
    std::ofstream os(tmp_path, std::ios::binary | std::ios::trunc);
    if (!os.is_open())
    {
        return false;
    }

    const uint32_t index = static_cast<uint32_t>(kf.index);
    const uint32_t cloud_path_len = static_cast<uint32_t>(kf.cloud_path.size());
    const uint32_t sc_rows = static_cast<uint32_t>(kf.descriptor.rows());
    const uint32_t sc_cols = static_cast<uint32_t>(kf.descriptor.cols());
    const uint32_t rk_size = static_cast<uint32_t>(kf.ring_key.size());
    const uint32_t cart_rows = static_cast<uint32_t>(kf.cart_descriptor.rows());
    const uint32_t cart_cols = static_cast<uint32_t>(kf.cart_descriptor.cols());
    const uint64_t checksum = computeDescriptorChecksum(kf.descriptor, kf.ring_key, kf.cart_descriptor);
    const int32_t num_ring = params.num_ring;
    const int32_t num_sector = params.num_sector;
    const double max_radius = params.max_radius;
    const double lidar_height = params.lidar_height;
    const uint8_t use_scpp = params.use_scpp ? 1U : 0U;
    const double scpp_search_ratio = params.scpp_search_ratio;
    const double cart_x_unit = params.cart_x_unit;
    const double cart_y_unit = params.cart_y_unit;
    const double cart_x_max = params.cart_x_max;
    const double cart_y_max = params.cart_y_max;
    const Eigen::Quaterniond q(kf.pose.rotation());
    const Eigen::Vector3d t = kf.pose.translation();
    const double pose_data[8] = {kf.time, t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()};

    writeBinary(os, kCacheMagic);
    writeBinary(os, kCacheVersion);
    writeBinary(os, index);
    writeBinary(os, cloud_path_len);
    writeBinary(os, sc_rows);
    writeBinary(os, sc_cols);
    writeBinary(os, rk_size);
    writeBinary(os, cart_rows);
    writeBinary(os, cart_cols);
    writeBinary(os, num_ring);
    writeBinary(os, num_sector);
    writeBinary(os, max_radius);
    writeBinary(os, lidar_height);
    writeBinary(os, use_scpp);
    writeBinary(os, scpp_search_ratio);
    writeBinary(os, cart_x_unit);
    writeBinary(os, cart_y_unit);
    writeBinary(os, cart_x_max);
    writeBinary(os, cart_y_max);
    writeBinary(os, checksum);
    os.write(reinterpret_cast<const char *>(pose_data), sizeof(pose_data));
    os.write(kf.cloud_path.data(), static_cast<std::streamsize>(kf.cloud_path.size()));
    if (kf.descriptor.size() > 0)
    {
        os.write(reinterpret_cast<const char *>(kf.descriptor.data()), sizeof(double) * static_cast<std::streamsize>(kf.descriptor.size()));
    }
    if (kf.ring_key.size() > 0)
    {
        os.write(reinterpret_cast<const char *>(kf.ring_key.data()), sizeof(double) * static_cast<std::streamsize>(kf.ring_key.size()));
    }
    if (kf.cart_descriptor.size() > 0)
    {
        os.write(reinterpret_cast<const char *>(kf.cart_descriptor.data()), sizeof(double) * static_cast<std::streamsize>(kf.cart_descriptor.size()));
    }
    os.close();
    if (!os)
    {
        std::filesystem::remove(tmp_path);
        return false;
    }
    std::filesystem::rename(tmp_path, path);
    return true;
}

bool readDescriptorCache(const std::string &path,
                         int expected_index,
                         const SCParams &params,
                         KeyFrame &kf)
{
    std::ifstream is(path, std::ios::binary);
    if (!is.is_open())
    {
        return false;
    }

    uint32_t magic = 0;
    uint32_t version = 0;
    uint32_t index = 0;
    uint32_t cloud_path_len = 0;
    uint32_t sc_rows = 0;
    uint32_t sc_cols = 0;
    uint32_t rk_size = 0;
    uint32_t cart_rows = 0;
    uint32_t cart_cols = 0;
    int32_t num_ring = 0;
    int32_t num_sector = 0;
    double max_radius = 0.0;
    double lidar_height = 0.0;
    uint8_t use_scpp = 0U;
    double scpp_search_ratio = 0.0;
    double cart_x_unit = 0.0;
    double cart_y_unit = 0.0;
    double cart_x_max = 0.0;
    double cart_y_max = 0.0;
    uint64_t checksum = 0;

    if (!readBinary(is, magic) || !readBinary(is, version) || !readBinary(is, index) ||
        !readBinary(is, cloud_path_len) || !readBinary(is, sc_rows) || !readBinary(is, sc_cols) ||
        !readBinary(is, rk_size) || !readBinary(is, cart_rows) || !readBinary(is, cart_cols) ||
        !readBinary(is, num_ring) || !readBinary(is, num_sector) || !readBinary(is, max_radius) ||
        !readBinary(is, lidar_height) || !readBinary(is, use_scpp) || !readBinary(is, scpp_search_ratio) ||
        !readBinary(is, cart_x_unit) || !readBinary(is, cart_y_unit) || !readBinary(is, cart_x_max) || !readBinary(is, cart_y_max) ||
        !readBinary(is, checksum))
    {
        return false;
    }

    if (magic != kCacheMagic || version != kCacheVersion || static_cast<int>(index) != expected_index ||
        static_cast<int>(sc_rows) != params.num_ring || static_cast<int>(sc_cols) != params.num_sector ||
        static_cast<int>(rk_size) != params.num_ring || num_ring != params.num_ring || num_sector != params.num_sector ||
        std::abs(max_radius - params.max_radius) > 1e-9 || std::abs(lidar_height - params.lidar_height) > 1e-9 ||
        (use_scpp != (params.use_scpp ? 1U : 0U)) || std::abs(scpp_search_ratio - params.scpp_search_ratio) > 1e-9 ||
        std::abs(cart_x_unit - params.cart_x_unit) > 1e-9 || std::abs(cart_y_unit - params.cart_y_unit) > 1e-9 ||
        std::abs(cart_x_max - params.cart_x_max) > 1e-9 || std::abs(cart_y_max - params.cart_y_max) > 1e-9)
    {
        return false;
    }

    double pose_data[8] = {0.0};
    is.read(reinterpret_cast<char *>(pose_data), sizeof(pose_data));
    if (!is)
    {
        return false;
    }

    std::string cloud_path(cloud_path_len, '\0');
    if (cloud_path_len > 0)
    {
        is.read(cloud_path.data(), static_cast<std::streamsize>(cloud_path_len));
        if (!is)
        {
            return false;
        }
    }

    kf.descriptor = ScanContext::SCDescriptor(static_cast<int>(sc_rows), static_cast<int>(sc_cols));
    kf.ring_key = ScanContext::RingKey(static_cast<int>(rk_size));
    kf.cart_descriptor = ScanContext::CartDescriptor(static_cast<int>(cart_rows), static_cast<int>(cart_cols));

    if (kf.descriptor.size() > 0)
    {
        is.read(reinterpret_cast<char *>(kf.descriptor.data()), sizeof(double) * static_cast<std::streamsize>(kf.descriptor.size()));
    }
    if (kf.ring_key.size() > 0)
    {
        is.read(reinterpret_cast<char *>(kf.ring_key.data()), sizeof(double) * static_cast<std::streamsize>(kf.ring_key.size()));
    }
    if (kf.cart_descriptor.size() > 0)
    {
        is.read(reinterpret_cast<char *>(kf.cart_descriptor.data()), sizeof(double) * static_cast<std::streamsize>(kf.cart_descriptor.size()));
    }
    if (!is)
    {
        return false;
    }

    if (checksum != computeDescriptorChecksum(kf.descriptor, kf.ring_key, kf.cart_descriptor))
    {
        return false;
    }

    kf.cloud_path = cloud_path;
    kf.descriptor_cache_path = std::filesystem::path(path).filename().string();
    return true;
}
} // namespace

MappingCore::MappingCore()
{
    last_keyframe_pose_ = Eigen::Isometry3d::Identity();
}

void MappingCore::setConfig(const Config &config)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    config_ = config;
}

void MappingCore::setScanContextParams(const SCParams &params)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    scan_context_ = ScanContext(params);
}

bool MappingCore::addFrame(const ScanContext::PointCloudType::Ptr &cloud, const Eigen::Isometry3d &pose, double time)
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
    kf->cloud = cloud;
    kf->cloud_path = "cloud_" + std::to_string(kf->index) + ".pcd";
    kf->descriptor_cache_path = descriptorFileName(kf->index);
    kf->descriptor = scan_context_.makeScanContext(*cloud);
    kf->ring_key = scan_context_.makeRingKey(kf->descriptor);
    kf->cart_descriptor = scan_context_.makeCartContext(*cloud);
    keyframes_.push_back(kf);

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
    std::filesystem::create_directories(directory);

    std::filesystem::remove(directory + "/sc_cache_manifest.yaml");

    std::ofstream ofs(directory + "/poses.txt", std::ios::out | std::ios::trunc);
    if (!ofs.is_open())
        return false;

    for (const auto &kf : keyframes_)
    {
        Eigen::Quaterniond q(kf->pose.rotation());
        Eigen::Vector3d t = kf->pose.translation();
        ofs << kf->index << " " << std::fixed << kf->time << " "
            << t.x() << " " << t.y() << " " << t.z() << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        const std::string cloud_name = "cloud_" + std::to_string(kf->index) + ".pcd";
        pcl::io::savePCDFileBinary(directory + "/" + cloud_name, *kf->cloud);
        KeyFrame tmp = *kf;
        tmp.cloud_path = cloud_name;
        tmp.descriptor_cache_path = descriptorFileName(kf->index);
        writeDescriptorCache(directory + "/" + tmp.descriptor_cache_path, tmp, scan_context_.getParams());
    }

    std::cout << "[Mapping] Saved " << keyframes_.size() << " keyframes to " << directory << std::endl;
    return true;
}

bool MappingCore::loadDatabase(const std::string &directory)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    keyframes_.clear();

    int cache_hits = 0;
    int cache_misses = 0;

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
        kf->cloud_path = "cloud_" + std::to_string(kf->index) + ".pcd";
        kf->descriptor_cache_path = descriptorFileName(kf->index);

        kf->cloud.reset(new ScanContext::PointCloudType());
        const std::string pcd_path = directory + "/" + kf->cloud_path;
        if (pcl::io::loadPCDFile(pcd_path, *kf->cloud) == -1)
        {
            std::cerr << "[Mapping] Failed to load " << pcd_path << std::endl;
            continue;
        }

        const bool loaded_cache = readDescriptorCache(directory + "/" + kf->descriptor_cache_path,
                                                      kf->index,
                                                      scan_context_.getParams(),
                                                      *kf);

        if (!loaded_cache)
        {
            kf->descriptor = scan_context_.makeScanContext(*kf->cloud);
            kf->ring_key = scan_context_.makeRingKey(kf->descriptor);
            kf->cart_descriptor = scan_context_.makeCartContext(*kf->cloud);
            cache_misses++;
            writeDescriptorCache(directory + "/" + kf->descriptor_cache_path, *kf, scan_context_.getParams());
        }
        else
        {
            cache_hits++;
        }

        keyframes_.push_back(kf);
    }

    if (!keyframes_.empty())
    {
        last_keyframe_pose_ = keyframes_.back()->pose;
    }

    std::filesystem::remove(directory + "/sc_cache_manifest.yaml");

    std::cout << "[Mapping] Loaded " << keyframes_.size() << " keyframes from " << directory
              << " (cache hits=" << cache_hits << ", misses=" << cache_misses << ")" << std::endl;
    return true;
}

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
    kf->cloud = cloud;
    kf->cloud_path = "cloud_" + std::to_string(kf->index) + ".pcd";
    kf->descriptor_cache_path = descriptorFileName(kf->index);
    kf->descriptor = sc;
    kf->ring_key = rk;
    kf->cart_descriptor = scan_context_.makeCartContext(*cloud);
    keyframes_.push_back(kf);
    if (out_kf)
        *out_kf = kf;

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
