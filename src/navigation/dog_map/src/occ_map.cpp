#include "dog_map/occ_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <oneapi/tbb/parallel_for.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_map>
#include <pcl/filters/radius_outlier_removal.h>
#include "dog_map/eigen_alias.hpp"

OccMap::OccMap(double min_obstacle_column_height,
               double min_obstacle_height, double max_obstacle_height,
               double ground_z_in_base, double half_map_size, double resolution_z,
               int frame_save, const std::string& pgm_yaml_path)
    : min_obstacle_column_height_(min_obstacle_column_height),
      min_obstacle_height_(min_obstacle_height),
      max_obstacle_height_(max_obstacle_height),
      ground_z_in_base_(ground_z_in_base),
      half_map_size_(half_map_size),
      resolution_z(resolution_z),
      frame_save_(frame_save),
      pgm_yaml_path(pgm_yaml_path) {
    if (resolution_z <= 0.0) {
        throw std::invalid_argument("resolution_z must be greater than zero");
    }
    if (min_obstacle_height_ < 0.0) {
        throw std::invalid_argument("min_obstacle_height must not be negative");
    }
    if (max_obstacle_height_ <= min_obstacle_height_) {
        throw std::invalid_argument(
            "max_obstacle_height must be greater than min_obstacle_height");
    }
    if (min_obstacle_column_height_ < 0.0) {
        throw std::invalid_argument(
            "min_obstacle_column_height must not be negative");
    }
    YAML::Node config = YAML::LoadFile(pgm_yaml_path);
    cloud_at_map_origin = yaml_get_value<std::vector<double>>(config, "origin");
    resolution = yaml_get_value<double>(config, "resolution");

    std::string pgm_path;
    int index = pgm_yaml_path.find_last_of("/");
    pgm_path = pgm_yaml_path.substr(0, index + 1) +
               yaml_get_value<std::string>(config, "image");
    cv::Mat map_img = cv::imread(pgm_path, cv::IMREAD_UNCHANGED);
    if (!map_img.empty()) {
        std::cout << " -- [DOG_MAP] Map loaded: size_x="
                  << map_img.cols * resolution
                  << ", size_y=" << map_img.rows * resolution << std::endl;
        InitMap(map_img);
    }
/*************fix map init */
#ifdef FIX_MAP
    static_fix_map_ = std::make_shared<static_fix_map::StaticFixMap>();
    static_fix_map_->InitPtr(std::make_shared<cv::Mat>(grid_map_), occ_map,
                             layer_z, ground_layer_z);
    static_fix_map_->SetMapInfo(cloud_at_map_origin[0], cloud_at_map_origin[1],
                                0.0, resolution);
#endif
    /*************888888888888888 */
    std::cout << " -- [DOG_MAP] OccMap initialized." << std::endl;
    std::cout << "[DOG_MAP] path " << pgm_path << std::endl;
    std::cout << " -- [DOG_MAP] half_map_size: " << half_map_size_
              << ", resolution_z: " << resolution_z << std::endl;
    std::cout<<" -- [DOG_MAP] OccMap resolution "<<resolution<<  std::endl;
    std::cout << " -- [DOG_MAP] ground_z_in_base: " << ground_z_in_base_
              << ", obstacle height band: (" << min_obstacle_height_ << ", "
              << max_obstacle_height_ << "]" << std::endl;
    std::cout << " -- [DOG_MAP] min_obstacle_column_height: "
              << min_obstacle_column_height_ << std::endl;
    std::cout << " -- [DOG_MAP] frame_save: " << frame_save_ << std::endl;
}
void OccMap::InitMap(cv::Mat& grid_map) {
    grid_map_ = grid_map;
    layer_z = std::max(
        1, static_cast<int>(std::ceil(max_obstacle_height_ / resolution_z)) + 1);
    const size_t column_count =
        static_cast<size_t>(grid_map_.rows) * grid_map_.cols;
    const size_t voxel_count = column_count * layer_z;
    occ_map = new uint8_t[voxel_count];
    observed_map_z_ = new float[column_count];
#ifndef DEBUG
    pub_map_ptr_ = new std::atomic<bool>[grid_map_.rows * grid_map_.cols];
#endif
    ground_layer_z = 0;
    std::fill_n(occ_map, voxel_count, 0);
    std::fill_n(observed_map_z_, column_count,
                std::numeric_limits<float>::quiet_NaN());
}
void OccMap::Clear() {
    const size_t column_count =
        static_cast<size_t>(grid_map_.rows) * grid_map_.cols;
    const size_t voxel_count = column_count * layer_z;
    std::fill_n(occ_map, voxel_count, 0);
    std::fill_n(observed_map_z_, column_count,
                std::numeric_limits<float>::quiet_NaN());
}

void OccMap::TimeGoON() {  //长期未观测到的障碍物清除
    std::vector<int> to_erase;
    to_erase.reserve(time_list_.size() / 4);  // 预估

    for (auto& [index, last_frame] : time_list_) {
        if (update_frame - last_frame > frame_save_) {
            occ_map[index] =
                std::max(static_cast<int>(occ_map[index]) + LOG_OCC_FREE / 2,
                         static_cast<int>(MIN_LOG));
        }
        if (occ_map[index] < THR_OCC) {
            to_erase.push_back(index);
        }
    }

    // 批量删除
    for (int idx : to_erase) {
        time_list_.erase(idx);
    }
}
using MapIt = std::map<int, long long>::iterator;
OccMap::PublicationStats OccMap::PubRes(
    pcl::PointCloud<pcl::PointXYZI>& pub_pc) {
    enum class ColumnState : uint8_t {
        kDuplicate,
        kSpanReject,
        kHoleReject,
        kAccepted
    };

    PublicationStats stats;
    stats.tracked_voxels = time_list_.size();

    //三维索引
#ifndef DEBUG
    std::fill_n(pub_map_ptr_, grid_map_.rows * grid_map_.cols, false);
#endif

    std::vector<std::pair<MapIt, ColumnState>> iterators;
    iterators.reserve(time_list_.size());
    for (auto it = time_list_.begin(); it != time_list_.end(); ++it) {
        iterators.emplace_back(it, ColumnState::kDuplicate);
    }

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, iterators.size()),
        [&](const tbb::blocked_range<size_t>& r) {
            for (size_t i = r.begin(); i != r.end(); ++i) {
                const int index = iterators[i].first->first;
                int rem = index % (grid_map_.cols * grid_map_.rows);
                int idy = rem / grid_map_.cols;
                int idx = rem % grid_map_.cols;
#ifndef DEBUG
                int key = idx + idy * grid_map_.cols;
                if (pub_map_ptr_[key].exchange(true)) continue;
#endif

                int max_z_occ_index = -1;
                int min_z_occ_index = 0;
                for (int zi = layer_z - 1; zi >= 0; zi--) {
                    if (IsOccupied(idx, idy, zi)) {
                        max_z_occ_index = zi;
                        break;
                    }
                }
                for (int zi = 0; zi < layer_z; zi++) {
                    if (IsOccupied(idx, idy, zi)) {
                        min_z_occ_index = zi;
                        break;
                    }
                }
                if (max_z_occ_index < 0) {
                    iterators[i].second = ColumnState::kSpanReject;
                    continue;
                }
                const int occupied_span =
                    max_z_occ_index - min_z_occ_index + 1;
                const double column_height = occupied_span * resolution_z;
                if (column_height < min_obstacle_column_height_) {
                    iterators[i].second = ColumnState::kSpanReject;
                    continue;
                }

                // 计算最大空洞大小
                int max_hole_size = 0;
                int curr_hole_size = 0;
                for (int zi = max_z_occ_index - 1; zi >= min_z_occ_index;
                     zi--) {
                    if (!IsOccupied(idx, idy, zi)) {
                        curr_hole_size++;
                    } else {
                        if (curr_hole_size > max_hole_size) {
                            max_hole_size = curr_hole_size;
                        }
                        curr_hole_size = 0;
                    }
                }
                max_hole_size = std::max(max_hole_size, curr_hole_size);
                if (max_hole_size * resolution_z > 0.15) {
                    iterators[i].second = ColumnState::kHoleReject;
                    continue;
                }
                iterators[i].second = ColumnState::kAccepted;
            }
        });

    for (const auto& [it, state] : iterators) {
        if (state == ColumnState::kDuplicate) {
            continue;
        }
        ++stats.candidate_columns;
        if (state == ColumnState::kSpanReject) {
            ++stats.span_reject;
        } else if (state == ColumnState::kHoleReject) {
            ++stats.hole_reject;
        } else if (state == ColumnState::kAccepted) {
            ++stats.accepted_columns;
            int index = it->first;
            int idz = index / (grid_map_.cols * grid_map_.rows);
            int rem = index % (grid_map_.cols * grid_map_.rows);
            int idy = rem / grid_map_.cols;
            int idx = rem % grid_map_.cols;
            pcl::PointXYZI pt;
            pt.x = idx * resolution + resolution / 2.0 + cloud_at_map_origin[0];
            pt.y = idy * resolution + resolution / 2.0 + cloud_at_map_origin[1];
            pt.z = std::isfinite(observed_map_z_[idx + idy * grid_map_.cols])
                       ? observed_map_z_[idx + idy * grid_map_.cols]
                       : 0.0f;
            pt.intensity = static_cast<float>(
                idz * resolution_z + resolution_z / 2.0);  // local height
            pub_pc.points.push_back(pt);

#ifdef FIX_MAP
            UpdateStaticMap(idx, idy);
#endif
        }
    }

    return stats;
}
OccMap::~OccMap() {
    delete[] occ_map;
    delete[] observed_map_z_;
#ifndef DEBUG
    delete[] pub_map_ptr_;
#endif
}
//俯瞰地图是否占据,任一高度层占据即为占据
bool OccMap::IsOccupied(const Vec2i& idx) {
    for (int zi = 0; zi < layer_z; zi++) {
        int index = idx[0] + idx[1] * grid_map_.cols +
                    zi * grid_map_.cols * grid_map_.rows;
        if (occ_map[index] >= THR_OCC) return true;
    }
    return false;
}
//指定某一个格子是否占据
bool OccMap::IsOccupied(int index) {
    if (occ_map[index] >= THR_OCC) return true;
    return false;
}
bool OccMap::IsOccupied(int ix, int iy, int iz) {
    int index = ix + iy * grid_map_.cols + iz * grid_map_.cols * grid_map_.rows;
    if (occ_map[index] >= THR_OCC) return true;
    return false;
}
bool OccMap::IsOccupied(const Vec3i& idx) {
    int index = idx[0] + idx[1] * grid_map_.cols +
                idx[2] * grid_map_.cols * grid_map_.rows;
    if (occ_map[index] >= THR_OCC) return true;
    return false;
}

bool OccMap::IsStaticMapPoint(const Vec2i& idx) {
    if (grid_map_.at<unsigned char>(idx[1], idx[0]) == 255) return true;
    return false;
}
bool OccMap::IsStaticMapPoint(int index) {
    int idx = index % grid_map_.cols;
    int idy = index / grid_map_.cols;

    if (grid_map_.at<unsigned char>(idy, idx) == 255) return true;

    return false;
}
Vec3f OccMap::PointInBase(const pcl::PointXYZ& point,
                            const Pose& pose) const {
    const Vec3f point_in_map(point.x, point.y, point.z);
    return pose.second.conjugate() * (point_in_map - pose.first);
}

double OccMap::HeightAboveGround(const pcl::PointXYZ& point,
                                 const Pose& pose) const {
    return PointInBase(point, pose).z() - ground_z_in_base_;
}

void OccMap::PassFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                        Pose pose,
                        pcl::PointCloud<pcl::PointXYZ>* ground_cloud) {
    pcl::PointCloud<pcl::PointXYZ> obstacle_candidates;
    obstacle_candidates.header = cloud_in->header;
    obstacle_candidates.points.reserve(cloud_in->points.size());

    if (ground_cloud != nullptr) {
        ground_cloud->clear();
        ground_cloud->header = cloud_in->header;
        ground_cloud->points.reserve(cloud_in->points.size());
    }

    for (const auto& point : cloud_in->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            continue;
        }
        const Vec3f point_in_base = PointInBase(point, pose);
        if (point_in_base.x() >= crop_min_x &&
            point_in_base.x() <= crop_max_x &&
            point_in_base.y() >= crop_min_y &&
            point_in_base.y() <= crop_max_y &&
            point_in_base.z() >= crop_min_z &&
            point_in_base.z() <= crop_max_z) {
            continue;
        }
        const double height_above_ground =
            point_in_base.z() - ground_z_in_base_;
        if (height_above_ground <= min_obstacle_height_) {
            if (ground_cloud != nullptr) {
                ground_cloud->points.push_back(point);
            }
            continue;
        }
        if (height_above_ground <= max_obstacle_height_) {
            obstacle_candidates.points.push_back(point);
        }
    }

    obstacle_candidates.width = obstacle_candidates.points.size();
    obstacle_candidates.height = 1;
    obstacle_candidates.is_dense = true;
    cloud_in->swap(obstacle_candidates);
    if (ground_cloud != nullptr) {
        ground_cloud->width = ground_cloud->points.size();
        ground_cloud->height = 1;
        ground_cloud->is_dense = true;
    }

    pcl::PassThrough<pcl::PointXYZ> pass;
    
    double min_x = pose.first[0] - half_map_size_;
    min_x = std::max(min_x, cloud_at_map_origin[0] + 0.2);
    double max_x = pose.first[0] + half_map_size_;
    max_x = std::min(
        max_x, resolution * grid_map_.cols + cloud_at_map_origin[0] - 0.2);
    double min_y = pose.first[1] - half_map_size_;
    min_y = std::max(min_y, cloud_at_map_origin[1] + 0.2);
    double max_y = pose.first[1] + half_map_size_;
    max_y = std::min(
        max_y, resolution * grid_map_.rows + cloud_at_map_origin[1] - 0.2);
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(min_x, max_x);
    pass.filter(*cloud_in);
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(min_y, max_y);
    pass.filter(*cloud_in);

    // 射线更新最终只落到 OccMap 的离散体素中。同一个体素里的重复
    // LiDAR 点会生成完全相同的占据终点，却让 racyHandle 重复遍历整条
    // 射线。MID360 一帧通常包含大量这种重复点，是地图处理落后数秒的
    // 主要来源。先按地图自身分辨率去重，不会降低地图可表达的精度。
    if (!cloud_in->empty()) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud_in);
        voxel_filter.setLeafSize(
            static_cast<float>(resolution),
            static_cast<float>(resolution),
            static_cast<float>(resolution_z));
        pcl::PointCloud<pcl::PointXYZ> downsampled;
        voxel_filter.filter(downsampled);
        cloud_in->swap(downsampled);
    }

    // 离群点滤波
    // if(cloud_in->size()==0) return ;
    //  pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    // ror.setInputCloud(cloud_in);
    // ror.setRadiusSearch(0.1); // 设置搜索半径 (如 0.15 米)
    // ror.setMinNeighborsInRadius(4); // 设定在该半径内至少所需的邻居点数，小于此数则视为离群点过滤掉
    // ror.filter(*cloud_in);
    
}
void OccMap::Update(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Pose pose,
                    pcl::PointCloud<pcl::PointXYZ>* ground_cloud) {
    //更新局部地图

    PassFilter(cloud, pose, ground_cloud);
       
    // TODO: 进行地图更新
    //射线起点格子
    int px =
        static_cast<int>((pose.first[0] - cloud_at_map_origin[0]) / resolution);
    int py =
        static_cast<int>((pose.first[1] - cloud_at_map_origin[1]) / resolution);
    int pz = static_cast<int>(
        std::floor(-ground_z_in_base_ / resolution_z));
    pz = std::clamp(pz, 0, layer_z - 1);
    Vec3i start_idx = Vec3i{px, py, pz};

    Vec3f res(resolution, resolution, resolution_z);

    // 2. 转换为物理空间坐标 (以中心或原点对齐)
    Vec3f start_f = start_idx.cast<double>().array() * res.array();
#ifndef TBB_DO

    for (auto& point : cloud->points) {
        int xi =
            static_cast<int>((point.x - cloud_at_map_origin[0]) / resolution);
        int yi =
            static_cast<int>((point.y - cloud_at_map_origin[1]) / resolution);
        int zi = static_cast<int>(std::floor(
            HeightAboveGround(point, pose) / resolution_z));
        if (xi < 0 || xi >= grid_map_.cols || yi < 0 ||
            yi >= grid_map_.rows || zi < 0 || zi >= layer_z) {
            continue;
        }
        Vec3i obs_idx = Vec3i{xi, yi, zi};
        racyHandle(start_idx, obs_idx, start_f, res);
    }

#else
    // TODO: 并行化free空间更新

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, cloud->points.size()),
        [&](const tbb::blocked_range<size_t>& r) {
            for (size_t i = r.begin(); i != r.end(); ++i) {
                int xi = static_cast<int>(
                    (cloud->points[i].x - cloud_at_map_origin[0]) / resolution);
                int yi = static_cast<int>(
                    (cloud->points[i].y - cloud_at_map_origin[1]) / resolution);

                int zi = static_cast<int>(std::floor(
                    HeightAboveGround(cloud->points[i], pose) /
                    resolution_z));
                if (xi < 0 || xi >= grid_map_.cols || yi < 0 ||
                    yi >= grid_map_.rows || zi < 0 || zi >= layer_z) {
                    continue;
                }
                Vec3i obs_idx = Vec3i{xi, yi, zi};
                racyHandle(start_idx, obs_idx, start_f, res);
            }
        });
    //障碍物添加

#endif

    for (auto& point : cloud->points) {
        int xi =
            static_cast<int>((point.x - cloud_at_map_origin[0]) / resolution);
        int yi =
            static_cast<int>((point.y - cloud_at_map_origin[1]) / resolution);
        if (xi < 0 || xi >= grid_map_.cols || yi < 0 ||
            yi >= grid_map_.rows) {
            continue;
        }
#ifdef SKIP_STATIC
        if (grid_map_.at<unsigned char>(grid_map_.rows - 1 - yi, xi) < 240)
            continue;  //静态障碍物不处理,偷运算
#endif
        int zi = static_cast<int>(std::floor(
            HeightAboveGround(point, pose) / resolution_z));
        if (zi < 0 || zi >= layer_z) {
            continue;
        }
        int index =
            xi + yi * grid_map_.cols + zi * grid_map_.cols * grid_map_.rows;
        observed_map_z_[xi + yi * grid_map_.cols] = point.z;
        //更新占据概率
        occ_map[index] =
            std::min(static_cast<int>(occ_map[index]) + LOG_OCC_HIT,
                     static_cast<int>(MAX_LOG));
        //加入观测列表
        if (occ_map[index] >= THR_OCC) {
            time_list_[index] = update_frame;
        }
    }
       
    // 更新未观测到的累计帧数
    TimeGoON();
     
    update_frame++;

}
void OccMap::UpdateStaticMap(int idx, int idy) {
    int img_idy = grid_map_.rows - 1 - idy;
    if (idx < 0 || idx >= grid_map_.cols || idy < 0 || idy >= grid_map_.rows)
        return;
    if (grid_map_.at<unsigned char>(img_idy, idx) > 252) {
        grid_map_.at<unsigned char>(img_idy, idx) =
            0;  // 将 pgm 上的占据点改为 0（占据）
    }
}
