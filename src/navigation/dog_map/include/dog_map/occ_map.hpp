#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_struct_traits.h>
#include <pcl/point_types.h>
#include <tbb/parallel_for.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/core/mat.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/crop_box.h>

#include "dog_map/eigen_alias.hpp"
#include "dog_map/static_fix_map.hpp"
using namespace analy_utils;
const float crop_min_x = -0.5f;
const float crop_min_y = -0.5f;
const float crop_min_z = -10.5f;
const float crop_max_x = 0.8f;
const float crop_max_y = 0.5f;
const float crop_max_z = 10.0f;
template <typename T>
T yaml_get_value(const YAML::Node& node, const std::string& key) {
    try {
        return node[key].as<T>();
    } catch (YAML::Exception& e) {
        std::stringstream ss;
        ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
        throw YAML::Exception(e.mark, ss.str());
    }
}

#define FORGET_FACTOR -30
class OccMap {
   public:
    struct PublicationStats {
        size_t tracked_voxels{0};
        size_t candidate_columns{0};
        size_t accepted_columns{0};
        size_t span_reject{0};
        size_t hole_reject{0};
    };

    OccMap(double min_obstacle_column_height, double min_obstacle_height,
           double max_obstacle_height, double ground_z_in_base,
           double half_map_size, double resolution_z, int frame_save,
           const std::string& pgm_yaml_path = "");
    ~OccMap();
    void InitMap(cv::Mat& grid_map);
    void Update(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Pose pose,
                pcl::PointCloud<pcl::PointXYZ>* ground_cloud = nullptr);
    bool IsOccupied(const Vec2i& idx);
    bool IsOccupied(const Vec3i& idx);
    bool IsOccupied(int index);
    bool IsOccupied(int ix, int iy, int iz);
    void Clear();
    inline void SetLOGParams( int8_t LOG_OCC_HIT, int8_t LOG_OCC_FREE , uint8_t THR_OCC, uint8_t MAX_LOG, uint8_t MIN_LOG ){
        this->LOG_OCC_HIT = LOG_OCC_HIT;
        this->LOG_OCC_FREE = LOG_OCC_FREE;
        this->THR_OCC = THR_OCC;
        this->MAX_LOG = MAX_LOG;
        this->MIN_LOG = MIN_LOG;
    }
    void UpdateStaticMap(int idx, int idy);
    static_fix_map::StaticFixMap* GetStaticFixMap() {
        return static_fix_map_.get();
    }
    bool IsStaticMapPoint(const Vec2i& idx);
    bool IsStaticMapPoint(int index);
    void TimeGoON();
    PublicationStats PubRes(pcl::PointCloud<pcl::PointXYZI>& pub_pc);
    void PassFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, Pose pose,
                    pcl::PointCloud<pcl::PointXYZ>* ground_cloud = nullptr);
    inline void racyHandle(const Vec3i& start_idx, const Vec3i& end_idx,
                           Vec3f& start_f, Vec3f& res) {
        // TODO  Amanatides-Woo
        // 射线步进初始化实现3D网格的射线处理,利用对数概率发现符合障碍物标准,加入time_list_

        Vec3f end_f = end_idx.cast<double>().array() * res.array();

        Vec3f dir = (end_f - start_f).normalized();
        Vec3i step = (end_f.array() > start_f.array())
                         .select(Vec3i(1, 1, 1), Vec3i(-1, -1, -1));

        // 3. 计算 tDelta
        // tDelta[i] = 跨越第 i 轴一个栅格所需的物理距离 t
        Vec3f tDelta;
        for (int i = 0; i < 3; ++i) {
            tDelta[i] =
                std::abs(dir[i]) > 1e-6f ? std::abs(res[i] / dir[i]) : 1e10f;
        }
        Vec3f tMax;
        for (int i = 0; i < 3; ++i) {
            if (std::abs(dir[i]) > 1e-6f) {
                // 下一个边界的物理坐标
                float next_boundary =
                    (start_idx[i] + (step[i] > 0 ? 1.0f : 0.0f)) * res[i];
                tMax[i] = (next_boundary - start_f[i]) / dir[i];
            } else {
                tMax[i] = 1e10f;
            }
        }
        Vec3i curr_idx = start_idx;

        // 4. 射线步进：更新路径上的 Free 状态
        while (true) {
            // 判断终止条件
            if (curr_idx == end_idx) break;
            int index = curr_idx[0] + curr_idx[1] * grid_map_.cols +
                        curr_idx[2] * grid_map_.cols * grid_map_.rows;
            if (index < 0 || index >= grid_map_.cols * grid_map_.rows * layer_z)
                break;
            // 更新 Free 状态
            occ_map[index] =
                std::max(static_cast<int>(occ_map[index]) + LOG_OCC_FREE,
                         static_cast<int>(MIN_LOG));

            // 选择下一个要跨越的栅格
            if (tMax[0] < tMax[1]) {
                if (tMax[0] < tMax[2]) {
                    // 跨越 x 轴栅格
                    curr_idx[0] += step[0];
                    tMax[0] += tDelta[0];
                } else {
                    // 跨越 z 轴栅格
                    curr_idx[2] += step[2];
                    tMax[2] += tDelta[2];
                }
            } else {
                if (tMax[1] < tMax[2]) {
                    // 跨越 y 轴栅格
                    curr_idx[1] += step[1];
                    tMax[1] += tDelta[1];
                } else {
                    // 跨越 z 轴栅格
                    curr_idx[2] += step[2];
                    tMax[2] += tDelta[2];
                }
            }
        }
    }

   private:
    Vec3f PointInBase(const pcl::PointXYZ& point, const Pose& pose) const;
    double HeightAboveGround(const pcl::PointXYZ& point,
                             const Pose& pose) const;

    double min_obstacle_column_height_;
    double min_obstacle_height_;
    double max_obstacle_height_;
    double ground_z_in_base_;
    double half_map_size_;
    double resolution_z;
    int frame_save_;
    std::string pgm_yaml_path;
   private:
    cv::Mat grid_map_;
    uint8_t* occ_map{nullptr};
    float* observed_map_z_{nullptr};
    double resolution;
    double local_map_size;
    int local_map_size_d;

    int layer_z;
    int ground_layer_z;
    long long update_frame = 0;
    std::vector<double> cloud_at_map_origin;
    std::shared_ptr<static_fix_map::StaticFixMap> static_fix_map_;
    std::atomic<bool>* pub_map_ptr_{nullptr};
    std::vector<int> static_indexs;       //静态地图占据点索引
    std::map<int, long long> time_list_;  //索引+未观测到的累计帧数


    int8_t LOG_OCC_HIT = 40;   // 占据增量
    int8_t LOG_OCC_FREE = -3;  // 空闲减量
    uint8_t THR_OCC = 30;      // 判定为障碍物的阈值
    uint8_t MAX_LOG = 100;   // 最大对数概率
    uint8_t MIN_LOG = 5;
};
