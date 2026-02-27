// Relocalization.hpp

#pragma once

#include <vector>
#include <mutex>
#include <memory>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <flann/flann.hpp>   // NEW: 高维 KD-tree
#include "common/eigen_types.h"
#include "scantext_module/ScanContext.hpp"
#include "scantext_module/Mapping.hpp"

namespace scantext {

class RelocalizationCore {
public:
    struct Config {
        // --- candidate search ---
        int ringkey_top_k = 10;     // KD-tree 取 topK (Optimized)
        int sc_top_k      = 10;     // SC 精匹配保留 topM
        int icp_top_k     = 3;      // 最后做 ICP 的候选数

        double sc_dist_thresh = 0.5;      // SC 距离阈值（越小越像）
        double icp_fitness_thresh = 0.05; // ICP fitness 阈值 (Strict 0.05m)

        // --- ICP params ---
        double icp_max_corr_dist = 2.0;
        int    icp_max_iter = 30;         // (Optimized <= 30)

        // --- local submap ---
        int    local_submap_kf_num = 10;   // 取候选 keyframe 前后 N 帧拼子图
        double local_map_voxel_leaf = 0.5; // 子图体素降采样
        double query_voxel_leaf = 0.5;     // query 云体素降采样

        // --- index rebuild ---
        int rebuild_index_every_n = 50;    // 新增 keyframe 累积到 N 再重建 KD-tree
    };

    enum class ReloState {
        IDLE,       // 空闲/已定位
        REQUESTED,  // 请求重定位 (未定位且TF未发布，或收到init_guess)
        RUNNING,    // 正在计算
        SUCC,       // 成功
        FAIL        // 失败
    };

    RelocalizationCore();
    ~RelocalizationCore() = default;

    void setConfig(const Config& config);

    // 一次性设置（离线加载地图）
    void setMap(const std::vector<std::shared_ptr<KeyFrame>>& keyframes);

    // 在线 mapping 增量添加
    void addKeyFrame(const std::shared_ptr<KeyFrame>& kf);

    // 新接口：直接吃 SC
    bool updateFromSC(const ScanContext::SCDescriptor& sc,
                      const ScanContext::RingKey& rk,
                      const ScanContext::PointCloudType::Ptr& cloud_ds,
                      const Eigen::Isometry3d& odom_pose,
                      double time,
                      const Eigen::Vector3d& velocity = Eigen::Vector3d::Zero());

    // 旧接口保留：内部算 SC 后转调 updateFromSC
    bool update(const ScanContext::PointCloudType::Ptr& cloud,
                const Eigen::Isometry3d& odom_pose,
                double time,
                const Eigen::Vector3d& velocity = Eigen::Vector3d::Zero());

    Eigen::Isometry3d getMapOdomTransform() const { return map_odom_transform_; }
    bool isLocalized() const { return is_localized_; }
    ReloState getState() const { return state_; }

    void setInitialPose(const Eigen::Isometry3d& pose, const Mat6d& cov);
    void resetRelocalization(); // Force reset to REQUESTED

private:
    struct Candidate {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        int idx = -1;
        double rk_dist = 1e9;
        double sc_dist = 1e9;
        int shift = 0;
        Eigen::Isometry3d init_pose = Eigen::Isometry3d::Identity();    // map_T_base init
        Eigen::Isometry3d refined_pose = Eigen::Isometry3d::Identity(); // map_T_base refined
        double icp_fitness = 1e9;
        bool icp_ok = false;
    };

    Config config_;
    ScanContext scan_context_;

    std::vector<std::shared_ptr<KeyFrame>> map_keyframes_;
    mutable std::mutex relo_mutex_;

    // --- localization status ---
    std::atomic<bool> is_localized_{false};
    std::atomic<ReloState> state_{ReloState::IDLE};
    Eigen::Isometry3d map_odom_transform_ = Eigen::Isometry3d::Identity();

    // --- manual init ---
    std::atomic<bool> has_manual_init_{false};
    Eigen::Isometry3d manual_init_pose_ = Eigen::Isometry3d::Identity();
    Mat6d manual_init_cov_ = Mat6d::Identity();

    // --- FLANN KD-tree index on RingKey (high-dim) ---
    int rk_dim_ = 0;
    size_t indexed_size_ = 0;
    std::vector<float> ringkey_dataset_; // contiguous [N x dim]
    flann::Matrix<float> ringkey_dataset_mat_;
    std::unique_ptr<flann::Index<flann::L2<float>>> ringkey_index_;
    int new_kf_since_rebuild_ = 0;

    // helpers
    void rebuildIndexLocked();
    std::vector<int> queryRingKeyCandidatesLocked(const ScanContext::RingKey& rk) const;

    ScanContext::PointCloudType::Ptr buildLocalSubmapLocked(int center_idx) const;
    ScanContext::PointCloudType::Ptr voxelDownsample(const ScanContext::PointCloudType::Ptr& in, double leaf) const;

    bool runICP(const ScanContext::PointCloudType::Ptr& source_base,
                const ScanContext::PointCloudType::Ptr& target_map,
                const Eigen::Isometry3d& init_map_T_base,
                Eigen::Isometry3d& refined_map_T_base,
                double& fitness) const;
};

} // namespace scantext
