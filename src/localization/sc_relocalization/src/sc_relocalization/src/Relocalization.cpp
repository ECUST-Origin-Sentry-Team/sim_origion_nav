// Relocalization.cpp

#include "scantext_module/Relocalization.hpp"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <unordered_map>

namespace scantext {

RelocalizationCore::RelocalizationCore()
    : ringkey_dataset_mat_(nullptr, 0, 0)
{
}

void RelocalizationCore::setConfig(const Config& config) {
    std::lock_guard<std::mutex> lock(relo_mutex_);
    config_ = config;
}

void RelocalizationCore::setScanContextParams(const SCParams& params) {
    std::lock_guard<std::mutex> lock(relo_mutex_);
    scan_context_ = ScanContext(params);
}

void RelocalizationCore::setMap(const std::vector<std::shared_ptr<KeyFrame>>& keyframes) {
    std::lock_guard<std::mutex> lock(relo_mutex_);
    
    // Robust check for invalid pointers
    map_keyframes_.clear();
    map_keyframes_.reserve(keyframes.size());
    
    for(const auto& kf : keyframes) {
        if(kf && kf->cloud && !kf->cloud->empty() && kf->ring_key.size() > 0) {
            map_keyframes_.push_back(kf);
        }
    }
    
    if (map_keyframes_.empty()) {
        std::cerr << "[Relocalization] Warning: setMap received empty or invalid keyframes!" << std::endl;
        new_kf_since_rebuild_ = 0;
        rebuildIndexLocked();
        return;
    }

    new_kf_since_rebuild_ = static_cast<int>(map_keyframes_.size()); // force rebuild
    rebuildIndexLocked();
}

void RelocalizationCore::resetRelocalization() {
    state_ = ReloState::REQUESTED;
    is_localized_ = false;
    has_manual_init_ = false;
    std::cout << "[Relocalization] State reset to REQUESTED." << std::endl;
}

void RelocalizationCore::addKeyFrame(const std::shared_ptr<KeyFrame>& kf) {
    if (!kf) return;
    std::lock_guard<std::mutex> lock(relo_mutex_);
    map_keyframes_.push_back(kf);
    new_kf_since_rebuild_++;

    // 累积到一定数量再重建 KD-tree，避免每帧重建
    if (new_kf_since_rebuild_ >= config_.rebuild_index_every_n) {
        rebuildIndexLocked();
    }
}

void RelocalizationCore::rebuildIndexLocked() {
    rk_dim_ = scan_context_.getParams().num_ring;

    const size_t N = map_keyframes_.size();
    if (N == 0 || rk_dim_ <= 0) {
        ringkey_index_.reset();
        ringkey_dataset_.clear();
        ringkey_dataset_mat_ = flann::Matrix<float>(nullptr, 0, 0);
        indexed_size_ = 0;
        new_kf_since_rebuild_ = 0;
        return;
    }

    ringkey_dataset_.resize(N * rk_dim_);

    for (size_t i = 0; i < N; ++i) {
        const auto& rk = map_keyframes_[i]->ring_key;
        if (rk.size() != rk_dim_) {
            // 维度不一致直接跳（也可以强制 resize/重算）
            for (int d = 0; d < rk_dim_; ++d) ringkey_dataset_[i * rk_dim_ + d] = 0.0f;
            continue;
        }
        for (int d = 0; d < rk_dim_; ++d) {
            ringkey_dataset_[i * rk_dim_ + d] = static_cast<float>(rk[d]);
        }
    }

    ringkey_dataset_mat_ = flann::Matrix<float>(ringkey_dataset_.data(), N, rk_dim_);
    ringkey_index_.reset(new flann::Index<flann::L2<float>>(ringkey_dataset_mat_, flann::KDTreeIndexParams(4)));
    ringkey_index_->buildIndex();

    indexed_size_ = N;
    new_kf_since_rebuild_ = 0;

    std::cout << "[Relocalization] Rebuilt RingKey KD-tree: N=" << N << " dim=" << rk_dim_ << std::endl;
}

std::vector<int> RelocalizationCore::queryRingKeyCandidatesLocked(const ScanContext::RingKey& rk) const {
    std::vector<int> out;

    const size_t N = map_keyframes_.size();
    if (N == 0) return out;

    const int topK = std::min(config_.ringkey_top_k, static_cast<int>(N));
    if (topK <= 0) return out;

    // 如果 KD-tree 不可用，退化线性
    if (!ringkey_index_ || indexed_size_ == 0 || rk.size() != rk_dim_) {
        std::vector<std::pair<double,int>> all;
        all.reserve(N);
        for (size_t i = 0; i < N; ++i) {
            const double dist = (map_keyframes_[i]->ring_key - rk).norm();
            all.push_back({dist, static_cast<int>(i)});
        }
        std::nth_element(all.begin(), all.begin() + topK, all.end());
        all.resize(topK);
        std::sort(all.begin(), all.end());
        for (auto& p : all) out.push_back(p.second);
        return out;
    }

    // KD-tree 查询
    std::vector<float> query(rk_dim_);
    for (int d = 0; d < rk_dim_; ++d) query[d] = static_cast<float>(rk[d]);

    flann::Matrix<float> query_mat(query.data(), 1, rk_dim_);

    std::vector<int> indices(topK);
    std::vector<float> dists(topK);
    flann::Matrix<int> indices_mat(indices.data(), 1, topK);
    flann::Matrix<float> dists_mat(dists.data(), 1, topK);

    ringkey_index_->knnSearch(query_mat, indices_mat, dists_mat, topK, flann::SearchParams(32));

    out.reserve(topK);
    for (int i = 0; i < topK; ++i) {
        if (indices[i] >= 0 && static_cast<size_t>(indices[i]) < indexed_size_) out.push_back(indices[i]);
    }

    // 如果 KD-tree 没包含最新那一段（indexed_size_ < N），再线性补一段
    if (indexed_size_ < N) {
        std::vector<std::pair<double,int>> extra;
        for (size_t i = indexed_size_; i < N; ++i) {
            const double dist = (map_keyframes_[i]->ring_key - rk).norm();
            extra.push_back({dist, static_cast<int>(i)});
        }
        // 合并进 out（简单做法：拼起来后再截断 topK）
        std::vector<std::pair<double,int>> merged;
        merged.reserve(out.size() + extra.size());
        for (int idx : out) {
            double dist = (map_keyframes_[idx]->ring_key - rk).norm();
            merged.push_back({dist, idx});
        }
        for (auto& e : extra) merged.push_back(e);

        std::nth_element(merged.begin(), merged.begin() + topK, merged.end());
        merged.resize(topK);
        std::sort(merged.begin(), merged.end());
        out.clear();
        for (auto& p : merged) out.push_back(p.second);
    }

    return out;
}

ScanContext::PointCloudType::Ptr RelocalizationCore::voxelDownsample(
    const ScanContext::PointCloudType::Ptr& in, double leaf) const
{
    if (!in) return in;
    if (leaf <= 1e-6) return in;

    pcl::VoxelGrid<ScanContext::PointType> vg;
    vg.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
    vg.setInputCloud(in);

    auto out = std::make_shared<ScanContext::PointCloudType>();
    vg.filter(*out);
    return out;
}

ScanContext::PointCloudType::Ptr RelocalizationCore::buildLocalSubmapLocked(int center_idx) const {
    if (center_idx < 0 || center_idx >= static_cast<int>(map_keyframes_.size())) return nullptr;

    const int N = static_cast<int>(map_keyframes_.size());
    const int win = config_.local_submap_kf_num;

    const int s = std::max(0, center_idx - win);
    const int e = std::min(N - 1, center_idx + win);

    auto submap = std::make_shared<ScanContext::PointCloudType>();
    submap->reserve(500000);

    for (int i = s; i <= e; ++i) {
        const auto& kf = map_keyframes_[i];
        if (!kf || !kf->cloud || kf->cloud->empty()) continue;

        ScanContext::PointCloudType tmp;
        const Eigen::Isometry3d pose_for_submap = projectTo4DoF(kf->pose);
        pcl::transformPointCloud(*kf->cloud, tmp, pose_for_submap.matrix());
        *submap += tmp;
    }

    // 降采样
    submap = voxelDownsample(submap, config_.local_map_voxel_leaf);
    return submap;
}

bool RelocalizationCore::runICP(const ScanContext::PointCloudType::Ptr& source_base,
                                const ScanContext::PointCloudType::Ptr& target_map,
                                const Eigen::Isometry3d& init_map_T_base,
                                Eigen::Isometry3d& refined_map_T_base,
                                double& fitness) const
{
    if (!source_base || !target_map) return false;
    if (source_base->empty() || target_map->empty()) return false;

    auto src = voxelDownsample(source_base, config_.query_voxel_leaf);
    auto tgt = voxelDownsample(target_map, config_.local_map_voxel_leaf);
    if (!src || !tgt || src->size() < 20 || tgt->size() < 20) return false;

    auto src_cov = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZI>, pcl::PointCloud<pcl::PointCovariance>>(
        *src, config_.query_voxel_leaf);
    auto tgt_cov = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZI>, pcl::PointCloud<pcl::PointCovariance>>(
        *tgt, config_.local_map_voxel_leaf);
    if (!src_cov || !tgt_cov || src_cov->size() < 20 || tgt_cov->size() < 20) return false;

    const int num_threads = std::max(1, config_.gicp_num_threads);
    const int num_neighbors = std::max(5, config_.gicp_num_neighbors);

    small_gicp::estimate_covariances_omp(*src_cov, num_neighbors, num_threads);
    small_gicp::estimate_covariances_omp(*tgt_cov, num_neighbors, num_threads);

    auto tgt_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        tgt_cov, small_gicp::KdTreeBuilderOMP(num_threads));

    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> gicp;
    gicp.reduction.num_threads = num_threads;
    gicp.rejector.max_dist_sq = config_.icp_max_corr_dist * config_.icp_max_corr_dist;
    gicp.optimizer.max_iterations = config_.icp_max_iter;

    const Eigen::Isometry3d init = projectTo4DoF(init_map_T_base);
    auto result = gicp.align(*tgt_cov, *src_cov, *tgt_tree, init);
    if (!result.converged) return false;

    const double denom = static_cast<double>(std::max<size_t>(1, result.num_inliers));
    fitness = result.error / denom;
    refined_map_T_base = projectTo4DoF(result.T_target_source);
    return true;
}

Eigen::Isometry3d RelocalizationCore::projectTo4DoF(const Eigen::Isometry3d& pose) const {
    if (!config_.use_4dof) {
        return pose;
    }

    const double yaw = std::atan2(pose.rotation()(1, 0), pose.rotation()(0, 0));

    Eigen::Isometry3d projected = Eigen::Isometry3d::Identity();
    projected.translation() = pose.translation();
    projected.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    if (config_.gravity_align) {
        // Gravity-aligned mode currently enforces zero roll/pitch in map frame.
        // Hook kept to support external gravity direction injection in the future.
        return projected;
    }

    return projected;
}

bool RelocalizationCore::updateFromSC(const ScanContext::SCDescriptor& sc,
                                     const ScanContext::RingKey& rk,
                                     const ScanContext::PointCloudType::Ptr& cloud_ds,
                                     const Eigen::Isometry3d& odom_pose,
                                     double /*time*/,
                                     const Eigen::Vector3d& /*velocity*/)
{
    std::lock_guard<std::mutex> lock(relo_mutex_);

    if (map_keyframes_.empty()) return false;
    if (!cloud_ds || cloud_ds->empty()) return false;

    // 0) 手动 initialpose：优先用 ICP 在手动初值附近收敛一次
    if (has_manual_init_) {
        // 找最近 keyframe 当中心（简单做法：线性找最近位置）
        int best = -1;
        double best_d = 1e18;
        for (int i = 0; i < (int)map_keyframes_.size(); ++i) {
            double d = (map_keyframes_[i]->pose.translation() - manual_init_pose_.translation()).norm();
            if (d < best_d) { best_d = d; best = i; }
        }

        auto submap = buildLocalSubmapLocked(best);
        Eigen::Isometry3d refined;
        double fit = 1e9;
        bool ok = runICP(cloud_ds, submap, projectTo4DoF(manual_init_pose_), refined, fit);

        if (ok && fit < config_.icp_fitness_thresh) {
            map_odom_transform_ = refined * odom_pose.inverse();
            is_localized_ = true;
            has_manual_init_ = false;
            return true;
        } else {
            // 手动初值没收敛，不直接判丢，继续走 SC 流程
            has_manual_init_ = false;
        }
    }

    // 1) RingKey KD-tree 获取候选
    auto cand_idx = queryRingKeyCandidatesLocked(rk);
    if (cand_idx.empty()) return false;

    // 2) 对候选做 SC 精匹配（带 shift）
    std::vector<Candidate> cands;
    cands.resize(cand_idx.size()); // Pre-allocate for OpenMP

    const int num_sector = scan_context_.getParams().num_sector;

    // OpenMP parallelization
    #pragma omp parallel for num_threads(2) schedule(dynamic)
    for (int i = 0; i < (int)cand_idx.size(); ++i) {
        int idx = cand_idx[i];
        // Note: map_keyframes_ is read-only here, safe for parallel read
        // But verify kf is valid (though checked in setMap/addKeyFrame)
        const auto& kf = map_keyframes_[idx];
        if (!kf) continue;

        auto res = scan_context_.distanceBtnScanContext(kf->descriptor, sc);
        const double sc_dist = res.first;
        const int shift = res.second;

        if (sc_dist > config_.sc_dist_thresh) {
            cands[i].idx = -1; // Mark invalid
            continue;
        }

        // shift -> yaw_diff（注意符号：这里用 -shift 更稳）
        const double yaw_diff = -static_cast<double>(shift) * 2.0 * M_PI / static_cast<double>(num_sector);
        Eigen::AngleAxisd Rz(yaw_diff, Eigen::Vector3d::UnitZ());

        Candidate c;
        c.idx = idx;
        c.sc_dist = sc_dist;
        c.match_dist = sc_dist;
        c.shift = shift;
        c.init_pose = projectTo4DoF(kf->pose * Eigen::Isometry3d(Rz)); // map_T_base 初值（平移用 keyframe 平移）
        cands[i] = c;
    }

    // Filter invalid candidates
    std::vector<Candidate> valid_cands;
    valid_cands.reserve(cands.size());
    for(const auto& c : cands) {
        if(c.idx != -1) valid_cands.push_back(c);
    }
    cands = std::move(valid_cands);

    if (cands.empty()) {
        state_ = ReloState::FAIL;
        return false;
    }

    std::sort(cands.begin(), cands.end(), [](const Candidate& a, const Candidate& b){
        return a.sc_dist < b.sc_dist;
    });

    const int scTopK = std::max(1, std::min(config_.sc_top_k, static_cast<int>(cands.size())));
    cands.resize(scTopK);

    if (config_.use_cart_context) {
        std::unordered_map<int, ScanContext::CartDescriptor> query_cart_cache;
        query_cart_cache.reserve(cands.size());
        for (auto& c : cands) {
            const auto& kf = map_keyframes_[c.idx];
            if (!kf || !cloud_ds) {
                continue;
            }

            const double yaw_diff = -static_cast<double>(c.shift) * 2.0 * M_PI / static_cast<double>(num_sector);
            auto it = query_cart_cache.find(c.shift);
            if (it == query_cart_cache.end()) {
                it = query_cart_cache.emplace(c.shift, scan_context_.makeCartContext(*cloud_ds, yaw_diff)).first;
            }
            c.cart_dist = scan_context_.distanceBtnCartContext(kf->cart_descriptor, it->second);
            if (std::isfinite(c.cart_dist)) {
                c.match_dist = (1.0 - config_.cart_weight) * c.sc_dist + config_.cart_weight * c.cart_dist;
            }
        }

        std::stable_sort(cands.begin(), cands.end(), [](const Candidate& a, const Candidate& b){
            return a.match_dist < b.match_dist;
        });
    }

    std::vector<Candidate> diverse_cands;
    diverse_cands.reserve(cands.size());
    for (const auto& cand : cands) {
        bool separated = true;
        if (config_.min_candidate_separation > 0.0) {
            for (const auto& kept : diverse_cands) {
                const double dist = (map_keyframes_[cand.idx]->pose.translation() -
                                     map_keyframes_[kept.idx]->pose.translation()).norm();
                if (dist < config_.min_candidate_separation) {
                    separated = false;
                    break;
                }
            }
        }

        if (separated || diverse_cands.empty()) {
            diverse_cands.push_back(cand);
        }
    }
    cands = std::move(diverse_cands);

    // 3) 取 topM 做 ICP refine（只做少量，保证快）
    const int topM = std::min(config_.icp_top_k, static_cast<int>(cands.size()));
    Candidate best = cands[0];
    
    // Only perform ICP if we have a valid cloud and candidates
    std::cout << "[Relocalization] Starting ICP on top " << topM << " candidates..." << std::endl;

    state_ = ReloState::RUNNING;

    for (int i = 0; i < topM; ++i) {
        auto& c = cands[i];
        auto submap = buildLocalSubmapLocked(c.idx);
        if (!submap || submap->empty()) continue;

        Eigen::Isometry3d refined;
        double fit = 1e9;
        bool ok = runICP(cloud_ds, submap, c.init_pose, refined, fit);
        c.icp_ok = ok;
        c.icp_fitness = fit;
        c.refined_pose = refined;

        if (ok && fit < best.icp_fitness) {
            best = c;
        }
    }

    // ICP 成功且 fitness 合格
    if (best.icp_ok && best.icp_fitness < config_.icp_fitness_thresh) {
        map_odom_transform_ = best.refined_pose * odom_pose.inverse();
        is_localized_ = true;
        state_ = ReloState::SUCC;

        // Print success info
        auto t = best.refined_pose.translation();
        auto r = best.refined_pose.rotation().eulerAngles(0, 1, 2);
        std::cout << "[Relocalization] ICP succeeded, score=" << best.icp_fitness
                  << ", pose=(" << t.x() << "," << t.y() << "," << t.z()
                  << "," << r.x() << "," << r.y() << "," << r.z() << ")" << std::endl;

        return true;
    }

    // 4) 如果没有 cloud 或 ICP 不收敛：退化用 SC 的 init_pose（精度差，但可用）
    //    你也可以选择直接 return false（更保守）
    // map_odom_transform_ = best.init_pose * odom_pose.inverse();
    // is_localized_ = true;
    // return true;
    
    state_ = ReloState::FAIL;
    return false;
}

bool RelocalizationCore::update(const ScanContext::PointCloudType::Ptr& cloud,
                               const Eigen::Isometry3d& odom_pose,
                               double time,
                               const Eigen::Vector3d& velocity)
{
    // wrapper：旧接口内部算 SC
    if (!cloud) return false;
    auto sc = scan_context_.makeScanContext(*cloud);
    auto rk = scan_context_.makeRingKey(sc);
    return updateFromSC(sc, rk, cloud, odom_pose, time, velocity);
}

void RelocalizationCore::setInitialPose(const Eigen::Isometry3d& pose, const Mat6d& cov) {
    std::lock_guard<std::mutex> lock(relo_mutex_);
    manual_init_pose_ = pose;
    manual_init_cov_ = cov;
    has_manual_init_ = true;
    is_localized_ = false;
}

} // namespace scantext
