#include <oneapi/tbb/parallel_for.h>

#include <algorithm>
#include <dog_map/static_fix_map.hpp>
#include <iostream>
namespace static_fix_map {
StaticFixMap::StaticFixMap() {
    std::cout << " -- [DOG_MAP] StaticFixMap constructor called." << std::endl;
}

bool StaticFixMap::InitPtr(std::shared_ptr<cv::Mat> pgm_ptr,
                           uint8_t* occ_map_ptr, int layer_z, int insert_z) {
    pgm_ptr_ = pgm_ptr;
    occ_map_ptr_ = occ_map_ptr;
    layer_z_ = layer_z;
    insert_z_ = insert_z;
    return true;
}

void StaticFixMap::AdjustAndFix(double robot_x, double robot_y,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (!pgm_ptr_ || !occ_map_ptr_) {
        std::cerr << " -- [DOG_MAP] StaticFixMap not initialized properly."
                  << std::endl;
        return;
    }
    int robot_px = static_cast<int>((robot_x - map_info_.origin_x) /
                                    map_info_.resolution_);
    int robot_py = static_cast<int>((robot_y - map_info_.origin_y) /
                                    map_info_.resolution_);
    int consider_cells =
        static_cast<int>(consider_length_ / map_info_.resolution_);
    int min_px = std::max(0, robot_px - consider_cells);
    int max_px = std::min(pgm_ptr_->cols - 1, robot_px + consider_cells);
    int min_py = std::max(0, robot_py - consider_cells);
    int max_py = std::min(pgm_ptr_->rows - 1, robot_py + consider_cells);
    layer_z_ = std::min(layer_z_, insert_z_ + 20);
    for (int i = min_py; i <= max_py; ++i) {
        for (int j = min_px; j <= max_px; ++j) {
            //遍历,对于那些pgm上是占据,但是occ_map是空闲的点,我们加入cloud中,同时,把pgm上的占据点改为0,或许可以使用TBB并行化加速
            uint8_t pixel_value =
                pgm_ptr_->at<uint8_t>(pgm_ptr_->rows - 1 - i, j);
            if (pixel_value < 250) {  // 阈值，255 是空闲，0 是障碍物
                // if (!IsOccEdige(j, i)) continue;  // 不是边缘点，跳过
                bool is_occupied = false;
                bool have_view = false;
                for (int idz = insert_z_; idz < layer_z_; idz++) {
                    int index = j + i * pgm_ptr_->cols +
                                idz * pgm_ptr_->cols * pgm_ptr_->rows;
                    if (occ_map_ptr_[index] == 0) continue;
                    have_view = true;
                    if (occ_map_ptr_[index] >
                        20) {  // 已经被占据,目前条件比较严格
                        is_occupied = true;
                        // std::cout << " -- [DOG_MAP] StaticFixMap: Point (" <<
                        // j << ", " << i
                        //           << ") is occupied in occ_map, skipping." <<
                        //           std::endl;
                        break;
                    }
                }
                if (!is_occupied && have_view) {
                    // 将该点加入点云
                    double wx =
                        map_info_.origin_x + (j + 0.5) * map_info_.resolution_;
                    double wy =
                        map_info_.origin_y + (i + 0.5) * map_info_.resolution_;
                    cloud->points.emplace_back(wx, wy, map_info_.origin_z);
                    // 将 pgm 上的占据点改为 255（空闲）
                    //   pgm_ptr_->at<uint8_t>(pgm_ptr_->rows - 1 - i, j) = 255;
                    //   std::cout << " -- [DOG_MAP] StaticFixMap: Added point
                    //   (" << wx << ", " << wy
                    //             << ") to cloud and updated pgm." <<
                    //             std::endl;
                }
            }
        }
    }
}
bool StaticFixMap::IsOccEdige(int idx, int idy) {
    // for (const auto& move : moves) {
    //     int new_idx = idx + move.first;
    //     int new_idy = idy + move.second;
    //     if (new_idx < 0 || new_idx >= pgm_ptr_->cols || new_idy < 0 ||
    //         new_idy >= pgm_ptr_->rows) {
    //         continue;  // 越界的点跳过
    //     }
    //     uint8_t pixel_value =
    //         pgm_ptr_->at<uint8_t>(pgm_ptr_->rows - 1 - new_idy, new_idx);
    //     if (pixel_value >= 250) {  // 阈值，255 是空闲，0 是障碍物
    //         return true;           // 周围有空闲点，说明是边缘点
    //     }
    // }
    return false;  // 周围都是占据点，不是边缘点
}
void StaticFixMap::AdjustAndFixByRacying2d(
    double robot_x, double robot_y,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
    // TODO
    // 利用2d射线处理思路,对cloud_in进行筛选,对那些位于考虑范围内的点进行以他为终点的xy射线处理,对路径中的pgm为占据的栅格,重写为free(255)并加入到cloud_out中
    if (!pgm_ptr_ || !cloud_in || !cloud_out) {
        return;
    }

    int robot_px = static_cast<int>((robot_x - map_info_.origin_x) /
                                    map_info_.resolution_);
    int robot_py = static_cast<int>((robot_y - map_info_.origin_y) /
                                    map_info_.resolution_);
    int consider_cells =
        static_cast<int>(consider_length_ / map_info_.resolution_);
    int min_px = std::max(0, robot_px - consider_cells);
    int max_px = std::min(pgm_ptr_->cols - 1, robot_px + consider_cells);
    int min_py = std::max(0, robot_py - consider_cells);
    int max_py = std::min(pgm_ptr_->rows - 1, robot_py + consider_cells);
    //射线边界
    for (const auto& pt : cloud_in->points) {
        int pt_px = static_cast<int>((pt.x - map_info_.origin_x) /
                                     map_info_.resolution_);
        int pt_py = static_cast<int>((pt.y - map_info_.origin_y) /
                                     map_info_.resolution_);
        if (pt_px < min_px || pt_px > max_px || pt_py < min_py ||
            pt_py > max_py) {
            continue;  // 超出考虑范围，跳过
        }
        // Bresenham 2D 画线算法进行射线检测 (从 robot 连线到 point)
        int x0 = robot_px;
        int y0 = robot_py;
        int x1 = pt_px;
        int y1 = pt_py;

        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;
        int free_count = 0;
        while (true) {
            // 边界检查
            if (x0 >= min_px && x0 < max_px && y0 >= min_py && y0 < max_py) {
                int pgm_y = pgm_ptr_->rows - 1 - y0;  // PGM的Y轴是反的
                uint8_t& pixel = pgm_ptr_->at<uint8_t>(pgm_y, x0);

                // 遇到占据点 (< 200 避开 205未知区域)
                if (pixel < 200) {
                    // 重写为 free
                    // pixel = 255;
                    free_count++;
                    double wx =
                        map_info_.origin_x + (x0 + 0.5) * map_info_.resolution_;
                    double wy =
                        map_info_.origin_y + (y0 + 0.5) * map_info_.resolution_;
                    cloud_out->points.emplace_back(wx, wy, map_info_.origin_z);
                    if (free_count > 3) {
                        break;  // 理论上车辆误差不会达到15cm以上吧
                    }
                }
            }

            if (x0 == x1 && y0 == y1) break;
            e2 = 2 * err;
            if (e2 >= dy) {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y0 += sy;
            }
        }
    }
}
}  // namespace static_fix_map
