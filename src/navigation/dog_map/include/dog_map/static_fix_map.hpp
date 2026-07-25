#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sys/types.h>

#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
namespace static_fix_map {
class StaticFixMap {
   public:
    StaticFixMap();
    ~StaticFixMap() = default;
    bool InitPtr(std::shared_ptr<cv::Mat> pgm_ptr, uint8_t* occ_map_ptr,
                 int layer_z, int insert_z);
    void SetMapInfo(double origin_x, double origin_y, double origin_z,
                    double resolution) {
        map_info_.origin_x = origin_x;
        map_info_.origin_y = origin_y;
        map_info_.origin_z = origin_z;
        map_info_.resolution_ = resolution;
    }
    void SetConsiderLength(double consider_length) {
        consider_length_ = consider_length;
    }
    void AdjustAndFix(double robot_x, double robot_y,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void AdjustAndFixByRacying2d(double robot_x, double robot_y,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
    bool IsOccEdige(int idx, int idy);

   private:
    uint8_t* occ_map_ptr_ = nullptr;
    std::shared_ptr<cv::Mat> pgm_ptr_;
    double consider_length_ = 2.5;  // 机器人前后左右
    int layer_z_ = -1;
    int insert_z_ = 0;
    struct {
        double origin_x;
        double origin_y;
        double origin_z;
        double resolution_ = 0.05;
    } map_info_;

    // std::vector<std::pair<int, int>> moves = {
    //     {0, 1},    // 上
    //     {0, -1},   // 下
    //     {-1, 0},   // 左
    //     {1, 0},    // 右
    //     {-1, 1},   // 左上
    //     {1, 1},    // 右上
    //     {-1, -1},  // 左下
    //     {1, -1}    // 右下
    // };
};
}  // namespace static_fix_map
