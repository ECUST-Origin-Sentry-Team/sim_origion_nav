#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>

class RegionBehaviorNode : public rclcpp::Node
{
public:
    RegionBehaviorNode() : Node("region_behavior_node")
    {
        this->declare_parameter("map_frame_id", "map");
        this->get_parameter("map_frame_id", map_frame_id_);
        this->declare_parameter("base_link_frame_id", "base_link");
        this->get_parameter("base_link_frame_id", base_link_frame_id_);

        std::string pkg_dir = ament_index_cpp::get_package_share_directory("region_behavior");
        std::string default_path = pkg_dir + "/config/1.json";
        this->declare_parameter("json_path", default_path);
        this->get_parameter("json_path", json_path_);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        if (!load_regions(json_path_))
        {
            RCLCPP_ERROR(this->get_logger(), "区域配置加载失败，节点退出。");
            rclcpp::shutdown();
            return;
        }

        region_int_pub_ = this->create_publisher<std_msgs::msg::Int32>("/region_int", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&RegionBehaviorNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "节点启动完成。");
    }

private:
    struct Region
    {
        std::string id;
        int type;
        std::vector<geometry_msgs::msg::Point> points;
    };

    std::vector<Region> regions_;
    std::string map_frame_id_;
    std::string base_link_frame_id_;
    std::string json_path_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr region_int_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Point curr_pos_;

    bool load_regions(const std::string &json_path)
    {
        std::ifstream file(json_path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开JSON文件: %s", json_path.c_str());
            return false;
        }

        nlohmann::json j;
        file >> j;

        for (const auto &region_json : j["regions"])
        {
            Region region;
            region.id = region_json["id"].get<std::string>();
            region.type = region_json["type"].get<int>();

            for (const auto &p : region_json["points"])
            {
                geometry_msgs::msg::Point pt;
                pt.x = p["x"].get<double>();
                pt.y = p["y"].get<double>();
                pt.z = p["z"].get<double>();
                region.points.push_back(pt);
            }

            regions_.push_back(region);
            RCLCPP_INFO(this->get_logger(), "加载区域成功: %s (顶点数: %zu)",
                        region.id.c_str(), region.points.size());
        }

        return !regions_.empty();
    }

    bool is_in_region(const geometry_msgs::msg::Point &pt, const Region &region)
    {
        bool inside = false;
        size_t n = region.points.size();

        for (size_t i = 0; i < n; ++i)
        {
            const auto &a = region.points[i];
            const auto &b = region.points[(i + 1) % n];

            // 判断点是否在边界上
            if ((pt.x == a.x && pt.y == a.y) ||
                (pt.x == b.x && pt.y == b.y))
                return true;

            if (((a.y > pt.y) != (b.y > pt.y)) &&
                (pt.x < (b.x - a.x) * (pt.y - a.y) / (b.y - a.y) + a.x))
            {
                inside = !inside;
            }
        }
        return inside;
    }

    int get_region_int(const geometry_msgs::msg::Point &pt)
    {
        for (const auto &region : regions_)
        {
            if (is_in_region(pt, region))
            {
                std::cout<<"region" <<region.id<<std::endl;
                return region.type;
            }
        }
        return -1;
    }

    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf = tf_buffer_->lookupTransform(map_frame_id_, base_link_frame_id_, tf2::TimePointZero);
            curr_pos_.x = tf.transform.translation.x;
            curr_pos_.y = tf.transform.translation.y;
            curr_pos_.z = tf.transform.translation.z;
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "TF查询失败: %s", e.what());
            return;
        }

        int region_int = get_region_int(curr_pos_);
        auto msg = std_msgs::msg::Int32();
        msg.data = region_int;
        region_int_pub_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RegionBehaviorNode>());
    rclcpp::shutdown();
    return 0;
}
