#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "region_behavior/srv/get_region.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class LocalPlanRegionChecker : public rclcpp::Node
{
public:
    LocalPlanRegionChecker() : Node("local_plan_region_checker")
    {
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = callback_group_;
        
        sub_plan_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10,
            std::bind(&LocalPlanRegionChecker::planCallback, this, std::placeholders::_1),
            sub_opts);

        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/region_debug_markers", 10);

        client_get_region_ = this->create_client<region_behavior::srv::GetRegion>(
            "/region_behavior/get_region",
            rmw_qos_profile_services_default,
            callback_group_);
            
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_plan_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Client<region_behavior::srv::GetRegion>::SharedPtr client_get_region_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    void planCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty() || msg->poses.size() < 2) return;

        std::vector<geometry_msgs::msg::Point> sampled_points;
        double target_dists[] = {0.0, 2.4, 4.8, 7.2, 9.6, 12.0};
        size_t dist_idx = 0;
        
        // Add start point
        sampled_points.push_back(msg->poses[0].pose.position);
        dist_idx++;

        double current_path_len = 0.0;
        for (size_t i = 0; i < msg->poses.size() - 1 && dist_idx < 6; ++i)
        {
            auto p1 = msg->poses[i].pose.position;
            auto p2 = msg->poses[i+1].pose.position;
            double seg_len = std::hypot(p2.x - p1.x, p2.y - p1.y);

            while (dist_idx < 6)
            {
                double target = target_dists[dist_idx];
                if (target <= current_path_len + seg_len)
                {
                    double ratio = (seg_len > 1e-6) ? (target - current_path_len) / seg_len : 0.0;
                    geometry_msgs::msg::Point pt;
                    pt.x = p1.x + (p2.x - p1.x) * ratio;
                    pt.y = p1.y + (p2.y - p1.y) * ratio;
                    pt.z = p1.z + (p2.z - p1.z) * ratio;
                    sampled_points.push_back(pt);
                    dist_idx++;
                }
                else
                {
                    break;
                }
            }
            current_path_len += seg_len;
        }

        // Visualize
        visualization_msgs::msg::MarkerArray ma;
        int id = 0;
        for (const auto& pt : sampled_points)
        {
            visualization_msgs::msg::Marker mk;
            mk.header = msg->header;
            mk.ns = "check_points";
            mk.id = id++;
            mk.type = visualization_msgs::msg::Marker::SPHERE;
            mk.action = visualization_msgs::msg::Marker::ADD;
            mk.pose.position = pt;
            mk.pose.orientation.w = 1.0;
            mk.scale.x = 0.2; mk.scale.y = 0.2; mk.scale.z = 0.2;
            mk.color.a = 1.0; mk.color.r = 1.0; mk.color.g = 0.0; mk.color.b = 0.0;
            ma.markers.push_back(mk);
        }
        pub_markers_->publish(ma);

        if (sampled_points.empty()) return;

        // Transform to map frame
        std::vector<geometry_msgs::msg::Point> map_points;
        std::string target_frame = "map"; 
        
        try {
            // Check if transform is available, if not, skip or use raw points if frames match
            if (msg->header.frame_id != target_frame) {
                // Wait for transform? 
                // We are in a callback, wait might block too long, but we have 200ms budget for service, maybe strict on TF?
                // Just try lookup.
                if (!tf_buffer_->canTransform(target_frame, msg->header.frame_id, tf2::TimePointZero)) {
                    RCLCPP_WARN(this->get_logger(), "Cannot transform from %s to %s", msg->header.frame_id.c_str(), target_frame.c_str());
                    return; 
                }
            }

            for (const auto& pt : sampled_points) {
                if (msg->header.frame_id == target_frame) {
                    map_points.push_back(pt);
                } else {
                    geometry_msgs::msg::PointStamped pt_stamped;
                    pt_stamped.header = msg->header;
                    pt_stamped.point = pt;
                    auto transformed = tf_buffer_->transform(pt_stamped, target_frame);
                    map_points.push_back(transformed.point);
                }
            }
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "TF Exception: %s", ex.what());
            return;
        }

        // Service Call
        auto request = std::make_shared<region_behavior::srv::GetRegion::Request>();
        request->locations = map_points;

        int retries = 2;
        bool success = false;
        
        for (int i = 0; i <= retries; ++i)
        {
             if (!client_get_region_->service_is_ready()) {
                 // Maybe wait a bit?
                 if (i < retries) {
                    rclcpp::sleep_for(std::chrono::milliseconds(10)); // Slight delay
                    continue; 
                 }
                 else break;
             }

             auto result_future = client_get_region_->async_send_request(request);
             if (result_future.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready)
             {
                 try {
                     auto response = result_future.get();
                     for (size_t k = 0; k < response->region_ids.size(); ++k)
                     {
                         if (response->region_ids[k] != -1)
                         {
                             RCLCPP_INFO(this->get_logger(), "Approaching Region ID: %d", response->region_ids[k]);
                             // Don't return yet, check all? "if one ... in specified region" -> output.
                             // Maybe we want to know WHICH one?
                             // I'll output and return, or output all?
                             // "if one of six ... output diagnostic info"
                             success = true;
                             return; 
                         }
                     }
                     success = true;
                     return;
                 } catch (const std::exception &e) {
                     RCLCPP_WARN(this->get_logger(), "Service call exception: %s", e.what());
                 }
             }
             else
             {
                 RCLCPP_WARN(this->get_logger(), "Service call timed out (attempt %d)", i + 1);
             }
        }
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get region info after retries.");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalPlanRegionChecker>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
