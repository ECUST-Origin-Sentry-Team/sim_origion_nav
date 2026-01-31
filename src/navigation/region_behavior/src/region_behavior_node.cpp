#include "region_behavior/region_behavior_node.hpp"

namespace region_behavior
{

RegionBehaviorNode::RegionBehaviorNode() : Node("region_behavior_node")
{
    this->declare_parameter("map_frame_id", "map");
    this->get_parameter("map_frame_id", map_frame_id_);
    this->declare_parameter("base_link_frame_id", "base_link");
    this->get_parameter("base_link_frame_id", base_link_frame_id_);

    std::string pkg_dir = ament_index_cpp::get_package_share_directory("region_behavior");
    std::string default_path = pkg_dir + "/config/regions.yaml";
    this->declare_parameter("yaml_path", default_path);
    this->get_parameter("yaml_path", yaml_path_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    if (!load_regions(yaml_path_))
    {
        RCLCPP_ERROR(this->get_logger(), "区域配置加载失败，节点退出。");
        rclcpp::shutdown();
        return;
    }

    region_int_pub_ = this->create_publisher<std_msgs::msg::Int32>("/region_int", 10);
    gimbal_cmd_pub_ = this->create_publisher<rm_interfaces::msg::GimbalRegionCmd>("/serial/gimbal_region_cmd", 10);
    uphill_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/serial/uphill", 10);
    speed_limit_pub_ = this->create_publisher<nav2_msgs::msg::SpeedLimit>(
        "/speed_limit", 10);
    arrow_pub = this->create_publisher<visualization_msgs::msg::Marker>(
        "/direction_arrow", 1);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&RegionBehaviorNode::timer_callback, this));

    get_region_service_ = this->create_service<region_behavior::srv::GetRegion>(
        "/region_behavior/get_region",
        std::bind(&RegionBehaviorNode::handle_get_region, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "节点启动完成。");
}

bool RegionBehaviorNode::load_regions(const std::string &yaml_path)
{
    try
    {
        YAML::Node config = YAML::LoadFile(yaml_path);
        
        if (!config["regions"])
        {
            RCLCPP_ERROR(this->get_logger(), "YAML文件中未找到 'regions' 节点");
            return false;
        }

        for (const auto &region_node : config["regions"])
        {
            Region region;
            region.id = region_node["id"].as<std::string>();
            region.type = region_node["type"].as<int>();

            for (const auto &p_node : region_node["points"])
            {
                geometry_msgs::msg::Point pt;
                pt.x = p_node["x"].as<double>();
                pt.y = p_node["y"].as<double>();
                pt.z = p_node["z"].as<double>();
                region.points.push_back(pt);
            }
            region.long_edge = compute_long_edge(region.points);
            regions_.push_back(region);
            RCLCPP_INFO(this->get_logger(), "加载区域成功: %s (顶点数: %zu)",
                        region.id.c_str(), region.points.size());
        }
    }
    catch (const YAML::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "加载YAML文件出错: %s", e.what());
        return false;
    }

    return !regions_.empty();
}

bool RegionBehaviorNode::is_in_region(const geometry_msgs::msg::Point &pt, const Region &region)
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

int RegionBehaviorNode::get_region_int(const geometry_msgs::msg::Point &pt, Region &region)
{
    for (const auto &region_ : regions_)
    {
        if (is_in_region(pt, region_))
        {
            region = region_;
            return region.type;
        }
    }
    return -1;
}

void RegionBehaviorNode::timer_callback()
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
    Region region_;
    Eigen::Vector2d curr_pos_vect(curr_pos_.x, curr_pos_.y);
    int region_int = get_region_int(curr_pos_, region_);
    auto msg = std_msgs::msg::Int32();
    msg.data = region_int;
    region_int_pub_->publish(msg);

    region_cmd_.chassis_mode = 2;
    region_cmd_.pass_special_region = 0;
    region_cmd_.pass_region_angle = 0.0;
    uphill_cmd_.data = false;
    nav2_msgs::msg::SpeedLimit speed_msg;
    /* 确保每次进区域都只发一个方向 */
    switch (region_int)
    {
    case -1: // 没进
        in_region_ = false;
        waiting_times = 0;
        direction = Eigen::Vector2d::Zero();
        break;
    case 1: // 颠簸路段
        // 如果没有让哨兵停止小陀螺，则停止
        if (waiting_times <= exp_waiting_times)
        {
            region_cmd_.chassis_mode = 1;
            region_cmd_.pass_special_region = 1;
            region_cmd_.pass_region_angle = 500.0; // 状态，表示需要停止小陀螺
            speed_msg.percentage = false;
            speed_msg.speed_limit = 0.5;
            speed_limit_pub_->publish(speed_msg);
            waiting_times++;
            break;
        }
        // 从没进到进
        if (!in_region_)
        {
            RCLCPP_INFO(this->get_logger(), "in_region_check");
            in_region_ = true;

            Eigen::Vector2d A = region_.long_edge.start;
            Eigen::Vector2d B = region_.long_edge.end;
            // A 更接近 B
            if ((curr_pos_vect - A).squaredNorm() < (curr_pos_vect - B).squaredNorm())
            {
                // curr 更靠近 A → 发布 A → B 的方向
                direction = (B - A).normalized();
                RCLCPP_INFO(this->get_logger(), "a to b");

                publish_rectangle_debug_marker(region_, direction, curr_pos_);
            }
            else
            {
                // curr 更靠近 B → 发布 B → A 的方向
                direction = (A - B).normalized();
                RCLCPP_INFO(this->get_logger(), "b to a");
                publish_rectangle_debug_marker(region_, direction, curr_pos_);
            }

            tf2::Quaternion q(
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w);
            double _, robot_yaw;
            tf2::Matrix3x3(q).getRPY(_, _, robot_yaw);
            double dir_yaw = std::atan2(direction.y(), direction.x());

            double angle = dir_yaw - robot_yaw;

            angle = std::fmod(angle + M_PI, 2 * M_PI);
            if (angle < 0)
            {
                angle += 2.0 * M_PI;
            }
            angle -= M_PI;
            in_region_angle = angle * 180.0 / M_PI;
        }

        region_cmd_.chassis_mode = 1;
        region_cmd_.pass_special_region = 1;
        region_cmd_.pass_region_angle = in_region_angle;
        speed_msg.percentage = false;
        speed_msg.speed_limit = 2.0;
        speed_limit_pub_->publish(speed_msg);
        break;
    // 上坡发超电   
    case 2: 
        uphill_cmd_.data= true;
        break;
    default:
        break;
    }
    /* start: 每tick均要发送 */
    uphill_pub_->publish(uphill_cmd_);
    gimbal_cmd_pub_->publish(region_cmd_);
    /* end:   每tick均要发送 */
}

void RegionBehaviorNode::publish_rectangle_debug_marker(
    const Region &region,
    const Eigen::Vector2d &direction,
    const geometry_msgs::msg::Point &p)
{
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = map_frame_id_;
    mk.header.stamp = now();
    mk.ns = "rect_debug";
    mk.id = 1;
    mk.type = visualization_msgs::msg::Marker::LINE_STRIP;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.scale.x = 0.05;
    mk.color.r = 1.0;
    mk.color.g = 1.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;

    for (auto &pt : region.points)
        mk.points.push_back(pt);
    mk.points.push_back(region.points[0]);
    arrow_pub->publish(mk);

    // ====================
    //     画方向箭头
    // ====================
    visualization_msgs::msg::Marker arrow;
    arrow.header = mk.header;
    arrow.ns = "rect_arrow";
    arrow.id = 2;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.scale.x = 0.1;
    arrow.scale.y = 0.2;
    arrow.scale.z = 0.2;
    arrow.color.r = 0.0;
    arrow.color.g = 1.0;
    arrow.color.b = 0.0;
    arrow.color.a = 1.0;

    geometry_msgs::msg::Point start = p;
    geometry_msgs::msg::Point end;
    end.x = start.x + direction.x();
    end.y = start.y + direction.y();
    end.z = start.z;

    arrow.points.push_back(start);
    arrow.points.push_back(end);

    arrow_pub->publish(arrow);

    // ====================
    //     点 P 的球
    // ====================
    visualization_msgs::msg::Marker sphere;
    sphere.header = mk.header;
    sphere.id = 3;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15;
    sphere.color.r = 0;
    sphere.color.g = 0;
    sphere.color.b = 1;
    sphere.color.a = 1;
    sphere.pose.position = p;

    arrow_pub->publish(sphere);
}

LongEdge RegionBehaviorNode::compute_long_edge(const std::vector<geometry_msgs::msg::Point> &pts)
{
    if (pts.size() != 4)
    {
        return {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};
    }

    Eigen::Vector2d A(pts[0].x, pts[0].y);
    Eigen::Vector2d B(pts[1].x, pts[1].y);
    Eigen::Vector2d C(pts[2].x, pts[2].y);
    Eigen::Vector2d D(pts[3].x, pts[3].y);

    std::array<LongEdge, 4> edges = {
        LongEdge{A, B},
        LongEdge{B, C},
        LongEdge{C, D},
        LongEdge{D, A}};

    double max_len = -1.0;
    LongEdge max_edge;

    for (const auto &e : edges)
    {
        double len = (e.end - e.start).norm();
        if (len > max_len)
        {
            max_len = len;
            max_edge = e;
        }
    }

    return max_edge;
}

void RegionBehaviorNode::handle_get_region(
    const std::shared_ptr<region_behavior::srv::GetRegion::Request> request,
    std::shared_ptr<region_behavior::srv::GetRegion::Response> response)
{
    Region temp_region;
    for (const auto &pt : request->locations)
    {
        int id = get_region_int(pt, temp_region);
        response->region_ids.push_back(id);
    }
}

} // namespace region_behavior

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<region_behavior::RegionBehaviorNode>());
    rclcpp::shutdown();
    return 0;
}
