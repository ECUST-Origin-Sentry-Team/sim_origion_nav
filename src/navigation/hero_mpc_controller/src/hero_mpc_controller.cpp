// Copyright 2025 Jinbo Liu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file hero_mpc_controller.cpp
 * @brief 基于 MINCO 轨迹和 acados MPC 的 Nav2 Controller 插件实现
 *
 * 本文件实现了 HeroMpcController 类的所有成员函数。
 *
 * 核心控制流程:
 * ==========================================================================
 * 1. configure(): 初始化参数、MPC 求解器、订阅器
 * 2. mincoTrajectoryCallback(): 接收并解析 MINCO 多项式轨迹
 * 3. computeVelocityCommands(): 每个控制周期执行
 *    a. 获取当前机器人状态（位置、速度）
 *    b. 检查轨迹有效性
 *    c. 计算轨迹相对时间
 *    d. 在 MINCO 轨迹上采样 N+1 个参考点
 *    e. 设置 MPC 初始状态和参考轨迹
 *    f. 调用 MPC 求解
 *    g. 提取速度指令并转换坐标系
 *    h. 发布速度指令
 *
 * 关键坐标系转换:
 * ==========================================================================
 * - Nav2 输入的 velocity 是机器人坐标系 (base_link)
 * - MPC 使用世界坐标系状态
 * - 输出 cmd_vel 是机器人坐标系
 *
 * 作者: Jinbo Liu
 * 日期: 2025.12.27
 */

#include "hero_mpc_controller/hero_mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace hero_mpc_controller {

// =============================================================================
// Nav2 Controller 接口实现
// =============================================================================

void HeroMpcController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    auto node = parent.lock();
    node_ = parent;
    if (!node) {
        throw nav2_core::PlannerException("无法锁定节点!");
    }

    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    // -------------------------------------------------------------------------
    // 声明和获取参数
    // -------------------------------------------------------------------------
    RCLCPP_INFO(logger_, "[%s] 正在配置 MPC 控制器...", plugin_name_.c_str());

    // 轨迹超时参数
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".trajectory_timeout",
                                      rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".trajectory_topic",
        rclcpp::ParameterValue(
            "smoother_server/minco_polynomial_trajectory"));
    node->get_parameter(plugin_name_ + ".trajectory_timeout", traj_timeout_);
    node->get_parameter(plugin_name_ + ".trajectory_topic",
                        trajectory_topic_);
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".trajectory_path_match_tolerance",
        rclcpp::ParameterValue(0.05));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".trajectory_path_stamp_tolerance",
        rclcpp::ParameterValue(0.01));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".minimum_trajectory_execution_time",
        rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".reference_projection_max_time_lead",
        rclcpp::ParameterValue(0.25));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".reference_projection_search_window",
        rclcpp::ParameterValue(2.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".reference_projection_sample_interval",
        rclcpp::ParameterValue(0.05));
    node->get_parameter(plugin_name_ + ".trajectory_path_match_tolerance",
                        trajectory_path_match_tolerance_);
    node->get_parameter(plugin_name_ + ".trajectory_path_stamp_tolerance",
                        trajectory_path_stamp_tolerance_);
    node->get_parameter(plugin_name_ + ".minimum_trajectory_execution_time",
                        minimum_trajectory_execution_time_);
    node->get_parameter(plugin_name_ + ".reference_projection_max_time_lead",
                        reference_projection_max_time_lead_);
    node->get_parameter(plugin_name_ + ".reference_projection_search_window",
                        reference_projection_search_window_);
    node->get_parameter(
        plugin_name_ + ".reference_projection_sample_interval",
        reference_projection_sample_interval_);

    // 速度约束参数
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_velocity",
                                      rclcpp::ParameterValue(2.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_omega",
                                      rclcpp::ParameterValue(1.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_acceleration",
                                      rclcpp::ParameterValue(4.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_alpha",
                                      rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_command_tracking_error",
        rclcpp::ParameterValue(0.6));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".command_state_timeout",
        rclcpp::ParameterValue(0.2));

    node->get_parameter(plugin_name_ + ".max_velocity", max_vel_);
    configured_max_vel_ = max_vel_;
    node->get_parameter(plugin_name_ + ".max_omega", max_omega_);
    node->get_parameter(plugin_name_ + ".max_acceleration", max_acc_);
    node->get_parameter(plugin_name_ + ".max_alpha", max_alpha_);
    node->get_parameter(plugin_name_ + ".max_command_tracking_error",
                        max_command_tracking_error_);
    node->get_parameter(plugin_name_ + ".command_state_timeout",
                        command_state_timeout_);

    // 航向跟踪参数
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".enable_yaw_tracking",
                                      rclcpp::ParameterValue(false));
    declare_parameter_if_not_declared(node, plugin_name_ + ".reference_yaw",
                                      rclcpp::ParameterValue(0.0));

    node->get_parameter(plugin_name_ + ".enable_yaw_tracking",
                        enable_yaw_tracking_);
    node->get_parameter(plugin_name_ + ".reference_yaw", reference_yaw_);

    // TF 容差
    double transform_tolerance = 0.1;
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".transform_tolerance",
                                      rclcpp::ParameterValue(0.1));
    node->get_parameter(plugin_name_ + ".transform_tolerance",
                        transform_tolerance);
    transform_tolerance_ = tf2::durationFromSec(transform_tolerance);

    // 控制频率
    declare_parameter_if_not_declared(node, "controller_frequency",
                                      rclcpp::ParameterValue(100.0));
    node->get_parameter("controller_frequency", control_frequency_);

    // 权重参数 Q (6 维状态)
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".weight_q",
        rclcpp::ParameterValue(
            std::vector<double>{400.0, 100.0, 0.0, 10.0, 10.0, 0.0}));
    node->get_parameter(plugin_name_ + ".weight_q", weight_q_);

    // 权重参数 R (3 维控制)
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".weight_r",
        rclcpp::ParameterValue(std::vector<double>{1.0, 1.0, 0.5}));
    node->get_parameter(plugin_name_ + ".weight_r", weight_r_);

    // Carrot Pose 发布器（用于发布预测点在 base_link 坐标系下的位置）
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".carrot_point_index", rclcpp::ParameterValue(5));
    node->get_parameter(plugin_name_ + ".carrot_point_index",
                        carrot_point_index_);

    declare_parameter_if_not_declared(
        node, plugin_name_ + ".collision_check_enabled",
        rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".collision_check_horizon",
        rclcpp::ParameterValue(0.8));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".collision_cost_threshold",
        rclcpp::ParameterValue(static_cast<int>(
            nav2_costmap_2d::LETHAL_OBSTACLE)));
    node->get_parameter(plugin_name_ + ".collision_check_enabled",
                        collision_check_enabled_);
    node->get_parameter(plugin_name_ + ".collision_check_horizon",
                        collision_check_horizon_);
    node->get_parameter(plugin_name_ + ".collision_cost_threshold",
                        collision_cost_threshold_);

    if (trajectory_topic_.empty() || !std::isfinite(traj_timeout_) ||
        traj_timeout_ < 0.0 || !std::isfinite(max_vel_) || max_vel_ <= 0.0 ||
        !std::isfinite(max_omega_) || max_omega_ <= 0.0 ||
        !std::isfinite(max_acc_) ||
        max_acc_ <= 0.0 || max_alpha_ <= 0.0 ||
        !std::isfinite(max_alpha_) || !std::isfinite(control_frequency_) ||
        control_frequency_ <= 0.0 || !std::isfinite(reference_yaw_) ||
        !std::isfinite(transform_tolerance) || transform_tolerance < 0.0 ||
        !std::isfinite(collision_check_horizon_) ||
        collision_check_horizon_ < 0.0 ||
        !std::isfinite(trajectory_path_match_tolerance_) ||
        trajectory_path_match_tolerance_ <= 0.0 ||
        !std::isfinite(trajectory_path_stamp_tolerance_) ||
        trajectory_path_stamp_tolerance_ < 0.0 ||
        !std::isfinite(minimum_trajectory_execution_time_) ||
        minimum_trajectory_execution_time_ < 0.0 ||
        !std::isfinite(reference_projection_max_time_lead_) ||
        reference_projection_max_time_lead_ < 0.0 ||
        !std::isfinite(reference_projection_search_window_) ||
        reference_projection_search_window_ <= 0.0 ||
        !std::isfinite(reference_projection_sample_interval_) ||
        reference_projection_sample_interval_ <= 0.0 ||
        reference_projection_sample_interval_ >
            reference_projection_search_window_ ||
        !std::isfinite(max_command_tracking_error_) ||
        max_command_tracking_error_ <= 0.0 ||
        !std::isfinite(command_state_timeout_) ||
        command_state_timeout_ <= 0.0 ||
        collision_cost_threshold_ <= nav2_costmap_2d::FREE_SPACE ||
        collision_cost_threshold_ >
            nav2_costmap_2d::LETHAL_OBSTACLE) {
        throw nav2_core::PlannerException(
            "MPC controller parameters contain invalid bounds");
    }

    // -------------------------------------------------------------------------
    // 初始化 MPC 求解器
    // -------------------------------------------------------------------------
    mpc_wrapper_ = std::make_unique<MpcWrapper>();
    if (!mpc_wrapper_->init()) {
        throw nav2_core::PlannerException("MPC 求解器初始化失败!");
    }

    // 设置权重矩阵
    const bool valid_q =
        weight_q_.size() == kStateSize &&
        std::all_of(weight_q_.begin(), weight_q_.end(),
                    [](double value) {
                        return std::isfinite(value) && value >= 0.0;
                    });
    const bool valid_r =
        weight_r_.size() == kInputSize &&
        std::all_of(weight_r_.begin(), weight_r_.end(),
                    [](double value) {
                        return std::isfinite(value) && value > 0.0;
                    });
    if (valid_q && valid_r) {
        StateWeightMatrix Q = StateWeightMatrix::Zero();
        InputWeightMatrix R = InputWeightMatrix::Zero();

        for (int i = 0; i < kStateSize; ++i) {
            Q(i, i) = weight_q_[i];
        }
        for (int i = 0; i < kInputSize; ++i) {
            R(i, i) = weight_r_[i];
        }

        if (!mpc_wrapper_->set_weights(Q, R)) {
            throw nav2_core::PlannerException(
                "Failed to set MPC weight matrices");
        }
        RCLCPP_INFO(logger_, "[%s] 权重矩阵设置成功", plugin_name_.c_str());
    } else {
        throw nav2_core::PlannerException(
            "MPC weight vectors contain invalid dimensions or values");
    }

    // 设置约束
    if (!mpc_wrapper_->set_control_bounds(max_acc_, max_alpha_) ||
        !mpc_wrapper_->set_velocity_bounds(max_vel_, max_omega_)) {
        throw nav2_core::PlannerException("MPC 约束设置失败!");
    }

    // -------------------------------------------------------------------------
    // 创建订阅器和发布器
    // -------------------------------------------------------------------------

    // MINCO 轨迹订阅器
    // 话题名称: ~/minco_polynomial_trajectory
    minco_traj_sub_ =
        node->create_subscription<rm_interfaces::msg::MincoTrajectory>(
            trajectory_topic_, rclcpp::QoS(10).reliable(),
            std::bind(&HeroMpcController::mincoTrajectoryCallback, this,
                      std::placeholders::_1));

    // 可视化发布器
    predicted_path_pub_ =
        node->create_publisher<visualization_msgs::msg::MarkerArray>(
            plugin_name_ + "/predicted_path", 10);
    reference_path_pub_ =
        node->create_publisher<visualization_msgs::msg::MarkerArray>(
            plugin_name_ + "/reference_path", 10);
    carrot_pose_pub_ = node->create_publisher<rm_interfaces::msg::NavOutput>(
        "/carrot_pose", rclcpp::SensorDataQoS());

    RCLCPP_INFO(
        logger_,
        "[%s] 配置完成: max_vel=%.2f, max_omega=%.2f, dt=%.4f, "
        "collision_cost_threshold=%d, path_match_tolerance=%.3f m, "
        "path_stamp_diagnostic_tolerance=%.3f s, command_tracking_error=%.3f "
        "m/s, trajectory_minimum_execution=%.2f s, "
        "reference_projection_lead=%.2f s, projection_window=%.2f s, "
        "projection_sample=%.3f s",
        plugin_name_.c_str(), max_vel_, max_omega_,
        MpcWrapper::get_timestep(), collision_cost_threshold_,
        trajectory_path_match_tolerance_,
        trajectory_path_stamp_tolerance_, max_command_tracking_error_,
        minimum_trajectory_execution_time_,
        reference_projection_max_time_lead_,
        reference_projection_search_window_,
        reference_projection_sample_interval_);

    resetCommandState();
}

void HeroMpcController::cleanup() {
    RCLCPP_INFO(logger_, "[%s] 正在清理...", plugin_name_.c_str());
    reset();
    minco_traj_sub_.reset();
    predicted_path_pub_.reset();
    reference_path_pub_.reset();
    carrot_pose_pub_.reset();
    mpc_wrapper_.reset();
}

void HeroMpcController::activate() {
    RCLCPP_INFO(logger_, "[%s] 激活", plugin_name_.c_str());
    resetCommandState();
    predicted_path_pub_->on_activate();
    reference_path_pub_->on_activate();
    carrot_pose_pub_->on_activate();
}

void HeroMpcController::deactivate() {
    RCLCPP_INFO(logger_, "[%s] 停用", plugin_name_.c_str());
    resetCommandState();
    predicted_path_pub_->on_deactivate();
    reference_path_pub_->on_deactivate();
    carrot_pose_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped HeroMpcController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& velocity,
    nav2_core::GoalChecker* /*goal_checker*/) {
    const rclcpp::Time now = clock_->now();
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header = pose.header;
    cmd_vel.header.stamp = now;

    std::string trajectory_frame;
    rclcpp::Time trajectory_start;
    double trajectory_duration = 0.0;
    uint32_t trajectory_id = 0;
    uint32_t promoted_trajectory_id = 0;
    double promoted_trajectory_age = 0.0;
    bool promoted_deferred_trajectory = false;
    std::uint64_t trajectory_version = 0;
    std::uint64_t trajectory_plan_generation = 0;
    std::vector<MincoSegment> trajectory_segments;
    bool reference_progress_was_valid = false;
    double projected_time_hint = 0.0;
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        if (pending_trajectory_.valid && global_plan_valid_) {
            double active_trajectory_age = 0.0;
            std::string pending_mismatch;
            const bool active_still_dwelling =
                shouldRetainActiveTrajectoryLocked(
                    global_plan_, now, active_trajectory_age);
            const bool pending_matches =
                trajectoryMatchesPlan(pending_trajectory_, global_plan_,
                                      pending_mismatch);
            if (!active_still_dwelling && pending_matches) {
                const double pending_age =
                    (now - pending_trajectory_.start_time).seconds();
                if (std::isfinite(pending_age) && pending_age >= -0.2 &&
                    pending_age <=
                        pending_trajectory_.total_duration + traj_timeout_) {
                    promoted_trajectory_id = pending_trajectory_.id;
                    promoted_trajectory_age = pending_age;
                    promoted_deferred_trajectory = true;
                    activateTrajectoryLocked(std::move(pending_trajectory_),
                                             plan_generation_);
                }
                pending_trajectory_ = MincoTrajectoryData{};
            }
        }

        if (!active_trajectory_.valid ||
            active_trajectory_.segments.empty() ||
            active_trajectory_.frame_id.empty() ||
            !global_plan_valid_ ||
            active_plan_generation_ != plan_generation_) {
            resetCommandState();
            throw nav2_core::PlannerException(
                "No MINCO trajectory matched to the current Nav2 path");
        }
        trajectory_frame = active_trajectory_.frame_id;
        trajectory_start = active_trajectory_.start_time;
        trajectory_duration = active_trajectory_.total_duration;
        trajectory_id = active_trajectory_.id;
        trajectory_version = active_trajectory_version_;
        trajectory_plan_generation = active_plan_generation_;
        trajectory_segments = active_trajectory_.segments;
        reference_progress_was_valid = reference_progress_valid_;
        projected_time_hint = active_projected_time_;
    }
    if (promoted_deferred_trajectory) {
        RCLCPP_INFO(logger_,
                    "[%s] Promoted deferred MINCO trajectory id=%u at "
                    "age=%.2f s",
                    plugin_name_.c_str(), promoted_trajectory_id,
                    promoted_trajectory_age);
    }

    const auto ensure_trajectory_is_current = [&]() {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        if (!active_trajectory_.valid ||
            active_trajectory_version_ != trajectory_version ||
            active_plan_generation_ != trajectory_plan_generation ||
            plan_generation_ != trajectory_plan_generation ||
            active_trajectory_.id != trajectory_id) {
            throw nav2_core::PlannerException(
                "Path or trajectory changed during MPC computation");
        }
    };

    const double trajectory_wall_age = (now - trajectory_start).seconds();
    if (!std::isfinite(trajectory_wall_age) ||
        trajectory_wall_age < -0.2 ||
        trajectory_wall_age > trajectory_duration + traj_timeout_) {
        {
            std::lock_guard<std::mutex> lock(traj_mutex_);
            if (active_trajectory_.valid &&
                active_trajectory_version_ == trajectory_version) {
                invalidateActiveTrajectoryLocked();
            }
            resetCommandState();
        }
        std::ostringstream message;
        message << "MINCO trajectory id=" << trajectory_id
                << " is stale (age=" << trajectory_wall_age
                << " s, duration=" << trajectory_duration
                << " s, timeout=" << traj_timeout_ << " s)";
        throw nav2_core::PlannerException(message.str());
    }

    geometry_msgs::msg::PoseStamped robot_pose = pose;
    if (pose.header.frame_id != trajectory_frame) {
        if (!tf_) {
            resetCommandState();
            throw nav2_core::PlannerException("TF buffer is unavailable");
        }
        try {
            const auto transform = tf_->lookupTransform(
                trajectory_frame, pose.header.frame_id, tf2::TimePointZero);
            tf2::doTransform(pose, robot_pose, transform);
        } catch (const tf2::TransformException& ex) {
            resetCommandState();
            throw nav2_core::PlannerException(
                std::string("Cannot transform robot pose into trajectory "
                            "frame: ") +
                ex.what());
        }
    }

    const double x = robot_pose.pose.position.x;
    const double y = robot_pose.pose.position.y;
    const double yaw = tf2::getYaw(robot_pose.pose.orientation);
    double vx_world = 0.0;
    double vy_world = 0.0;
    bodyToWorld(velocity.linear.x, velocity.linear.y, yaw, vx_world, vy_world);
    const double omega = velocity.angular.z;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
        !std::isfinite(vx_world) || !std::isfinite(vy_world) ||
        !std::isfinite(omega) ||
        !mpc_wrapper_->set_state(x, y, yaw, vx_world, vy_world, omega)) {
        resetCommandState();
        throw nav2_core::PlannerException("Invalid measured MPC state");
    }

    // 轨迹的 start_time 是 Smoother 的生成时刻，不代表车辆已经执行到了
    // 相同相位。对延迟激活或实车跟踪落后的轨迹，直接使用墙钟年龄会让
    // MPC 持续追赶远处参考点并切弯。这里先把实车位置投影到轨迹，再用
    // 墙钟时间作为“最多推进到哪里”的软目标。
    const double wall_reference_time =
        std::clamp(trajectory_wall_age, 0.0, trajectory_duration);
    double projection_search_start = 0.0;
    double projection_search_end = std::min(
        trajectory_duration,
        wall_reference_time + reference_projection_search_window_);
    if (reference_progress_was_valid) {
        projection_search_start =
            std::clamp(projected_time_hint, 0.0, trajectory_duration);
        projection_search_end =
            std::min(trajectory_duration,
                     projection_search_start +
                         reference_projection_search_window_);
    }
    if (projection_search_end < projection_search_start) {
        projection_search_end = projection_search_start;
    }

    double projection_squared_distance =
        std::numeric_limits<double>::infinity();
    const double projected_time = projectPositionOntoTrajectory(
        trajectory_segments, x, y, projection_search_start,
        projection_search_end, reference_projection_sample_interval_,
        projection_squared_distance);
    if (!std::isfinite(projected_time) ||
        !std::isfinite(projection_squared_distance)) {
        resetCommandState();
        throw nav2_core::PlannerException(
            "Failed to project robot position onto MINCO trajectory");
    }

    double t_current = 0.0;
    double committed_projected_time = projected_time;
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        if (!active_trajectory_.valid ||
            active_trajectory_version_ != trajectory_version ||
            active_plan_generation_ != trajectory_plan_generation ||
            plan_generation_ != trajectory_plan_generation ||
            active_trajectory_.id != trajectory_id) {
            resetCommandState();
            throw nav2_core::PlannerException(
                "Path or trajectory changed during reference projection");
        }

        // 投影进度和最终参考相位均不允许倒退。墙钟只允许把参考推到
        // “投影进度 + lead”以内；车辆停住时参考也会冻结在近处，而不是
        // 沿整条 MINCO 曲线继续跑远。
        if (reference_progress_valid_) {
            committed_projected_time =
                std::max(active_projected_time_, committed_projected_time);
        }
        committed_projected_time =
            std::clamp(committed_projected_time, 0.0, trajectory_duration);

        const double maximum_reference_time =
            std::min(trajectory_duration,
                     committed_projected_time +
                         reference_projection_max_time_lead_);
        double candidate_reference_time =
            std::clamp(wall_reference_time, committed_projected_time,
                       maximum_reference_time);
        if (reference_progress_valid_) {
            candidate_reference_time =
                std::max(active_reference_time_, candidate_reference_time);
        }

        active_projected_time_ = committed_projected_time;
        active_reference_time_ =
            std::clamp(candidate_reference_time, 0.0, trajectory_duration);
        reference_progress_valid_ = true;
        t_current = active_reference_time_;
    }

    // 采样参考轨迹并设置到 MPC
    const int N = MpcWrapper::get_horizon_steps();
    const double dt = MpcWrapper::get_timestep();
    TrajectoryState current_reference{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for (int k = 0; k <= N; ++k) {
        double t_k = t_current + k * dt;
        TrajectoryState ref_state =
            sampleTrajectorySegments(trajectory_segments, t_k);
        if (!std::isfinite(ref_state.x) || !std::isfinite(ref_state.y) ||
            !std::isfinite(ref_state.vx) || !std::isfinite(ref_state.vy) ||
            !std::isfinite(ref_state.ax) || !std::isfinite(ref_state.ay)) {
            resetCommandState();
            throw nav2_core::PlannerException(
                "MINCO trajectory produced a non-finite reference");
        }
        if (k == 0) {
            current_reference = ref_state;
        }

        // 计算参考航向角
        // 如果启用航向跟踪，根据速度方向计算；否则使用固定值
        double psi_ref;
        if (enable_yaw_tracking_) {
            // 使用速度方向作为参考航向
            double vel_norm = std::hypot(ref_state.vx, ref_state.vy);
            if (vel_norm > 0.1) {
                psi_ref = std::atan2(ref_state.vy, ref_state.vx);
            } else {
                psi_ref = yaw;  // 速度太小时保持当前航向
            }
        } else {
            psi_ref = reference_yaw_;
        }

        // 计算参考角速度（航向变化率）
        double omega_ref = 0.0;

        // 计算参考角加速度
        double alpha_ref = 0.0;

        if (k < N) {
            // 路径代价参考 (9 维): [p_x, p_y, ψ, v_x, v_y, ω, a_x, a_y, α]
            RefVector yref;
            yref << ref_state.x, ref_state.y, psi_ref, ref_state.vx,
                ref_state.vy, omega_ref, ref_state.ax, ref_state.ay, alpha_ref;

            if (!mpc_wrapper_->set_reference(k, yref)) {
                resetCommandState();
                throw nav2_core::PlannerException(
                    "Failed to set MPC stage reference");
            }
        } else {
            // 终端代价参考 (6 维): [p_x, p_y, ψ, v_x, v_y, ω]
            EndRefVector yref_e;
            yref_e << ref_state.x, ref_state.y, psi_ref, ref_state.vx,
                ref_state.vy, omega_ref;

            if (!mpc_wrapper_->set_terminal_reference(yref_e)) {
                resetCommandState();
                throw nav2_core::PlannerException(
                    "Failed to set MPC terminal reference");
            }
        }
    }

    ensure_trajectory_is_current();

    const int status = mpc_wrapper_->solve();
    if (status != 0) {
        resetCommandState();
        throw nav2_core::PlannerException("MPC solve failed with status " +
                                        std::to_string(status));
    }

    const auto predicted_states = mpc_wrapper_->get_predicted_states();
    if (collision_check_enabled_ &&
        !isCollisionFree(predicted_states, trajectory_frame)) {
        {
            std::lock_guard<std::mutex> lock(traj_mutex_);
            if (active_trajectory_.valid &&
                active_trajectory_version_ == trajectory_version) {
                invalidateActiveTrajectoryLocked();
            }
            resetCommandState();
        }
        throw nav2_core::PlannerException(
            "MPC prediction collides with the local costmap");
    }

    if (predicted_states.size() < 2 || !predicted_states[1].allFinite()) {
        resetCommandState();
        throw nav2_core::PlannerException(
            "MPC returned an invalid next state");
    }
    ensure_trajectory_is_current();

    // acados 输出的是加速度 u0，而底盘接口接收速度。若每一帧都把速度
    // 命令重新锚到有延迟的实测里程计，只能反复发布 measured + 一个离散
    // 步长的增量，速度无法跨控制周期积累。这里保留上一帧已发布命令，
    // 并按实际控制周期积分 u0；同时用实测速度误差球限制积分状态，
    // 防止坏里程计或底盘失联时无限“蓄力”。
    const StateVector &next_state = predicted_states[1];
    const InputVector optimal_control = mpc_wrapper_->get_optimal_control();
    if (!optimal_control.allFinite()) {
        resetCommandState();
        throw nav2_core::PlannerException(
            "MPC returned a non-finite optimal control");
    }

    const Eigen::Vector2d measured_velocity(vx_world, vy_world);
    Eigen::Vector2d commanded_acceleration(optimal_control(kAccX),
                                           optimal_control(kAccY));
    const double acceleration_norm = commanded_acceleration.norm();
    if (acceleration_norm > max_acc_) {
        commanded_acceleration *= max_acc_ / acceleration_norm;
    }

    const double nominal_command_dt = 1.0 / control_frequency_;
    double command_dt = nominal_command_dt;
    Eigen::Vector2d command_base = measured_velocity;
    Eigen::Vector2d command_velocity = measured_velocity;
    std::uint64_t command_state_version = 0;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_state_version = command_state_version_;
        bool use_existing_command_state = command_state_valid_;
        if (use_existing_command_state) {
            const double elapsed = (now - last_command_time_).seconds();
            const double existing_tracking_error =
                (last_command_velocity_world_ - measured_velocity).norm();
            if (std::isfinite(elapsed) && elapsed > 0.0 &&
                elapsed <= command_state_timeout_ &&
                last_command_velocity_world_.allFinite() &&
                std::isfinite(existing_tracking_error) &&
                existing_tracking_error <=
                    max_command_tracking_error_ + 1.0e-6) {
                command_dt = elapsed;
                command_base = last_command_velocity_world_;
            } else {
                use_existing_command_state = false;
            }
        }

        if (!use_existing_command_state) {
            command_dt = nominal_command_dt;
            command_base = measured_velocity;
        }

        Eigen::Vector2d candidate =
            command_base + commanded_acceleration * command_dt;

        const double candidate_speed = candidate.norm();
        if (candidate_speed > max_vel_) {
            candidate *= max_vel_ / candidate_speed;
        }

        Eigen::Vector2d tracking_error = candidate - measured_velocity;
        const double tracking_error_norm = tracking_error.norm();
        if (tracking_error_norm > max_command_tracking_error_) {
            candidate = measured_velocity +
                        tracking_error *
                            (max_command_tracking_error_ / tracking_error_norm);
        }

        // 跟踪误差投影可能改变候选命令，因此最后再次约束相邻两帧
        // 命令的欧氏增量，保证对角线运动也不超过 max_acc_。
        Eigen::Vector2d command_delta = candidate - command_base;
        const double max_command_delta = max_acc_ * command_dt;
        const double command_delta_norm = command_delta.norm();
        if (command_delta_norm > max_command_delta) {
            command_delta *= max_command_delta / command_delta_norm;
        }
        command_velocity = command_base + command_delta;

        const double command_speed = command_velocity.norm();
        if (command_speed > max_vel_) {
            command_velocity *= max_vel_ / command_speed;
        }
    }

    double vx_cmd_world = command_velocity.x();
    double vy_cmd_world = command_velocity.y();
    const double omega_delta =
        std::clamp(next_state(kOmega) - omega, -max_alpha_ * command_dt,
                   max_alpha_ * command_dt);
    double omega_cmd = omega + omega_delta;

    double vx_cmd_body, vy_cmd_body;
    worldToBody(vx_cmd_world, vy_cmd_world, yaw, vx_cmd_body, vy_cmd_body);

    double vel_norm = std::hypot(vx_cmd_body, vy_cmd_body);
    if (vel_norm > max_vel_) {
        double scale = max_vel_ / vel_norm;
        vx_cmd_body *= scale;
        vy_cmd_body *= scale;
    }

    omega_cmd = std::clamp(omega_cmd, -max_omega_, max_omega_);

    // 轨迹版本检查和命令状态提交按 traj -> command 的固定锁顺序完成。
    // 若生命周期回调或另一控制计算已经修改命令状态，
    // 拒绝提交过期结果。
    {
        std::lock_guard<std::mutex> trajectory_lock(traj_mutex_);
        if (!active_trajectory_.valid ||
            active_trajectory_version_ != trajectory_version ||
            active_plan_generation_ != trajectory_plan_generation ||
            plan_generation_ != trajectory_plan_generation ||
            active_trajectory_.id != trajectory_id) {
            throw nav2_core::PlannerException(
                "Path or trajectory changed before command commit");
        }

        std::lock_guard<std::mutex> command_lock(command_mutex_);
        if (command_state_version_ != command_state_version) {
            throw nav2_core::PlannerException(
                "Command state changed before command commit");
        }
        last_command_velocity_world_ = command_velocity;
        last_command_time_ = now;
        command_state_valid_ = true;
        ++command_state_version_;
    }

    // 设置输出
    cmd_vel.twist.linear.x = vx_cmd_body;
    cmd_vel.twist.linear.y = vy_cmd_body;
    cmd_vel.twist.angular.z = omega_cmd;

    RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 1000,
        "[%s] cmd trajectory=%u ref_t=%.3f/%.3f s, wall_t=%.3f, "
        "projection_t=%.3f, projection_error=%.3f m, "
        "ref_v_world=(%.3f, %.3f), measured_world=(%.3f, %.3f), "
        "u0=(%.3f, %.3f), command_base=(%.3f, %.3f), "
        "cmd_body=(%.3f, %.3f, %.3f), tracking_gap=%.3f",
        plugin_name_.c_str(), trajectory_id, t_current, trajectory_duration,
        wall_reference_time, committed_projected_time,
        std::sqrt(projection_squared_distance),
        current_reference.vx, current_reference.vy, vx_world, vy_world,
        commanded_acceleration.x(), commanded_acceleration.y(),
        command_base.x(), command_base.y(), vx_cmd_body, vy_cmd_body, omega_cmd,
        (command_velocity - measured_velocity).norm());

    publishPredictedPath(predicted_states, trajectory_frame);
    publishReferencePath(t_current, trajectory_segments, trajectory_frame);
    return cmd_vel;
}

void HeroMpcController::setPlan(const nav_msgs::msg::Path& path) {
    bool path_valid = path.poses.size() >= 2 && !path.header.frame_id.empty();
    for (const auto& pose : path.poses) {
        path_valid =
            path_valid && std::isfinite(pose.pose.position.x) &&
            std::isfinite(pose.pose.position.y) &&
            (pose.header.frame_id.empty() ||
             pose.header.frame_id == path.header.frame_id);
    }
    if (!path_valid) {
        {
            std::lock_guard<std::mutex> lock(traj_mutex_);
            ++plan_generation_;
            global_plan_ = path;
            global_plan_valid_ = false;
            pending_trajectory_ = MincoTrajectoryData{};
            invalidateActiveTrajectoryLocked();
            resetCommandState();
        }
        RCLCPP_ERROR(logger_,
                     "[%s] Rejected invalid Nav2 path; active MINCO "
                     "trajectory was invalidated",
                     plugin_name_.c_str());
        throw nav2_core::PlannerException(
            "HeroMpcController received an invalid path");
    }

    bool activated_pending = false;
    bool retained_active = false;
    bool retained_for_minimum_execution = false;
    bool kept_deferred_pending = false;
    bool goal_changed = true;
    bool active_after_update = false;
    double active_trajectory_age = 0.0;
    uint32_t selected_id = 0;
    uint32_t deferred_id = 0;
    std::uint64_t selected_generation = 0;
    std::string pending_mismatch;
    std::string active_mismatch;
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        if (global_plan_valid_ && !global_plan_.poses.empty() &&
            global_plan_.header.frame_id == path.header.frame_id) {
            const auto& old_goal = global_plan_.poses.back().pose.position;
            const auto& new_goal = path.poses.back().pose.position;
            const double goal_error =
                std::hypot(old_goal.x - new_goal.x, old_goal.y - new_goal.y);
            goal_changed =
                !std::isfinite(goal_error) ||
                goal_error > trajectory_path_match_tolerance_;
        }

        global_plan_ = path;
        global_plan_valid_ = true;
        selected_generation = ++plan_generation_;

        const bool pending_matches =
            pending_trajectory_.valid &&
            trajectoryMatchesPlan(pending_trajectory_, global_plan_,
                                  pending_mismatch);
        const bool active_matches =
            active_trajectory_.valid &&
            trajectoryMatchesGoal(active_trajectory_, global_plan_,
                                  active_mismatch);
        const bool keep_active_for_minimum_execution =
            active_matches &&
            shouldRetainActiveTrajectoryLocked(
                global_plan_, clock_->now(), active_trajectory_age);

        if (keep_active_for_minimum_execution) {
            selected_id = active_trajectory_.id;
            deferred_id = pending_matches ? pending_trajectory_.id : 0;
            kept_deferred_pending = pending_matches;
            active_plan_generation_ = selected_generation;
            ++active_trajectory_version_;
            retained_active = true;
            retained_for_minimum_execution = true;
        } else if (pending_matches) {
            selected_id = pending_trajectory_.id;
            activateTrajectoryLocked(std::move(pending_trajectory_),
                                     selected_generation);
            pending_trajectory_ = MincoTrajectoryData{};
            activated_pending = true;
        } else if (active_matches) {
            selected_id = active_trajectory_.id;
            active_plan_generation_ = selected_generation;
            ++active_trajectory_version_;
            retained_active = true;
        } else {
            invalidateActiveTrajectoryLocked();
        }

        // setPlan() 已为当前控制目标建立了 frame/目标端点边界；不匹配的
        // 候选不能留给未来目标，否则延迟消息可能串到后续导航任务。
        if (!activated_pending && !kept_deferred_pending) {
            pending_trajectory_ = MincoTrajectoryData{};
        }
        active_after_update = active_trajectory_.valid;
        if (goal_changed || !active_after_update) {
            resetCommandState();
        }
    }

    if (retained_for_minimum_execution) {
        RCLCPP_INFO(
            logger_,
            "[%s] Keeping MINCO trajectory id=%u at t=%.2f s until %.2f s "
            "minimum execution time; deferred candidate id=%u",
            plugin_name_.c_str(), selected_id, active_trajectory_age,
            minimum_trajectory_execution_time_, deferred_id);
    } else if (activated_pending) {
        RCLCPP_INFO(logger_,
                    "[%s] Activated early MINCO trajectory id=%u for path "
                    "generation=%llu",
                    plugin_name_.c_str(), selected_id,
                    static_cast<unsigned long long>(selected_generation));
    } else if (retained_active) {
        RCLCPP_DEBUG(logger_,
                     "[%s] Retained matching MINCO trajectory id=%u for "
                     "path generation=%llu",
                     plugin_name_.c_str(), selected_id,
                     static_cast<unsigned long long>(selected_generation));
    } else {
        RCLCPP_WARN(
            logger_,
            "[%s] Path generation=%llu has no matching MINCO trajectory; "
            "controller will wait. pending='%s', active='%s'",
            plugin_name_.c_str(),
            static_cast<unsigned long long>(selected_generation),
            pending_mismatch.empty() ? "none" : pending_mismatch.c_str(),
            active_mismatch.empty() ? "none" : active_mismatch.c_str());
    }
}

void HeroMpcController::setSpeedLimit(const double& speed_limit,
                                      const bool& percentage) {
    double requested_limit = configured_max_vel_;
    if (speed_limit != nav2_costmap_2d::NO_SPEED_LIMIT) {
        requested_limit =
            percentage ? configured_max_vel_ * speed_limit / 100.0
                       : speed_limit;
    }
    if (!std::isfinite(requested_limit) || requested_limit <= 0.0) {
        RCLCPP_ERROR(logger_, "[%s] Ignoring invalid speed limit %.3f",
                     plugin_name_.c_str(), requested_limit);
        return;
    }

    max_vel_ = std::min(configured_max_vel_, requested_limit);
    if (mpc_wrapper_ &&
        !mpc_wrapper_->set_velocity_bounds(max_vel_, max_omega_)) {
        resetCommandState();
        throw nav2_core::PlannerException(
            "Failed to update MPC speed limit");
    }
    resetCommandState();
}

void HeroMpcController::reset() {
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        ++plan_generation_;
        global_plan_ = nav_msgs::msg::Path{};
        global_plan_valid_ = false;
        pending_trajectory_ = MincoTrajectoryData{};
        invalidateActiveTrajectoryLocked();
        resetCommandState();
    }
}

// MINCO 轨迹处理函数
void HeroMpcController::mincoTrajectoryCallback(
    const rm_interfaces::msg::MincoTrajectory::SharedPtr msg) {
    if (!msg) {
        return;
    }
    const size_t num_segments = msg->durations.size();
    const size_t expected_coeffs = num_segments * 12;
    if (msg->header.frame_id.empty() || num_segments == 0 ||
        num_segments > 500 || msg->coefficients.size() != expected_coeffs) {
        RCLCPP_ERROR(logger_, "[%s] Rejected malformed MINCO trajectory",
                     plugin_name_.c_str());
        return;
    }

    MincoTrajectoryData parsed_trajectory;
    parsed_trajectory.segments.reserve(num_segments);

    // 系数顺序: [段0_x_c5, 段0_x_c4, 段0_x_c3, 段0_x_c2, 段0_x_c1, 段0_x_c0,
    //           段0_y_c5, 段0_y_c4, 段0_y_c3, 段0_y_c2, 段0_y_c1, 段0_y_c0,
    //           段1_x_c5, ...]
    for (size_t i = 0; i < num_segments; ++i) {
        MincoSegment seg;
        seg.duration = msg->durations[i];
        if (!std::isfinite(seg.duration) || seg.duration <= 1.0e-4 ||
            seg.duration > 60.0) {
            RCLCPP_ERROR(logger_, "[%s] Invalid segment duration at %zu",
                         plugin_name_.c_str(), i);
            return;
        }

        const size_t base_idx = i * 12;
        for (size_t j = 0; j < 6; ++j) {
            seg.x_coeffs[j] = msg->coefficients[base_idx + j];
            seg.y_coeffs[j] = msg->coefficients[base_idx + 6 + j];
            if (!std::isfinite(seg.x_coeffs[j]) ||
                !std::isfinite(seg.y_coeffs[j])) {
                RCLCPP_ERROR(logger_,
                             "[%s] Non-finite trajectory coefficient",
                             plugin_name_.c_str());
                return;
            }
        }

        for (int sample = 0; sample <= 10; ++sample) {
            const double t =
                seg.duration * static_cast<double>(sample) / 10.0;
            const double px = evaluatePolynomial(seg.x_coeffs, t);
            const double py = evaluatePolynomial(seg.y_coeffs, t);
            const double vx =
                evaluatePolynomialDerivative(seg.x_coeffs, t);
            const double vy =
                evaluatePolynomialDerivative(seg.y_coeffs, t);
            const double ax =
                evaluatePolynomialSecondDerivative(seg.x_coeffs, t);
            const double ay =
                evaluatePolynomialSecondDerivative(seg.y_coeffs, t);
            if (!std::isfinite(px) || !std::isfinite(py) ||
                !std::isfinite(vx) || !std::isfinite(vy) ||
                !std::isfinite(ax) || !std::isfinite(ay) ||
                std::hypot(vx, vy) >
                    configured_max_vel_ * 1.05 + 1.0e-6 ||
                std::hypot(ax, ay) > max_acc_ * 1.05 + 1.0e-6) {
                RCLCPP_ERROR(logger_,
                             "[%s] Trajectory segment %zu violates "
                             "finite/dynamic bounds",
                             plugin_name_.c_str(), i);
                return;
            }
        }

        if (!parsed_trajectory.segments.empty()) {
            const auto &previous = parsed_trajectory.segments.back();
            const double previous_t = previous.duration;
            const double position_jump = std::hypot(
                evaluatePolynomial(previous.x_coeffs, previous_t) -
                    evaluatePolynomial(seg.x_coeffs, 0.0),
                evaluatePolynomial(previous.y_coeffs, previous_t) -
                    evaluatePolynomial(seg.y_coeffs, 0.0));
            const double velocity_jump = std::hypot(
                evaluatePolynomialDerivative(previous.x_coeffs, previous_t) -
                    evaluatePolynomialDerivative(seg.x_coeffs, 0.0),
                evaluatePolynomialDerivative(previous.y_coeffs, previous_t) -
                    evaluatePolynomialDerivative(seg.y_coeffs, 0.0));
            const double acceleration_jump = std::hypot(
                evaluatePolynomialSecondDerivative(previous.x_coeffs,
                                                   previous_t) -
                    evaluatePolynomialSecondDerivative(seg.x_coeffs, 0.0),
                evaluatePolynomialSecondDerivative(previous.y_coeffs,
                                                   previous_t) -
                    evaluatePolynomialSecondDerivative(seg.y_coeffs, 0.0));
            if (position_jump > 1.0e-3 || velocity_jump > 1.0e-2 ||
                acceleration_jump > 1.0e-1) {
                RCLCPP_ERROR(logger_,
                             "[%s] Discontinuous MINCO segment at %zu",
                             plugin_name_.c_str(), i);
                return;
            }
        }

        parsed_trajectory.segments.push_back(seg);
        parsed_trajectory.total_duration += seg.duration;
    }

    const rclcpp::Time parsed_start(msg->start_time);
    const double parsed_age = (clock_->now() - parsed_start).seconds();
    if (!std::isfinite(parsed_trajectory.total_duration) ||
        parsed_trajectory.total_duration <= 0.0 ||
        parsed_trajectory.total_duration > 300.0 || parsed_age < -0.5 ||
        parsed_age >
            parsed_trajectory.total_duration + traj_timeout_) {
        RCLCPP_ERROR(
            logger_,
            "[%s] Rejected stale/future MINCO trajectory id=%u "
            "(age=%.3f s, duration=%.3f s)",
            plugin_name_.c_str(), msg->trajectory_id, parsed_age,
            parsed_trajectory.total_duration);
        return;
    }

    parsed_trajectory.start_time = parsed_start;
    parsed_trajectory.frame_id = msg->header.frame_id;
    parsed_trajectory.id = msg->trajectory_id;
    parsed_trajectory.path_stamp_nanoseconds =
        rclcpp::Time(msg->header.stamp).nanoseconds();
    parsed_trajectory.valid = true;

    bool activated = false;
    bool staged = false;
    bool deferred_for_minimum_execution = false;
    double active_trajectory_age = 0.0;
    uint32_t active_trajectory_id = 0;
    std::uint64_t matched_generation = 0;
    std::string mismatch_reason;
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        const auto out_of_order_with = [&](const MincoTrajectoryData& other) {
            // header.stamp 在周期重规划链路中不是可靠的发布顺序标识。
            // start_time 由 Smoother 在轨迹通过验证后写入，能够阻止延迟
            // 到达的旧轨迹覆盖更新后的活动/候选轨迹。
            return other.valid &&
                   parsed_trajectory.start_time +
                           rclcpp::Duration::from_seconds(0.01) <
                       other.start_time;
        };
        if (out_of_order_with(active_trajectory_) ||
            out_of_order_with(pending_trajectory_)) {
            RCLCPP_WARN(logger_,
                        "[%s] Ignoring out-of-order MINCO trajectory id=%u",
                        plugin_name_.c_str(), parsed_trajectory.id);
            return;
        }

        if (global_plan_valid_ &&
            trajectoryMatchesPlan(parsed_trajectory, global_plan_,
                                  mismatch_reason)) {
            if (shouldRetainActiveTrajectoryLocked(
                    global_plan_, clock_->now(), active_trajectory_age)) {
                active_trajectory_id = active_trajectory_.id;
                pending_trajectory_ = std::move(parsed_trajectory);
                staged = true;
                deferred_for_minimum_execution = true;
            } else {
                matched_generation = plan_generation_;
                activateTrajectoryLocked(std::move(parsed_trajectory),
                                         matched_generation);
                pending_trajectory_ = MincoTrajectoryData{};
                activated = true;
            }
        } else {
            pending_trajectory_ = std::move(parsed_trajectory);
            staged = true;
        }
    }

    if (deferred_for_minimum_execution) {
        RCLCPP_INFO(
            logger_,
            "[%s] Deferred MINCO trajectory id=%u; active id=%u has only "
            "executed %.2f/%.2f s",
            plugin_name_.c_str(), msg->trajectory_id, active_trajectory_id,
            active_trajectory_age, minimum_trajectory_execution_time_);
    } else if (activated) {
        RCLCPP_INFO(logger_,
                    "[%s] Activated MINCO trajectory id=%u for path "
                    "generation=%llu",
                    plugin_name_.c_str(), msg->trajectory_id,
                    static_cast<unsigned long long>(matched_generation));
    } else if (staged) {
        RCLCPP_DEBUG(logger_,
                     "[%s] Staged MINCO trajectory id=%u until matching "
                     "setPlan(): %s",
                     plugin_name_.c_str(), msg->trajectory_id,
                     mismatch_reason.empty() ? "no current path"
                                             : mismatch_reason.c_str());
    }
}

bool HeroMpcController::trajectoryMatchesPlan(
    const MincoTrajectoryData& trajectory, const nav_msgs::msg::Path& path,
    std::string& mismatch_reason) const {
    mismatch_reason.clear();
    if (!trajectory.valid || trajectory.segments.empty()) {
        mismatch_reason = "trajectory is empty";
        return false;
    }
    if (path.poses.size() < 2 || path.header.frame_id.empty()) {
        mismatch_reason = "path is empty";
        return false;
    }
    if (trajectory.frame_id != path.header.frame_id) {
        mismatch_reason = "frame mismatch";
        return false;
    }

    const std::int64_t path_stamp_nanoseconds =
        rclcpp::Time(path.header.stamp).nanoseconds();
    if (trajectory.path_stamp_nanoseconds != 0 &&
        path_stamp_nanoseconds != 0) {
        const double stamp_error =
            std::abs(static_cast<double>(
                trajectory.path_stamp_nanoseconds - path_stamp_nanoseconds)) *
            1.0e-9;
        if (stamp_error > trajectory_path_stamp_tolerance_) {
            // ComputePathToPose、SmoothPath 和 FollowPath 位于 1 Hz
            // PipelineSequence 的不同 action 边界。它们的 header 时间戳
            // 可能相差一次平滑耗时，不能把这个差值当作轨迹身份。真正的
            // 关联由 frame 和几何首尾点完成。
            RCLCPP_DEBUG_THROTTLE(
                logger_, *clock_, 2000,
                "[%s] Ignoring %.3f s path/trajectory header skew; "
                "validating frame and endpoints",
                plugin_name_.c_str(), stamp_error);
        }
    }

    const auto& first_segment = trajectory.segments.front();
    const auto& last_segment = trajectory.segments.back();
    const double trajectory_start_x =
        evaluatePolynomial(first_segment.x_coeffs, 0.0);
    const double trajectory_start_y =
        evaluatePolynomial(first_segment.y_coeffs, 0.0);
    const double trajectory_goal_x =
        evaluatePolynomial(last_segment.x_coeffs, last_segment.duration);
    const double trajectory_goal_y =
        evaluatePolynomial(last_segment.y_coeffs, last_segment.duration);
    const auto& path_start = path.poses.front().pose.position;
    const auto& path_goal = path.poses.back().pose.position;
    const double start_error =
        std::hypot(trajectory_start_x - path_start.x,
                   trajectory_start_y - path_start.y);
    const double goal_error =
        std::hypot(trajectory_goal_x - path_goal.x,
                   trajectory_goal_y - path_goal.y);
    if (!std::isfinite(start_error) || !std::isfinite(goal_error) ||
        start_error > trajectory_path_match_tolerance_ ||
        goal_error > trajectory_path_match_tolerance_) {
        mismatch_reason =
            "endpoint error start=" + std::to_string(start_error) +
            " m, goal=" + std::to_string(goal_error) + " m";
        return false;
    }
    return true;
}

bool HeroMpcController::trajectoryMatchesGoal(
    const MincoTrajectoryData& trajectory, const nav_msgs::msg::Path& path,
    std::string& mismatch_reason) const {
    mismatch_reason.clear();
    if (!trajectory.valid || trajectory.segments.empty()) {
        mismatch_reason = "trajectory is empty";
        return false;
    }
    if (path.poses.size() < 2 || path.header.frame_id.empty()) {
        mismatch_reason = "path is empty";
        return false;
    }
    if (trajectory.frame_id != path.header.frame_id) {
        mismatch_reason = "frame mismatch";
        return false;
    }

    const auto& last_segment = trajectory.segments.back();
    const double trajectory_goal_x =
        evaluatePolynomial(last_segment.x_coeffs, last_segment.duration);
    const double trajectory_goal_y =
        evaluatePolynomial(last_segment.y_coeffs, last_segment.duration);
    const auto& path_goal = path.poses.back().pose.position;
    const double goal_error =
        std::hypot(trajectory_goal_x - path_goal.x,
                   trajectory_goal_y - path_goal.y);
    if (!std::isfinite(goal_error) ||
        goal_error > trajectory_path_match_tolerance_) {
        mismatch_reason =
            "goal endpoint error=" + std::to_string(goal_error) + " m";
        return false;
    }
    return true;
}

void HeroMpcController::activateTrajectoryLocked(
    MincoTrajectoryData&& trajectory, std::uint64_t plan_generation) {
    active_trajectory_ = std::move(trajectory);
    active_trajectory_.valid = true;
    active_plan_generation_ = plan_generation;
    ++active_trajectory_version_;
}

void HeroMpcController::invalidateActiveTrajectoryLocked() {
    active_trajectory_ = MincoTrajectoryData{};
    active_plan_generation_ = 0;
    ++active_trajectory_version_;
}

bool HeroMpcController::shouldRetainActiveTrajectoryLocked(
    const nav_msgs::msg::Path& path, const rclcpp::Time& now,
    double& trajectory_age) const {
    trajectory_age = 0.0;
    if (minimum_trajectory_execution_time_ <= 0.0 ||
        !active_trajectory_.valid || active_trajectory_.segments.empty() ||
        active_trajectory_.total_duration <= 0.0) {
        return false;
    }

    std::string mismatch_reason;
    if (!trajectoryMatchesGoal(active_trajectory_, path, mismatch_reason)) {
        return false;
    }

    trajectory_age = (now - active_trajectory_.start_time).seconds();
    const double hold_duration =
        std::min(minimum_trajectory_execution_time_,
                 active_trajectory_.total_duration);
    return std::isfinite(trajectory_age) && trajectory_age >= 0.0 &&
           trajectory_age < hold_duration;
}

void HeroMpcController::resetCommandState() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    last_command_velocity_world_.setZero();
    last_command_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    command_state_valid_ = false;
    ++command_state_version_;
}

TrajectoryState HeroMpcController::sampleTrajectory(double t) const {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    return sampleTrajectorySegments(active_trajectory_.segments, t);
}

TrajectoryState HeroMpcController::sampleTrajectorySegments(
    const std::vector<MincoSegment>& segments, double t) {
    TrajectoryState state{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (segments.empty()) {
        return state;
    }

    size_t seg_idx;
    double local_t;
    if (!findSegment(segments, t, seg_idx, local_t)) {
        const auto& last_seg = segments.back();
        const double t_end = last_seg.duration;

        state.x = evaluatePolynomial(last_seg.x_coeffs, t_end);
        state.y = evaluatePolynomial(last_seg.y_coeffs, t_end);
        state.vx = evaluatePolynomialDerivative(last_seg.x_coeffs, t_end);
        state.vy = evaluatePolynomialDerivative(last_seg.y_coeffs, t_end);
        state.ax = evaluatePolynomialSecondDerivative(last_seg.x_coeffs, t_end);
        state.ay = evaluatePolynomialSecondDerivative(last_seg.y_coeffs, t_end);

        return state;
    }

    const auto& seg = segments[seg_idx];

    state.x = evaluatePolynomial(seg.x_coeffs, local_t);
    state.y = evaluatePolynomial(seg.y_coeffs, local_t);
    state.vx = evaluatePolynomialDerivative(seg.x_coeffs, local_t);
    state.vy = evaluatePolynomialDerivative(seg.y_coeffs, local_t);
    state.ax = evaluatePolynomialSecondDerivative(seg.x_coeffs, local_t);
    state.ay = evaluatePolynomialSecondDerivative(seg.y_coeffs, local_t);
    return state;
}

double HeroMpcController::projectPositionOntoTrajectory(
    const std::vector<MincoSegment>& segments, double x, double y,
    double search_start, double search_end, double sample_interval,
    double& squared_distance) {
    squared_distance = std::numeric_limits<double>::infinity();
    const double invalid_time = std::numeric_limits<double>::quiet_NaN();

    if (segments.empty() || !std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(search_start) || !std::isfinite(search_end) ||
        !std::isfinite(sample_interval) || sample_interval <= 0.0) {
        return invalid_time;
    }

    double total_duration = 0.0;
    for (const auto& segment : segments) {
        if (!std::isfinite(segment.duration) || segment.duration <= 0.0) {
            return invalid_time;
        }
        total_duration += segment.duration;
    }
    if (!std::isfinite(total_duration) || total_duration <= 0.0) {
        return invalid_time;
    }

    const double clamped_start =
        std::clamp(search_start, 0.0, total_duration);
    const double clamped_end = std::clamp(search_end, 0.0, total_duration);
    if (clamped_end < clamped_start) {
        return invalid_time;
    }

    const auto distance_at = [&segments, x, y](double t) {
        const TrajectoryState state = sampleTrajectorySegments(segments, t);
        const double dx = state.x - x;
        const double dy = state.y - y;
        if (!std::isfinite(dx) || !std::isfinite(dy)) {
            return std::numeric_limits<double>::infinity();
        }
        return dx * dx + dy * dy;
    };

    double best_time = clamped_start;
    squared_distance = distance_at(best_time);
    const double search_width = clamped_end - clamped_start;
    if (search_width == 0.0) {
        return std::isfinite(squared_distance) ? best_time : invalid_time;
    }

    const double coarse_step_count = std::ceil(search_width / sample_interval);
    if (!std::isfinite(coarse_step_count) ||
        coarse_step_count >
            static_cast<double>(std::numeric_limits<std::size_t>::max() - 1)) {
        return invalid_time;
    }

    const auto coarse_steps =
        std::max<std::size_t>(1, static_cast<std::size_t>(coarse_step_count));
    for (std::size_t i = 1; i <= coarse_steps; ++i) {
        const double t =
            clamped_start +
            std::min(search_width, static_cast<double>(i) * sample_interval);
        const double candidate_distance = distance_at(t);
        if (candidate_distance < squared_distance) {
            best_time = t;
            squared_distance = candidate_distance;
        }
    }
    if (!std::isfinite(squared_distance)) {
        return invalid_time;
    }

    // The coarse minimum identifies the local branch of the trajectory. Refine
    // only inside its neighboring sample interval so self-intersections cannot
    // make the projection jump to a distant branch.
    double lower =
        std::max(clamped_start, best_time - sample_interval);
    double upper = std::min(clamped_end, best_time + sample_interval);
    constexpr double kInverseGoldenRatio = 0.6180339887498948482;
    double left = upper - kInverseGoldenRatio * (upper - lower);
    double right = lower + kInverseGoldenRatio * (upper - lower);
    double left_distance = distance_at(left);
    double right_distance = distance_at(right);

    for (int iteration = 0; iteration < 24; ++iteration) {
        if (left_distance <= right_distance) {
            upper = right;
            right = left;
            right_distance = left_distance;
            left = upper - kInverseGoldenRatio * (upper - lower);
            left_distance = distance_at(left);
        } else {
            lower = left;
            left = right;
            left_distance = right_distance;
            right = lower + kInverseGoldenRatio * (upper - lower);
            right_distance = distance_at(right);
        }
    }

    const double refined_time = 0.5 * (lower + upper);
    const double refined_distance = distance_at(refined_time);
    if (refined_distance < squared_distance) {
        best_time = refined_time;
        squared_distance = refined_distance;
    }
    return best_time;
}

double HeroMpcController::evaluatePolynomial(
    const std::array<double, 6>& coeffs, double t) {
    // p(t) = c5*t^5 + c4*t^4 + c3*t^3 + c2*t^2 + c1*t + c0
    // 使用 Horner 方法提高数值稳定性和效率
    // p(t) = ((((c5*t + c4)*t + c3)*t + c2)*t + c1)*t + c0
    return ((((coeffs[0] * t + coeffs[1]) * t + coeffs[2]) * t + coeffs[3]) *
                t +
            coeffs[4]) *
               t +
           coeffs[5];
}

double HeroMpcController::evaluatePolynomialDerivative(
    const std::array<double, 6>& coeffs, double t) {
    // p'(t) = 5*c5*t^4 + 4*c4*t^3 + 3*c3*t^2 + 2*c2*t + c1
    // 使用 Horner: (((5*c5*t + 4*c4)*t + 3*c3)*t + 2*c2)*t + c1
    return (((5.0 * coeffs[0] * t + 4.0 * coeffs[1]) * t + 3.0 * coeffs[2]) *
                t +
            2.0 * coeffs[3]) *
               t +
           coeffs[4];
}

double HeroMpcController::evaluatePolynomialSecondDerivative(
    const std::array<double, 6>& coeffs, double t) {
    // p''(t) = 20*c5*t^3 + 12*c4*t^2 + 6*c3*t + 2*c2
    // 使用 Horner: ((20*c5*t + 12*c4)*t + 6*c3)*t + 2*c2
    return ((20.0 * coeffs[0] * t + 12.0 * coeffs[1]) * t + 6.0 * coeffs[2]) *
               t +
           2.0 * coeffs[3];
}

bool HeroMpcController::findSegment(
    const std::vector<MincoSegment>& segments, double t,
    size_t& segment_index, double& local_t) {
    if (t < 0.0) {
        segment_index = 0;
        local_t = 0.0;
        return true;
    }

    double accumulated_time = 0.0;
    for (size_t i = 0; i < segments.size(); ++i) {
        if (t <= accumulated_time + segments[i].duration) {
            segment_index = i;
            local_t = t - accumulated_time;
            return true;
        }
        accumulated_time += segments[i].duration;
    }

    // t 超出轨迹范围
    return false;
}

// =============================================================================
// 坐标系转换函数
// =============================================================================

void HeroMpcController::bodyToWorld(double vx_body, double vy_body, double yaw,
                                    double& vx_world, double& vy_world) {
    // 旋转矩阵 R(yaw):
    // [ cos(yaw)  -sin(yaw) ]
    // [ sin(yaw)   cos(yaw) ]
    //
    // v_world = R * v_body
    double c = std::cos(yaw);
    double s = std::sin(yaw);

    vx_world = vx_body * c - vy_body * s;
    vy_world = vx_body * s + vy_body * c;
}

void HeroMpcController::worldToBody(double vx_world, double vy_world,
                                    double yaw, double& vx_body,
                                    double& vy_body) {
    // 逆旋转矩阵 R^T(yaw) = R(-yaw):
    // [  cos(yaw)  sin(yaw) ]
    // [ -sin(yaw)  cos(yaw) ]
    //
    // v_body = R^T * v_world
    double c = std::cos(yaw);
    double s = std::sin(yaw);

    vx_body = vx_world * c + vy_world * s;
    vy_body = -vx_world * s + vy_world * c;
}

double HeroMpcController::normalizeAngle(double angle) {
    constexpr double kPi = 3.14159265358979323846;
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

// =============================================================================
// 可视化函数
// =============================================================================

bool HeroMpcController::isCollisionFree(
    const std::vector<StateVector>& predicted_states,
    const std::string& trajectory_frame) const {
    if (!costmap_ros_) {
        RCLCPP_ERROR_THROTTLE(
            logger_, *clock_, 1000,
            "[%s] MPC safety check unavailable: local costmap is null",
            plugin_name_.c_str());
        return false;
    }
    if (predicted_states.empty() || trajectory_frame.empty()) {
        RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 1000,
            "[%s] MPC safety check rejected an empty prediction or frame "
            "(states=%zu, frame='%s')",
            plugin_name_.c_str(), predicted_states.size(),
            trajectory_frame.c_str());
        return false;
    }

    auto *costmap = costmap_ros_->getCostmap();
    const auto footprint = costmap_ros_->getRobotFootprint();
    const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
    if (!costmap) {
        RCLCPP_ERROR_THROTTLE(
            logger_, *clock_, 1000,
            "[%s] MPC safety check unavailable: Costmap2D is null",
            plugin_name_.c_str());
        return false;
    }
    if (footprint.size() < 3 || costmap_frame.empty() ||
        !std::isfinite(costmap->getResolution()) ||
        costmap->getResolution() <= 0.0) {
        RCLCPP_ERROR_THROTTLE(
            logger_, *clock_, 1000,
            "[%s] MPC safety check configuration is invalid: footprint=%zu "
            "points, costmap_frame='%s', resolution=%.6f",
            plugin_name_.c_str(), footprint.size(), costmap_frame.c_str(),
            costmap->getResolution());
        return false;
    }

    geometry_msgs::msg::TransformStamped frame_transform;
    const bool transform_required = trajectory_frame != costmap_frame;
    if (transform_required) {
        if (!tf_) {
            RCLCPP_ERROR_THROTTLE(
                logger_, *clock_, 1000,
                "[%s] Cannot transform MPC prediction from '%s' to '%s': "
                "TF buffer is null",
                plugin_name_.c_str(), trajectory_frame.c_str(),
                costmap_frame.c_str());
            return false;
        }
        try {
            frame_transform = tf_->lookupTransform(
                costmap_frame, trajectory_frame, tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR_THROTTLE(
                logger_, *clock_, 1000,
                "[%s] Cannot transform MPC prediction to costmap: %s",
                plugin_name_.c_str(), ex.what());
            return false;
        }
    }

    const int max_stage = std::min(
        static_cast<int>(predicted_states.size()) - 1,
        static_cast<int>(
            std::ceil(collision_check_horizon_ / MpcWrapper::get_timestep())));

    // acados 对 v_x / v_y 使用逐轴 box 约束，而控制器在最终输出时才对
    // 平移速度施加欧氏范数限制。这里若用 hypot(v_x, v_y) 与 max_vel_
    // 比较，会把合法的对角运动误报为“碰撞”（例如两个分量均为 1.6 m/s）。
    // stage 0 是实测状态，可能暂时超出配置上限；拒绝速度指令会使控制器
    // 无法制动或恢复，所以 stage 0 只检查有限性。未来 stage 则按求解器
    // 的真实逐轴约束做一致性校验。
    const double velocity_tolerance =
        std::max(1.0e-6, max_vel_ * 0.02);
    const double omega_tolerance =
        std::max(1.0e-6, max_omega_ * 0.02);
    for (int stage = 0; stage <= max_stage; ++stage) {
        const auto & state = predicted_states[stage];
        if (!state.allFinite()) {
            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 500,
                "[%s] MPC prediction rejected: non-finite state at stage "
                "%d (x=%.3f, y=%.3f, yaw=%.3f, vx=%.3f, vy=%.3f, "
                "omega=%.3f)",
                plugin_name_.c_str(), stage, state(kPosX), state(kPosY),
                state(kPsi), state(kVelX), state(kVelY), state(kOmega));
            return false;
        }
        if (stage > 0 &&
            (std::abs(state(kVelX)) > max_vel_ + velocity_tolerance ||
             std::abs(state(kVelY)) > max_vel_ + velocity_tolerance ||
             std::abs(state(kOmega)) > max_omega_ + omega_tolerance)) {
            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 500,
                "[%s] MPC prediction rejected: dynamic bound violation at "
                "stage %d (vx=%.3f, vy=%.3f, omega=%.3f; per-axis limits "
                "vx/vy=%.3f, omega=%.3f)",
                plugin_name_.c_str(), stage, state(kVelX), state(kVelY),
                state(kOmega), max_vel_, max_omega_);
            return false;
        }
    }

    nav2_costmap_2d::FootprintCollisionChecker<
        nav2_costmap_2d::Costmap2D *>
        collision_checker(costmap);
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
        *(costmap->getMutex()));

    const auto check_state = [&](const StateVector &state, int stage,
                                 int substep) {
        geometry_msgs::msg::PoseStamped prediction;
        prediction.header.frame_id = trajectory_frame;
        prediction.header.stamp = clock_->now();
        prediction.pose.position.x = state(kPosX);
        prediction.pose.position.y = state(kPosY);
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, state(kPsi));
        prediction.pose.orientation = tf2::toMsg(orientation);

        if (transform_required) {
            geometry_msgs::msg::PoseStamped transformed;
            tf2::doTransform(prediction, transformed, frame_transform);
            prediction = transformed;
        }

        const double yaw = tf2::getYaw(prediction.pose.orientation);
        const double pose_x = prediction.pose.position.x;
        const double pose_y = prediction.pose.position.y;
        unsigned int cell_x = 0;
        unsigned int cell_y = 0;
        if (!costmap->worldToMap(pose_x, pose_y, cell_x, cell_y)) {
            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 500,
                "[%s] MPC prediction rejected: footprint center is outside "
                "the local costmap at stage %d/%d (pose=(%.3f, %.3f, "
                "%.3f), source_frame='%s', costmap_frame='%s')",
                plugin_name_.c_str(), stage, substep, pose_x, pose_y, yaw,
                trajectory_frame.c_str(), costmap_frame.c_str());
            return false;
        }

        const unsigned char center_cost = costmap->getCost(cell_x, cell_y);
        const double cost = collision_checker.footprintCostAtPose(
            pose_x, pose_y, yaw, footprint);
        const double effective_cost =
            std::max(cost, static_cast<double>(center_cost));
        if (!std::isfinite(cost)) {
            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 500,
                "[%s] MPC prediction rejected: non-finite footprint cost at "
                "stage %d/%d (pose=(%.3f, %.3f, %.3f))",
                plugin_name_.c_str(), stage, substep, pose_x, pose_y, yaw);
            return false;
        }
        if (cost < 0.0) {
            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 500,
                "[%s] MPC prediction rejected: footprint extends outside "
                "the local costmap at stage %d/%d (pose=(%.3f, %.3f, "
                "%.3f), footprint_cost=%.1f)",
                plugin_name_.c_str(), stage, substep, pose_x, pose_y, yaw,
                cost);
            return false;
        }
        if (center_cost == nav2_costmap_2d::NO_INFORMATION ||
            cost == static_cast<double>(
                        nav2_costmap_2d::NO_INFORMATION)) {
            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 500,
                "[%s] MPC prediction rejected: unknown costmap cell at "
                "stage %d/%d (pose=(%.3f, %.3f, %.3f), center_cost=%u, "
                "footprint_cost=%.1f)",
                plugin_name_.c_str(), stage, substep, pose_x, pose_y, yaw,
                static_cast<unsigned int>(center_cost), cost);
            return false;
        }
        if (effective_cost >=
            static_cast<double>(collision_cost_threshold_)) {
            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 500,
                "[%s] MPC prediction rejected: lethal footprint collision "
                "at stage %d/%d (pose=(%.3f, %.3f, %.3f), center_cost=%u, "
                "footprint_cost=%.1f, threshold=%d)",
                plugin_name_.c_str(), stage, substep, pose_x, pose_y, yaw,
                static_cast<unsigned int>(center_cost), cost,
                collision_cost_threshold_);
            return false;
        }
        return true;
    };

    if (!check_state(predicted_states.front(), 0, 0)) {
        return false;
    }
    const double spatial_step = 0.5 * costmap->getResolution();
    double footprint_radius = 0.0;
    for (const auto & point : footprint) {
        footprint_radius =
            std::max(footprint_radius, std::hypot(point.x, point.y));
    }
    for (int stage = 1; stage <= max_stage; ++stage) {
        const auto &previous = predicted_states[stage - 1];
        const auto &current = predicted_states[stage];
        const double segment_length = std::hypot(
            current(kPosX) - previous(kPosX),
            current(kPosY) - previous(kPosY));
        const double yaw_delta =
            normalizeAngle(current(kPsi) - previous(kPsi));
        // 对旋转也加密采样。仅按平移距离采样会在原地转向时漏检 footprint
        // 顶角扫过的障碍；平移距离与角向扫掠弧长之和给出保守步数。
        const double swept_length =
            segment_length + footprint_radius * std::abs(yaw_delta);
        const int substeps =
            std::max(1, static_cast<int>(
                            std::ceil(swept_length / spatial_step)));
        for (int substep = 1; substep <= substeps; ++substep) {
            const double ratio =
                static_cast<double>(substep) / substeps;
            StateVector interpolated =
                previous + ratio * (current - previous);
            interpolated(kPsi) =
                normalizeAngle(previous(kPsi) + ratio * yaw_delta);
            if (!check_state(interpolated, stage, substep)) {
                return false;
            }
        }
    }
    return true;
}

void HeroMpcController::publishPredictedPath(
    const std::vector<StateVector>& predicted_states,
    const std::string& frame_id) {
    rclcpp::Time current_time = clock_->now();

    // -------------------------------------------------------------------------
    // 发布 Carrot Pose（指定预测点在 base_link 坐标系下的位置）
    // -------------------------------------------------------------------------
    if (carrot_point_index_ >= 0 &&
        static_cast<size_t>(carrot_point_index_) < predicted_states.size()) {
        const auto& carrot_state = predicted_states[carrot_point_index_];

        const std::string base_frame = costmap_ros_->getBaseFrameID();
        try {
            auto transform = tf_->lookupTransform(
                base_frame, frame_id, tf2::TimePointZero);

            geometry_msgs::msg::PoseStamped carrot_in_trajectory;
            carrot_in_trajectory.header.frame_id = frame_id;
            carrot_in_trajectory.header.stamp = current_time;
            carrot_in_trajectory.pose.position.x = carrot_state(kPosX);
            carrot_in_trajectory.pose.position.y = carrot_state(kPosY);
            carrot_in_trajectory.pose.position.z = 0.0;
            carrot_in_trajectory.pose.orientation.w = 1.0;

            // 转换到 base_link 坐标系
            geometry_msgs::msg::PoseStamped carrot_in_base;
            tf2::doTransform(carrot_in_trajectory, carrot_in_base, transform);
            carrot_in_base.header.frame_id = base_frame;
            carrot_in_base.header.stamp = current_time;

            rm_interfaces::msg::NavOutput nav_output_msg;
            nav_output_msg.header = carrot_in_base.header;
            nav_output_msg.mode = 0;  // 0: normal navigation
            nav_output_msg.point = carrot_in_base;
            carrot_pose_pub_->publish(nav_output_msg);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                                 "[%s] 无法将 carrot 转换到机器人坐标系: %s",
                                 plugin_name_.c_str(), ex.what());
        }
    }

    // -------------------------------------------------------------------------
    // 发布预测轨迹可视化 Marker
    // -------------------------------------------------------------------------
    if (predicted_path_pub_->get_subscription_count() == 0) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;

    // 遍历预测状态生成球体 Marker
    for (size_t i = 0; i < predicted_states.size(); ++i) {
        const auto& state = predicted_states[i];

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = current_time;
        marker.ns = "predicted_trajectory";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 设置位置
        marker.pose.position.x = state(kPosX);
        marker.pose.position.y = state(kPosY);
        marker.pose.position.z = 0.0;

        // 方向默认即可，球体不敏感
        marker.pose.orientation.w = 1.0;

        // 设置尺寸 (直径 m)
        // 可以根据时间 i 渐变大小，或者固定大小
        double scale = 0.1;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;

        // 设置颜色 (RGBA)
        // 例如：绿色，透明度随时间递减
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(
            0.1);  // 生命周期短一点，自动消失防止重影

        marker_array.markers.push_back(marker);
    }

    predicted_path_pub_->publish(marker_array);
}

void HeroMpcController::publishReferencePath(
    double t_start, const std::vector<MincoSegment>& trajectory_segments,
    const std::string& frame_id) {
    if (reference_path_pub_->get_subscription_count() == 0) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    rclcpp::Time current_time = clock_->now();

    const int N = MpcWrapper::get_horizon_steps();
    const double dt = MpcWrapper::get_timestep();

    for (int k = 0; k <= N; ++k) {
        double t_k = t_start + k * dt;
        TrajectoryState ref =
            sampleTrajectorySegments(trajectory_segments, t_k);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = current_time;
        marker.ns = "reference_trajectory";
        marker.id = static_cast<int>(k);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 设置位置
        marker.pose.position.x = ref.x;
        marker.pose.position.y = ref.y;
        marker.pose.position.z = 0.0;

        // 方向默认即可
        marker.pose.orientation.w = 1.0;

        // 设置尺寸 (直径 m)
        double scale = 0.08;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;

        // 设置颜色 (RGBA) - 红色
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0.1);

        marker_array.markers.push_back(marker);
    }

    reference_path_pub_->publish(marker_array);
}

}  // namespace hero_mpc_controller

// 注册 Nav2 插件
PLUGINLIB_EXPORT_CLASS(hero_mpc_controller::HeroMpcController,
                       nav2_core::Controller)
