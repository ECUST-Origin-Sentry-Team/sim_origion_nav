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
 * @file hero_mpc_controller.hpp
 * @brief 基于 MINCO 轨迹和 acados MPC 的 Nav2 Controller 插件 (Layer 3)
 *
 * 本文件实现了 Nav2 的 Controller 插件接口，用于全向轮机器人的轨迹跟踪控制。
 *
 * 系统架构：
 * ==========================================================================
 *                    +-------------------+
 *                    |  MINCO Smoother   |
 *                    |   (Path Planner)  |
 *                    +--------+----------+
 *                             |
 *                             | MincoTrajectory (多项式系数)
 *                             v
 *    +--------------------------------------------------------+
 *    |               HeroMpcController (Nav2 Plugin)          |
 *    |                                                        |
 *    |  +------------------+    +-------------------------+   |
 *    |  | Trajectory       |    |    MpcWrapper           |   |
 *    |  | Sampler          |--->|    (acados 封装)        |   |
 *    |  | (多项式采样)      |    +-------------------------+   |
 *    |  +------------------+                                  |
 *    +--------------------------------------------------------+
 *                             |
 *                             | geometry_msgs::Twist (cmd_vel)
 *                             v
 *                    +-------------------+
 *                    |   Robot Base      |
 *                    +-------------------+
 *
 * 坐标系说明：
 * ==========================================================================
 * - MPC 模型使用 **世界坐标系 (World Frame)** 下的状态
 *   - 位置: (p_x, p_y) 在 map/odom 坐标系下
 *   - 速度: (v_x, v_y) 是世界坐标系速度
 *
 * - Nav2 输入:
 *   - pose: 世界坐标系 (map/odom frame)
 *   - velocity: **机器人本体坐标系 (base_link frame)**
 *
 * - Nav2 输出:
 *   - cmd_vel: **机器人本体坐标系 (base_link frame)**
 *
 * - 因此需要进行坐标系转换：
 *   1. 输入时: 将 base_link 速度旋转到 world 速度
 *   2. 输出时: 将 MPC 计算的 world 速度旋转回 base_link 速度
 *
 * 作者: Jinbo Liu
 * 日期: 2025.12.27
 */

#ifndef HERO_MPC_CONTROLLER__HERO_MPC_CONTROLLER_HPP_
#define HERO_MPC_CONTROLLER__HERO_MPC_CONTROLLER_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// ROS 2 核心
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Nav2 接口
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

// TF2
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"

// 消息类型
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rm_interfaces/msg/nav_output.hpp"

// MINCO 轨迹消息
#include "rm_interfaces/msg/minco_trajectory.hpp"

// MPC Wrapper
#include "hero_mpc_controller/mpc_wrapper.hpp"
#include "hero_mpc_controller/visibility_control.h"

namespace hero_mpc_controller {

/**
 * @struct MincoSegment
 * @brief MINCO 轨迹的单段多项式
 *
 * 每段是五次多项式: p(t) = c5*t^5 + c4*t^4 + c3*t^3 + c2*t^2 + c1*t + c0
 * 系数存储顺序: [c5, c4, c3, c2, c1, c0] (从高次到低次)
 */
struct MincoSegment {
    /// X 维度系数 [c5, c4, c3, c2, c1, c0]
    std::array<double, 6> x_coeffs;

    /// Y 维度系数 [c5, c4, c3, c2, c1, c0]
    std::array<double, 6> y_coeffs;

    /// 该段的持续时间 [s]
    double duration;
};

/**
 * @struct MincoTrajectoryData
 * @brief 已解析、等待与 Nav2 Path 建立关联的 MINCO 轨迹
 */
struct MincoTrajectoryData {
    std::vector<MincoSegment> segments;
    rclcpp::Time start_time{0, 0, RCL_ROS_TIME};
    double total_duration{0.0};
    uint32_t id{0};
    std::string frame_id;
    std::int64_t path_stamp_nanoseconds{0};
    bool valid{false};
};

/**
 * @struct TrajectoryState
 * @brief 轨迹采样点的完整状态
 *
 * 包含位置、速度、加速度信息，用于构建 MPC 参考轨迹
 */
struct TrajectoryState {
    double x;   ///< 位置 X [m]
    double y;   ///< 位置 Y [m]
    double vx;  ///< 速度 X [m/s]
    double vy;  ///< 速度 Y [m/s]
    double ax;  ///< 加速度 X [m/s²]
    double ay;  ///< 加速度 Y [m/s²]
};

/**
 * @class HeroMpcController
 * @brief 基于 acados MPC 的 Nav2 Controller 插件
 *
 * 主要功能：
 * 1. 订阅 MINCO 多项式轨迹
 * 2. 根据当前时间在轨迹上采样参考点
 * 3. 调用 MPC 求解器计算最优控制
 * 4. 输出机器人速度指令
 */
class HeroMpcController : public nav2_core::Controller {
   public:
    HERO_MPC_CONTROLLER_PUBLIC
    HeroMpcController() = default;

    HERO_MPC_CONTROLLER_PUBLIC
    ~HeroMpcController() override = default;

    /**
     * @brief 配置控制器
     *
     * 声明和获取参数，初始化 MPC 求解器，创建订阅器和发布器
     *
     * @param parent 父节点弱指针
     * @param name 插件名称
     * @param tf TF2 缓冲区
     * @param costmap_ros Costmap2DROS 指针
     */
    HERO_MPC_CONTROLLER_PUBLIC
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    /**
     * @brief 清理控制器资源
     */
    HERO_MPC_CONTROLLER_PUBLIC
    void cleanup() override;

    /**
     * @brief 激活控制器
     */
    HERO_MPC_CONTROLLER_PUBLIC
    void activate() override;

    /**
     * @brief 停用控制器
     */
    HERO_MPC_CONTROLLER_PUBLIC
    void deactivate() override;

    /**
     * @brief 计算速度指令
     *
     * 核心控制循环：
     * 1. 获取当前机器人状态
     * 2. 检查轨迹有效性
     * 3. 采样 MINCO 轨迹生成参考
     * 4. 调用 MPC 求解
     * 5. 坐标系转换后输出速度指令
     *
     * @param pose 当前位姿（世界坐标系）
     * @param velocity 当前速度（机器人坐标系）
     * @param goal_checker 目标检查器
     * @return 速度指令（机器人坐标系）
     */
    HERO_MPC_CONTROLLER_PUBLIC
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped& pose,
        const geometry_msgs::msg::Twist& velocity,
        nav2_core::GoalChecker* goal_checker) override;

    /**
     * @brief 设置全局路径
     *
     * 控制参考仍来自 MINCO 多项式，但离散 Path 用于确认该轨迹确实属于
     * 当前 Nav2 控制目标。
     *
     * @param path 全局路径
     */
    HERO_MPC_CONTROLLER_PUBLIC
    void setPlan(const nav_msgs::msg::Path& path) override;

    /**
     * @brief 设置速度限制
     *
     * @param speed_limit 速度限制
     * @param percentage 是否为百分比
     */
    HERO_MPC_CONTROLLER_PUBLIC
    void setSpeedLimit(const double& speed_limit,
                       const bool& percentage) override;

    // ROS 2 Humble 的 nav2_core::Controller 不包含 reset() 虚接口。
    // 保留为插件内部状态清理函数，由 cleanup() 主动调用。
    HERO_MPC_CONTROLLER_PUBLIC
    void reset();

   protected:
    // =========================================================================
    // MINCO 轨迹处理函数
    // =========================================================================

    /**
     * @brief MINCO 轨迹回调函数
     *
     * 解析消息并缓存轨迹数据
     *
     * @param msg MINCO 轨迹消息
     */
    void mincoTrajectoryCallback(
        const rm_interfaces::msg::MincoTrajectory::SharedPtr msg);

    /**
     * @brief 检查新多项式轨迹是否确实来自给定的 Nav2 Path
     *
     * MINCO 发布可能早于或晚于 ControllerServer 调用 setPlan()。通过
     * frame 和首尾点建立关联；时间戳只用于诊断，因为 1 Hz 重规划时
     * Planner/Smoother/Controller 看到的 header 时间戳不一定相同。
     */
    bool trajectoryMatchesPlan(const MincoTrajectoryData& trajectory,
                               const nav_msgs::msg::Path& path,
                               std::string& mismatch_reason) const;

    /**
     * @brief 检查活动轨迹是否仍属于重规划后的同一个导航目标
     *
     * 重规划会把 Path 起点推进到机器人当前位置，所以不能再要求活动
     * 轨迹的原始起点与新 Path 起点一致。frame 和目标端点必须保持匹配，
     * 防止把上一导航目标的轨迹用于新目标。
     */
    bool trajectoryMatchesGoal(const MincoTrajectoryData& trajectory,
                               const nav_msgs::msg::Path& path,
                               std::string& mismatch_reason) const;

    /// 调用前必须持有 traj_mutex_
    void activateTrajectoryLocked(MincoTrajectoryData&& trajectory,
                                  std::uint64_t plan_generation);

    /// 调用前必须持有 traj_mutex_
    void invalidateActiveTrajectoryLocked();

    /**
     * @brief 当前活动轨迹是否仍处于最小执行驻留期
     *
     * 周期重规划可以继续运行，但同一目标的新轨迹不会每秒都把控制相位
     * 重置到 t=0。新目标、轨迹结束或活动轨迹失效时不会驻留。
     * 调用前必须持有 traj_mutex_。
     */
    bool shouldRetainActiveTrajectoryLocked(
        const nav_msgs::msg::Path& path, const rclcpp::Time& now,
        double& trajectory_age) const;

    /// 清除速度命令积分状态；下一周期从实测速度重新建立安全锚点
    void resetCommandState();

    /**
     * @brief 在 MINCO 轨迹上采样指定时间点的状态
     *
     * 根据当前时间 t，计算轨迹上的位置、速度、加速度
     * 如果 t 超出轨迹范围，返回终点状态
     *
     * @param t 相对于轨迹起始的时间 [s]
     * @return 采样状态
     */
    TrajectoryState sampleTrajectory(double t) const;

    /// 从不可变的轨迹快照采样，供一次 MPC 求解全过程使用
    static TrajectoryState sampleTrajectorySegments(
        const std::vector<MincoSegment>& segments, double t);

    /**
     * @brief 将实车位置投影到 MINCO 轨迹的有限时间区间
     *
     * 先按固定时间间隔粗采样，再在最优采样点附近做一维细化。搜索区间由
     * 控制进度状态限制，避免自交或回头路径把参考相位跳到远处的另一支。
     *
     * @param segments 不可变轨迹快照
     * @param x 实车世界坐标 X
     * @param y 实车世界坐标 Y
     * @param search_start 搜索起始时间 [s]
     * @param search_end 搜索结束时间 [s]
     * @param sample_interval 粗采样间隔 [s]
     * @param squared_distance [out] 投影点到实车的平方距离 [m²]
     * @return 投影点对应的轨迹时间 [s]
     */
    static double projectPositionOntoTrajectory(
        const std::vector<MincoSegment>& segments, double x, double y,
        double search_start, double search_end, double sample_interval,
        double& squared_distance);

    /**
     * @brief 评估五次多项式
     *
     * p(t) = c[0]*t^5 + c[1]*t^4 + c[2]*t^3 + c[3]*t^2 + c[4]*t + c[5]
     * 注意系数顺序: [c5, c4, c3, c2, c1, c0] 即高次在前
     *
     * @param coeffs 多项式系数 [c5, c4, c3, c2, c1, c0]
     * @param t 时间参数
     * @return 多项式值
     */
    static double evaluatePolynomial(const std::array<double, 6>& coeffs,
                                     double t);

    /**
     * @brief 评估五次多项式的一阶导数（速度）
     *
     * p'(t) = 5*c5*t^4 + 4*c4*t^3 + 3*c3*t^2 + 2*c2*t + c1
     *
     * @param coeffs 多项式系数 [c5, c4, c3, c2, c1, c0]
     * @param t 时间参数
     * @return 一阶导数值
     */
    static double evaluatePolynomialDerivative(
        const std::array<double, 6>& coeffs, double t);

    /**
     * @brief 评估五次多项式的二阶导数（加速度）
     *
     * p''(t) = 20*c5*t^3 + 12*c4*t^2 + 6*c3*t + 2*c2
     *
     * @param coeffs 多项式系数 [c5, c4, c3, c2, c1, c0]
     * @param t 时间参数
     * @return 二阶导数值
     */
    static double evaluatePolynomialSecondDerivative(
        const std::array<double, 6>& coeffs, double t);

    /**
     * @brief 查找给定时间所在的轨迹段
     *
     * @param t 相对于轨迹起始的时间 [s]
     * @param segment_index [out] 段索引
     * @param local_t [out] 段内局部时间
     * @return 是否在轨迹范围内
     */
    static bool findSegment(const std::vector<MincoSegment>& segments,
                            double t, size_t& segment_index,
                            double& local_t);

    // =========================================================================
    // 坐标系转换函数
    // =========================================================================

    /**
     * @brief 将机器人坐标系速度转换为世界坐标系速度
     *
     * 旋转公式:
     *   v_world_x = v_body_x * cos(yaw) - v_body_y * sin(yaw)
     *   v_world_y = v_body_x * sin(yaw) + v_body_y * cos(yaw)
     *
     * @param vx_body 机器人坐标系 X 速度
     * @param vy_body 机器人坐标系 Y 速度
     * @param yaw 机器人航向角
     * @param vx_world [out] 世界坐标系 X 速度
     * @param vy_world [out] 世界坐标系 Y 速度
     */
    static void bodyToWorld(double vx_body, double vy_body, double yaw,
                            double& vx_world, double& vy_world);

    /**
     * @brief 将世界坐标系速度转换为机器人坐标系速度
     *
     * 逆旋转公式:
     *   v_body_x = v_world_x * cos(yaw) + v_world_y * sin(yaw)
     *   v_body_y = -v_world_x * sin(yaw) + v_world_y * cos(yaw)
     *
     * @param vx_world 世界坐标系 X 速度
     * @param vy_world 世界坐标系 Y 速度
     * @param yaw 机器人航向角
     * @param vx_body [out] 机器人坐标系 X 速度
     * @param vy_body [out] 机器人坐标系 Y 速度
     */
    static void worldToBody(double vx_world, double vy_world, double yaw,
                            double& vx_body, double& vy_body);

    /**
     * @brief 角度归一化到 [-π, π]
     *
     * @param angle 输入角度 [rad]
     * @return 归一化后的角度 [rad]
     */
    static double normalizeAngle(double angle);

    // =========================================================================
    // 可视化函数
    // =========================================================================

    /**
     * @brief 发布预测轨迹用于可视化
     *
     * @param predicted_states 预测状态序列
     */
    void publishPredictedPath(const std::vector<StateVector>& predicted_states,
                              const std::string& frame_id);

    /**
     * @brief 发布参考轨迹用于可视化
     *
     * @param t_start 起始时间
     * @param trajectory_segments 当前控制周期使用的轨迹快照
     * @param frame_id 轨迹坐标系
     */
    void publishReferencePath(
        double t_start, const std::vector<MincoSegment>& trajectory_segments,
        const std::string& frame_id);

    bool isCollisionFree(
        const std::vector<StateVector>& predicted_states,
        const std::string& trajectory_frame) const;

   protected:
    // =========================================================================
    // 节点和 ROS 组件
    // =========================================================================

    /// 父节点弱指针
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

    /// 插件名称
    std::string plugin_name_;

    /// 日志器
    rclcpp::Logger logger_{rclcpp::get_logger("HeroMpcController")};

    /// 时钟
    rclcpp::Clock::SharedPtr clock_;

    /// TF2 缓冲区
    std::shared_ptr<tf2_ros::Buffer> tf_;

    /// Costmap
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

    // =========================================================================
    // 订阅器和发布器
    // =========================================================================

    /// MINCO 轨迹订阅器
    rclcpp::Subscription<rm_interfaces::msg::MincoTrajectory>::SharedPtr
        minco_traj_sub_;

    /// 预测路径发布器（可视化）
    rclcpp_lifecycle::LifecyclePublisher<
        visualization_msgs::msg::MarkerArray>::SharedPtr predicted_path_pub_;

    /// 参考路径发布器（可视化）
    rclcpp_lifecycle::LifecyclePublisher<
        visualization_msgs::msg::MarkerArray>::SharedPtr reference_path_pub_;

    /// Carrot Pose 发布器（预测点在 base_link 坐标系下的位置）
    rclcpp_lifecycle::LifecyclePublisher<
        rm_interfaces::msg::NavOutput>::SharedPtr carrot_pose_pub_;

    // =========================================================================
    // MPC 相关
    // =========================================================================

    /// MPC 求解器封装
    std::unique_ptr<MpcWrapper> mpc_wrapper_;

    /// 速度命令积分状态互斥锁（不得在持有它时再获取 traj_mutex_）
    mutable std::mutex command_mutex_;

    /// 上一控制周期发布的世界坐标系平移速度
    Eigen::Vector2d last_command_velocity_world_{Eigen::Vector2d::Zero()};

    /// 上一有效速度命令的时间
    rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};

    /// 是否已经建立速度命令积分状态
    bool command_state_valid_{false};

    /// 命令状态版本，防止并发 reset/compute 提交过期积分结果
    std::uint64_t command_state_version_{0};

    // =========================================================================
    // MINCO 轨迹数据
    // =========================================================================

    /// 轨迹数据互斥锁
    mutable std::mutex traj_mutex_;

    /// 已与当前 Path 完成关联、允许控制循环使用的轨迹
    MincoTrajectoryData active_trajectory_;

    /// 可能先于 setPlan() 到达、等待与下一条 Path 关联的轨迹
    MincoTrajectoryData pending_trajectory_;

    /// 活动轨迹版本；控制计算期间发生切换时用于阻止输出混合结果
    std::uint64_t active_trajectory_version_{0};

    /// 活动轨迹所关联的 Path 代次
    std::uint64_t active_plan_generation_{0};

    /// 当前活动轨迹真正进入控制器的时间；不复用 Smoother 的生成时间
    rclcpp::Time active_trajectory_activation_time_{0, 0, RCL_ROS_TIME};

    /**
     * @brief 当前活动轨迹的闭环参考进度
     *
     * 这些成员只允许在持有 traj_mutex_ 时访问。setPlan() 保留同一条
     * 活动轨迹时也保留该进度；只有真正激活另一条轨迹或失效时才重置。
     */
    bool reference_progress_valid_{false};
    double active_projected_time_{0.0};
    double active_reference_time_{0.0};

    // =========================================================================
    // 参数
    // =========================================================================

    /// 轨迹超时时间 [s]，超过此时间未更新则停止
    double traj_timeout_{1.0};

    /// MINCO 轨迹话题；保持相对名称以支持 Nav2 namespace
    std::string trajectory_topic_{"smoother_server/minco_polynomial_trajectory"};

    /// MINCO 多项式首尾点与 Nav2 Path 首尾点的最大匹配误差 [m]
    double trajectory_path_match_tolerance_{0.05};

    /// MINCO header 与 Nav2 Path header 的时间戳差诊断阈值 [s]
    double trajectory_path_stamp_tolerance_{0.01};

    /// 同一目标的活动轨迹至少连续执行该时长
    double minimum_trajectory_execution_time_{3.0};

    /// 墙钟参考相位最多领先实车投影进度的时间 [s]
    double reference_projection_max_time_lead_{0.25};

    /// 已初始化后每周期向前搜索投影点的最大时间窗口 [s]
    double reference_projection_search_window_{2.0};

    /// 位置投影的粗采样时间间隔 [s]
    double reference_projection_sample_interval_{0.05};

    /// 最大线速度 [m/s]
    double max_vel_{2.0};

    /// 参数配置的基础最大线速度，用于恢复 speed-limit
    double configured_max_vel_{2.0};

    /// 最大角速度 [rad/s]
    double max_omega_{1.5};

    /// 最大线加速度 [m/s²]
    double max_acc_{2.0};

    /// 最大角加速度 [rad/s²]
    double max_alpha_{1.0};

    /// 控制器频率 [Hz]
    double control_frequency_{100.0};

    /// 发布命令与实测速度之间允许的最大差值 [m/s]
    double max_command_tracking_error_{0.6};

    /// 控制回调中断超过该时长后丢弃速度命令积分状态 [s]
    double command_state_timeout_{0.2};

    /// TF 转换容差
    tf2::Duration transform_tolerance_;

    /// 权重矩阵 Q (状态)
    std::vector<double> weight_q_;

    /// 权重矩阵 R (控制)
    std::vector<double> weight_r_;

    /// 是否启用航向跟踪
    bool enable_yaw_tracking_{false};

    /// 参考航向角（当禁用航向跟踪时使用）
    double reference_yaw_{0.0};

    /// Carrot 预测点索引（用于发布 carrot_pose）
    int carrot_point_index_{5};

    /// 是否对 MPC 预测轨迹执行 costmap footprint 碰撞检查
    bool collision_check_enabled_{true};

    /// 每次检查的预测时域 [s]
    double collision_check_horizon_{0.8};

    /**
     * @brief footprintCostAtPose 的碰撞阈值
     *
     * 完整 footprint 检查默认采用 Nav2 LETHAL_OBSTACLE (254) 语义，
     * 避免把 inscribed inflation 与 footprint 几何重复计算。参数最大只允许
     * 为 254，保证 lethal/unknown 始终拒绝。
     */
    int collision_cost_threshold_{
        static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)};

    // =========================================================================
    // 全局路径缓存（兼容 Nav2 接口）
    // =========================================================================

    /// 全局路径
    nav_msgs::msg::Path global_plan_;

    /// 是否已有可关联的有效 Path
    bool global_plan_valid_{false};

    /// 每次 setPlan() 递增，防止控制循环输出跨 Path 的计算结果
    std::uint64_t plan_generation_{0};
};

}  // namespace hero_mpc_controller

#endif  // HERO_MPC_CONTROLLER__HERO_MPC_CONTROLLER_HPP_
