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
 * @file mpc_wrapper.hpp
 * @brief 基于 acados 的 MPC 求解器封装层 (Layer 2)
 *
 * 本文件实现了对 acados 生成的 C 代码的 C++ 封装，不包含任何 ROS 依赖。
 * 适用于全向轮移动机器人的动力学 MPC 轨迹跟踪控制。
 *
 * 动力学模型说明：
 * - 状态变量 (nx=6): [p_x, p_y, ψ, v_x, v_y, ω]^T
 *   - p_x, p_y: 世界坐标系下的位置 [m]
 *   - ψ: 航向角 (Yaw) [rad]
 *   - v_x, v_y: 世界坐标系下的速度 [m/s]
 *   - ω: 角速度 [rad/s]
 *
 * - 控制变量 (nu=3): [a_x, a_y, α]^T
 *   - a_x, a_y: 世界坐标系下的加速度 [m/s²]
 *   - α: 角加速度 [rad/s²]
 *
 * - 代价函数参考 (ny=9): [x; u] = [p_x, p_y, ψ, v_x, v_y, ω, a_x, a_y, α]
 * - 终端代价参考 (ny_e=6): [p_x, p_y, ψ, v_x, v_y, ω]
 *
 * 作者: Jinbo Liu
 * 日期: 2025.12.27
 */

#ifndef HERO_MPC_CONTROLLER__MPC_WRAPPER_HPP_
#define HERO_MPC_CONTROLLER__MPC_WRAPPER_HPP_

#include <array>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>

// Eigen 用于矩阵运算
#include <Eigen/Dense>

// =============================================================================
// acados 生成代码的头文件
// 注意：这些头文件由 acados 代码生成器自动生成
// =============================================================================
extern "C" {
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_omnidirectional_robot_dynamic.h"
}

namespace hero_mpc_controller {

// =============================================================================
// 编译时常量定义（与 Python 模型和 acados 代码生成配置一致）
// =============================================================================

/// 预测时域步数 N
static constexpr int kHorizonSteps = OMNIDIRECTIONAL_ROBOT_DYNAMIC_N;

/// 状态维度 nx = 6 [p_x, p_y, ψ, v_x, v_y, ω]
static constexpr int kStateSize = OMNIDIRECTIONAL_ROBOT_DYNAMIC_NX;

/// 控制维度 nu = 3 [a_x, a_y, α]
static constexpr int kInputSize = OMNIDIRECTIONAL_ROBOT_DYNAMIC_NU;

/// 路径代价参考维度 ny = nx + nu = 9
static constexpr int kRefSize = OMNIDIRECTIONAL_ROBOT_DYNAMIC_NY;

/// 终端代价参考维度 ny_e = nx = 6
static constexpr int kEndRefSize = OMNIDIRECTIONAL_ROBOT_DYNAMIC_NYN;

/// 预测时域总时间 Tf [s]
static constexpr double kPredictionHorizon = 2.0;

/// MPC 采样时间 dt = Tf / N [s]
static constexpr double kMpcTimestep = kPredictionHorizon / kHorizonSteps;

// =============================================================================
// 状态和控制索引枚举
// =============================================================================

/// 状态变量索引
enum StateIndex {
    kPosX = 0,  ///< 世界坐标系 X 位置
    kPosY = 1,  ///< 世界坐标系 Y 位置
    kPsi = 2,   ///< 航向角 (Yaw)
    kVelX = 3,  ///< 世界坐标系 X 速度
    kVelY = 4,  ///< 世界坐标系 Y 速度
    kOmega = 5  ///< 角速度
};

/// 控制变量索引
enum InputIndex {
    kAccX = 0,  ///< X 方向加速度
    kAccY = 1,  ///< Y 方向加速度
    kAlpha = 2  ///< 角加速度
};

// =============================================================================
// 类型定义
// =============================================================================

/// 状态向量类型 (6x1)
using StateVector = Eigen::Matrix<double, kStateSize, 1>;

/// 控制向量类型 (3x1)
using InputVector = Eigen::Matrix<double, kInputSize, 1>;

/// 路径参考向量类型 (9x1)
using RefVector = Eigen::Matrix<double, kRefSize, 1>;

/// 终端参考向量类型 (6x1)
using EndRefVector = Eigen::Matrix<double, kEndRefSize, 1>;

/// 状态权重矩阵类型 (6x6)
using StateWeightMatrix = Eigen::Matrix<double, kStateSize, kStateSize>;

/// 控制权重矩阵类型 (3x3)
using InputWeightMatrix = Eigen::Matrix<double, kInputSize, kInputSize>;

/// 路径代价权重矩阵类型 (9x9)
using CostWeightMatrix = Eigen::Matrix<double, kRefSize, kRefSize>;

// =============================================================================
// MPC 求解器封装类
// =============================================================================

/**
 * @class MpcWrapper
 * @brief 封装 acados 生成的 OCP 求解器，提供简洁的 C++ 接口
 *
 * 本类不包含任何 ROS 依赖，可独立进行单元测试。
 * 主要功能：
 * 1. 初始化和释放 acados 求解器资源
 * 2. 设置代价函数权重
 * 3. 设置当前状态和参考轨迹
 * 4. 求解 OCP 并获取最优控制量
 *
 * 使用示例：
 * @code
 *   MpcWrapper mpc;
 *   mpc.init();
 *   mpc.set_state(x, y, psi, vx, vy, omega);
 *   for (int k = 0; k <= N; ++k) {
 *       mpc.set_reference(k, ref_data);
 *   }
 *   int status = mpc.solve();
 *   if (status == 0) {
 *       auto u = mpc.get_optimal_control();
 *       auto x_next = mpc.get_optimal_state(1);
 *   }
 * @endcode
 */
class MpcWrapper {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 默认构造函数
     */
    MpcWrapper();

    /**
     * @brief 析构函数，释放 acados 求解器资源
     */
    ~MpcWrapper();

    // 禁用拷贝（acados 求解器资源不可拷贝）
    MpcWrapper(const MpcWrapper &) = delete;
    MpcWrapper &operator=(const MpcWrapper &) = delete;

    // 允许移动
    MpcWrapper(MpcWrapper &&other) noexcept;
    MpcWrapper &operator=(MpcWrapper &&other) noexcept;

    // =========================================================================
    // 初始化接口
    // =========================================================================

    /**
     * @brief 初始化 acados OCP 求解器
     *
     * 调用 acados 的创建函数分配求解器资源，并设置默认参数。
     *
     * @return true 初始化成功
     * @return false 初始化失败
     */
    bool init();

    /**
     * @brief 检查求解器是否已初始化
     *
     * @return true 已初始化
     * @return false 未初始化
     */
    bool is_initialized() const { return is_initialized_; }

    // =========================================================================
    // 参数设置接口
    // =========================================================================

    /**
     * @brief 设置代价函数权重矩阵
     *
     * 代价函数形式: J = Σ (y - y_ref)^T W (y - y_ref) + (y_N - y_ref_N)^T W_e
     * (y_N - y_ref_N) 其中 y = [x; u], y_N = x
     *
     * @param Q 状态权重矩阵 (6x6)，建议对角阵
     * @param R 控制权重矩阵 (3x3)，建议对角阵
     * @return true 设置成功
     * @return false 设置失败
     */
    bool set_weights(const StateWeightMatrix &Q, const InputWeightMatrix &R);

    /**
     * @brief 设置终端代价权重矩阵
     *
     * @param Q_e 终端状态权重矩阵 (6x6)
     * @return true 设置成功
     * @return false 设置失败
     */
    bool set_terminal_weights(const StateWeightMatrix &Q_e);

    /**
     * @brief 设置控制量约束（加速度限制）
     *
     * @param a_max 最大线加速度 [m/s²]
     * @param alpha_max 最大角加速度 [rad/s²]
     * @return true 设置成功
     * @return false 设置失败
     */
    bool set_control_bounds(double a_max, double alpha_max);

    /**
     * @brief 设置速度状态约束
     *
     * @param v_max 最大线速度 [m/s]
     * @param omega_max 最大角速度 [rad/s]
     * @return true 设置成功
     * @return false 设置失败
     */
    bool set_velocity_bounds(double v_max, double omega_max);

    // =========================================================================
    // 状态和参考轨迹设置接口
    // =========================================================================

    /**
     * @brief 设置当前机器人状态（MPC 初始状态约束）
     *
     * 这是 MPC 求解的边界条件 x_0 = x_current。
     *
     * @param x 世界坐标系 X 位置 [m]
     * @param y 世界坐标系 Y 位置 [m]
     * @param psi 航向角 [rad]
     * @param vx 世界坐标系 X 速度 [m/s]
     * @param vy 世界坐标系 Y 速度 [m/s]
     * @param omega 角速度 [rad/s]
     * @return true 设置成功
     * @return false 设置失败
     */
    bool set_state(double x, double y, double psi, double vx, double vy,
                   double omega);

    /**
     * @brief 设置当前机器人状态（向量形式）
     *
     * @param state 状态向量 [p_x, p_y, ψ, v_x, v_y, ω]^T
     * @return true 设置成功
     * @return false 设置失败
     */
    bool set_state(const StateVector &state);

    /**
     * @brief 设置预测时域内某一步的参考轨迹
     *
     * 参考数据格式 (路径代价, k < N):
     *   ref_data = [p_x_ref, p_y_ref, ψ_ref, v_x_ref, v_y_ref, ω_ref, a_x_ref,
     * a_y_ref, α_ref]
     *
     * 终端代价 (k == N):
     *   ref_data = [p_x_ref, p_y_ref, ψ_ref, v_x_ref, v_y_ref, ω_ref]
     *
     * @param stage 预测步索引 k ∈ [0, N]
     * @param ref_data 参考数据，k < N 时为 9 维，k == N 时为 6 维
     * @return true 设置成功
     * @return false 设置失败
     */
    bool set_reference(int stage, const double *ref_data);

    /**
     * @brief 设置路径代价参考（向量形式）
     *
     * @param stage 预测步索引 k ∈ [0, N-1]
     * @param ref 参考向量 (9x1)
     * @return true 设置成功
     * @return false 设置失败
     */
    bool set_reference(int stage, const RefVector &ref);

    /**
     * @brief 设置终端代价参考
     *
     * @param ref_e 终端参考向量 (6x1)
     * @return true 设置成功
     * @return false 设置失败
     */
    bool set_terminal_reference(const EndRefVector &ref_e);

    // =========================================================================
    // 求解接口
    // =========================================================================

    /**
     * @brief 求解 OCP 问题
     *
     * 在调用此函数前，必须先设置：
     * 1. 当前状态 (set_state)
     * 2. 参考轨迹 (set_reference)
     *
     * @return int acados 求解状态码
     *         - 0: 求解成功
     *         - 1: QP 求解失败
     *         - 2: 最大迭代次数
     *         - 其他: 参见 acados 文档
     */
    int solve();

    /**
     * @brief 获取求解时间
     *
     * @return double 上一次 solve() 的耗时 [s]
     */
    double get_solve_time() const { return solve_time_; }

    // =========================================================================
    // 结果获取接口
    // =========================================================================

    /**
     * @brief 获取最优控制量 u_0
     *
     * @return InputVector 最优控制量 [a_x, a_y, α]^T
     */
    InputVector get_optimal_control() const;

    /**
     * @brief 获取预测时域内某一步的控制量
     *
     * @param stage 预测步索引 k ∈ [0, N-1]
     * @return InputVector 控制量 [a_x, a_y, α]^T
     */
    InputVector get_control(int stage) const;

    /**
     * @brief 获取预测的下一个状态 x_1
     *
     * 用于提取速度指令：x_1 中的 [v_x, v_y, ω] 可直接作为速度指令。
     *
     * @return StateVector 预测状态 [p_x, p_y, ψ, v_x, v_y, ω]^T
     */
    StateVector get_next_state() const;

    /**
     * @brief 获取预测时域内某一步的状态
     *
     * @param stage 预测步索引 k ∈ [0, N]
     * @return StateVector 状态 [p_x, p_y, ψ, v_x, v_y, ω]^T
     */
    StateVector get_state(int stage) const;

    /**
     * @brief 获取所有预测状态（用于可视化）
     *
     * @return std::vector<StateVector> 预测状态序列 [x_0, x_1, ..., x_N]
     */
    std::vector<StateVector> get_predicted_states() const;

    /**
     * @brief 获取所有预测控制（用于可视化）
     *
     * @return std::vector<InputVector> 预测控制序列 [u_0, u_1, ..., u_{N-1}]
     */
    std::vector<InputVector> get_predicted_controls() const;

    // =========================================================================
    // 工具函数
    // =========================================================================

    /**
     * @brief 获取预测时域步数 N
     */
    static constexpr int get_horizon_steps() { return kHorizonSteps; }

    /**
     * @brief 获取 MPC 采样时间 dt
     */
    static constexpr double get_timestep() { return kMpcTimestep; }

    /**
     * @brief 获取预测时域总时间 Tf
     */
    static constexpr double get_prediction_horizon() {
        return kPredictionHorizon;
    }

   private:
    /// acados OCP 求解器句柄
    omnidirectional_robot_dynamic_solver_capsule *acados_ocp_capsule_{nullptr};

    /// acados NLP 配置
    ocp_nlp_config *nlp_config_{nullptr};

    /// acados NLP 维度
    ocp_nlp_dims *nlp_dims_{nullptr};

    /// acados NLP 输入结构
    ocp_nlp_in *nlp_in_{nullptr};

    /// acados NLP 输出结构
    ocp_nlp_out *nlp_out_{nullptr};

    /// acados NLP 求解器
    ocp_nlp_solver *nlp_solver_{nullptr};

    /// 是否已初始化
    bool is_initialized_{false};

    /// 上一次求解耗时 [s]
    double solve_time_{0.0};

    /// 当前状态缓存
    StateVector current_state_;

    /// 路径代价权重矩阵 W (9x9)
    CostWeightMatrix W_;

    /// 终端代价权重矩阵 W_e (6x6)
    StateWeightMatrix W_e_;
};

}  // namespace hero_mpc_controller

#endif  // HERO_MPC_CONTROLLER__MPC_WRAPPER_HPP_
