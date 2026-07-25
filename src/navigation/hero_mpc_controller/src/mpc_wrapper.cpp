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
 * @file mpc_wrapper.cpp
 * @brief 基于 acados 的 MPC 求解器封装层实现
 *
 * 本文件实现了 MpcWrapper 类的所有成员函数。
 * 核心功能是将 acados 生成的 C 代码接口封装为易用的 C++ 类。
 *
 * 关键实现细节：
 * 1. 使用 acados 的 ocp_nlp 接口设置参数和获取结果
 * 2. 代价函数采用 NONLINEAR_LS 形式，y = [x; u]
 * 3. 权重矩阵 W 需要手动组装为块对角矩阵
 *
 * 作者: Jinbo Liu
 * 日期: 2025.12.27
 */

#include "hero_mpc_controller/mpc_wrapper.hpp"

#include <chrono>
#include <stdexcept>

namespace hero_mpc_controller {

// =============================================================================
// 构造函数和析构函数
// =============================================================================

MpcWrapper::MpcWrapper()
    : acados_ocp_capsule_(nullptr),
      nlp_config_(nullptr),
      nlp_dims_(nullptr),
      nlp_in_(nullptr),
      nlp_out_(nullptr),
      nlp_solver_(nullptr),
      is_initialized_(false),
      solve_time_(0.0) {
    // 初始化状态缓存
    current_state_.setZero();

    // 初始化默认权重矩阵
    // 参考 Python 模型中的权重设置
    W_.setZero();
    W_e_.setZero();

    // 默认状态权重 Q (与 Python 模型一致)
    StateWeightMatrix Q = StateWeightMatrix::Zero();
    Q(kPosX, kPosX) = 400.0;  // p_x 位置误差权重
    Q(kPosY, kPosY) = 100.0;  // p_y 位置误差权重
    Q(kPsi, kPsi) = 20.0;     // ψ 航向误差权重
    Q(kVelX, kVelX) = 10.0;   // v_x 速度误差权重
    Q(kVelY, kVelY) = 10.0;   // v_y 速度误差权重
    Q(kOmega, kOmega) = 5.0;  // ω 角速度误差权重

    // 默认控制权重 R (与 Python 模型一致)
    InputWeightMatrix R = InputWeightMatrix::Zero();
    R(kAccX, kAccX) = 1.0;    // a_x 加速度权重
    R(kAccY, kAccY) = 1.0;    // a_y 加速度权重
    R(kAlpha, kAlpha) = 0.5;  // α 角加速度权重

    // 组装 W 矩阵: W = [Q, 0; 0, R]
    W_.block<kStateSize, kStateSize>(0, 0) = Q;
    W_.block<kInputSize, kInputSize>(kStateSize, kStateSize) = R;

    // 默认终端权重
    W_e_(kPosX, kPosX) = 200.0;
    W_e_(kPosY, kPosY) = 200.0;
    W_e_(kPsi, kPsi) = 50.0;
    W_e_(kVelX, kVelX) = 20.0;
    W_e_(kVelY, kVelY) = 20.0;
    W_e_(kOmega, kOmega) = 10.0;
}

MpcWrapper::~MpcWrapper() {
    // 释放 acados 求解器资源
    if (acados_ocp_capsule_ != nullptr) {
        omnidirectional_robot_dynamic_acados_free(acados_ocp_capsule_);
        omnidirectional_robot_dynamic_acados_free_capsule(acados_ocp_capsule_);
        acados_ocp_capsule_ = nullptr;
    }
    is_initialized_ = false;
}

// 移动构造函数
MpcWrapper::MpcWrapper(MpcWrapper&& other) noexcept
    : acados_ocp_capsule_(other.acados_ocp_capsule_),
      nlp_config_(other.nlp_config_),
      nlp_dims_(other.nlp_dims_),
      nlp_in_(other.nlp_in_),
      nlp_out_(other.nlp_out_),
      nlp_solver_(other.nlp_solver_),
      is_initialized_(other.is_initialized_),
      solve_time_(other.solve_time_),
      current_state_(other.current_state_),
      W_(other.W_),
      W_e_(other.W_e_) {
    // 清空源对象的指针
    other.acados_ocp_capsule_ = nullptr;
    other.nlp_config_ = nullptr;
    other.nlp_dims_ = nullptr;
    other.nlp_in_ = nullptr;
    other.nlp_out_ = nullptr;
    other.nlp_solver_ = nullptr;
    other.is_initialized_ = false;
}

// 移动赋值运算符
MpcWrapper& MpcWrapper::operator=(MpcWrapper&& other) noexcept {
    if (this != &other) {
        // 释放当前资源
        if (acados_ocp_capsule_ != nullptr) {
            omnidirectional_robot_dynamic_acados_free(acados_ocp_capsule_);
            omnidirectional_robot_dynamic_acados_free_capsule(
                acados_ocp_capsule_);
        }

        // 移动资源
        acados_ocp_capsule_ = other.acados_ocp_capsule_;
        nlp_config_ = other.nlp_config_;
        nlp_dims_ = other.nlp_dims_;
        nlp_in_ = other.nlp_in_;
        nlp_out_ = other.nlp_out_;
        nlp_solver_ = other.nlp_solver_;
        is_initialized_ = other.is_initialized_;
        solve_time_ = other.solve_time_;
        current_state_ = other.current_state_;
        W_ = other.W_;
        W_e_ = other.W_e_;

        // 清空源对象的指针
        other.acados_ocp_capsule_ = nullptr;
        other.nlp_config_ = nullptr;
        other.nlp_dims_ = nullptr;
        other.nlp_in_ = nullptr;
        other.nlp_out_ = nullptr;
        other.nlp_solver_ = nullptr;
        other.is_initialized_ = false;
    }
    return *this;
}

// =============================================================================
// 初始化接口
// =============================================================================

bool MpcWrapper::init() {
    // 防止重复初始化
    if (is_initialized_) {
        std::cerr << "[MpcWrapper] 警告: 求解器已初始化，跳过" << std::endl;
        return true;
    }

    // -------------------------------------------------------------------------
    // 步骤 1: 创建 acados OCP 求解器 capsule
    // -------------------------------------------------------------------------
    acados_ocp_capsule_ = omnidirectional_robot_dynamic_acados_create_capsule();
    if (acados_ocp_capsule_ == nullptr) {
        std::cerr << "[MpcWrapper] 错误: 无法创建 acados capsule" << std::endl;
        return false;
    }

    // -------------------------------------------------------------------------
    // 步骤 2: 创建求解器（使用默认参数）
    // -------------------------------------------------------------------------
    int status =
        omnidirectional_robot_dynamic_acados_create(acados_ocp_capsule_);
    if (status != 0) {
        std::cerr << "[MpcWrapper] 错误: acados 求解器创建失败，状态码 = "
                  << status << std::endl;
        omnidirectional_robot_dynamic_acados_free_capsule(acados_ocp_capsule_);
        acados_ocp_capsule_ = nullptr;
        return false;
    }

    // -------------------------------------------------------------------------
    // 步骤 3: 获取求解器内部结构指针
    // -------------------------------------------------------------------------
    nlp_config_ = omnidirectional_robot_dynamic_acados_get_nlp_config(
        acados_ocp_capsule_);
    nlp_dims_ =
        omnidirectional_robot_dynamic_acados_get_nlp_dims(acados_ocp_capsule_);
    nlp_in_ =
        omnidirectional_robot_dynamic_acados_get_nlp_in(acados_ocp_capsule_);
    nlp_out_ =
        omnidirectional_robot_dynamic_acados_get_nlp_out(acados_ocp_capsule_);
    nlp_solver_ = omnidirectional_robot_dynamic_acados_get_nlp_solver(
        acados_ocp_capsule_);

    // -------------------------------------------------------------------------
    // 步骤 4: 设置默认权重矩阵
    // -------------------------------------------------------------------------
    // 路径代价权重 W (对于 k = 0, ..., N-1)
    for (int k = 0; k < kHorizonSteps; ++k) {
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, k, "W",
                               W_.data());
    }

    // 终端代价权重 W_e (对于 k = N)
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, kHorizonSteps, "W",
                           W_e_.data());

    std::cout << "[MpcWrapper] acados 求解器初始化成功" << std::endl;
    std::cout << "  - 预测步数 N = " << kHorizonSteps << std::endl;
    std::cout << "  - 采样时间 dt = " << kMpcTimestep << " s" << std::endl;
    std::cout << "  - 状态维度 nx = " << kStateSize << std::endl;
    std::cout << "  - 控制维度 nu = " << kInputSize << std::endl;

    is_initialized_ = true;
    return true;
}

// =============================================================================
// 参数设置接口
// =============================================================================

bool MpcWrapper::set_weights(const StateWeightMatrix& Q,
                             const InputWeightMatrix& R) {
    if (!is_initialized_) {
        std::cerr << "[MpcWrapper] 错误: 求解器未初始化" << std::endl;
        return false;
    }

    // 组装 W 矩阵: W = [Q, 0; 0, R]
    W_.setZero();
    W_.block<kStateSize, kStateSize>(0, 0) = Q;
    W_.block<kInputSize, kInputSize>(kStateSize, kStateSize) = R;

    // 设置路径代价权重 (k = 0, ..., N-1)
    for (int k = 0; k < kHorizonSteps; ++k) {
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, k, "W",
                               W_.data());
    }

    return true;
}

bool MpcWrapper::set_terminal_weights(const StateWeightMatrix& Q_e) {
    if (!is_initialized_) {
        std::cerr << "[MpcWrapper] 错误: 求解器未初始化" << std::endl;
        return false;
    }

    W_e_ = Q_e;

    // 设置终端代价权重 (k = N)
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, kHorizonSteps, "W",
                           W_e_.data());

    return true;
}

bool MpcWrapper::set_control_bounds(double a_max, double alpha_max) {
    if (!is_initialized_) {
        std::cerr << "[MpcWrapper] 错误: 求解器未初始化" << std::endl;
        return false;
    }

    // 控制量下界: [-a_max, -a_max, -alpha_max]
    InputVector lbu;
    lbu << -a_max, -a_max, -alpha_max;

    // 控制量上界: [a_max, a_max, alpha_max]
    InputVector ubu;
    ubu << a_max, a_max, alpha_max;

    // 设置所有阶段的控制约束
    for (int k = 0; k < kHorizonSteps; ++k) {
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                      k, "lbu", static_cast<void*>(lbu.data()));
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                      k, "ubu", static_cast<void*>(ubu.data()));
    }

    return true;
}

bool MpcWrapper::set_velocity_bounds(double v_max, double omega_max) {
    if (!is_initialized_) {
        std::cerr << "[MpcWrapper] 错误: 求解器未初始化" << std::endl;
        return false;
    }

    // 速度状态约束索引: [v_x, v_y, ω] 对应状态索引 [3, 4, 5]
    // 下界
    Eigen::Vector3d lbx;
    lbx << -v_max, -v_max, -omega_max;

    // 上界
    Eigen::Vector3d ubx;
    ubx << v_max, v_max, omega_max;

    // 设置所有阶段的状态约束 (k = 1, ..., N)
    // 注意: k = 0 由初始状态约束设定
    for (int k = 1; k <= kHorizonSteps; ++k) {
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                      k, "lbx", static_cast<void*>(lbx.data()));
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                      k, "ubx", static_cast<void*>(ubx.data()));
    }

    return true;
}

// =============================================================================
// 状态和参考轨迹设置接口
// =============================================================================

bool MpcWrapper::set_state(double x, double y, double psi, double vx, double vy,
                           double omega) {
    StateVector state;
    state << x, y, psi, vx, vy, omega;
    return set_state(state);
}

bool MpcWrapper::set_state(const StateVector& state) {
    if (!is_initialized_) {
        std::cerr << "[MpcWrapper] 错误: 求解器未初始化" << std::endl;
        return false;
    }

    current_state_ = state;

    // -------------------------------------------------------------------------
    // 关键步骤: 设置 MPC 初始状态约束 x_0 = x_current
    // 在 acados 中，通过设置 lbx_0 = ubx_0 = x_current 来固定初始状态
    // -------------------------------------------------------------------------
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0,
                                  "lbx",
                                  static_cast<void*>(current_state_.data()));
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0,
                                  "ubx",
                                  static_cast<void*>(current_state_.data()));

    return true;
}

bool MpcWrapper::set_reference(int stage, const double* ref_data) {
    if (!is_initialized_) {
        std::cerr << "[MpcWrapper] 错误: 求解器未初始化" << std::endl;
        return false;
    }

    if (stage < 0 || stage > kHorizonSteps) {
        std::cerr << "[MpcWrapper] 错误: 无效的阶段索引 " << stage << std::endl;
        return false;
    }

    if (stage < kHorizonSteps) {
        // 路径代价参考 (k = 0, ..., N-1)
        // y_ref = [p_x, p_y, ψ, v_x, v_y, ω, a_x, a_y, α]
        ocp_nlp_cost_model_set(
            nlp_config_, nlp_dims_, nlp_in_, stage, "yref",
            const_cast<void*>(static_cast<const void*>(ref_data)));
    } else {
        // 终端代价参考 (k = N)
        // y_ref_e = [p_x, p_y, ψ, v_x, v_y, ω]
        ocp_nlp_cost_model_set(
            nlp_config_, nlp_dims_, nlp_in_, stage, "yref",
            const_cast<void*>(static_cast<const void*>(ref_data)));
    }

    return true;
}

bool MpcWrapper::set_reference(int stage, const RefVector& ref) {
    return set_reference(stage, ref.data());
}

bool MpcWrapper::set_terminal_reference(const EndRefVector& ref_e) {
    return set_reference(kHorizonSteps, ref_e.data());
}

// =============================================================================
// 求解接口
// =============================================================================

int MpcWrapper::solve() {
    if (!is_initialized_) {
        std::cerr << "[MpcWrapper] 错误: 求解器未初始化" << std::endl;
        return -1;
    }

    // 记录求解开始时间
    auto start_time = std::chrono::high_resolution_clock::now();

    // -------------------------------------------------------------------------
    // 调用 acados 求解器
    // -------------------------------------------------------------------------
    int status =
        omnidirectional_robot_dynamic_acados_solve(acados_ocp_capsule_);

    // 记录求解结束时间
    auto end_time = std::chrono::high_resolution_clock::now();
    solve_time_ = std::chrono::duration<double>(end_time - start_time).count();

    // 输出求解状态
    if (status != 0) {
        std::cerr << "[MpcWrapper] 警告: acados 求解返回非零状态 " << status
                  << std::endl;
    }

    return status;
}

// =============================================================================
// 结果获取接口
// =============================================================================

InputVector MpcWrapper::get_optimal_control() const { return get_control(0); }

InputVector MpcWrapper::get_control(int stage) const {
    InputVector u;
    u.setZero();

    if (!is_initialized_) {
        std::cerr << "[MpcWrapper] 错误: 求解器未初始化" << std::endl;
        return u;
    }

    if (stage < 0 || stage >= kHorizonSteps) {
        std::cerr << "[MpcWrapper] 错误: 无效的阶段索引 " << stage << std::endl;
        return u;
    }

    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, stage, "u", u.data());

    return u;
}

StateVector MpcWrapper::get_next_state() const { return get_state(1); }

StateVector MpcWrapper::get_state(int stage) const {
    StateVector x;
    x.setZero();

    if (!is_initialized_) {
        std::cerr << "[MpcWrapper] 错误: 求解器未初始化" << std::endl;
        return x;
    }

    if (stage < 0 || stage > kHorizonSteps) {
        std::cerr << "[MpcWrapper] 错误: 无效的阶段索引 " << stage << std::endl;
        return x;
    }

    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, stage, "x", x.data());

    return x;
}

std::vector<StateVector> MpcWrapper::get_predicted_states() const {
    std::vector<StateVector> states(kHorizonSteps + 1);

    if (!is_initialized_) {
        std::cerr << "[MpcWrapper] 错误: 求解器未初始化" << std::endl;
        return states;
    }

    for (int k = 0; k <= kHorizonSteps; ++k) {
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, k, "x",
                        states[k].data());
    }

    return states;
}

std::vector<InputVector> MpcWrapper::get_predicted_controls() const {
    std::vector<InputVector> controls(kHorizonSteps);

    if (!is_initialized_) {
        std::cerr << "[MpcWrapper] 错误: 求解器未初始化" << std::endl;
        return controls;
    }

    for (int k = 0; k < kHorizonSteps; ++k) {
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, k, "u",
                        controls[k].data());
    }

    return controls;
}

}  // namespace hero_mpc_controller
