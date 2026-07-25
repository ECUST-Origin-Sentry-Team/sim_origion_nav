#
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# The 2-Clause BSD License
#

"""
全向轮移动机器人动力学模型 (升级版)

本文件定义了包含速度状态的全向轮机器人动力学模型，用于高性能 MPC 轨迹跟踪控制。
相比纯运动学模型，动力学模型能更好地处理加速度约束和速度平滑性。

=============================================================================
数学模型说明
=============================================================================

【状态变量 (State Variables, nx=6)】:
    - p_x    : 机器人在世界坐标系下的 X 位置 [m]
    - p_y    : 机器人在世界坐标系下的 Y 位置 [m]
    - ψ      : 机器人的航向角（Yaw）[rad]
    - v_x    : 世界坐标系下的 X 方向速度 [m/s]
    - v_y    : 世界坐标系下的 Y 方向速度 [m/s]
    - ω      : 绕 Z 轴的角速度 [rad/s]

【控制变量 (Control Variables, nu=3)】:
    - F_x    : 世界坐标系下的 X 方向控制力/质量 (即加速度 a_x) [m/s²]
    - F_y    : 世界坐标系下的 Y 方向控制力/质量 (即加速度 a_y) [m/s²]
    - M_z    : 绕 Z 轴的控制力矩/惯量 (即角加速度 α) [rad/s²]

【为什么选择世界坐标系速度？】
    参考文档中的模型使用世界坐标系速度，这有以下优势：
    1. 状态转移矩阵 A 和 B 为常数矩阵（不依赖于 θ），便于线性化和 QP 求解
    2. 轨迹规划器通常输出世界坐标系下的参考轨迹
    3. 横向误差和纵向误差的定义更直观

    离散化的状态转移方程（参考文档）：
    x_{k+1} = A * x_k + B * u_k

    其中（假设 m=1, Iz=1 归一化）：
    A = [1  0  0  Δt  0   0 ]      B = [0    0    0   ]
        [0  1  0  0   Δt  0 ]          [0    0    0   ]
        [0  0  1  0   0   Δt]          [0    0    0   ]
        [0  0  0  1   0   0 ]          [Δt   0    0   ]
        [0  0  0  0   1   0 ]          [0    Δt   0   ]
        [0  0  0  0   0   1 ]          [0    0    Δt  ]

【连续时间动力学方程】:
    ṗ_x = v_x
    ṗ_y = v_y
    ψ̇   = ω
    v̇_x = F_x / m = a_x  (假设 m=1)
    v̇_y = F_y / m = a_y  (假设 m=1)
    ω̇   = M_z / I_z = α  (假设 I_z=1)

作者: Jinbo Liu
日期: 2025.12.22
"""

from acados_template import AcadosModel
import casadi as ca
import numpy as np


def export_omnidirectional_dynamic_model() -> AcadosModel:
    """
    导出全向轮移动机器人的 6 状态动力学模型。
    
    状态: [p_x, p_y, ψ, v_x, v_y, ω]^T  (6维)
    控制: [a_x, a_y, α]^T               (3维，加速度)
    
    Returns:
        AcadosModel: 包含完整动力学方程的 acados 模型对象
    """
    
    # =========================================================================
    # 1. 创建模型对象
    # =========================================================================
    model_name = 'omnidirectional_robot_dynamic'
    model = AcadosModel()
    
    # =========================================================================
    # 2. 定义状态变量 (State Variables, nx=6)
    # =========================================================================
    
    # 位置状态 (Position)
    p_x = ca.SX.sym('p_x')      # 世界坐标系 X 位置 [m]
    p_y = ca.SX.sym('p_y')      # 世界坐标系 Y 位置 [m]
    psi = ca.SX.sym('psi')      # 航向角/偏航角 [rad]
    
    # 速度状态 (Velocity) - 使用世界坐标系速度
    v_x = ca.SX.sym('v_x')      # 世界坐标系 X 速度 [m/s]
    v_y = ca.SX.sym('v_y')      # 世界坐标系 Y 速度 [m/s]
    omega = ca.SX.sym('omega')  # 角速度 [rad/s]
    
    # 组合状态向量: x = [p_x, p_y, ψ, v_x, v_y, ω]^T
    states = ca.vertcat(p_x, p_y, psi, v_x, v_y, omega)
    
    # =========================================================================
    # 3. 定义控制变量 (Control Variables, nu=3)
    # =========================================================================
    
    # 加速度控制 (Acceleration control)
    # 物理意义: F/m 或 τ/I，这里假设质量和惯量归一化为 1
    a_x = ca.SX.sym('a_x')          # X 方向加速度 [m/s²]
    a_y = ca.SX.sym('a_y')          # Y 方向加速度 [m/s²]
    alpha = ca.SX.sym('alpha')      # 角加速度 [rad/s²]
    
    # 组合控制向量: u = [a_x, a_y, α]^T
    controls = ca.vertcat(a_x, a_y, alpha)
    
    # =========================================================================
    # 4. 定义状态导数变量 (用于隐式动力学)
    # =========================================================================
    p_x_dot = ca.SX.sym('p_x_dot')
    p_y_dot = ca.SX.sym('p_y_dot')
    psi_dot = ca.SX.sym('psi_dot')
    v_x_dot = ca.SX.sym('v_x_dot')
    v_y_dot = ca.SX.sym('v_y_dot')
    omega_dot = ca.SX.sym('omega_dot')
    
    states_dot = ca.vertcat(p_x_dot, p_y_dot, psi_dot, v_x_dot, v_y_dot, omega_dot)
    
    # =========================================================================
    # 5. 定义连续时间动力学方程 (Continuous-time Dynamics)
    # =========================================================================
    # 
    # 双积分器模型（世界坐标系）:
    #   ṗ_x = v_x         (位置导数 = 速度)
    #   ṗ_y = v_y
    #   ψ̇   = ω           (航向导数 = 角速度)
    #   v̇_x = a_x         (速度导数 = 加速度/控制输入)
    #   v̇_y = a_y
    #   ω̇   = α
    #
    # 这是一个简单的线性双积分器系统，非常适合 MPC 优化。
    # 实际的电机动力学可以通过调整加速度约束来近似。
    
    f_expl = ca.vertcat(
        v_x,        # ṗ_x = v_x
        v_y,        # ṗ_y = v_y
        omega,      # ψ̇ = ω
        a_x,        # v̇_x = a_x (控制输入)
        a_y,        # v̇_y = a_y (控制输入)
        alpha       # ω̇ = α (控制输入)
    )
    
    # 隐式动力学: f_impl = x_dot - f(x, u) = 0
    f_impl = states_dot - f_expl
    
    # =========================================================================
    # 6. 填充 AcadosModel 结构
    # =========================================================================
    model.name = model_name
    model.x = states
    model.xdot = states_dot
    model.u = controls
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl
    
    # =========================================================================
    # 7. 设置标签（用于绘图）
    # =========================================================================
    model.x_labels = [
        r'$p_x$ [m]',
        r'$p_y$ [m]', 
        r'$\psi$ [rad]',
        r'$v_x$ [m/s]',
        r'$v_y$ [m/s]',
        r'$\omega$ [rad/s]'
    ]
    
    model.u_labels = [
        r'$a_x$ [m/s²]',
        r'$a_y$ [m/s²]',
        r'$\alpha$ [rad/s²]'
    ]
    
    model.t_label = r'$t$ [s]'
    
    print(f"[INFO] 成功创建全向轮动力学模型: {model_name}")
    print(f"       状态维度 (nx): {states.rows()}")
    print(f"       控制维度 (nu): {controls.rows()}")
    
    return model


def export_omnidirectional_model() -> AcadosModel:
    """
    导出全向轮移动机器人的 3 状态运动学模型（向后兼容）。
    
    状态: [p_x, p_y, ψ]^T       (3维)
    控制: [v_x, v_y, ω]^T       (3维，速度)
    
    注意: 这是简化的运动学模型，建议使用 export_omnidirectional_dynamic_model()
    """
    
    model_name = 'omnidirectional_robot'
    model = AcadosModel()
    
    # 状态变量 (3维)
    p_x = ca.SX.sym('p_x')
    p_y = ca.SX.sym('p_y')
    psi = ca.SX.sym('psi')
    states = ca.vertcat(p_x, p_y, psi)
    
    # 控制变量 (3维) - 本体坐标系速度
    v_x_body = ca.SX.sym('v_x')    # 本体纵向速度
    v_y_body = ca.SX.sym('v_y')    # 本体横向速度
    omega = ca.SX.sym('omega')     # 角速度
    controls = ca.vertcat(v_x_body, v_y_body, omega)
    
    # 状态导数
    p_x_dot = ca.SX.sym('p_x_dot')
    p_y_dot = ca.SX.sym('p_y_dot')
    psi_dot = ca.SX.sym('psi_dot')
    states_dot = ca.vertcat(p_x_dot, p_y_dot, psi_dot)
    
    # 运动学方程 (本体速度 -> 世界速度)
    f_expl = ca.vertcat(
        v_x_body * ca.cos(psi) - v_y_body * ca.sin(psi),
        v_x_body * ca.sin(psi) + v_y_body * ca.cos(psi),
        omega
    )
    
    f_impl = states_dot - f_expl
    
    # 填充模型
    model.name = model_name
    model.x = states
    model.xdot = states_dot
    model.u = controls
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl
    
    model.x_labels = [r'$p_x$ [m]', r'$p_y$ [m]', r'$\psi$ [rad]']
    model.u_labels = [r'$v_x$ [m/s]', r'$v_y$ [m/s]', r'$\omega$ [rad/s]']
    
    print(f"[INFO] 成功创建全向轮运动学模型: {model_name}")
    print(f"       状态维度 (nx): {states.rows()}")
    print(f"       控制维度 (nu): {controls.rows()}")
    
    return model


# =============================================================================
# 模块测试
# =============================================================================
if __name__ == '__main__':
    print("=" * 70)
    print("测试全向轮机器人模型")
    print("=" * 70)
    
    print("\n--- 动力学模型 (6状态) ---")
    model_dyn = export_omnidirectional_dynamic_model()
    print(f"  状态: {model_dyn.x}")
    print(f"  控制: {model_dyn.u}")
    print(f"  显式动力学: {model_dyn.f_expl_expr}")
    
    print("\n--- 运动学模型 (3状态, 向后兼容) ---")
    model_kin = export_omnidirectional_model()
    print(f"  状态: {model_kin.x}")
    print(f"  控制: {model_kin.u}")
    
    print("\n模型创建成功！")
