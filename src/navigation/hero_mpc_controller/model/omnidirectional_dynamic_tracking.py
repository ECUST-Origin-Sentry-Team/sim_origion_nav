#
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# The 2-Clause BSD License
#

"""
全向轮移动机器人 MPC 动力学轨迹跟踪控制器 (6状态 + 二次采样策略)

本脚本实现了基于动力学模型的高性能 MPC 轨迹跟踪控制器。

=============================================================================
主要特性
=============================================================================

1. 【6状态动力学模型】
   - 状态: [p_x, p_y, ψ, v_x, v_y, ω]^T
   - 控制: [a_x, a_y, α]^T (加速度)
   - 优势: 能显式约束加速度，提升速度平滑性

2. 【二次采样策略 (Secondary Trajectory Resampling)】
   核心思想: 根据机器人当前速度方向与参考轨迹速度方向的一致性，
   动态调整预测视界内参考点的时间间隔。

   计算公式:
       α_k = max(0, (v_prim · v_ref) / (|v_prim| × |v_ref|))
           = max(0, cos(角度差))
   
   其中:
       v_prim = [v_x, v_y]^T    当前机器人速度（世界坐标系）
       v_ref  = 参考轨迹在该点的速度方向
   
   采样间隔:
       dt_k = α_k × dt_base
   
   物理意义:
       - α ≈ 1: 速度方向一致，正常采样间隔
       - α ≈ 0: 速度方向偏差大（或反向），缩短采样间隔
       - 当偏离轨迹时，参考点"聚集"在附近，帮助快速回到轨迹

3. 【微分平坦前馈控制】
   全向轮机器人是微分平坦系统，可解析计算参考控制量：
   - 参考速度: 轨迹的一阶导数
   - 参考加速度: 轨迹的二阶导数

作者: Jinbo Liu
日期: 2025.12.22
"""

import numpy as np
import matplotlib.pyplot as plt
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver
from typing import Tuple, Literal, List

# 导入 6 状态动力学模型
from omnidirectional_model import export_omnidirectional_dynamic_model


# =============================================================================
# 轨迹生成器类 (升级版 - 支持二阶导数)
# =============================================================================
class DynamicTrajectoryGenerator:
    """
    动力学轨迹生成器
    
    支持的轨迹类型：
    - 'figure8': 8字形轨迹 (Lissajous曲线)
    - 'circle': 圆形轨迹
    """
    
    def __init__(self, 
                 trajectory_type: Literal['figure8', 'circle'] = 'figure8',
                 scale: float = 1.0,
                 period: float = 10.0,
                 center: Tuple[float, float] = (0.0, 0.0)):
        """
        初始化轨迹生成器
        
        Args:
            trajectory_type: 轨迹类型 ('figure8' 或 'circle')
            scale: 轨迹缩放因子 [m]
            period: 完成一个周期的时间 [s]
            center: 轨迹中心点 (x_c, y_c) [m]
        """
        self.trajectory_type = trajectory_type
        self.scale = scale
        self.period = period
        self.center = center
        self.omega_traj = 2 * np.pi / period  # 轨迹角频率 [rad/s]
        
        print(f"[轨迹生成器] 类型: {trajectory_type}, 缩放: {scale}m, 周期: {period}s")
    
    def get_position(self, t: float) -> Tuple[float, float, float]:
        """
        获取 t 时刻的位置和航向
        
        Returns:
            (p_x, p_y, psi): 位置和航向角
        """
        omega = self.omega_traj
        
        if self.trajectory_type == 'figure8':
            # 8字形轨迹: x = A*sin(ωt), y = A*sin(2ωt)/2
            p_x = self.center[0] + self.scale * np.sin(omega * t)
            p_y = self.center[1] + self.scale * np.sin(2 * omega * t) / 2
            
            # 航向角 = 速度方向
            dx = self.scale * omega * np.cos(omega * t)
            dy = self.scale * omega * np.cos(2 * omega * t)
            psi = np.arctan2(dy, dx)
            
        elif self.trajectory_type == 'circle':
            # 圆形轨迹
            p_x = self.center[0] + self.scale * np.cos(omega * t)
            p_y = self.center[1] + self.scale * np.sin(omega * t)
            psi = omega * t + np.pi / 2  # 切线方向
            
        else:
            raise ValueError(f"未知轨迹类型: {self.trajectory_type}")
        
        return p_x, p_y, psi
    
    def get_velocity(self, t: float) -> Tuple[float, float, float]:
        """
        获取 t 时刻的世界坐标系速度 (一阶导数)
        
        Returns:
            (v_x, v_y, omega): 世界坐标系速度
        """
        omega = self.omega_traj
        
        if self.trajectory_type == 'figure8':
            v_x = self.scale * omega * np.cos(omega * t)
            v_y = self.scale * omega * np.cos(2 * omega * t)
            
            # 航向角变化率: θ̇ = (ẋÿ - ẏẍ) / (ẋ² + ẏ²)
            a_x = -self.scale * omega**2 * np.sin(omega * t)
            a_y = -2 * self.scale * omega**2 * np.sin(2 * omega * t)
            
            denom = v_x**2 + v_y**2
            if denom > 1e-6:
                omega_z = (v_x * a_y - v_y * a_x) / denom
            else:
                omega_z = 0.0
                
        elif self.trajectory_type == 'circle':
            v_x = -self.scale * omega * np.sin(omega * t)
            v_y = self.scale * omega * np.cos(omega * t)
            omega_z = omega
            
        else:
            raise ValueError(f"未知轨迹类型: {self.trajectory_type}")
        
        return v_x, v_y, omega_z
    
    def get_acceleration(self, t: float) -> Tuple[float, float, float]:
        """
        获取 t 时刻的世界坐标系加速度 (二阶导数)
        
        这是动力学模型的关键！用于计算参考控制量。
        
        Returns:
            (a_x, a_y, alpha): 世界坐标系加速度
        """
        omega = self.omega_traj
        
        if self.trajectory_type == 'figure8':
            # 二阶导数
            a_x = -self.scale * omega**2 * np.sin(omega * t)
            a_y = -2 * self.scale * omega**2 * np.sin(2 * omega * t)
            
            # 角加速度 (通过数值微分或解析计算)
            # 这里简化处理，实际应用中可能需要更精确的计算
            v_x, v_y, _ = self.get_velocity(t)
            
            # 三阶导数 (jerk)
            j_x = -self.scale * omega**3 * np.cos(omega * t)
            j_y = -4 * self.scale * omega**3 * np.cos(2 * omega * t)
            
            denom = v_x**2 + v_y**2
            if denom > 1e-6:
                # α = d/dt[(ẋÿ - ẏẍ)/(ẋ² + ẏ²)]
                # 简化：假设角加速度较小
                alpha = (v_x * j_y - v_y * j_x) / denom - \
                        2 * (v_x * a_y - v_y * a_x) * (v_x * a_x + v_y * a_y) / (denom**2)
            else:
                alpha = 0.0
                
        elif self.trajectory_type == 'circle':
            a_x = -self.scale * omega**2 * np.cos(omega * t)
            a_y = -self.scale * omega**2 * np.sin(omega * t)
            alpha = 0.0  # 匀速圆周运动，角加速度为零
            
        else:
            raise ValueError(f"未知轨迹类型: {self.trajectory_type}")
        
        return a_x, a_y, alpha
    
    def get_full_state(self, t: float) -> np.ndarray:
        """
        获取 t 时刻的完整 6 维状态
        
        Returns:
            state: [p_x, p_y, psi, v_x, v_y, omega]
        """
        p_x, p_y, psi = self.get_position(t)
        v_x, v_y, omega_z = self.get_velocity(t)
        return np.array([p_x, p_y, psi, v_x, v_y, omega_z])
    
    def get_ref_for_mpc(self, t: float) -> np.ndarray:
        """
        获取 MPC 代价函数的参考向量 (状态 + 控制)
        
        Returns:
            yref: [p_x, p_y, psi, v_x, v_y, omega, a_x, a_y, alpha] (9维)
        """
        state = self.get_full_state(t)
        a_x, a_y, alpha = self.get_acceleration(t)
        return np.concatenate([state, np.array([a_x, a_y, alpha])])
    
    def get_velocity_vector(self, t: float) -> np.ndarray:
        """
        获取 t 时刻的 2D 速度向量 (用于二次采样计算)
        
        Returns:
            v: [v_x, v_y]
        """
        v_x, v_y, _ = self.get_velocity(t)
        return np.array([v_x, v_y])
    
    def generate_full_trajectory(self, t_start: float, t_end: float, 
                                  dt: float = 0.01) -> Tuple[np.ndarray, np.ndarray]:
        """
        生成完整的参考轨迹（用于可视化）
        
        Returns:
            (t_array, state_array): 时间数组和状态数组 [N, 6]
        """
        t_array = np.arange(t_start, t_end, dt)
        state_array = np.array([self.get_full_state(t) for t in t_array])
        return t_array, state_array


# =============================================================================
# 二次采样策略类
# =============================================================================
class SecondaryResamplingStrategy:
    """
    二次轨迹采样策略
    
    核心思想:
    根据机器人当前速度与参考轨迹速度的方向一致性，动态调整采样间隔。
    
    公式:
        α = max(0, cos(θ_diff)) = max(0, v_prim · v_ref / (|v_prim| × |v_ref|))
        dt_k = α × dt_base
    
    当机器人偏离轨迹（速度方向不一致）时:
    - α 值减小 → 采样间隔缩短
    - 参考点"聚集"在轨迹的近端
    - 帮助机器人快速回到轨迹上
    """
    
    def __init__(self, 
                 dt_base: float,
                 alpha_min: float = 0.1,
                 velocity_threshold: float = 0.01):
        """
        初始化二次采样策略
        
        Args:
            dt_base: 基础采样间隔 [s]
            alpha_min: 最小 α 值（防止采样间隔过小）
            velocity_threshold: 速度阈值，低于此值时使用默认 α=1
        """
        self.dt_base = dt_base
        self.alpha_min = alpha_min
        self.velocity_threshold = velocity_threshold
        
        # 记录 α 值历史（用于可视化和调试）
        self.alpha_history: List[float] = []
    
    def compute_alpha(self, 
                      v_robot: np.ndarray, 
                      v_ref: np.ndarray) -> float:
        """
        计算 α 因子
        
        α = max(α_min, cos(v_robot, v_ref))
          = max(α_min, v_robot · v_ref / (|v_robot| × |v_ref|))
        
        Args:
            v_robot: 机器人当前 2D 速度 [v_x, v_y]
            v_ref: 参考轨迹 2D 速度 [v_ref_x, v_ref_y]
            
        Returns:
            alpha: 采样因子 ∈ [α_min, 1.0]
        """
        # 计算速度幅值
        norm_robot = np.linalg.norm(v_robot)
        norm_ref = np.linalg.norm(v_ref)
        
        # 如果速度太小，使用默认 α=1
        if norm_robot < self.velocity_threshold or norm_ref < self.velocity_threshold:
            return 1.0
        
        # 计算余弦相似度 (点积 / 模长乘积)
        cos_theta = np.dot(v_robot, v_ref) / (norm_robot * norm_ref)
        
        # 限制在 [0, 1] 范围内（负值表示反向，此时 α=0 → α_min）
        alpha = max(0.0, cos_theta)
        
        # 应用最小 α 值
        alpha = max(self.alpha_min, alpha)
        
        return alpha
    
    def resample_trajectory(self,
                            traj_gen: DynamicTrajectoryGenerator,
                            t_current: float,
                            v_robot: np.ndarray,
                            N: int) -> Tuple[List[np.ndarray], List[float], List[float]]:
        """
        使用二次采样策略生成预测视界内的参考轨迹
        
        算法流程:
        1. 从当前时间 t_current 开始
        2. 对于每个预测步 k = 0, 1, ..., N-1:
           a. 获取参考轨迹在 t_k 处的速度 v_ref
           b. 计算 α_k = f(v_robot, v_ref)
           c. 计算采样间隔 dt_k = α_k × dt_base
           d. 更新时间 t_{k+1} = t_k + dt_k
           e. 获取 t_k 处的参考状态和控制
        
        Args:
            traj_gen: 轨迹生成器
            t_current: 当前时间 [s]
            v_robot: 机器人当前 2D 速度 [v_x, v_y]
            N: 预测步数
            
        Returns:
            (yref_list, t_list, alpha_list):
                - yref_list: N+1 个参考向量的列表
                - t_list: N+1 个时间点的列表
                - alpha_list: N 个 α 值的列表
        """
        yref_list = []
        t_list = []
        alpha_list = []
        
        t_k = t_current
        
        for k in range(N):
            # 获取参考轨迹在 t_k 处的参考速度
            v_ref = traj_gen.get_velocity_vector(t_k)
            
            # 计算 α 因子
            # 注意: 对于第一个点，使用机器人当前速度
            # 对于后续点，可以使用前一步的预测速度（简化：仍用当前速度）
            alpha_k = self.compute_alpha(v_robot, v_ref)
            alpha_list.append(alpha_k)
            
            # 获取 t_k 时刻的参考（9维: 状态6 + 控制3）
            yref_k = traj_gen.get_ref_for_mpc(t_k)
            yref_list.append(yref_k)
            t_list.append(t_k)
            
            # 计算下一个时间点
            dt_k = alpha_k * self.dt_base
            t_k = t_k + dt_k
        
        # 终端参考（仅状态，6维）
        yref_e = traj_gen.get_full_state(t_k)
        yref_list.append(yref_e)
        t_list.append(t_k)
        
        # 记录第一个 α 值（用于可视化）
        if len(alpha_list) > 0:
            self.alpha_history.append(alpha_list[0])
        
        return yref_list, t_list, alpha_list
    
    def clear_history(self):
        """清除 α 历史记录"""
        self.alpha_history = []


# =============================================================================
# OCP 设置函数 (6 状态动力学模型)
# =============================================================================
def setup_dynamic_ocp() -> AcadosOcp:
    """
    设置基于 6 状态动力学模型的最优控制问题
    
    状态: [p_x, p_y, ψ, v_x, v_y, ω]^T (6维)
    控制: [a_x, a_y, α]^T (3维，加速度)
    """
    ocp = AcadosOcp()

    # =========================================================================
    # 加载动力学模型
    # =========================================================================
    model = export_omnidirectional_dynamic_model()
    ocp.model = model

    nx = model.x.rows()  # 6
    nu = model.u.rows()  # 3

    print(f"\n[OCP 设置] 动力学模型")
    print(f"         状态维度 nx = {nx} (位置3 + 速度3)")
    print(f"         控制维度 nu = {nu} (加速度)")

    # =========================================================================
    # 预测时域参数
    # =========================================================================
    N = 40         # 预测步数
    Tf = 2.0       # 预测总时间 [s]
    dt = Tf / N

    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = Tf

    print(f"[OCP 设置] N = {N}, Tf = {Tf}s, dt = {dt:.4f}s")

    # =========================================================================
    # 代价函数 (Nonlinear Least Squares)
    # =========================================================================
    # 代价 = (y - y_ref)^T W (y - y_ref)
    # y = [x; u] = [p_x, p_y, ψ, v_x, v_y, ω, a_x, a_y, α]
    
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.model.cost_y_expr = ca.vertcat(model.x, model.u)

    # 状态权重 Q (6x6)
    # 位置误差权重较大，速度误差权重中等
    Q = np.diag([
        400.0,   # p_x 位置误差
        100.0,   # p_y 位置误差
        20.0,    # ψ 航向误差
        10.0,    # v_x 速度误差
        10.0,    # v_y 速度误差
        5.0      # ω 角速度误差
    ])

    # 控制权重 R (3x3)
    # 惩罚加速度，确保平滑
    R = np.diag([
        1.0,     # a_x 加速度
        1.0,     # a_y 加速度
        0.5      # α 角加速度
    ])

    # 组合权重矩阵 W
    W = np.block([
        [Q, np.zeros((nx, nu))],
        [np.zeros((nu, nx)), R]
    ])
    ocp.cost.W = W
    ocp.cost.yref = np.zeros(nx + nu)  # 9维

    print(f"[OCP 设置] 路径代价权重 W: {W.shape}")

    # 终端代价 (仅状态)
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.model.cost_y_expr_e = model.x
    Q_e = np.diag([200.0, 200.0, 50.0, 20.0, 20.0, 10.0])
    ocp.cost.W_e = Q_e
    ocp.cost.yref_e = np.zeros(nx)  # 6维

    print(f"[OCP 设置] 终端代价权重 W_e: {Q_e.shape}")

    # =========================================================================
    # 约束条件
    # =========================================================================
    # 控制量约束 (加速度)
    a_max = 2.0       # 最大线加速度 [m/s²]
    alpha_max = 1.0   # 最大角加速度 [rad/s²]

    ocp.constraints.lbu = np.array([-a_max, -a_max, -alpha_max])
    ocp.constraints.ubu = np.array([a_max, a_max, alpha_max])
    ocp.constraints.idxbu = np.array([0, 1, 2])

    print(f"[OCP 设置] 控制约束: a_x, a_y ∈ [-{a_max}, {a_max}] m/s²")
    print(f"                   α ∈ [-{alpha_max}, {alpha_max}] rad/s²")

    # 状态约束 (速度限制)
    v_max = 2.0       # 最大线速度 [m/s]
    omega_max = 1.5   # 最大角速度 [rad/s]
    
    # 对速度状态施加约束 (状态索引 3, 4, 5)
    ocp.constraints.lbx = np.array([-v_max, -v_max, -omega_max])
    ocp.constraints.ubx = np.array([v_max, v_max, omega_max])
    ocp.constraints.idxbx = np.array([3, 4, 5])  # v_x, v_y, ω
    
    print(f"[OCP 设置] 速度约束: v_x, v_y ∈ [-{v_max}, {v_max}] m/s")
    print(f"                   ω ∈ [-{omega_max}, {omega_max}] rad/s")

    # 初始状态约束
    ocp.constraints.x0 = np.zeros(nx)

    # =========================================================================
    # 求解器选项
    # =========================================================================
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.nlp_solver_max_iter = 1
    ocp.solver_options.print_level = 0

    print(f"[OCP 设置] QP: {ocp.solver_options.qp_solver}")
    print(f"[OCP 设置] NLP: {ocp.solver_options.nlp_solver_type}")

    return ocp


# =============================================================================
# 闭环仿真函数 (带二次采样)
# =============================================================================
def run_dynamic_tracking_simulation(
    ocp_solver: AcadosOcpSolver,
    traj_gen: DynamicTrajectoryGenerator,
    resampler: SecondaryResamplingStrategy,
    x0: np.ndarray,
    T_sim: float,
    dt_sim: float,
    use_resampling: bool = True
) -> dict:
    """
    运行带二次采样策略的动力学 MPC 仿真
    
    Args:
        ocp_solver: acados OCP 求解器
        traj_gen: 轨迹生成器
        resampler: 二次采样策略
        x0: 初始状态 [p_x, p_y, ψ, v_x, v_y, ω]
        T_sim: 仿真总时间 [s]
        dt_sim: 仿真步长 [s]
        use_resampling: 是否使用二次采样策略
        
    Returns:
        result: 包含所有仿真数据的字典
    """
    # 获取参数
    nx = ocp_solver.acados_ocp.dims.nx  # 6
    nu = ocp_solver.acados_ocp.dims.nu  # 3
    N = ocp_solver.acados_ocp.solver_options.N_horizon
    dt_mpc = ocp_solver.acados_ocp.solver_options.tf / N
    
    N_sim = int(T_sim / dt_sim)
    
    # 初始化存储
    x_traj = np.zeros((N_sim + 1, nx))
    u_traj = np.zeros((N_sim, nu))
    ref_traj = np.zeros((N_sim + 1, nx))
    errors_pos = np.zeros(N_sim + 1)      # 位置误差
    errors_vel = np.zeros(N_sim + 1)      # 速度误差
    solve_times = np.zeros(N_sim)
    alpha_values = np.zeros(N_sim)        # α 因子
    
    # 设置初始状态
    x_current = x0.copy()
    x_traj[0, :] = x_current
    ref_traj[0, :] = traj_gen.get_full_state(0.0)
    errors_pos[0] = np.linalg.norm(x_current[:2] - ref_traj[0, :2])
    errors_vel[0] = np.linalg.norm(x_current[3:5] - ref_traj[0, 3:5])
    
    # 清除 α 历史
    resampler.clear_history()
    
    print("\n" + "=" * 70)
    print(f"开始动力学 MPC 仿真 {'(使用二次采样)' if use_resampling else '(均匀采样)'}")
    print("=" * 70)
    print(f"初始状态: p=[{x0[0]:.2f}, {x0[1]:.2f}], ψ={x0[2]:.2f}, v=[{x0[3]:.2f}, {x0[4]:.2f}]")
    print(f"仿真时间: {T_sim}s, 步数: {N_sim}, 步长: {dt_sim}s")
    print("-" * 70)
    
    # 主仿真循环
    for i in range(N_sim):
        t_current = i * dt_sim
        
        # -----------------------------------------------------------------
        # 步骤 1: 设置初始状态约束
        # -----------------------------------------------------------------
        ocp_solver.set(0, "lbx", x_current)
        ocp_solver.set(0, "ubx", x_current)
        
        # -----------------------------------------------------------------
        # 步骤 2: 生成预测视界内的参考轨迹
        # -----------------------------------------------------------------
        if use_resampling:
            # 使用二次采样策略
            v_robot = x_current[3:5]  # 当前机器人 2D 速度
            yref_list, t_list, alpha_list = resampler.resample_trajectory(
                traj_gen, t_current, v_robot, N
            )
            alpha_values[i] = alpha_list[0] if len(alpha_list) > 0 else 1.0
            
            # 设置参考
            for k in range(N):
                ocp_solver.set(k, "yref", yref_list[k])
            ocp_solver.set(N, "yref", yref_list[N])  # 终端（6维）
            
        else:
            # 使用均匀采样（传统方法）
            alpha_values[i] = 1.0
            for k in range(N):
                t_pred = t_current + k * dt_mpc
                yref_k = traj_gen.get_ref_for_mpc(t_pred)
                ocp_solver.set(k, "yref", yref_k)
            
            t_terminal = t_current + N * dt_mpc
            yref_e = traj_gen.get_full_state(t_terminal)
            ocp_solver.set(N, "yref", yref_e)
        
        # -----------------------------------------------------------------
        # 步骤 3: 求解 OCP
        # -----------------------------------------------------------------
        status = ocp_solver.solve()
        solve_times[i] = ocp_solver.get_stats("time_tot")
        
        if status != 0:
            print(f"[警告] 步 {i}: 求解失败 (status={status})")
        
        # -----------------------------------------------------------------
        # 步骤 4: 获取并应用最优控制
        # -----------------------------------------------------------------
        u_optimal = ocp_solver.get(0, "u")
        u_traj[i, :] = u_optimal
        
        # -----------------------------------------------------------------
        # 步骤 5: 系统状态更新 (显式欧拉积分)
        # -----------------------------------------------------------------
        # 动力学: ẋ = [v_x, v_y, ω, a_x, a_y, α]
        a_x, a_y, alpha = u_optimal
        
        x_next = np.zeros(nx)
        x_next[0] = x_current[0] + dt_sim * x_current[3]   # p_x += v_x * dt
        x_next[1] = x_current[1] + dt_sim * x_current[4]   # p_y += v_y * dt
        x_next[2] = x_current[2] + dt_sim * x_current[5]   # ψ += ω * dt
        x_next[3] = x_current[3] + dt_sim * a_x            # v_x += a_x * dt
        x_next[4] = x_current[4] + dt_sim * a_y            # v_y += a_y * dt
        x_next[5] = x_current[5] + dt_sim * alpha          # ω += α * dt
        
        x_current = x_next
        
        # 保存数据
        x_traj[i + 1, :] = x_current
        ref_traj[i + 1, :] = traj_gen.get_full_state((i + 1) * dt_sim)
        errors_pos[i + 1] = np.linalg.norm(x_current[:2] - ref_traj[i + 1, :2])
        errors_vel[i + 1] = np.linalg.norm(x_current[3:5] - ref_traj[i + 1, 3:5])
        
        # 打印进度
        if (i + 1) % 50 == 0 or i == 0:
            print(f"Step {i+1:4d}/{N_sim}: t={t_current:.2f}s, "
                  f"pos_err={errors_pos[i+1]*1000:.1f}mm, "
                  f"α={alpha_values[i]:.3f}, "
                  f"solve={solve_times[i]*1000:.2f}ms")
    
    # 统计
    print("-" * 70)
    print(f"仿真完成!")
    print(f"位置误差: 平均={np.mean(errors_pos)*1000:.2f}mm, 最大={np.max(errors_pos)*1000:.2f}mm")
    print(f"速度误差: 平均={np.mean(errors_vel):.3f}m/s")
    print(f"α 因子: 平均={np.mean(alpha_values):.3f}, 最小={np.min(alpha_values):.3f}")
    print(f"求解时间: 平均={np.mean(solve_times)*1000:.3f}ms")
    print("=" * 70)
    
    # 返回结果
    t_traj = np.arange(N_sim + 1) * dt_sim
    
    return {
        'x_traj': x_traj,
        'u_traj': u_traj,
        't_traj': t_traj,
        'ref_traj': ref_traj,
        'errors_pos': errors_pos,
        'errors_vel': errors_vel,
        'alpha_values': alpha_values,
        'solve_times': solve_times
    }


# =============================================================================
# 可视化函数
# =============================================================================
def plot_dynamic_tracking_results(
    result: dict,
    traj_gen: DynamicTrajectoryGenerator,
    save_path: str = None
):
    """
    绘制动力学 MPC 仿真结果
    
    包含:
    - 2D 轨迹对比
    - 位置误差
    - 速度状态
    - 控制量（加速度）
    - α 因子变化曲线
    """
    # 设置字体
    plt.rcParams['text.usetex'] = False
    
    import matplotlib.font_manager as fm
    chinese_fonts = ['WenQuanYi Micro Hei', 'Noto Sans CJK SC', 'SimHei', 'DejaVu Sans']
    available_fonts = [f.name for f in fm.fontManager.ttflist]
    selected_font = next((f for f in chinese_fonts if f in available_fonts), 'DejaVu Sans')
    use_chinese = selected_font != 'DejaVu Sans'
    if use_chinese:
        plt.rcParams['font.family'] = selected_font
        plt.rcParams['axes.unicode_minus'] = False
    
    # 提取数据
    x_traj = result['x_traj']
    u_traj = result['u_traj']
    t_traj = result['t_traj']
    ref_traj = result['ref_traj']
    errors_pos = result['errors_pos']
    alpha_values = result['alpha_values']
    
    # 生成完整参考轨迹
    t_ref, ref_full = traj_gen.generate_full_trajectory(0, t_traj[-1], 0.01)
    
    # 创建 3x3 子图
    fig, axes = plt.subplots(3, 3, figsize=(16, 12))
    
    title = '全向轮机器人动力学 MPC 轨迹跟踪 (二次采样)' if use_chinese else \
            'Omnidirectional Robot Dynamic MPC Tracking (Secondary Resampling)'
    fig.suptitle(title, fontsize=14)
    
    # 标签
    L = {
        'actual': '实际' if use_chinese else 'Actual',
        'ref': '参考' if use_chinese else 'Reference',
        'time': '时间 [s]' if use_chinese else 'Time [s]',
        'pos_err': '位置误差 [mm]' if use_chinese else 'Position Error [mm]',
        'alpha': 'α 因子' if use_chinese else 'Alpha Factor',
    }
    
    # ----- 1. 2D 轨迹 -----
    ax = axes[0, 0]
    ax.plot(ref_full[:, 0], ref_full[:, 1], 'r--', lw=1.5, alpha=0.7, label=L['ref'])
    ax.plot(x_traj[:, 0], x_traj[:, 1], 'b-', lw=2, label=L['actual'])
    ax.plot(x_traj[0, 0], x_traj[0, 1], 'go', ms=10, label='Start')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('2D Trajectory')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # ----- 2. 位置误差 -----
    ax = axes[0, 1]
    ax.plot(t_traj, errors_pos * 1000, 'b-', lw=2)
    ax.axhline(np.mean(errors_pos) * 1000, color='r', ls='--', 
               label=f'Mean: {np.mean(errors_pos)*1000:.1f}mm')
    ax.set_xlabel(L['time'])
    ax.set_ylabel(L['pos_err'])
    ax.set_title('Position Error')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # ----- 3. α 因子 -----
    ax = axes[0, 2]
    t_u = t_traj[:-1]
    ax.plot(t_u, alpha_values, 'g-', lw=2)
    ax.axhline(np.mean(alpha_values), color='r', ls='--',
               label=f'Mean: {np.mean(alpha_values):.3f}')
    ax.set_xlabel(L['time'])
    ax.set_ylabel(L['alpha'])
    ax.set_title('Resampling Factor α')
    ax.set_ylim([0, 1.1])
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # ----- 4. X 位置 -----
    ax = axes[1, 0]
    ax.plot(t_traj, ref_traj[:, 0], 'r--', lw=1.5, alpha=0.7, label=L['ref'])
    ax.plot(t_traj, x_traj[:, 0], 'b-', lw=2, label=L['actual'])
    ax.set_xlabel(L['time'])
    ax.set_ylabel('$p_x$ [m]')
    ax.set_title('X Position')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # ----- 5. Y 位置 -----
    ax = axes[1, 1]
    ax.plot(t_traj, ref_traj[:, 1], 'r--', lw=1.5, alpha=0.7, label=L['ref'])
    ax.plot(t_traj, x_traj[:, 1], 'b-', lw=2, label=L['actual'])
    ax.set_xlabel(L['time'])
    ax.set_ylabel('$p_y$ [m]')
    ax.set_title('Y Position')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # ----- 6. 速度 v_x -----
    ax = axes[1, 2]
    ax.plot(t_traj, ref_traj[:, 3], 'r--', lw=1.5, alpha=0.7, label=L['ref'])
    ax.plot(t_traj, x_traj[:, 3], 'b-', lw=2, label=L['actual'])
    ax.set_xlabel(L['time'])
    ax.set_ylabel('$v_x$ [m/s]')
    ax.set_title('X Velocity')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # ----- 7. 速度 v_y -----
    ax = axes[2, 0]
    ax.plot(t_traj, ref_traj[:, 4], 'r--', lw=1.5, alpha=0.7, label=L['ref'])
    ax.plot(t_traj, x_traj[:, 4], 'b-', lw=2, label=L['actual'])
    ax.set_xlabel(L['time'])
    ax.set_ylabel('$v_y$ [m/s]')
    ax.set_title('Y Velocity')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # ----- 8. 加速度 a_x, a_y -----
    ax = axes[2, 1]
    ax.plot(t_u, u_traj[:, 0], 'b-', lw=2, label='$a_x$')
    ax.plot(t_u, u_traj[:, 1], 'g-', lw=2, label='$a_y$')
    ax.set_xlabel(L['time'])
    ax.set_ylabel('Acceleration [m/s²]')
    ax.set_title('Linear Acceleration')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # ----- 9. 角加速度 α -----
    ax = axes[2, 2]
    ax.plot(t_u, u_traj[:, 2], 'm-', lw=2)
    ax.set_xlabel(L['time'])
    ax.set_ylabel('$\\alpha$ [rad/s²]')
    ax.set_title('Angular Acceleration')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\n[INFO] 图片已保存: {save_path}")
    
    plt.show()


# =============================================================================
# 主函数
# =============================================================================
def main():
    """
    主函数: 运行全向轮机器人动力学 MPC 轨迹跟踪仿真
    """
    print("\n" + "=" * 70)
    print("全向轮移动机器人动力学 MPC 控制器")
    print("6 状态模型 + 二次轨迹采样策略")
    print("=" * 70)
    
    # =========================================================================
    # 步骤 1: 创建轨迹生成器
    # =========================================================================
    print("\n[步骤 1] 创建轨迹生成器...")
    traj_gen = DynamicTrajectoryGenerator(
        trajectory_type='figure8',
        scale=1.5,
        period=10.0,
        center=(0.0, 0.0)
    )
    
    # =========================================================================
    # 步骤 2: 设置 OCP 并创建求解器
    # =========================================================================
    print("\n[步骤 2] 配置最优控制问题...")
    ocp = setup_dynamic_ocp()
    
    print("\n[步骤 3] 创建 acados 求解器...")
    ocp_solver = AcadosOcpSolver(ocp)
    print("[INFO] C 代码已生成")
    
    # =========================================================================
    # 步骤 3: 创建二次采样策略
    # =========================================================================
    print("\n[步骤 4] 配置二次采样策略...")
    dt_mpc = ocp.solver_options.tf / ocp.solver_options.N_horizon
    resampler = SecondaryResamplingStrategy(
        dt_base=dt_mpc,
        alpha_min=0.1,
        velocity_threshold=0.01
    )
    print(f"         dt_base = {dt_mpc:.4f}s, alpha_min = 0.1")
    
    # =========================================================================
    # 步骤 4: 设置仿真参数
    # =========================================================================
    # 初始状态：从轨迹起点开始，但给一些初始偏差以测试二次采样效果
    x0_ref = traj_gen.get_full_state(0.0)
    
    # 添加初始偏差
    x0 = x0_ref.copy()
    x0[0] += 0.3   # X 偏差 0.3m
    x0[1] += 0.0   # Y 偏差 0.2m
    x0[3] = 0.0    # 初始速度为零
    x0[4] = 0.0
    
    print(f"\n[INFO] 参考初始状态: p=[{x0_ref[0]:.2f}, {x0_ref[1]:.2f}], v=[{x0_ref[3]:.2f}, {x0_ref[4]:.2f}]")
    print(f"[INFO] 实际初始状态: p=[{x0[0]:.2f}, {x0[1]:.2f}], v=[{x0[3]:.2f}, {x0[4]:.2f}]")
    print(f"[INFO] 初始位置偏差: {np.linalg.norm(x0[:2] - x0_ref[:2])*1000:.1f} mm")
    
    T_sim = 15.0    # 仿真时间
    dt_sim = 0.01   # 仿真步长 (50 Hz)
    
    # =========================================================================
    # 步骤 5: 运行仿真
    # =========================================================================
    print("\n[步骤 5] 运行轨迹跟踪仿真...")
    result = run_dynamic_tracking_simulation(
        ocp_solver, traj_gen, resampler, x0, T_sim, dt_sim,
        use_resampling=True  # 使用二次采样
    )
    
    # =========================================================================
    # 步骤 6: 绘制结果
    # =========================================================================
    print("\n[步骤 6] 绘制仿真结果...")
    plot_dynamic_tracking_results(
        result, traj_gen,
        save_path='omnidirectional_dynamic_tracking.png'
    )
    
    # 清理
    del ocp_solver
    
    print("\n[完成] 动力学 MPC 仿真已完成!")
    print("=" * 70)


if __name__ == '__main__':
    main()
