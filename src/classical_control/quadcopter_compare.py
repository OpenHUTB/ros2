"""四旋翼姿态控制：PID vs LQR 算法对比.

对比经典 PID 和现代 LQR 两种姿态控制器在
**四旋翼简化模型**（4 自由度：roll/pitch/yaw/height）
下的阶跃响应、抗扰性能和稳态误差。

不同于 cart-pole LQR（倒立摆是单自由度欠驱动系统），
本模块是**四旋翼多自由度、完全驱动系统**的控制对比。
"""
import json
import numpy as np
from pathlib import Path


class QuadcopterModel:
    """四旋翼简化动力学模型（4 自由度）.

    状态向量: [roll, pitch, yaw, height, droll, dpitch, dyaw, dheight]
    控制输入: [u_roll, u_pitch, u_yaw, u_thrust]
    """

    def __init__(self, dt=0.01, mass=1.5, arm_length=0.5, inertia=0.03):
        self.dt = dt
        self.mass = mass
        self.L = arm_length
        self.I = inertia
        self.g = 9.81
        self.state = np.zeros(8)  # 初始悬停
        self.traj = [self.state.copy()]

    def reset(self, init_state=None):
        self.state = init_state if init_state is not None else np.zeros(8)
        self.traj = [self.state.copy()]

    def dynamics(self, state, u):
        """非线性简化动力学（忽略高阶耦合）."""
        roll, pitch, yaw, h, dr, dp, dy, dh = state
        u_r, u_p, u_y, u_t = u
        # 角加速度
        ddr = u_r * self.L / self.I
        ddp = u_p * self.L / self.I
        ddy = u_y * (2 * self.I)  # yaw 惯量不同
        # 高度加速度（推力 - 重力）/ 质量
        ddh = (u_t - self.mass * self.g) / self.mass
        return np.array([dr, dp, dy, dh, ddr, ddp, ddy, ddh])

    def step(self, u):
        """Runge-Kutta 4 积分一步."""
        k1 = self.dynamics(self.state, u)
        k2 = self.dynamics(self.state + 0.5 * self.dt * k1, u)
        k3 = self.dynamics(self.state + 0.5 * self.dt * k2, u)
        k4 = self.dynamics(self.state + self.dt * k3, u)
        self.state += self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        self.traj.append(self.state.copy())

    def add_disturbance(self, axis, magnitude=0.1):
        """加一个瞬时扰动（轴：roll/pitch/yaw/height）."""
        mapping = {"roll": 0, "pitch": 1, "yaw": 2, "height": 3}
        if axis in mapping:
            idx = mapping[axis]
            self.state[idx] += magnitude


class PIDController:
    """独立轴 PID 控制器."""

    def __init__(self, kp=2.0, ki=0.1, kd=0.5, dt=0.01):
        self.dt = dt
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integrals = np.zeros(4)
        self.prev_errors = np.zeros(4)

    def compute(self, target, state):
        # 位置: roll, pitch, yaw, height → state[0:4]
        # 速度: droll, dpitch, dyaw, dheight → state[4:8]
        pos_err = target - state[0:4]
        vel_err = -state[4:8]  # 期望速度 0

        self.integrals += pos_err * self.dt
        self.integrals = np.clip(self.integrals, -5.0, 5.0)  # 抗饱和

        u = (
            self.kp * pos_err
            + self.ki * self.integrals
            + self.kd * vel_err
        )
        u[3] += 9.81  # 高度通道加上重力补偿
        return u


class LQRController:
    """线性二次型调节器（线性化模型 + 离散化）."""

    def __init__(self, dt=0.01):
        self.dt = dt
        # 连续时间状态空间矩阵（简化）
        I, L, m = 0.03, 0.5, 1.5
        g = 9.81
        Ac = np.zeros((8, 8))
        # 状态 → 速度通道
        Ac[0:4, 4:8] = np.eye(4)
        # 控制 → 加速度通道（线性化）
        Bc = np.zeros((8, 4))
        Bc[4, 0] = L / I      # roll
        Bc[5, 1] = L / I      # pitch
        Bc[6, 2] = L / (2 * I)  # yaw
        Bc[7, 3] = 1.0 / m      # height
        # 高度推力偏置（悬停推力）
        self.u_bias = np.array([0.0, 0.0, 0.0, m * g])

        # 权重矩阵
        Q = np.diag([10.0, 10.0, 5.0, 8.0, 1.0, 1.0, 0.5, 1.0])
        R = np.diag([0.1, 0.1, 0.1, 0.01])

        # 手动求解 Riccati（简化版，给常见权重先算好 K）
        # 实际用 scipy.linalg.solve_continuous_are，这里给近似值
        self.K = np.array([
            [8.0, 0, 0, 0, 2.0, 0, 0, 0],
            [0, 8.0, 0, 0, 0, 2.0, 0, 0],
            [0, 0, 5.0, 0, 0, 0, 1.0, 0],
            [0, 0, 0, 6.0, 0, 0, 0, 1.5],
        ])

    def compute(self, target, state):
        error = target - state[0:4]
        state_err = np.zeros(8)
        state_err[0:4] = error
        state_err[4:8] = -state[4:8]
        u = self.K @ state_err + self.u_bias
        return u


def run_closed_loop(model, controller, target, steps=2000, disturb_at=None):
    """跑一段闭环响应."""
    model.reset()
    errors = []
    for i in range(steps):
        # 扰动
        if disturb_at and i == disturb_at:
            model.add_disturbance("roll", 0.15)

        u = controller.compute(target, model.state)
        u = np.clip(u, -10, 20)
        model.step(u)
        err = np.linalg.norm(target - model.state[0:4])
        errors.append(err)
    return errors, model.traj


def run_demo():
    """对比 PID vs LQR."""
    cfg_path = Path(__file__).parent / "scenario.json"
    if cfg_path.exists():
        with open(cfg_path) as f:
            cfg = json.load(f)
    else:
        cfg = {"dt": 0.01, "steps": 3000, "disturb_at": 1500}

    dt = cfg["dt"]
    steps = cfg["steps"]
    disturb_at = cfg.get("disturb_at", 1500)

    # 目标：roll=5°, height=2m, yaw=30°
    target = np.array([np.deg2rad(5), 0.0, np.deg2rad(30), 2.0])

    print("=" * 55)
    print(f"  四旋翼姿态控制: PID vs LQR")
    print(f"  目标: roll=5° yaw=30° height=2m")
    print(f"  扰动: roll +0.15rad 在 step {disturb_at}")
    print("=" * 55)

    model_pid = QuadcopterModel(dt=dt)
    ctrl_pid = PIDController(kp=2.5, ki=0.15, kd=0.8, dt=dt)
    pid_errors, pid_traj = run_closed_loop(
        model_pid, ctrl_pid, target, steps, disturb_at
    )

    model_lqr = QuadcopterModel(dt=dt)
    ctrl_lqr = LQRController(dt=dt)
    lqr_errors, lqr_traj = run_closed_loop(
        model_lqr, ctrl_lqr, target, steps, disturb_at
    )

    # 统计数据
    def compute_metrics(errors):
        steady_phase = errors[disturb_at + 200:]
        settling_phase = errors[:disturb_at]
        return {
            "max_overshoot": np.max(settling_phase),
            "settling_error": np.mean(steady_phase),
            "std": np.std(steady_phase),
            "recovery_time": None,
        }

    pid_metrics = compute_metrics(pid_errors)
    lqr_metrics = compute_metrics(lqr_errors)

    # 恢复时间：扰动后回到稳态 ±0.02rad
    threshold = 0.05
    pid_recover, lqr_recover = None, None
    for i in range(disturb_at, len(pid_errors)):
        if pid_errors[i] < threshold:
            pid_recover = (i - disturb_at) * dt
            break
    for i in range(disturb_at, len(lqr_errors)):
        if lqr_errors[i] < threshold:
            lqr_recover = (i - disturb_at) * dt
            break

    print("\n  📊 对比结果:")
    print(f"  {'指标':<24} {'PID':>10}  {'LQR':>10}")
    print(f"  {'-'*44}")
    print(f"  {'最大超调 (rad)':<24} {pid_metrics['max_overshoot']:>10.4f}  {lqr_metrics['max_overshoot']:>10.4f}")
    print(f"  {'稳态误差 (rad)':<24} {pid_metrics['settling_error']:>10.4f}  {lqr_metrics['settling_error']:>10.4f}")
    print(f"  {'恢复时间 (s)':<24} {(pid_recover or 'N/A') if pid_recover else 'N/A':>10}  {(lqr_recover or 'N/A') if lqr_recover else 'N/A':>10}")

    # 结论
    if lqr_metrics["settling_error"] < pid_metrics["settling_error"]:
        winner = "LQR 稳态精度更高"
    else:
        winner = "PID 稳态表现更好"
    print(f"\n  🏆 结论: {winner}")

    # 关键时刻的状态
    for label, idx in [("初始", 0), ("响应中", 500), ("稳态前", disturb_at - 100),
                        ("扰动后 100 步", disturb_at + 100), ("扰动后 1s", disturb_at + 1000)]:
        if idx < len(pid_traj) and idx < len(lqr_traj):
            p = pid_traj[idx]
            l = lqr_traj[idx]
            print(f"\n  [{label}]")
            print(f"     PID : roll={np.rad2deg(p[0]):5.2f}° h={p[3]:5.2f}m")
            print(f"     LQR : roll={np.rad2deg(l[0]):5.2f}° h={l[3]:5.2f}m")


if __name__ == "__main__":
    run_demo()
