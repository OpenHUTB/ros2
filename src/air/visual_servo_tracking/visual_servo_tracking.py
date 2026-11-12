"""
无人机视觉伺服目标跟踪
======================

无人机配备俯视单目相机，目标是地面上的移动目标。本模块实现
**基于图像特征的视觉伺服（IBVS）**：

核心思路：
  1. 相机以固定帧速率采集图像，检测目标在图像平面的投影点
  2. 从 3D 深度（已知高度）和相机模型反推目标相对位姿
  3. 计算图像误差 e = s - s*（当前特征 − 目标特征）
  4. 用图像 Jacobian L 把图像空间误差映射到 6-DOF 控制输入
  5. 再投影到无人机的 4-DOF 控制（roll/pitch/yaw/altitude）

与仓库已有的 winter 场景圆周飞行不同：
  · 圆周飞行是**无目标**的轨迹跟踪（写死圆周路径）
  · 本模块是**闭环目标跟踪**（用实时视觉特征）
"""

import numpy as np


# ---------------------------------------------------------------------------
# 简化针孔相机模型
# ---------------------------------------------------------------------------
class PinholeCamera:
    def __init__(self, focal_m=0.016, sensor_px_m=3.0e-6,
                 resolution=(640, 480)):
        self.focal = focal_m
        self.px_per_m = 1.0 / sensor_px_m
        self.fx = focal_m * self.px_per_m
        self.fy = self.fx
        self.cx = resolution[0] / 2.0
        self.cy = resolution[1] / 2.0
        self.width, self.height = resolution

    def project(self, point_in_cam):
        """将 3D 点（相机坐标系）投影到图像平面像素坐标。"""
        X, Y, Z = point_in_cam
        if Z <= 0.01:
            return None
        u = self.fx * X / Z + self.cx
        v = self.fy * Y / Z + self.cy
        return np.array([u, v])

    def is_in_image(self, uv):
        if uv is None:
            return False
        return (0 <= uv[0] <= self.width) and (0 <= uv[1] <= self.height)


# ---------------------------------------------------------------------------
# 无人机简化动力学（4-DOF）
# ---------------------------------------------------------------------------
class DroneIBVS:
    def __init__(self, x0=None, height0=12.0):
        # 状态: [x, y, z, roll, pitch, yaw, vx, vy, vz]
        if x0 is None:
            x0 = np.array([0.0, 0.0, height0,
                           0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0])
        self.state = x0.copy()
        # 目标高度保持
        self.target_height = height0

    @property
    def pos(self):
        return self.state[:3].copy()

    @property
    def attitude(self):
        return self.state[3:6].copy()

    def step(self, u, dt=0.05):
        """4-DOF 控制输入 u = [roll_cmd, pitch_cmd, yaw_rate_cmd, vz_cmd]."""
        # 简化：roll/pitch 直接对位置积分（小角度假设）
        roll, pitch, yaw_rate, vz = u
        self.state[3] += (roll - self.state[3]) * 2.0 * dt
        self.state[4] += (pitch - self.state[4]) * 2.0 * dt
        self.state[5] += yaw_rate * dt
        self.state[5] = (self.state[5] + np.pi) % (2 * np.pi) - np.pi
        # 位置
        self.state[6] = self.state[6] + (-pitch) * 3.0 * dt - \
                        self.state[6] * 0.1 * dt
        self.state[7] = self.state[7] + (roll) * 3.0 * dt - \
                        self.state[7] * 0.1 * dt
        self.state[8] = (vz - self.state[8]) * 2.0 * dt
        self.state[0] += self.state[6] * dt
        self.state[1] += self.state[7] * dt
        self.state[2] += self.state[8] * dt


# ---------------------------------------------------------------------------
# 视觉伺服控制器
# ---------------------------------------------------------------------------
class IBVSController:
    def __init__(self, camera, lambda_gain=0.3, desired_uv=None):
        self.cam = camera
        self.lam = lambda_gain
        # 目标特征（图像中心）
        if desired_uv is None:
            self.s_star = np.array([camera.cx, camera.cy])
        else:
            self.s_star = np.asarray(desired_uv, dtype=float)

    def compute(self, target_in_world, drone):
        """根据目标世界坐标和无人机状态计算控制输入。"""
        # 1. 变换到相机坐标系（俯视相机，姿态补偿）
        d_pos = drone.pos
        t_cam = target_in_world - d_pos  # 平移
        # 俯视，相机朝向 -z 世界 → 变换
        R_drone = self._rot_drone(drone.attitude)
        t_cam = R_drone @ t_cam
        # 相机光轴沿 -z（世界）→ Z_cam 为正数（目标在相机下方）
        # 简化：俯视时 Z_cam ≈ 无人机高度（目标高度 0）
        Z = drone.pos[2]
        if Z < 0.1:
            return np.zeros(4), None

        # 2. 投影
        t_cam_approx = np.array([target_in_world[0] - drone.pos[0],
                                 target_in_world[1] - drone.pos[1],
                                 Z])
        uv = self.cam.project(t_cam_approx)

        # 3. 图像误差
        if uv is None:
            return np.zeros(4), None
        e = uv - self.s_star

        # 4. 图像 Jacobian（仅平移 + yaw，简化）
        # L = [[-fx/Z,  0,    u/Z],
        #      [ 0,    -fy/Z, v/Z]]
        fx, fy = self.cam.fx, self.cam.fy
        u, v = uv
        L = np.array([
            [-fx / Z,   0.0,    u / Z],
            [ 0.0,   -fy / Z,  v / Z],
        ])

        # 5. 最小二乘求解 2 自由度期望速度（vx, vy）
        v_xy = np.linalg.lstsq(L[:, :2], -self.lam * e, rcond=None)[0]

        # 6. yaw 控制（让目标在图像中居中时 yaw=0）
        yaw_err = np.arctan2(t_cam_approx[1], t_cam_approx[0])
        yaw_rate = -2.0 * yaw_err

        # 7. 速度 → roll/pitch（小角度：vx ≈ -pitch * 3, vy ≈ roll * 3）
        pitch_cmd = -np.clip(v_xy[0] / 3.0, -0.3, 0.3)
        roll_cmd = np.clip(v_xy[1] / 3.0, -0.3, 0.3)

        # 8. 高度保持（目标不在视野时升高）
        vz_cmd = 0.0
        if not self.cam.is_in_image(uv):
            vz_cmd = -0.5  # 下降找目标

        u = np.array([roll_cmd, pitch_cmd, np.clip(yaw_rate, -1.0, 1.0),
                      vz_cmd])
        return u, uv

    def _rot_drone(self, att):
        """小角度近似。"""
        roll, pitch, yaw = att
        cy, sy = np.cos(yaw), np.sin(yaw)
        return np.array([
            [cy,  -sy,  0.0],
            [sy,   cy,  0.0],
            [0.0,  0.0,  1.0],
        ])


# ---------------------------------------------------------------------------
# 目标运动模型
# ---------------------------------------------------------------------------
class MovingTarget:
    def __init__(self, x0=None, velocity=None):
        if x0 is None:
            x0 = np.array([15.0, 10.0, 0.0])
        self.pos = x0.copy()
        if velocity is None:
            velocity = np.array([0.8, 0.3, 0.0])
        self.vel = velocity.copy()
        self.t = 0.0

    def step(self, dt=0.05):
        # 匀速 + 轻微正弦扰动
        self.t += dt
        self.pos[0] += self.vel[0] * dt + 0.3 * np.sin(self.t * 0.8) * dt
        self.pos[1] += self.vel[1] * dt + 0.2 * np.cos(self.t * 0.6) * dt
        # 边界反弹（±25m）
        for k in [0, 1]:
            if abs(self.pos[k]) > 25.0:
                self.vel[k] *= -1
                self.pos[k] = np.sign(self.pos[k]) * 25.0


# ---------------------------------------------------------------------------
# 仿真
# ---------------------------------------------------------------------------
def simulate(duration=60.0, dt=0.05, height0=12.0):
    camera = PinholeCamera()
    drone = DroneIBVS(height0=height0)
    target = MovingTarget()
    ctrl = IBVSController(camera, lambda_gain=0.3)

    steps = int(duration / dt)
    history = {
        "time": np.linspace(0, duration, steps + 1),
        "dist": np.zeros(steps + 1),      # 水平距离
        "uv_err": np.zeros(steps + 1),    # 图像误差像素
        "in_fov": np.zeros(steps + 1),    # 在图像内标记
    }

    def dist_xy():
        return np.linalg.norm(drone.pos[:2] - target.pos[:2])

    history["dist"][0] = dist_xy()

    for t in range(steps):
        u, uv = ctrl.compute(target.pos, drone)
        drone.step(u, dt=dt)
        target.step(dt=dt)

        history["dist"][t + 1] = dist_xy()
        if uv is not None:
            err = np.linalg.norm(uv - ctrl.s_star)
            history["uv_err"][t + 1] = err
            history["in_fov"][t + 1] = camera.is_in_image(uv)

    return drone, target, history


# ---------------------------------------------------------------------------
# 控制台输出
# ---------------------------------------------------------------------------
def run_demo():
    print("=" * 60)
    print("  无人机视觉伺服目标跟踪 (IBVS)")
    print("=" * 60)
    print()
    print("  📷 相机: 640x480 @ 16mm")
    print("  🚁 初始高度: 12.0 m")
    print("  🎯 目标: 地面移动 (0.8, 0.3 m/s + 正弦扰动)")
    print()

    drone, target, hist = simulate(duration=60.0, dt=0.05)

    print("-" * 60)
    print("  📊 跟踪结果 (60s):")
    print("-" * 60)

    print(f"  起始水平距离: {hist['dist'][0]:.2f} m")
    print(f"  最终水平距离: {hist['dist'][-1]:.2f} m")
    print(f"  平均水平距离: {np.mean(hist['dist']):.2f} m")
    print(f"  图像平均误差: {np.mean(hist['uv_err'][1:]):.2f} 像素")
    print(f"  目标在视野时间: "
          f"{np.sum(hist['in_fov']) * 0.05:.1f} s / 60 s "
          f"({np.mean(hist['in_fov']) * 100:.0f}%)")

    # 收敛判断：平均水平距离 < 1.5m
    converged = np.mean(hist['dist'][int(len(hist['dist']) / 2):]) < 1.5
    print(f"  🏆 是否收敛: {'✅ 是' if converged else '⚠️ 否'}")
    print()

    print("  最终状态:")
    print(f"    无人机位置: ({drone.pos[0]:.2f}, "
          f"{drone.pos[1]:.2f}, {drone.pos[2]:.2f}) m")
    print(f"    目标位置:   ({target.pos[0]:.2f}, "
          f"{target.pos[1]:.2f}, {target.pos[2]:.2f}) m")
    print("=" * 60)


if __name__ == "__main__":
    run_demo()
