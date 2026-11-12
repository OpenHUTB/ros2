"""AirSim IMU 环境：IMU 时序窗进，真值姿态角出（作为监督标签）。

任务：从带噪声的 IMU 读数估计机体 roll/pitch，属于姿态解算（状态估计）。
网络只看 IMU，真值姿态由 AirSim 四元数给出，只用于监督，不存在泄漏。
"""
from __future__ import annotations

from collections import deque

import airsim
import numpy as np

IMU_DIM = 6      # ax, ay, az, gx, gy, gz
WINDOW = 5       # 时序窗长度
ATT_SCALE = 1.0  # 姿态角尺度（rad）


def yaw_from_quat(q) -> float:
    w, x, y, z = q
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def euler_from_quat(q):
    """四元数 (w, x, y, z) -> (roll, pitch)，ZYX 顺序，单位弧度。"""
    w, x, y, z = q
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    return float(roll), float(pitch)


class ImuAttEnv:
    def __init__(self, ip="127.0.0.1", port=41451, vehicle="Drone1"):
        self.client = airsim.MultirotorClient(ip=ip, port=port)
        self.client.confirmConnection()
        self.vehicle = vehicle
        self.buffer = deque(maxlen=WINDOW)
        self.client.enableApiControl(True, vehicle_name=vehicle)
        self.client.armDisarm(True, vehicle_name=vehicle)

    # ---------------------------------------------------------------- 传感器
    def imu(self):
        """返回 6 维 IMU 读数（加速度 m/s^2、角速度 rad/s）。"""
        d = self.client.getImuData(vehicle_name=self.vehicle)
        return np.array([
            d.linear_acceleration.x_val, d.linear_acceleration.y_val, d.linear_acceleration.z_val,
            d.angular_velocity.x_val, d.angular_velocity.y_val, d.angular_velocity.z_val,
        ], dtype=np.float32)

    def imu_window(self):
        """返回展平后的时序窗（WINDOW*6 维）。窗口不足时用首帧补齐。"""
        vec = self.imu()
        if not self.buffer:
            for _ in range(WINDOW):
                self.buffer.append(vec)
        else:
            self.buffer.append(vec)
        return np.concatenate(list(self.buffer), axis=0).astype(np.float32)

    def attitude(self):
        """真值 roll/pitch（弧度），与 pose() 同一口径（kinematics_estimated 的四元数）。"""
        _, q = self.pose()
        return euler_from_quat(q)

    def pose(self):
        st = self.client.getMultirotorState(vehicle_name=self.vehicle).kinematics_estimated
        p = st.position
        q = st.orientation
        return (p.x_val, p.y_val, p.z_val), (q.w_val, q.x_val, q.y_val, q.z_val)

    def scene_rgb(self):
        req = airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
        resp = self.client.simGetImages([req], vehicle_name=self.vehicle)[0]
        arr = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
        n = arr.size // (resp.height * resp.width)
        a = arr.reshape(resp.height, resp.width, n)
        if n == 4:
            a = a[:, :, 2::-1]
        return np.ascontiguousarray(a[:, :, :3])

    # ---------------------------------------------------------------- 控制
    def reset(self, x=0.0, y=0.0, z=-5.0, yaw_deg=0.0):
        self.client.enableApiControl(True, vehicle_name=self.vehicle)
        self.client.armDisarm(True, vehicle_name=self.vehicle)
        pose = airsim.Pose(airsim.Vector3r(x, y, z),
                           airsim.to_quaternion(0, 0, np.deg2rad(yaw_deg)))
        self.client.simSetVehiclePose(pose, True, vehicle_name=self.vehicle)
        self.client.moveToZAsync(z, 3, vehicle_name=self.vehicle).join()
        self.buffer.clear()

    def fly(self, forward, yaw_rate, z_target, duration=0.2, wait=True):
        """沿机头方向以 forward 前进，按 yaw_rate 偏航，高度闭环。

        wait=False 时立即返回 future，由调用方在指令执行中段采样，
        这样读到的是「正在飞」时的姿态而不是停下后的水平姿态。
        """
        p, q = self.pose()
        yaw = yaw_from_quat(q)
        vz = float(np.clip(0.8 * (z_target - p[2]), -1.5, 1.5))
        fut = self.client.moveByVelocityAsync(
            float(forward) * float(np.cos(yaw)),
            float(forward) * float(np.sin(yaw)),
            vz, float(duration),
            yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=float(np.rad2deg(yaw_rate))),
            vehicle_name=self.vehicle,
        )
        if wait:
            fut.join()
        return fut

    def land(self):
        self.client.landAsync(vehicle_name=self.vehicle).join()

    def close(self):
        try:
            self.client.armDisarm(False, vehicle_name=self.vehicle)
            self.client.enableApiControl(False, vehicle_name=self.vehicle)
        except Exception:
            pass
