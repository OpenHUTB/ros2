"""AirSim 风扰环境：注入侧风，无人机用速度指令对抗。

任务属于课程任务 ①「物理仿真控制」：在外部扰动下维持定点悬停。
"""
from __future__ import annotations

from collections import deque

import airsim
import numpy as np

IMU_DIM = 6
WINDOW = 5
CMD_SCALE = 3.0     # 补偿速度归一化尺度（m/s）


def yaw_from_quat(q) -> float:
    w, x, y, z = q
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


class WindEnv:
    def __init__(self, ip="127.0.0.1", port=41451, vehicle="Drone1"):
        self.client = airsim.MultirotorClient(ip=ip, port=port)
        self.client.confirmConnection()
        self.vehicle = vehicle
        self.buffer = deque(maxlen=WINDOW)
        self.client.enableApiControl(True, vehicle_name=vehicle)
        self.client.armDisarm(True, vehicle_name=vehicle)

    # ------------------------------------------------------------ 传感器
    def imu(self):
        d = self.client.getImuData(vehicle_name=self.vehicle)
        return np.array([
            d.linear_acceleration.x_val, d.linear_acceleration.y_val, d.linear_acceleration.z_val,
            d.angular_velocity.x_val, d.angular_velocity.y_val, d.angular_velocity.z_val,
        ], dtype=np.float32)

    def imu_window(self):
        vec = self.imu()
        if not self.buffer:
            for _ in range(WINDOW):
                self.buffer.append(vec)
        else:
            self.buffer.append(vec)
        return np.concatenate(list(self.buffer), axis=0)

    def pose(self):
        st = self.client.getMultirotorState(vehicle_name=self.vehicle).kinematics_estimated
        p = st.position
        q = st.orientation
        return np.array([p.x_val, p.y_val, p.z_val], dtype=np.float64), \
               (q.w_val, q.x_val, q.y_val, q.z_val)

    def scene_rgb(self):
        req = airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
        resp = self.client.simGetImages([req], vehicle_name=self.vehicle)[0]
        arr = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
        n = arr.size // (resp.height * resp.width)
        return np.ascontiguousarray(arr.reshape(resp.height, resp.width, n)[:, :, 2::-1])

    # ------------------------------------------------------------ 扰动
    def set_wind(self, wx, wy, wz=0.0):
        """注入世界系风速（m/s）。"""
        self.client.simSetWind(airsim.Vector3r(float(wx), float(wy), float(wz)))

    def clear_wind(self):
        self.client.simSetWind(airsim.Vector3r(0.0, 0.0, 0.0))

    # ------------------------------------------------------------ 控制
    def reset(self, x=0.0, y=0.0, z=-5.0, yaw_deg=0.0):
        self.clear_wind()
        self.client.enableApiControl(True, vehicle_name=self.vehicle)
        self.client.armDisarm(True, vehicle_name=self.vehicle)
        pose = airsim.Pose(airsim.Vector3r(x, y, z),
                           airsim.to_quaternion(0, 0, np.deg2rad(yaw_deg)))
        self.client.simSetVehiclePose(pose, True, vehicle_name=self.vehicle)
        self.client.moveToZAsync(z, 3, vehicle_name=self.vehicle).join()
        self.buffer.clear()

    def apply_velocity(self, dvx, dvy, z_target, duration=0.25, wait=False):
        """世界系速度补偿指令（不打折扣地按世界系下发）。"""
        p, _ = self.pose()
        vz = float(np.clip(0.8 * (z_target - p[2]), -1.5, 1.5))
        fut = self.client.moveByVelocityAsync(
            float(dvx), float(dvy), vz, float(duration),
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0.0),
            vehicle_name=self.vehicle,
        )
        if wait:
            fut.join()
        return fut

    def land(self):
        self.clear_wind()
        self.client.landAsync(vehicle_name=self.vehicle).join()

    def close(self):
        try:
            self.clear_wind()
            self.client.armDisarm(False, vehicle_name=self.vehicle)
            self.client.enableApiControl(False, vehicle_name=self.vehicle)
        except Exception:
            pass
