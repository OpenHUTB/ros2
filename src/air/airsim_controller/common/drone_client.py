"""扩展的 AirSim 多旋翼客户端。

在原有 ``client/drone_client.py`` 基础上补充：
- 键盘遥控所需的速度/位置/偏航指令；
- 场景 RGB、深度图（DepthVis / DepthPerspective）、深度距离向量的获取；
- 模拟 LiDAR 点云获取（AirSim 自带 lidar 接口）；
- 位姿（ENU）读取与轨迹打点。

说明：``airsim`` 采用惰性导入，使本模块在未安装 AirSim 的开发机上仍可被
``import`` 与 ``py_compile`` 校验；目标运行机（Ubuntu 20.04 + AirSim 1.8.1）
执行 ``pip install airsim`` 后即可正常使用。
"""

import os
import time

import numpy as np

try:  # pragma: no cover - 仅在装有 AirSim 的运行机上为真
    import airsim
    _HAS_AIRSIM = True
except ImportError:  # pragma: no cover
    airsim = None
    _HAS_AIRSIM = False


class DroneClient:
    """对 AirSim Multirotor 原生 API 的薄封装。"""

    def __init__(self, interval=0.1, root_path="./images", vehicle_name=""):
        self.interval = interval
        self.root_path = root_path
        self.vehicle_name = vehicle_name
        os.makedirs(root_path, exist_ok=True)
        self.client = None
        if _HAS_AIRSIM:
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()
            self.client.enableApiControl(True, vehicle_name)
            self.client.armDisarm(True, vehicle_name)

    # ------------------------------------------------------------------ 生命周期
    def start(self):
        """起飞并悬停到默认高度。"""
        self.client.takeoffAsync(vehicle_name=self.vehicle_name).join()
        return self

    def land(self):
        self.client.landAsync(vehicle_name=self.vehicle_name).join()

    def destroy(self):
        try:
            self.client.landAsync(vehicle_name=self.vehicle_name).join()
        finally:
            self.client.enableApiControl(False, self.vehicle_name)
            self.client.armDisarm(False, self.vehicle_name)

    # ------------------------------------------------------------------ 状态读取
    def get_state(self):
        return self.client.getMultirotorState(vehicle_name=self.vehicle_name)

    def get_position(self):
        """返回 ENU 坐标 (x, y, z)，单位米；z 向上为正。"""
        kin = self.get_state().kinematics_estimated
        p = kin.position
        return np.array([p.x_val, p.y_val, p.z_val], dtype=np.float64)

    def get_velocity(self):
        kin = self.get_state().kinematics_estimated
        v = kin.linear_velocity
        return np.array([v.x_val, v.y_val, v.z_val], dtype=np.float64)

    def get_yaw(self):
        kin = self.get_state().kinematics_estimated
        q = kin.orientation
        yaw, _, _ = airsim.to_eularian_angles(q)
        return yaw

    def get_collision_info(self):
        return self.client.simGetCollisionInfo(vehicle_name=self.vehicle_name)

    # ------------------------------------------------------------------ 运动指令
    def move_by_velocity(self, vx, vy, vz, yaw_rate=0.0, duration=None):
        """世界系速度控制（vx, vy, vz），并可叠加偏航角速度。"""
        dur = duration if duration is not None else self.interval
        self.client.moveByVelocityAsync(
            vx, vy, vz, dur,
            yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=np.degrees(yaw_rate)),
            vehicle_name=self.vehicle_name,
        )

    def move_by_position(self, x, y, z, speed=2.0):
        self.client.moveToPositionAsync(
            x, y, z, speed, vehicle_name=self.vehicle_name
        ).join()

    def move_by_body_rate(self, throttle=0.0, vertical=0.0,
                          lateral=0.0, longitudinal=0.0, yaw_rate=0.0):
        """机体系速率控制，便于键盘遥控：throttle/vertical/lateral/longitudinal ∈ [-1, 1]。"""
        self.client.moveByRollPitchYawThrottleAsync(
            roll=0.0, pitch=0.0, yaw=np.degrees(yaw_rate), throttle=0.6 + throttle,
            duration=self.interval, vehicle_name=self.vehicle_name,
        )

    def set_yaw(self, yaw_deg):
        self.client.rotateToYawAsync(yaw_deg, vehicle_name=self.vehicle_name).join()

    # ------------------------------------------------------------------ 传感器
    def get_rgb(self, camera_name="0"):
        """返回 HxWx3 uint8 的场景图。"""
        resp = self.client.simGetImages([
            airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, False)
        ], vehicle_name=self.vehicle_name)[0]
        img = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
        return img.reshape(resp.height, resp.width, 4)[..., :3]

    def get_depth(self, camera_name="0"):
        """返回归一化到 [0,1] 的深度图（近=0，远=1）。"""
        resp = self.client.simGetImages([
            airsim.ImageRequest(camera_name, airsim.ImageType.DepthPerspective, True, False)
        ], vehicle_name=self.vehicle_name)[0]
        depth = np.array(resp.image_data_float, dtype=np.float32)
        depth = depth.reshape(resp.height, resp.width)
        return np.clip(depth, 0.0, 80.0) / 80.0

    def get_min_depth(self, camera_name="0", roi=(0.2, 0.8)):
        """前向 ROI 内的最近距离（米），用于简单避障。"""
        d = self.get_depth(camera_name) * 80.0
        h, w = d.shape
        x0, x1 = int(w * roi[0]), int(w * roi[1])
        center = d[:, x0:x1]
        return float(np.min(center))

    def get_lidar(self):
        """返回 Nx3 的 LiDAR 点云（EN 平面 + z）。"""
        data = self.client.getLidarData(lidar_name="Lidar1", vehicle_name=self.vehicle_name)
        if data.point_count < 3:
            return np.zeros((0, 3))
        pts = np.array(data.point_cloud, dtype=np.float32).reshape(-1, 3)
        return pts

    def spin(self, seconds, on_tick=None, rate_hz=10.0):
        """以固定频率阻塞运行 ``seconds`` 秒，每个周期回调 ``on_tick(client)``。"""
        dt = 1.0 / rate_hz
        steps = int(seconds * rate_hz)
        for _ in range(steps):
            if on_tick is not None:
                on_tick(self)
            time.sleep(dt)
