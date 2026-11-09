"""AirSim 前视深度环境：为 TTC 预测采集画面与真值。

TTC = 最近障碍距离 / 朝障碍的速度，单位秒。
"""
from __future__ import annotations

import time

import airsim
import numpy as np

TTC_MAX = 20.0     # 归一化上限（秒）


def yaw_from_quat(q):
    w, x, y, z = q
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _resize_bilinear(src, out_h, out_w):
    h, w = src.shape
    ys = np.linspace(0, h - 1, out_h)
    xs = np.linspace(0, w - 1, out_w)
    y0 = np.floor(ys).astype(int); y1 = np.minimum(y0 + 1, h - 1)
    x0 = np.floor(xs).astype(int); x1 = np.minimum(x0 + 1, w - 1)
    wy = (ys - y0)[:, None]; wx = (xs - x0)[None, :]
    a = src[np.ix_(y0, x0)]; b = src[np.ix_(y0, x1)]
    c = src[np.ix_(y1, x0)]; d = src[np.ix_(y1, x1)]
    return ((a * (1 - wx) + b * wx) * (1 - wy) + (c * (1 - wx) + d * wx) * wy)


class TtcEnv:
    def __init__(self, ip="127.0.0.1", port=41451, vehicle="Drone1", size=64):
        self.client = airsim.MultirotorClient(ip=ip, port=port)
        self.client.confirmConnection()
        self.vehicle = vehicle
        self.size = size
        self.client.enableApiControl(True, vehicle_name=vehicle)
        self.client.armDisarm(True, vehicle_name=vehicle)

    # ------------------------------------------------------------ 传感器
    def depth(self):
        req = airsim.ImageRequest("0", airsim.ImageType.DepthPlanar, True, False)
        r = self.client.simGetImages([req], vehicle_name=self.vehicle)[0]
        d = np.array(r.image_data_float, dtype=np.float32).reshape(r.height, r.width)
        d = np.clip(d, 0.0, 30.0)
        small = _resize_bilinear(d, self.size, self.size)
        return (small / 30.0).astype(np.float32)

    def depth_raw(self):
        req = airsim.ImageRequest("0", airsim.ImageType.DepthPlanar, True, False)
        r = self.client.simGetImages([req], vehicle_name=self.vehicle)[0]
        return np.array(r.image_data_float, dtype=np.float32).reshape(r.height, r.width)

    def scene_rgb(self):
        req = airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
        resp = self.client.simGetImages([req], vehicle_name=self.vehicle)[0]
        arr = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
        n = arr.size // (resp.height * resp.width)
        return np.ascontiguousarray(arr.reshape(resp.height, resp.width, n)[:, :, 2::-1])

    def pose(self):
        st = self.client.getMultirotorState(vehicle_name=self.vehicle).kinematics_estimated
        p = st.position
        q = st.orientation
        return np.array([p.x_val, p.y_val, p.z_val]), (q.w_val, q.x_val, q.y_val, q.z_val)

    # ------------------------------------------------------------ 控制
    def reset(self, x=-12.0, y=0.0, z=-4.0, yaw_deg=0.0):
        self.client.enableApiControl(True, vehicle_name=self.vehicle)
        self.client.armDisarm(True, vehicle_name=self.vehicle)
        pose = airsim.Pose(airsim.Vector3r(x, y, z),
                           airsim.to_quaternion(0, 0, np.deg2rad(yaw_deg)))
        self.client.simSetVehiclePose(pose, True, vehicle_name=self.vehicle)
        self.client.moveToZAsync(z, 3, vehicle_name=self.vehicle).join()

    def fly_forward(self, speed, z_target, duration=0.25, wait=True):
        """沿机头方向前进（世界系分解）."""
        _, q = self.pose()
        yaw = yaw_from_quat(q)
        vz = float(np.clip(0.8 * (z_target - self.pose()[0][2]), -1.5, 1.5))
        fut = self.client.moveByVelocityAsync(
            speed * np.cos(yaw), speed * np.sin(yaw), vz, duration,
            yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=np.rad2deg(yaw)),
            vehicle_name=self.vehicle)
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
