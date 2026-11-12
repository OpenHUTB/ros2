"""AirSim 深度避障环境封装。

只暴露三件事：
* ``reset()``  把无人机放到指定位姿
* ``depth()``  取前视相机的深度图并降采样到 64x64
* ``step()``   按（偏航角速度, 目标高度, 前飞速度）飞一小段

所有控制都走 AirSim 的高层 API，不做底层电机混控，保证任何装了 AirSim 的
机器都能复现。
"""
from __future__ import annotations

import time

import numpy as np

try:
    import airsim
except ImportError as exc:  # pragma: no cover
    raise ImportError("未安装 airsim：pip install airsim==1.8.1") from exc


def _resize_bilinear(img: np.ndarray, h: int, w: int) -> np.ndarray:
    """纯 NumPy 双线性降采样，避免额外依赖 cv2。"""
    src_h, src_w = img.shape[:2]
    if (src_h, src_w) == (h, w):
        return img
    ys = (np.arange(h) + 0.5) * (src_h / h) - 0.5
    xs = (np.arange(w) + 0.5) * (src_w / w) - 0.5
    ys = np.clip(ys, 0, src_h - 1)
    xs = np.clip(xs, 0, src_w - 1)
    y0 = np.floor(ys).astype(np.int32)
    x0 = np.floor(xs).astype(np.int32)
    y1 = np.minimum(y0 + 1, src_h - 1)
    x1 = np.minimum(x0 + 1, src_w - 1)
    wy = (ys - y0)[:, None]
    wx = (xs - x0)[None, :]
    a = img[y0][:, x0]
    b = img[y0][:, x1]
    c = img[y1][:, x0]
    d = img[y1][:, x1]
    out = (a * (1 - wx) * (1 - wy) + b * wx * (1 - wy)
           + c * (1 - wx) * wy + d * wx * wy)
    return out.astype(np.float32)


def yaw_from_quat(q) -> float:
    """四元数 (w,x,y,z) -> 偏航角（弧度）。"""
    w, x, y, z = q
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


class DepthAvoidEnv:
    """前视深度相机 + 高层速度控制。"""

    def __init__(self, ip="127.0.0.1", port=41451, camera="0", out_size=64,
                 max_depth=30.0):
        self.client = airsim.MultirotorClient(ip=ip, port=port)
        self.client.confirmConnection()
        self.camera = camera
        self.out_size = out_size
        self.max_depth = max_depth
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

    # ---------------------------------------------------------------- 状态
    def pose(self):
        st = self.client.getMultirotorState().kinematics_estimated
        p = st.position
        q = st.orientation
        return (p.x_val, p.y_val, p.z_val), (q.w_val, q.x_val, q.y_val, q.z_val)

    def depth(self):
        """返回 (64, 64) 的归一化深度图，1.0 表示很远/天空。"""
        req = airsim.ImageRequest(self.camera, airsim.ImageType.DepthPlanar, True, False)
        resp = self.client.simGetImages([req])[0]
        d = np.array(resp.image_data_float, dtype=np.float32).reshape(resp.height, resp.width)
        d = np.nan_to_num(d, nan=self.max_depth, posinf=self.max_depth)
        d = _resize_bilinear(d, self.out_size, self.out_size)
        d = np.clip(d, 0.0, self.max_depth) / self.max_depth
        return d

    def scene_rgb(self, width=320, height=240):
        """取一帧真实画面。AirSim 的 Scene 图是 BGRA，这里转成 RGB。"""
        req = airsim.ImageRequest(self.camera, airsim.ImageType.Scene, False, False)
        resp = self.client.simGetImages([req])[0]
        arr = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
        n = arr.size // (resp.height * resp.width)
        arr = arr.reshape(resp.height, resp.width, n)
        rgb = arr[:, :, 2::-1] if n >= 3 else arr[:, :, :3]
        return np.ascontiguousarray(rgb)

    # ---------------------------------------------------------------- 控制
    def reset(self, x=0.0, y=0.0, z=-5.0, yaw_deg=0.0):
        """只重新摆放位姿，不重置整个场景（比重启场景快得多）。

        摆好之后用 ``moveToZAsync`` 把高度钉住：AirSim 的 hover 不锁高度，
        直接开飞会掉高。
        """
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        pose = airsim.Pose(airsim.Vector3r(x, y, z),
                           airsim.to_quaternion(0, 0, np.deg2rad(yaw_deg)))
        self.client.simSetVehiclePose(pose, True)
        time.sleep(0.2)
        self.client.moveToZAsync(z, 3).join()
        time.sleep(0.2)

    def step(self, yaw_rate, z_target, vx=2.5, duration=0.25):
        """飞一小段：沿机头方向前进 ``vx``，按 ``yaw_rate`` 偏航，高度用误差反馈闭环。

        ``moveByVelocityAsync`` 的 vx/vy 是世界系分量，不是机体系。所以必须把
        「沿机头前进」分解成 (cos yaw, sin yaw) 两个世界系分量，否则不管机头
        怎么转，无人机都只会沿 +x 直线飞。
        """
        p, q = self.pose()
        yaw = yaw_from_quat(q)
        vz = float(np.clip(0.8 * (z_target - p[2]), -1.5, 1.5))
        yaw_mode = airsim.YawMode(is_rate=True,
                                  yaw_or_rate=float(np.rad2deg(yaw_rate)))
        self.client.moveByVelocityAsync(
            float(vx) * float(np.cos(yaw)),
            float(vx) * float(np.sin(yaw)),
            vz, float(duration), yaw_mode=yaw_mode).join()

    def land(self):
        self.client.landAsync().join()

    def close(self):
        try:
            self.client.armDisarm(False)
            self.client.enableApiControl(False)
        except Exception:  # noqa: BLE001
            pass
