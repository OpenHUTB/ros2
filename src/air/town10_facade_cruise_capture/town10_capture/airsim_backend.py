"""单线程 AirSim RPC 适配；只使用服务端实际返回的状态与 RGB 图像。"""

import io
import math
import time

import numpy as np
from PIL import Image

from .config import ned_to_enu


def quaternion_matrix(q):
    x, y, z, w = np.asarray(q, dtype=float)
    n = x * x + y * y + z * z + w * w
    if not math.isfinite(n) or n < 1e-12:
        raise ValueError("姿态四元数无效")
    x, y, z, w = np.array([x, y, z, w]) / math.sqrt(n)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_quaternion(matrix):
    """旋转矩阵转xyzw；特征值形式覆盖180度旋转。"""
    m = np.asarray(matrix)
    k = (
        np.array(
            [
                [
                    m[0, 0] - m[1, 1] - m[2, 2],
                    m[0, 1] + m[1, 0],
                    m[0, 2] + m[2, 0],
                    m[2, 1] - m[1, 2],
                ],
                [
                    m[0, 1] + m[1, 0],
                    m[1, 1] - m[0, 0] - m[2, 2],
                    m[1, 2] + m[2, 1],
                    m[0, 2] - m[2, 0],
                ],
                [
                    m[0, 2] + m[2, 0],
                    m[1, 2] + m[2, 1],
                    m[2, 2] - m[0, 0] - m[1, 1],
                    m[1, 0] - m[0, 1],
                ],
                [m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1], m.trace()],
            ]
        )
        / 3
    )
    values, vectors = np.linalg.eigh(k)
    q = vectors[:, np.argmax(values)]
    return q if q[3] >= 0 else -q


def vector(value):
    return np.array([value.x_val, value.y_val, value.z_val], dtype=float)


class AirSimBackend:
    def __init__(self, config, client=None, api=None):
        if api is None:
            try:
                import airsim as api
            except ImportError as exc:
                raise RuntimeError(
                    "AirSim后端需要 pip install airsim；MuJoCo后端不需要该依赖"
                ) from exc
        self.api, self.config = api, config
        self.client = (
            client
            if client is not None
            else api.MultirotorClient(ip=config.host, port=config.port, timeout_value=5)
        )
        self.active = False
        self.last_image_metadata = {}
        self.depth_samples = None
        self.depth_statistics = {}
        self._observation_time = -math.inf
        self._pixels = None
        self.client.confirmConnection()
        collision = self.client.simGetCollisionInfo(vehicle_name=config.vehicle)
        self.collision_timestamp_baseline = int(
            getattr(collision, "time_stamp", getattr(collision, "timestamp", 0))
        )
        self._collision_was_active = bool(collision.has_collided)
        self.client.enableApiControl(True, vehicle_name=config.vehicle)
        self.active = True
        try:
            landed = self.client.getMultirotorState(
                vehicle_name=config.vehicle
            ).landed_state
            if not config.takeoff and landed == self.api.LandedState.Landed:
                raise RuntimeError("无人机尚未起飞；先悬停后运行，或显式指定 --takeoff")
            if config.takeoff:
                self.client.armDisarm(True, vehicle_name=config.vehicle)
                self.client.takeoffAsync(
                    timeout_sec=15, vehicle_name=config.vehicle
                ).join()
            self.client.hoverAsync(vehicle_name=config.vehicle).join()
            self.anchor, _, self.orientation = self.state()
            q_ned = self.client.getMultirotorState(
                vehicle_name=config.vehicle
            ).kinematics_estimated.orientation
            r = quaternion_matrix([q_ned.x_val, q_ned.y_val, q_ned.z_val, q_ned.w_val])
            self.yaw = math.degrees(math.atan2(r[1, 0], r[0, 0]))
            yaw = math.radians(self.yaw)
            self.forward = ned_to_enu([math.cos(yaw), math.sin(yaw), 0.0])
            self.right = ned_to_enu([-math.sin(yaw), math.cos(yaw), 0.0])
            self.ranges()
            if self.depth_statistics["front_depth_min_m"] < 1.2:
                raise RuntimeError("初始前向深度低于1.2m，保持悬停并拒绝巡航")
        except BaseException:
            self.close()
            raise

    def state(self):
        state = self.client.getMultirotorState(vehicle_name=self.config.vehicle)
        kin = state.kinematics_estimated
        position, velocity = (
            ned_to_enu(vector(kin.position)),
            ned_to_enu(vector(kin.linear_velocity)),
        )
        q = kin.orientation
        r = quaternion_matrix([q.x_val, q.y_val, q.z_val, q.w_val])
        c = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        orientation = matrix_quaternion(c @ r @ np.diag([1, -1, -1]))
        self.timestamp_ns = int(state.timestamp)
        if not np.isfinite(np.concatenate((position, velocity, orientation))).all():
            raise RuntimeError("AirSim状态包含非有限数值")
        return position, velocity, orientation

    def check_collision(self):
        collision = self.client.simGetCollisionInfo(vehicle_name=self.config.vehicle)
        timestamp = int(
            getattr(collision, "time_stamp", getattr(collision, "timestamp", 0))
        )
        fresh = timestamp > self.collision_timestamp_baseline or (
            timestamp == 0 and not self._collision_was_active
        )
        self._collision_was_active = bool(collision.has_collided)
        if collision.has_collided and fresh:
            raise RuntimeError("AirSim报告碰撞，终止巡航并请求悬停")

    def step(self, command, dt=0.05):
        command = np.asarray(command, dtype=float)
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError("速度指令必须为三个有限数")
        ned = ned_to_enu(command)  # 此变换矩阵是自身的逆。
        self.client.moveByVelocityAsync(
            *map(float, ned),
            duration=dt * 2,
            drivetrain=self.api.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=self.api.YawMode(False, self.yaw),
            vehicle_name=self.config.vehicle,
        )

    def _observe(self, force=False):
        if not force and time.monotonic() - self._observation_time < 0.2:
            return
        requests = [
            self.api.ImageRequest(
                self.config.camera, self.api.ImageType.Scene, False, True
            ),
            self.api.ImageRequest(
                self.config.camera, self.api.ImageType.DepthPlanar, True, False
            ),
        ]
        responses = self.client.simGetImages(requests, vehicle_name=self.config.vehicle)
        if len(responses) != 2:
            raise RuntimeError("相机未同时返回Scene和DepthPlanar")
        response = responses[0]
        if response.width <= 0 or response.height <= 0:
            raise RuntimeError("相机图像宽高无效")
        raw = bytes(response.image_data_uint8)
        # OpenHUTB 2.10的未压缩Scene缓冲为BGR；请求PNG并显式解码RGB消除通道歧义。
        try:
            with Image.open(io.BytesIO(raw)) as encoded:
                pixels = np.asarray(encoded.convert("RGB")).copy()
        except Exception as exc:
            raise RuntimeError("Scene未返回可解码的压缩图像") from exc
        if pixels.shape != (response.height, response.width, 3):
            raise RuntimeError("Scene图像尺寸与响应宽高不一致")
        self.last_image_metadata = {"sensor_timestamp_ns": int(response.time_stamp)}
        if hasattr(response, "camera_position"):
            self.last_image_metadata["camera_position_enu_m"] = ned_to_enu(
                vector(response.camera_position)
            ).tolist()
        if hasattr(response, "camera_orientation"):
            q = response.camera_orientation
            self.last_image_metadata["camera_orientation_ned_xyzw"] = [
                float(q.x_val),
                float(q.y_val),
                float(q.z_val),
                float(q.w_val),
            ]
        depth_response = responses[1]
        depth = np.asarray(depth_response.image_data_float, dtype=float)
        if (
            depth_response.width < 3
            or depth_response.height < 3
            or depth.size != depth_response.width * depth_response.height
        ):
            raise RuntimeError("DepthPlanar宽高或数据长度无效")
        depth = depth.reshape(depth_response.height, depth_response.width)
        # 中央3×3网格覆盖画面40%..60%，避免天空主导前向墙面估计。
        ys = np.rint(np.linspace(0.4, 0.6, 3) * (depth.shape[0] - 1)).astype(int)
        xs = np.rint(np.linspace(0.4, 0.6, 3) * (depth.shape[1] - 1)).astype(int)
        samples = depth[np.ix_(ys, xs)].reshape(-1)
        valid = np.isfinite(samples) & (samples > 0.05) & (samples < 1000.0)
        if valid.sum() < 5:
            raise RuntimeError("DepthPlanar九点中有效值不足5个，停止巡航")
        self.depth_samples = np.where(valid, samples, float(np.median(samples[valid])))
        self.depth_statistics = {
            "front_depth_min_m": float(np.min(samples[valid])),
            "front_depth_median_m": float(np.median(samples[valid])),
            "front_depth_valid_count": int(valid.sum()),
            "depth_sensor_timestamp_ns": int(depth_response.time_stamp),
        }
        self.last_image_metadata.update(self.depth_statistics)
        self._pixels = pixels
        self._observation_time = time.monotonic()

    def ranges(self):
        self._observe()
        # 将真实Planar深度转换为既有九束输入格式，投影后仍等于原始Planar深度。
        from .scene import RAY_ANGLES

        return np.array(
            [
                (value - 0.14) / (math.cos(a) * math.cos(b))
                for value, (a, b) in zip(self.depth_samples, RAY_ANGLES)
            ]
        )

    def image(self, camera="front", force=False):
        self._observe(force=force)
        return self._pixels.copy()

    def close(self):
        if self.active:
            try:
                self.client.hoverAsync(vehicle_name=self.config.vehicle).join()
            finally:
                self.client.enableApiControl(False, vehicle_name=self.config.vehicle)
                self.active = False
