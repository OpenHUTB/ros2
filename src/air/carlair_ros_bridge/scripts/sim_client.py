#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CarlaAir / AirSim 客户端封装与坐标系换算（ROS Noetic, Python 3.8）.

本模块把 AirSim 的 NED（北-东-地）坐标统一换算为 ROS 的 ENU（东-北-天）坐标，
并提供连接、自检、状态读取与指令下发的薄封装。所有 ROS 节点都通过它访问仿真器，
避免坐标换算散落在各处。

换算关系
--------
位置：  ENU = C · NED，其中 C = [[0,1,0],[1,0,0],[0,0,-1]]
        即  x_enu = y_ned,  y_enu = x_ned,  z_enu = -z_ned
姿态：  同一物理姿态在 ENU 下的旋转矩阵  R_enu = C · R_ned
        （C·v_ned = v_enu 对同一物理向量成立，故旋转矩阵左乘 C）
速度：  线速度、角速度同为世界系向量，v_enu = C · v_ned

注意：AirSim 四元数顺序为 (w, x, y, z)，ROS 为 (x, y, z, w)。
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np

# NED -> ENU 的坐标变换矩阵（正交，det = +1）
C_NED2ENU = np.array([[0.0, 1.0, 0.0],
                      [1.0, 0.0, 0.0],
                      [0.0, 0.0, -1.0]], dtype=np.float64)


# --------------------------------------------------------------------- 基础换算
def ned_to_enu(vec: np.ndarray) -> np.ndarray:
    """NED 向量 -> ENU 向量."""
    return C_NED2ENU @ np.asarray(vec, dtype=np.float64)


def enu_to_ned(vec: np.ndarray) -> np.ndarray:
    """ENU 向量 -> NED 向量（C 正交且对称，逆变换即自身）."""
    return C_NED2ENU @ np.asarray(vec, dtype=np.float64)


def quat_wxyz_to_matrix(q_wxyz) -> np.ndarray:
    """AirSim 四元数 (w, x, y, z) -> 3x3 旋转矩阵（机体系 -> 世界系）."""
    w, x, y, z = [float(v) for v in q_wxyz]
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def matrix_to_quat_xyzw(R: np.ndarray) -> Tuple[float, float, float, float]:
    """3x3 旋转矩阵 -> ROS 四元数 (x, y, z, w)（用 Shepperd 分支法，数值稳定）."""
    m = np.asarray(R, dtype=np.float64)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n)


def airsim_quat_to_ros_quat(q_wxyz) -> Tuple[float, float, float, float]:
    """AirSim(NED) 四元数 -> ROS(ENU) 四元数 (x, y, z, w)."""
    R_enu = C_NED2ENU @ quat_wxyz_to_matrix(q_wxyz)
    return matrix_to_quat_xyzw(R_enu)


def yaw_from_quat_xyzw(q) -> float:
    """从 ROS 四元数取偏航角（ENU 下，绕 z 轴，逆时针为正）."""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# --------------------------------------------------------------------- 图像解码
def _to_uint8_array(data) -> np.ndarray:
    """把 AirSim 返回的图像字节（bytes / list[int]）统一成 uint8 一维数组."""
    if isinstance(data, (bytes, bytearray, memoryview)):
        return np.frombuffer(bytes(data), dtype=np.uint8)
    return np.asarray(data, dtype=np.uint8).ravel()


def decode_rgb(resp) -> np.ndarray:
    """ImageResponse(Scene / Segmentation) -> (H, W, 3) uint8，通道顺序 BGR.

    AirSim 的 `image_data_uint8` 是 w*h*4 的 BGRA 平面数组（compress=False 时），
    丢掉 alpha 后前三个通道正好是 B、G、R，可直接填 ROS 的 `bgr8` 编码，
    因此本函数不依赖 OpenCV。若返回 3 通道则按 BGR 直接使用。
    """
    w, h = int(resp.width), int(resp.height)
    if w <= 0 or h <= 0:
        raise ValueError("图像尺寸非法: %sx%s" % (resp.width, resp.height))
    buf = _to_uint8_array(resp.image_data_uint8)
    pix = buf.size // (w * h)
    if pix >= 4:
        img = buf[:w * h * 4].reshape(h, w, 4)[:, :, :3]
    elif pix == 3:
        img = buf[:w * h * 3].reshape(h, w, 3)
    else:
        raise ValueError("无法解析的图像数据: %d 字节 / %dx%d" % (buf.size, w, h))
    return np.ascontiguousarray(img)


def decode_depth(resp) -> np.ndarray:
    """ImageResponse(DepthPlanar) -> (H, W) float32，单位米（平面深度）.

    require `pixels_as_float=True`，此时数据在 `image_data_float` 中。
    """
    w, h = int(resp.width), int(resp.height)
    arr = np.asarray(resp.image_data_float, dtype=np.float32).ravel()
    if w <= 0 or h <= 0 or arr.size != w * h:
        raise ValueError("深度图尺寸不匹配: %d 个浮点数 / %dx%d" % (arr.size, w, h))
    return arr.reshape(h, w)


# --------------------------------------------------------------------- 点云解码
def lidar_points_to_enu(cloud, pose=None, frame: str = "vehicle_inertial") -> np.ndarray:
    """AirSim LiDAR 点云 -> ENU（N x 3）.

    AirSim 的 `DataFrame` 设置决定点云所在坐标系（见 AirSim 官方 lidar 文档）：
      * ``vehicle_inertial``（默认）：点已在**载具惯性系**（NED 世界系，单位米），
        与 `/uav/odom` 同源，只需做 NED -> ENU 的轴变换；
      * ``sensor_local``：点在**雷达局部系**，需先用雷达位姿
        ``p_world = R(q_pose) · p_sensor + t_pose`` 变换到惯性系，再做轴变换。
    雷达位姿 `LidarData.pose` 的语义为"雷达在载具惯性系中的位姿"（NED）。
    """
    pts = np.asarray(cloud, dtype=np.float64).ravel()
    if pts.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    if pts.size % 3 != 0:
        raise ValueError("点云长度 %d 不是 3 的整数倍" % pts.size)
    pts = pts.reshape(-1, 3)

    if frame == "sensor_local":
        if pose is None:
            raise ValueError("frame=sensor_local 时必须提供雷达位姿 pose")
        q = [pose.orientation.w_val, pose.orientation.x_val,
             pose.orientation.y_val, pose.orientation.z_val]
        t = np.array([pose.position.x_val, pose.position.y_val, pose.position.z_val],
                     dtype=np.float64)
        # 传感器局部系 -> 载具惯性系（NED）
        pts = pts.dot(quat_wxyz_to_matrix(q).T) + t
    elif frame != "vehicle_inertial":
        raise ValueError("未知的点云坐标系: %r" % (frame,))

    return np.ascontiguousarray(pts.dot(C_NED2ENU.T))


# --------------------------------------------------------------------- 运行期兼容
def _raise_msgpack_limits() -> None:
    """放宽 msgpack 解包器的长度上限（兼容 msgpack >= 0.5）.

    msgpack 0.6.x 的 ``Unpacker`` 默认 ``max_array_len = 131072``，而 AirSim 把
    深度图以 float 数组返回（640x480 = 307200 个元素），解包时会抛出
    ``ValueError: 307200 exceeds max_array_len(131072)``，导致取图连接被关闭、
    ``/camera/image_raw`` 等话题永远没有消息（节点表现为"无输出、很慢"）。

    这里在创建 AirSim 客户端之前，把相关上限放宽到 2^31-1。
    msgpack < 0.5（如 0.4.x）没有这些参数，探测失败时直接跳过，不影响老环境。
    """
    try:
        import msgpack  # noqa: WPS433 (运行期按需导入)
    except ImportError:
        return
    if getattr(msgpack, "_carlair_patched", False):
        return
    orig = msgpack.Unpacker
    try:
        orig(max_array_len=2 ** 31 - 1)  # 探测该版本是否支持长度上限参数
    except TypeError:
        return  # 老版本没有上限，无需 patch
    def _patched(*args, **kwargs):
        kwargs.setdefault("max_array_len", 2 ** 31 - 1)
        kwargs.setdefault("max_bin_len", 2 ** 31 - 1)
        kwargs.setdefault("max_str_len", 2 ** 31 - 1)
        kwargs.setdefault("max_map_len", 2 ** 31 - 1)
        return orig(*args, **kwargs)
    msgpack.Unpacker = _patched
    msgpack._carlair_patched = True


# --------------------------------------------------------------------- 客户端封装
class SimClient(object):
    """CarlaAir / AirSim 客户端的薄封装（只依赖 airsim 包，缺失时给出清晰报错）."""

    def __init__(self, host: str = "127.0.0.1", port: int = 41451,
                 vehicle_name: str = "", timeout: float = 10.0):
        self.host = host
        self.port = port
        self.vehicle_name = vehicle_name
        self.timeout = timeout
        self.client = None
        self._airsim = None

    # ---------------------------------------------------------------- 连接
    def connect(self) -> None:
        """连接仿真器；失败时抛出带操作提示的 RuntimeError."""
        try:
            import airsim  # noqa: WPS433 (运行期按需导入，便于无仿真时也能跑纯数学测试)
        except ImportError as exc:
            raise RuntimeError(
                "未安装 airsim 包。请在 CarlaAir 的 conda 环境中执行: pip install airsim"
            ) from exc
        _raise_msgpack_limits()  # 必须先于 MultirotorClient 创建，否则图像取不到
        self._airsim = airsim
        self.client = airsim.MultirotorClient(ip=self.host, port=self.port)
        self.client.confirmConnection()
        self.client.enableApiControl(True, vehicle_name=self.vehicle_name)
        self.client.armDisarm(True, vehicle_name=self.vehicle_name)

    def is_connected(self) -> bool:
        return self.client is not None

    # ---------------------------------------------------------------- 状态读取
    def get_state_enu(self):
        """返回 (position_enu(3), quat_xyzw(4), lin_vel_enu(3), ang_vel_enu(3))."""
        state = self.client.getMultirotorState(vehicle_name=self.vehicle_name)
        k = state.kinematics_estimated
        pos_enu = ned_to_enu([k.position.x_val, k.position.y_val, k.position.z_val])
        q_enu = airsim_quat_to_ros_quat(
            [k.orientation.w_val, k.orientation.x_val,
             k.orientation.y_val, k.orientation.z_val])
        vel_enu = ned_to_enu([k.linear_velocity.x_val,
                              k.linear_velocity.y_val,
                              k.linear_velocity.z_val])
        ang_enu = ned_to_enu([k.angular_velocity.x_val,
                              k.angular_velocity.y_val,
                              k.angular_velocity.z_val])
        return pos_enu, q_enu, vel_enu, ang_enu

    def get_collision(self) -> bool:
        state = self.client.getMultirotorState(vehicle_name=self.vehicle_name)
        return bool(state.collision.has_collided)

    # ---------------------------------------------------------------- 指令下发
    def takeoff(self, timeout: float = 20.0) -> None:
        self.client.takeoffAsync(timeout_sec=timeout,
                                 vehicle_name=self.vehicle_name).join()

    def land(self, timeout: float = 20.0) -> None:
        self.client.landAsync(timeout_sec=timeout,
                              vehicle_name=self.vehicle_name).join()

    def hover(self) -> None:
        self.client.hoverAsync(vehicle_name=self.vehicle_name)

    def move_by_velocity_enu(self, vel_enu, duration: float = 0.5,
                             yaw_rate: float = 0.0) -> None:
        """按 ENU 速度指令飞行（内部换算成 NED 后调用 AirSim）."""
        v = enu_to_ned(np.asarray(vel_enu, dtype=np.float64))
        yaw_mode = self._airsim.YawMode(is_rate=True, yaw_or_rate=math.degrees(yaw_rate))
        self.client.moveByVelocityAsync(
            float(v[0]), float(v[1]), float(v[2]), float(duration),
            yaw_mode=yaw_mode, vehicle_name=self.vehicle_name)

    def move_by_velocity_body(self, forward: float, left: float, up: float,
                              duration: float = 0.2, yaw_rate: float = 0.0) -> None:
        """按机体系速度飞行（前 / 左 / 上，符合 ROS 习惯）.

        机体系换算：AirSim 机体轴为 前-右-下，ROS 机体轴为 前-左-上，
        故 (forward, left, up) -> (forward, -left, -up)，无需世界系坐标变换。
        """
        yaw_mode = self._airsim.YawMode(is_rate=True, yaw_or_rate=math.degrees(yaw_rate))
        self.client.moveByVelocityBodyFrameAsync(
            float(forward), float(-left), float(-up), float(duration),
            yaw_mode=yaw_mode, vehicle_name=self.vehicle_name)

    def move_to_position_enu(self, pos_enu, speed: float = 2.0,
                             timeout: float = 30.0) -> None:
        """飞抵 ENU 目标点（阻塞至到达或超时）."""
        p = enu_to_ned(np.asarray(pos_enu, dtype=np.float64))
        self.client.moveToPositionAsync(
            float(p[0]), float(p[1]), float(p[2]), float(speed),
            timeout_sec=timeout, vehicle_name=self.vehicle_name).join()

    def reset(self) -> None:
        self.client.reset()
        self.client.enableApiControl(True, vehicle_name=self.vehicle_name)
        self.client.armDisarm(True, vehicle_name=self.vehicle_name)

    # ---------------------------------------------------------------- 传感器
    def get_images(self, camera: str = "front_rgb", image_type: int = 0,
                   pixels_as_float: bool = False):
        """取一帧图像；image_type: 0=RGB(Scene) 1=DepthPlanar 3=DepthVis 5=Segmentation."""
        req = [self._airsim.ImageRequest(camera, image_type, pixels_as_float, False)]
        resp = self.client.simGetImages(req, vehicle_name=self.vehicle_name)
        if not resp:
            return None
        return resp[0]

    def get_images_bundle(self, specs):
        """一次 RPC 取多路图像。

        specs: [(camera_name, image_type, pixels_as_float), ...]
        返回 [(camera_name, ImageResponse), ...]，顺序与入参一致。
        多路合并成一次调用可以避免每路一次网络往返，是保证 20 Hz 的关键。
        """
        req = [self._airsim.ImageRequest(cam, itype, as_float, False)
               for cam, itype, as_float in specs]
        resps = self.client.simGetImages(req, vehicle_name=self.vehicle_name)
        return list(zip([s[0] for s in specs], resps))

    def get_lidar_data(self, lidar_name: str = "lidar1"):
        """取一帧原始雷达数据（含 point_cloud 与雷达位姿）."""
        return self.client.getLidarData(lidar_name=lidar_name,
                                        vehicle_name=self.vehicle_name)

    def get_lidar_points_enu(self, lidar_name: str = "lidar1",
                             frame: str = "vehicle_inertial") -> Optional[np.ndarray]:
        """取一帧激光雷达点云并换算到 ENU（N x 3），无数据时返回 None.

        frame 需与 settings.json 中该雷达的 `DataFrame` 保持一致：
        `vehicle_inertial`（默认，世界系）或 `sensor_local`（雷达局部系）。
        """
        data = self.get_lidar_data(lidar_name)
        pts = lidar_points_to_enu(data.point_cloud, pose=getattr(data, "pose", None),
                                  frame=frame)
        return None if pts.shape[0] == 0 else pts


def self_check(host: str, port: int, vehicle_name: str = "") -> List[Tuple[str, bool, str]]:
    """环境自检：返回 [(项目, 是否通过, 说明)]，供 main.py 打印."""
    results: List[Tuple[str, bool, str]] = []
    try:
        import airsim  # noqa: F401
        results.append(("airsim 包可用", True, "已导入"))
    except ImportError as exc:
        results.append(("airsim 包可用", False, "pip install airsim（在 carlaAir 环境中）"))
        return results

    cli = SimClient(host=host, port=port, vehicle_name=vehicle_name)
    try:
        cli.connect()
        results.append(("连接仿真器 %s:%d" % (host, port), True, "confirmConnection 成功"))
    except Exception as exc:  # noqa: BLE001 - 自检需捕获所有异常并给出提示
        results.append(("连接仿真器 %s:%d" % (host, port), False,
                        "确认 CarlaAir.sh 已启动，且 host 可达。错误: %s" % exc))
        return results

    try:
        pos, quat, vel, _ = cli.get_state_enu()
        results.append(("读取位姿", True,
                        "pos_enu=(%.2f, %.2f, %.2f) yaw=%.1f°"
                        % (pos[0], pos[1], pos[2], math.degrees(yaw_from_quat_xyzw(quat)))))
    except Exception as exc:  # noqa: BLE001
        results.append(("读取位姿", False, str(exc)))

    try:
        img = cli.get_images("front_rgb", 0)
        ok = img is not None and getattr(img, "width", 0) > 0
        results.append(("相机取帧 front_rgb", ok,
                        "尺寸 %sx%s" % (getattr(img, "width", "?"),
                                        getattr(img, "height", "?")) if ok else "无图像"))
    except Exception as exc:  # noqa: BLE001
        results.append(("相机取帧 front_rgb", False, str(exc)))

    try:
        pts = cli.get_lidar_points_enu("lidar1")
        if pts is None or pts.shape[0] == 0:
            results.append(("激光雷达点云 lidar1", False, "无数据"))
        else:
            lo, hi = pts.min(axis=0), pts.max(axis=0)
            # 打印 ENU 包围盒：既能确认点云有效，也能核对 DataFrame 语义
            # （vehicle_inertial 时点云应与 /uav/odom 落在同一 world 系内）
            results.append(("激光雷达点云 lidar1", True,
                            "点数 %d, x∈[%.1f, %.1f] y∈[%.1f, %.1f] z∈[%.1f, %.1f]"
                            % (pts.shape[0], lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])))
    except Exception as exc:  # noqa: BLE001
        results.append(("激光雷达点云 lidar1", False, str(exc)))
    return results
