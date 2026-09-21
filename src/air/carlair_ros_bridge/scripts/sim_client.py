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
    def get_images(self, camera: str = "front_rgb", image_type: int = 0):
        """取一帧图像；image_type: 0=RGB(Scene) 1=DepthPlanar 5=Segmentation."""
        req = [self._airsim.ImageRequest(camera, image_type, False, False)]
        resp = self.client.simGetImages(req, vehicle_name=self.vehicle_name)
        if not resp:
            return None
        return resp[0]

    def get_lidar_points_enu(self, lidar_name: str = "lidar1") -> Optional[np.ndarray]:
        """取一帧激光雷达点云并换算到 ENU（N x 3），无数据时返回 None."""
        data = self.client.getLidarData(lidar_name=lidar_name,
                                        vehicle_name=self.vehicle_name)
        pts = np.asarray(data.point_cloud, dtype=np.float64)
        if pts.size == 0:
            return None
        pts = pts.reshape(-1, 3)
        return np.array([ned_to_enu(p) for p in pts])


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
        results.append(("激光雷达点云 lidar1", pts is not None,
                        "点数 %d" % (0 if pts is None else pts.shape[0])))
    except Exception as exc:  # noqa: BLE001
        results.append(("激光雷达点云 lidar1", False, str(exc)))
    return results
