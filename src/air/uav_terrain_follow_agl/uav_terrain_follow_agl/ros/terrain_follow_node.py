#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地形跟随在线节点。

订阅
    ``/uav/odom``            nav_msgs/Odometry       位置 + 速度（ENU）
    ``/range/agl``           sensor_msgs/Range       雷达高度表（对地距离）
    ``/terrain/profile``     std_msgs/Float32MultiArray
        从当前位置起、沿航向等间隔（``profile_step_m``）的高程序列，共
        ``profile_len`` 个点——相当于机载 DEM 沿航迹切出的一条剖面。
        规划端只需把「前方地形有多高」发过来，节点不需要拿到整张高程库。

发布
    ``/uav/altitude_setpoint``  std_msgs/Float64   高度设定（米，ENU z）
    ``/uav/climb_rate_setpoint`` std_msgs/Float64  爬升率设定（米/秒）
    ``/terrain_follow/diag``    std_msgs/String    JSON 诊断

``FollowState`` 不 import rclpy，可以单独单测；``main()`` 里才建节点。
"""
from __future__ import annotations

import json
import math
from typing import Dict, Optional

import numpy as np

from ..core.control import ConstantMSL, DemPreview, ReactiveAGL, VerticalLimits

#: 参数默认值（与 config/uav_terrain_follow_agl.yaml 保持一致）
DEFAULT_PARAMS: Dict[str, object] = {
    "agl_set_m": 15.0,
    "strategy": "dem_preview",
    "climb_rate_max_mps": 3.0,
    "climb_accel_max_mps2": 2.0,
    "profile_step_m": 2.0,
    "profile_len": 76,
    "trim_ki": 0.02,
    "trim_limit_m": 10.0,
    "reactive_kp": 1.2,
    "reactive_ki": 0.25,
    "control_dt": 0.05,
    "odom_topic": "/uav/odom",
    "agl_topic": "/range/agl",
    "profile_topic": "/terrain/profile",
    "altitude_topic": "/uav/altitude_setpoint",
    "climb_topic": "/uav/climb_rate_setpoint",
    "diag_topic": "/terrain_follow/diag",
}

STRATEGY_NAMES = ("constant_msl", "reactive_agl", "dem_preview")


def node_params(overrides: Optional[Dict[str, object]] = None) -> Dict[str, object]:
    """默认参数 + 覆盖，并做一次范围校验。"""
    params = dict(DEFAULT_PARAMS)
    if overrides:
        params.update({k: v for k, v in overrides.items() if v is not None})
    if params["strategy"] not in STRATEGY_NAMES:
        raise ValueError(f"strategy 必须是 {STRATEGY_NAMES} 之一，"
                         f"收到 {params['strategy']!r}")
    if not 1.0 <= float(params["agl_set_m"]) <= 120.0:
        raise ValueError(f"agl_set_m 应在 1~120 m，收到 {params['agl_set_m']!r}")
    if float(params["climb_rate_max_mps"]) <= 0.0:
        raise ValueError("climb_rate_max_mps 必须为正")
    if int(params["profile_len"]) < 2:
        raise ValueError(f"profile_len 至少 2，收到 {params['profile_len']!r}")
    if float(params["profile_step_m"]) <= 0.0:
        raise ValueError("profile_step_m 必须为正")
    if float(params["control_dt"]) <= 0.0:
        raise ValueError("control_dt 必须为正")
    return params


class FollowState:
    """无 ROS 依赖的状态机：喂 odom / 高度表 / 前视剖面，吐高度与爬升率设定。"""

    def __init__(self, params: Optional[Dict[str, object]] = None):
        self.params = node_params(params)
        limits = VerticalLimits(
            climb_rate_mps=float(self.params["climb_rate_max_mps"]),
            climb_accel_mps2=float(self.params["climb_accel_max_mps2"]))
        self._profile: Optional[np.ndarray] = None
        name = str(self.params["strategy"])
        if name == "dem_preview":
            length = int(self.params["profile_len"])
            step = float(self.params["profile_step_m"])
            self.strategy = DemPreview(
                limits, horizon_m=step * (length - 1), step_m=step,
                ki=float(self.params["trim_ki"]),
                trim_limit=float(self.params["trim_limit_m"]),
                profile_hook=self._heights)
        elif name == "reactive_agl":
            self.strategy = ReactiveAGL(
                limits, kp=float(self.params["reactive_kp"]),
                ki=float(self.params["reactive_ki"]))
        else:
            self.strategy = ConstantMSL()

        self.x = 0.0
        self.z = float("nan")
        self.vx = 0.0
        self.agl: Optional[float] = None
        self.agl_count = 0
        self.profile_count = 0
        self.ready = False
        self.z_cmd = 0.0
        self.vz_cmd = 0.0
        self._inited = False

    # ---------------------------------------------------------------- 输入
    def _heights(self):
        if self._profile is None:
            raise ValueError("还没有收到前视剖面")
        return self._profile

    def on_odom(self, x: float, z: float, vx: float) -> None:
        for name, val in (("x", x), ("z", z), ("vx", vx)):
            if not math.isfinite(float(val)):
                raise ValueError(f"odom 的 {name} 不是有限数：{val!r}")
        self.x, self.z, self.vx = float(x), float(z), float(vx)

    def on_agl(self, agl_m: float) -> None:
        if not math.isfinite(float(agl_m)) or float(agl_m) < 0.0:
            raise ValueError(f"高度表读数应为非负有限数，收到 {agl_m!r}")
        self.agl = float(agl_m)
        self.agl_count += 1

    def on_profile(self, heights) -> None:
        arr = np.asarray(list(heights), dtype=float)
        if arr.ndim != 1 or arr.size < 2:
            raise ValueError(f"前视剖面应为一维且至少 2 点，收到形状 {arr.shape}")
        if not np.all(np.isfinite(arr)):
            raise ValueError("前视剖面含非有限值")
        self._profile = arr
        self.profile_count += 1

    # ---------------------------------------------------------------- 计算
    def _init_once(self) -> None:
        if self._inited:
            return
        if math.isnan(self.z):
            raise ValueError("还没收到 odom，无法初始化高度设定")
        self.strategy.reset(None, None, self.x, self.z,
                            float(self.params["agl_set_m"]))
        self.z_cmd = float(self.z)
        self.vz_cmd = 0.0
        self._inited = True
        self.ready = True

    def step(self) -> tuple:
        """推进一个控制周期，返回 (z_cmd, vz_cmd)。"""
        self._init_once()
        dt = float(self.params["control_dt"])
        if isinstance(self.strategy, DemPreview) and self._profile is None:
            # 没有前视剖面时退化为保持当前高度，并在诊断里暴露 profile_count=0
            return self.z_cmd, 0.0
        self.z_cmd, self.vz_cmd = self.strategy.step(
            0.0, dt, self.x, self.z, self.vx, self.agl)
        return self.z_cmd, self.vz_cmd

    def diag(self) -> Dict[str, object]:
        out: Dict[str, object] = {
            "strategy": self.strategy.name,
            "agl_set_m": float(self.params["agl_set_m"]),
            "x_m": round(self.x, 3),
            "z_m": None if math.isnan(self.z) else round(self.z, 3),
            "vx_mps": round(self.vx, 3),
            "agl_meas_m": None if self.agl is None else round(self.agl, 3),
            "z_cmd_m": round(float(self.z_cmd), 3),
            "vz_cmd_mps": round(float(self.vz_cmd), 3),
            "agl_samples": self.agl_count,
            "profile_updates": self.profile_count,
            "ready": self.ready,
        }
        if isinstance(self.strategy, DemPreview):
            out["trim_m"] = round(self.strategy.trim, 3)
            out["z_ref_m"] = round(self.strategy.reference, 3)
        if isinstance(self.strategy, ReactiveAGL):
            out["error_m"] = round(self.strategy.error, 3)
        return out


def main(argv=None):  # pragma: no cover - 需要 rclpy 与 ROS 2 环境
    import sys

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Range
    from std_msgs.msg import Float32MultiArray, Float64, String

    rclpy.init(args=argv)
    node = Node("terrain_follow_node")

    overrides = {}
    for key, default in DEFAULT_PARAMS.items():
        overrides[key] = node.declare_parameter(key, default).value
    params = node_params(overrides)
    state = FollowState(params)

    pub_alt = node.create_publisher(Float64, str(params["altitude_topic"]), 10)
    pub_climb = node.create_publisher(Float64, str(params["climb_topic"]), 10)
    pub_diag = node.create_publisher(String, str(params["diag_topic"]), 10)

    def on_odom(msg: Odometry) -> None:
        state.on_odom(msg.pose.pose.position.x, msg.pose.pose.position.z,
                      msg.twist.twist.linear.x)

    def on_agl(msg: Range) -> None:
        state.on_agl(float(msg.range))

    def on_profile(msg: Float32MultiArray) -> None:
        state.on_profile(msg.data)

    node.create_subscription(Odometry, str(params["odom_topic"]), on_odom, 20)
    node.create_subscription(Range, str(params["agl_topic"]), on_agl, 20)
    node.create_subscription(Float32MultiArray, str(params["profile_topic"]),
                             on_profile, 5)

    def tick() -> None:
        try:
            z_cmd, vz_cmd = state.step()
        except ValueError as exc:            # 还没拿到 odom
            node.get_logger().warn(f"跳过一拍：{exc}", throttle_duration_sec=2.0)
            return
        pub_alt.publish(Float64(data=float(z_cmd)))
        pub_climb.publish(Float64(data=float(vz_cmd)))
        pub_diag.publish(String(data=json.dumps(state.diag(), ensure_ascii=False)))
        node.get_logger().debug(
            f"z_cmd={z_cmd:.2f} vz_cmd={vz_cmd:.2f} agl={state.agl}")

    node.create_timer(float(params["control_dt"]), tick)
    node.get_logger().info(
        f"地形跟随节点已启动：strategy={params['strategy']} "
        f"AGL={params['agl_set_m']} m 爬升率上限={params['climb_rate_max_mps']} m/s")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    import sys

    sys.exit(main())
