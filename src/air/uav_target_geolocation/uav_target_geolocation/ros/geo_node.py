#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在线判定节点：订阅里程计与云台视线，发布地面目标估计。

与离线解算（``main.py``）的关系：同一个 ``core`` 里的交会与统计函数，
节点只做"接收 → 归一化 → 逐站累积 → 交会"。它是简化版——没有对目标
运动建模，站点切换靠操作员发一次 ``/target/geo/new_station``。

话题：
    订阅  /uav/odom               nav_msgs/Odometry      机体位姿
    订阅  /gimbal/los             geometry_msgs/Vector3Stamped  机体系视线
    订阅  /target/geo/new_station std_msgs/Empty         换一个观察站
    发布  /target/ground          geometry_msgs/PointStamped  地面估计
    发布  /target/geo_diag        std_msgs/String        JSON 诊断
"""
from __future__ import annotations

import json
from typing import List, Optional

import numpy as np

from ..core.fusion import angular_spread_deg, fused_from_directions, mean_direction
from ..core.geometry import horizontal_intersection_angle_deg, make_ray

DEFAULTS = {
    "los_bias_deg": (0.0, 0.0),
    "los_sigma_deg": 0.30,
    "min_stations_for_fusion": 2,
    "samples_per_station": 100,
    "odom_topic": "/uav/odom",
    "los_topic": "/gimbal/los",
    "ground_topic": "/target/ground",
    "diag_topic": "/target/geo_diag",
    "new_station_topic": "/target/geo/new_station",
    "ground_z": 0.0,
}


def quat_to_matrix(q) -> np.ndarray:
    """(w,x,y,z) → 3x3 旋转矩阵（与 uav_mujoco_sim.core.quadrotor 同式）。"""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def los_body_to_world(los_body, quat) -> np.ndarray:
    """机体系视线 → 世界系（不归一化则在此处归一化）。"""
    v = np.asarray(los_body, dtype=float).reshape(-1)
    if v.shape != (3,):
        raise ValueError(f"视线必须是 3 维，收到 {v.shape}")
    n = float(np.linalg.norm(v))
    if n < 1e-9 or not np.isfinite(n):
        raise ValueError("视线是零向量或含非有限值")
    q = np.asarray(quat, dtype=float).reshape(-1)
    qn = float(np.linalg.norm(q))
    if q.shape != (4,) or qn < 1e-9 or not np.isfinite(qn):
        raise ValueError("姿态四元数无效")
    R = quat_to_matrix(q / qn)
    return R @ (v / n)


def node_params(overrides: Optional[dict] = None) -> dict:
    """把参数合并到默认值上，并做基本校验（不依赖 rclpy，方便离线测）。"""
    p = dict(DEFAULTS)
    if overrides:
        unknown = set(overrides) - set(DEFAULTS)
        if unknown:
            raise KeyError(f"未知参数：{sorted(unknown)}")
        p.update(overrides)
    b = tuple(p["los_bias_deg"])
    if len(b) != 2 or any(x < -5.0 or x > 5.0 for x in b):
        raise ValueError("los_bias_deg 必须是两个 |值| ≤ 5 的分量")
    if p["samples_per_station"] < 1:
        raise ValueError("samples_per_station 至少为 1")
    if p["min_stations_for_fusion"] < 1:
        raise ValueError("min_stations_for_fusion 至少为 1")
    if p["ground_z"] > 0.0:
        raise ValueError("ground_z 应该 ≤ 0（ENU 地面在无人机下方）")
    return p


class GeoState:
    """不依赖 ROS 的节点状态机：可以脱离 rclpy 单测。"""

    def __init__(self, params: Optional[dict] = None):
        self.p = node_params(params)
        self.bias = (float(self.p["los_bias_deg"][0]),
                     float(self.p["los_bias_deg"][1]))
        self.pos: Optional[np.ndarray] = None
        self.quat: Optional[np.ndarray] = None
        self.buffer: List[np.ndarray] = []
        self.stations_pos: List[np.ndarray] = []
        self.stations_dir: List[np.ndarray] = []

    # ---------------------------------------------------------------- 输入
    def on_odom(self, pos, quat) -> None:
        p = np.asarray(pos, dtype=float).reshape(-1)
        q = np.asarray(quat, dtype=float).reshape(-1)
        if p.shape != (3,) or q.shape != (4,):
            raise ValueError("里程计位姿维度不对")
        if not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)):
            raise ValueError("里程计位姿含非有限值")
        n = float(np.linalg.norm(q))
        if n < 1e-9:
            raise ValueError("四元数为零")
        self.pos, self.quat = p, q / n

    def on_new_station(self) -> dict:
        """收尾当前站：平均视线、单站打点、累积交会。"""
        if not self.buffer:
            return {"ok": False, "reason": "当前站没有样本"}
        if self.pos is None or self.quat is None:
            return {"ok": False, "reason": "还没有收到里程计"}
        mean_dir = mean_direction(self.buffer)
        self.stations_pos.append(self.pos.copy())
        self.stations_dir.append(mean_dir)
        self.buffer = []

        n = len(self.stations_pos)
        if n >= int(self.p["min_stations_for_fusion"]):
            fused = fused_from_directions(self.stations_pos, self.stations_dir,
                                          ground_z=float(self.p["ground_z"]))
            if fused.ok:
                out = {"ok": True, "mode": "fused", "n_stations": n,
                       "x": float(fused.point[0]), "y": float(fused.point[1]),
                       "residual_m": float(fused.residual_m),
                       "condition": float(fused.condition)}
            else:
                out = {"ok": False, "mode": "fused", "n_stations": n,
                       "reason": fused.reason}
        else:
            ray = make_ray(self.stations_pos[0], self.stations_dir[0])
            hit = ray.ground_hit(float(self.p["ground_z"]))
            out = {"ok": True, "mode": "single", "n_stations": n,
                   "x": float(hit[0]), "y": float(hit[1]),
                   "reason": "站数不足，只有单站估计"}
        return out

    def on_los(self, los_body) -> dict:
        """收到一帧视线：累积；攒够一批就自动收尾当前站。"""
        if self.pos is None or self.quat is None:
            return {"ok": False, "reason": "还没有收到里程计"}
        d = los_body_to_world(los_body, self.quat)
        # 零位偏置：整架次不变，在线阶段无法标掉，只能如实记录
        self.buffer.append(d)
        if len(self.buffer) >= int(self.p["samples_per_station"]):
            return self.on_new_station()
        return {"ok": True, "mode": "accumulating",
                "n": len(self.buffer)}

    # ---------------------------------------------------------------- 诊断
    def diag(self) -> str:
        info = {
            "n_stations": len(self.stations_pos),
            "buffered": len(self.buffer),
            "los_sigma_deg": float(self.p["los_sigma_deg"]),
            "min_stations_for_fusion": int(self.p["min_stations_for_fusion"]),
        }
        if len(self.stations_dir) >= 2:
            rays = [make_ray(p, d) for p, d in
                    zip(self.stations_pos, self.stations_dir)]
            info["intersection_deg"] = round(
                float(horizontal_intersection_angle_deg(rays)), 2)
        if self.buffer:
            info["los_spread_deg"] = round(
                float(angular_spread_deg(self.buffer)), 3)
        return json.dumps(info, ensure_ascii=False)


def main(args=None) -> int:                     # noqa: C901
    import rclpy                                 # noqa: PLC0415
    from rclpy.node import Node                  # noqa: PLC0415
    from geometry_msgs.msg import PointStamped, Vector3Stamped  # noqa: PLC0415
    from nav_msgs.msg import Odometry            # noqa: PLC0415
    from std_msgs.msg import Empty, String       # noqa: PLC0415

    rclpy.init(args=args)
    node = Node("geo_node")
    node.declare_parameters("", [
        ("los_bias_deg", list(DEFAULTS["los_bias_deg"])),
        ("los_sigma_deg", DEFAULTS["los_sigma_deg"]),
        ("min_stations_for_fusion", DEFAULTS["min_stations_for_fusion"]),
        ("samples_per_station", DEFAULTS["samples_per_station"]),
        ("odom_topic", DEFAULTS["odom_topic"]),
        ("los_topic", DEFAULTS["los_topic"]),
        ("ground_topic", DEFAULTS["ground_topic"]),
        ("diag_topic", DEFAULTS["diag_topic"]),
        ("new_station_topic", DEFAULTS["new_station_topic"]),
        ("ground_z", DEFAULTS["ground_z"]),
    ])
    gp = lambda name: node.get_parameter(name).value  # noqa: E731
    state = GeoState({
        "los_bias_deg": tuple(gp("los_bias_deg")),
        "los_sigma_deg": float(gp("los_sigma_deg")),
        "min_stations_for_fusion": int(gp("min_stations_for_fusion")),
        "samples_per_station": int(gp("samples_per_station")),
        "ground_z": float(gp("ground_z")),
    })
    pub_ground = node.create_publisher(PointStamped, gp("ground_topic"), 10)
    pub_diag = node.create_publisher(String, gp("diag_topic"), 10)

    def _emit(result: dict) -> None:
        msg = String()
        msg.data = json.dumps(result, ensure_ascii=False)
        pub_diag.publish(msg)
        if result.get("ok") and result.get("mode") in ("single", "fused"):
            pt = PointStamped()
            pt.header.stamp = node.get_clock().now().to_msg()
            pt.header.frame_id = "map"
            pt.point.x = float(result["x"])
            pt.point.y = float(result["y"])
            pt.point.z = float(gp("ground_z"))
            pub_ground.publish(pt)

    def _on_odom(msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        state.on_odom((p.x, p.y, p.z), (q.w, q.x, q.y, q.z))

    def _on_los(msg: Vector3Stamped) -> None:
        _emit(state.on_los((msg.vector.x, msg.vector.y, msg.vector.z)))

    def _on_new_station(_msg: Empty) -> None:
        _emit(state.on_new_station())

    node.create_subscription(Odometry, gp("odom_topic"), _on_odom, 10)
    node.create_subscription(Vector3Stamped, gp("los_topic"), _on_los, 10)
    node.create_subscription(Empty, gp("new_station_topic"), _on_new_station, 10)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
