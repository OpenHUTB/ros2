#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 建图 + 导航 —— ROS 2 Humble 节点（神经网络规划）。

职责：
  * 连接 CARLA、生成自车、挂载激光雷达
  * **建图**：边行驶边把雷达命中点与射线沿途投影到 2D 占用栅格（对数几率贝叶斯更新）
  * **导航**：用**规划神经网络**由（目标方位 + 障碍左右分布 + 最近距离）输出 (油门, 转向)
  * 广播栅格地图、轨迹、状态话题

发布：  /carla/ego_vehicle/occupancy_grid   (nav_msgs/OccupancyGrid)
        /carla/ego_vehicle/path             (nav_msgs/Path)
        /carla/ego_vehicle/odometry         (nav_msgs/Odometry)
        /carla/ego_vehicle/planned_cmd      (std_msgs/Float32MultiArray, [throttle, steer])
订阅：  /carla/ego_vehicle/goal             (std_msgs/Float32MultiArray, [x, y])

启动：  ros2 launch carla_mapping_navigation main.launch.py host:=<宿主机IP> goal:="20,8"
"""

import math
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32MultiArray

_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from carla_mapping_navigation import carla_common as cc          # noqa: E402
from carla_mapping_navigation.nn_models import MLPPolicy         # noqa: E402

DT = 0.05
GRID_M = 0.5
GRID_N = 200


class MappingNavigationNode(Node):
    """占用栅格建图 + 神经网络规划导航节点。"""

    def __init__(self):
        super().__init__("carla_mapping_navigation_node")

        # ---------------- 参数 ----------------
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 2000)
        self.declare_parameter("town", "Town05")
        self.declare_parameter("model_path", "models/nn_plan.json")
        self.declare_parameter("goal_x", 20.0)
        self.declare_parameter("goal_y", 8.0)
        self.declare_parameter("goal_tol", 1.5)
        self.declare_parameter("map_publish_period", 20)   # 每 N 个 tick 发一次地图

        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)
        self.town = self.get_parameter("town").value
        model_path = self.get_parameter("model_path").value
        self.goal = (float(self.get_parameter("goal_x").value),
                     float(self.get_parameter("goal_y").value))
        self.goal_tol = float(self.get_parameter("goal_tol").value)
        self.map_period = int(self.get_parameter("map_publish_period").value)

        # ---------------- 规划神经网络 ----------------
        self.net = self._load_or_train(model_path)

        # ---------------- 占用栅格（导入主模块的类，保持逻辑一致） ----------------
        from carla_mapping_navigation.main import OccupancyGrid, build_map_from_scan, obs_features
        self._OccGrid = OccupancyGrid
        self._build_map = build_map_from_scan
        self._obs_features = obs_features
        self.grid = None

        # ---------------- 话题 ----------------
        # 栅格地图体量较大，用 transient_local 让后加入的订阅者也能拿到
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.pub_grid = self.create_publisher(OccupancyGrid, "/carla/ego_vehicle/occupancy_grid", map_qos)
        self.pub_path = self.create_publisher(Path, "/carla/ego_vehicle/path", 10)
        self.pub_odom = self.create_publisher(Odometry, "/carla/ego_vehicle/odometry", 10)
        self.pub_cmd = self.create_publisher(Float32MultiArray, "/carla/ego_vehicle/planned_cmd", 10)
        self.create_subscription(Float32MultiArray, "/carla/ego_vehicle/goal", self._on_goal, 10)

        # ---------------- CARLA ----------------
        self.client = None
        self.world = None
        self.vehicle = None
        self.sensors = []
        self.pc = None
        self.path = []
        self.tick_count = 0
        self._connect()

        self.create_timer(DT, self._on_timer)
        self.get_logger().info(
            f"建图+导航 NN 节点已启动：host={self.host}:{self.port} town={self.town} "
            f"目标=({self.goal[0]},{self.goal[1]})")

    # ------------------------------------------------------------------ 模型
    def _load_or_train(self, model_path):
        if os.path.isfile(model_path):
            try:
                net = MLPPolicy.load(model_path)
                self.get_logger().info(f"已加载规划神经网络：{model_path}")
                return net
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"模型加载失败({exc})，改为现场训练")
        self.get_logger().info("未找到规划模型，现场训练（纯 numpy，约数秒）...")
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "mn_main", os.path.join(_HERE, "main.py"))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        net, _hist = mod.train_planning(epochs=150, out=model_path, verbose=0)
        return net

    # ------------------------------------------------------------------ 连接
    def _connect(self):
        try:
            self.client, self.world = cc.connect(self.host, self.port, self.town)
            self.vehicle, tf = cc.spawn_vehicle(self.world)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"连接 CARLA 失败：{exc}")
            self.get_logger().error(
                "请先启动 CARLA 服务端；启动方式与 IP/端口查看见 "
                "https://openhutb.github.io/ros2/set_up_and_connect_to_carla/")
            return
        self.grid = self._OccGrid(center=(tf.location.x, tf.location.y))
        self.sensors.append(cc.make_lidar(
            self.world, self.vehicle, self._on_lidar, tick=True))
        self.get_logger().info(
            f"自车已生成于 ({tf.location.x:.1f},{tf.location.y:.1f})，激光雷达就绪；"
            f"栅格 {GRID_N}×{GRID_N} @ {GRID_M} m/格")

    def _on_lidar(self, pc):
        self.pc = np.frombuffer(pc.raw_data, dtype=np.float32).reshape(-1, 4)

    def _on_goal(self, msg):
        if len(msg.data) >= 2:
            self.goal = (float(msg.data[0]), float(msg.data[1]))
            self.get_logger().info(f"收到新导航目标：({self.goal[0]}, {self.goal[1]})")

    # ------------------------------------------------------------------ 主循环
    def _on_timer(self):
        if self.world is None or self.vehicle is None:
            return

        pos = cc.get_location(self.vehicle)
        yaw = cc.get_yaw(self.vehicle)

        # ---- 建图：本帧雷达 → 占用栅格 ----
        if self.pc is not None:
            self._build_map(self.grid, self.pc, pos[0], pos[1], yaw)
            self.path.append(Point(x=float(pos[0]), y=float(pos[1]), z=0.0))

        # ---- 到达判定 ----
        dx, dy = self.goal[0] - pos[0], self.goal[1] - pos[1]
        d = math.hypot(dx, dy)
        if d < self.goal_tol:
            cc.apply_control(self.vehicle, throttle=0.0, steer=0.0, brake=0.8)
            self.world.tick()
            self.get_logger().info(f"已抵达目标 ({self.goal[0]},{self.goal[1]})，停车。")
            return

        # ---- 规划 NN ----
        goal_ang = math.atan2(dy, dx)
        diff = (goal_ang - yaw + math.pi) % (2 * math.pi) - math.pi
        state, raw = self._obs_features(self.pc, diff)
        out = self.net.predict(state[None, :])[0]
        throttle = float(np.clip(out[0], 0.0, 1.0))
        steer = float(np.clip(out[1], -1.0, 1.0))
        cc.apply_control(self.vehicle, throttle=throttle, steer=steer)
        self.pub_cmd.publish(Float32MultiArray(data=[throttle, steer]))

        self.world.tick()
        self.tick_count += 1

        # ---- 广播 ----
        self._publish_odom(pos, yaw)
        if self.tick_count % self.map_period == 0:
            self._publish_grid()
            self._publish_path()
        if self.tick_count % 100 == 0:
            _g, lc, rc, nearest = raw
            self.get_logger().info(
                f"距目标={d:.1f}m NN(油门={throttle:.2f},转向={steer:+.2f}) "
                f"障碍(L{lc}/R{rc},{nearest:.1f}m) 占据格={self.grid.occupied_count}")

    # ------------------------------------------------------------------ 广播
    def _publish_grid(self):
        """把占用栅格发布为 nav_msgs/OccupancyGrid（RViz 可直接可视化）。"""
        if self.grid is None:
            return
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = float(self.grid.m)
        msg.info.width = int(self.grid.n)
        msg.info.height = int(self.grid.n)
        # 左下角世界坐标（栅格以中心定义，行方向 y 增大）
        msg.info.origin.position.x = float(self.grid.center[0] - self.grid.n * self.grid.m / 2.0)
        msg.info.origin.position.y = float(self.grid.center[1] - self.grid.n * self.grid.m / 2.0)
        msg.info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        # 概率 → OccupancyGrid 的 0..100（-1 表示未知）
        p = self.grid.probability()
        data = np.where((p > 0.55) | (p < 0.45), (p * 100).astype(np.int8), np.int8(-1))
        msg.data = data.ravel().tolist()
        self.pub_grid.publish(msg)

    def _publish_path(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        for pt in self.path[-2000:]:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position = pt
            ps.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def _publish_odom(self, pos, yaw):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "ego_vehicle"
        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        self.pub_odom.publish(msg)

    def destroy_node(self):
        for s in self.sensors:
            try:
                s.stop()
                s.destroy()
            except Exception:  # noqa: BLE001
                pass
        if self.vehicle is not None:
            try:
                self.vehicle.destroy()
            except Exception:  # noqa: BLE001
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MappingNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
