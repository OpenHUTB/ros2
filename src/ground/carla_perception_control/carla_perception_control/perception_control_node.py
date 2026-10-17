#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 传感器感知 + 给定轨迹跟踪 —— ROS 2 Humble 节点。

职责：
  * 连接 CARLA、生成自车、挂载 RGB / 深度 / 雷达三路传感器
  * 驱动同步步进（world.tick）
  * 用**感知神经网络**由传感器特征输出障碍类别
  * 用**控制神经网络**由 (航向差, 距离) 输出转向，沿给定轨迹行驶
  * 广播感知结果与车辆状态话题，便于 RViz / 其它节点订阅

订阅：  /carla/ego_vehicle/waypoints   (std_msgs/Float32MultiArray, [x1,y1,x2,y2,...])
发布：  /carla/ego_vehicle/rgb_front/image  (sensor_msgs/Image, rgb8)
        /carla/ego_vehicle/odometry         (nav_msgs/Odometry)
        /carla/ego_vehicle/perception       (std_msgs/Int32, 0=无目标 1=偏左 2=偏右)
        /carla/ego_vehicle/speed            (std_msgs/Float32)

启动：  ros2 launch carla_perception_control main.launch.py host:=<宿主机IP>
"""

import math
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray, Int32

# 允许以脚本方式直接运行（launch 传入 main.py 时路径不同）
_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from carla_perception_control import carla_common as cc            # noqa: E402
from carla_perception_control.nn_models import MLPClassifier, MLPPolicy  # noqa: E402

DT = 0.05
DEFAULT_WAYPOINTS = [36.0, -5.0, 40.0, -8.0, 40.0, 12.0, 25.0, 20.0]
CLS_NAME = {0: "无目标", 1: "偏左", 2: "偏右"}


class PerceptionControlNode(Node):
    """感知（NN）+ 轨迹跟踪控制（NN）节点。"""

    def __init__(self):
        super().__init__("carla_perception_control_node")

        # ---------------- 参数 ----------------
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 2000)
        self.declare_parameter("town", "Town05")
        self.declare_parameter("ego_blueprint", "vehicle.tesla.model3")
        self.declare_parameter("model_path", "models/nn_percept.json")
        self.declare_parameter("throttle_max", 0.6)
        self.declare_parameter("use_nn_control", True)

        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)
        self.town = self.get_parameter("town").value
        self.throttle_max = float(self.get_parameter("throttle_max").value)
        self.use_nn_control = bool(self.get_parameter("use_nn_control").value)
        model_path = self.get_parameter("model_path").value

        # ---------------- 神经网络 ----------------
        self.sens, self.ctrl = self._load_or_init_models(model_path)

        # ---------------- 话题 ----------------
        self.pub_img = self.create_publisher(Image, "/carla/ego_vehicle/rgb_front/image", 10)
        self.pub_odom = self.create_publisher(Odometry, "/carla/ego_vehicle/odometry", 10)
        self.pub_cls = self.create_publisher(Int32, "/carla/ego_vehicle/perception", 10)
        self.pub_speed = self.create_publisher(Float32, "/carla/ego_vehicle/speed", 10)
        self.create_subscription(Float32MultiArray, "/carla/ego_vehicle/waypoints",
                                 self._on_waypoints, 10)

        # ---------------- CARLA ----------------
        self.waypoints = [(DEFAULT_WAYPOINTS[i], DEFAULT_WAYPOINTS[i + 1])
                          for i in range(0, len(DEFAULT_WAYPOINTS), 2)]
        self.client = None
        self.world = None
        self.vehicle = None
        self.sensors = []
        self.holder = {"rgb": None, "depth": None, "lidar": None}
        self.frames_seen = 0
        self._connect()

        self.create_timer(DT, self._on_timer)
        self.get_logger().info(
            f"感知+控制 NN 节点已启动：host={self.host}:{self.port} town={self.town} "
            f"路点={len(self.waypoints)}")

    # ------------------------------------------------------------------ 模型
    def _load_or_init_models(self, model_path):
        """加载已训练模型；若不存在则现场训练（保证节点总能运行）。"""
        import json
        if os.path.isfile(model_path):
            try:
                with open(model_path, encoding="utf-8") as f:
                    obj = json.load(f)
                sens = MLPClassifier(obj["sens"]["layers"])
                sens.W = [np.asarray(w, dtype=np.float32) for w in obj["sens"]["W"]]
                sens.b = [np.asarray(t, dtype=np.float32) for t in obj["sens"]["b"]]
                ctrl = MLPPolicy(obj["ctrl"]["layers"])
                ctrl.W = [np.asarray(w, dtype=np.float32) for w in obj["ctrl"]["W"]]
                ctrl.b = [np.asarray(t, dtype=np.float32) for t in obj["ctrl"]["b"]]
                self.get_logger().info(f"已加载神经网络模型：{model_path}")
                return sens, ctrl
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"模型加载失败({exc})，改为现场训练")
        self.get_logger().info("未找到模型文件，现场训练感知/控制神经网络（纯 numpy）...")
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "pc_main", os.path.join(_HERE, "main.py"))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        feat_X, feat_Y, ctrl_X, ctrl_Y = mod.synth_dataset(400)
        sens, ctrl, _h = mod.train(feat_X, feat_Y, ctrl_X, ctrl_Y, epochs=150)
        return sens, ctrl

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
        self.sensors.append(cc.make_rgb_camera(
            self.world, self.vehicle, lambda im: self._on_rgb(im), tick=True))
        self.sensors.append(cc.make_depth_camera(
            self.world, self.vehicle, lambda d: self.holder.__setitem__("depth", d), tick=True))
        self.sensors.append(cc.make_lidar(
            self.world, self.vehicle, lambda pc: self._on_lidar(pc), tick=True))
        self.get_logger().info(f"自车已生成于 {tf.location}，三路传感器就绪（RGB/深度/雷达）")

    def _on_rgb(self, img):
        self.holder["rgb"] = img
        self.frames_seen += 1

    def _on_lidar(self, pc):
        self.holder["lidar"] = np.frombuffer(pc.raw_data, dtype=np.float32).reshape(-1, 4)

    def _on_waypoints(self, msg):
        vals = list(msg.data)
        if len(vals) >= 2 and len(vals) % 2 == 0:
            self.waypoints = [(vals[i], vals[i + 1]) for i in range(0, len(vals), 2)]
            self.get_logger().info(f"收到新路点序列，共 {len(self.waypoints)} 个")

    # ------------------------------------------------------------------ 主循环
    def _on_timer(self):
        if self.world is None or self.vehicle is None:
            return
        from carla_perception_control.main import extract_features, nn_control, pure_pursuit

        pos = cc.get_location(self.vehicle)
        yaw = cc.get_yaw(self.vehicle)

        # ---- 感知：三路传感器 → 感知神经网络 ----
        feat, raw = extract_features(self.holder["rgb"], self.holder["depth"], self.holder["lidar"])
        cls = int(self.sens.predict(feat[None, :])[0])
        self.pub_cls.publish(Int32(data=cls))

        # ---- 控制：控制神经网络（或纯跟踪对比）----
        if self.use_nn_control:
            throttle, steer = nn_control(pos, yaw, self.waypoints, self.ctrl, self.throttle_max)
        else:
            throttle, steer = pure_pursuit(pos, yaw, self.waypoints)
        cc.apply_control(self.vehicle, throttle=throttle, steer=steer)

        # ---- 广播画面与状态 ----
        if self.holder["rgb"] is not None:
            self._publish_image(self.holder["rgb"])
        self._publish_odom(pos, yaw)
        self.pub_speed.publish(Float32(data=float(cc.get_speed(self.vehicle))))

        self.world.tick()

        if self.frames_seen and self.frames_seen % 100 == 0:
            off, d_, fn, fd = raw
            self.get_logger().info(
                f"NN感知={CLS_NAME[cls]} steer={steer:+.2f} 速度={cc.get_speed(self.vehicle):.2f}m/s "
                f"rgb_off={off:+.2f} depth={d_:.0f} 雷达前方({fn},{fd})")

    def _publish_image(self, rgb):
        msg = Image()
        msg.height, msg.width = rgb.shape[:2]
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = rgb.shape[1] * 3
        msg.data = rgb.tobytes()
        self.pub_img.publish(msg)

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
        # 恢复异步模式：connect() 打开了 synchronous_mode，此时服务端只在
        # 客户端 world.tick() 时推进。节点退出后没人再 tick，CARLA 窗口会
        # 看起来「卡住不动」，后续再连也像死机。必须在这里交还给服务端自动运行。
        if self.world is not None:
            if cc.restore_async(self.world):
                self.get_logger().info("已销毁传感器与自车，世界已恢复异步模式。")
            else:
                self.get_logger().warn(
                    "已销毁传感器与自车；世界仍为同步模式，如需恢复可重启 CARLA 服务端。")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionControlNode()
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
