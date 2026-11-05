#!/usr/bin/env python3
"""ROS 2 仿真桥接节点：在 ROS 里把 MuJoCo 小车真实跑起来，和推理节点组成闭环。

发布：
  /scan    sensor_msgs/LaserScan   25 束前视射线距离（MuJoCo mj_ray 打出）
  /odom    nav_msgs/Odometry       车体位姿（给航向反馈）
订阅：
  /cmd_vel geometry_msgs/Twist      推理节点算出的线速度 / 角速度

每个控制周期推进 MuJoCo 物理 -> 用最近一次 cmd_vel 驱动小车 -> 发布 scan/odom
-> 记录真实轨迹。跑完全部场景后把轨迹画成图，退出。

只在装有 mujoco + ROS 2（如 Ubuntu Humble）的环境下运行，配合 ros_node.py，
让"神经网络端到端避障"成为可复现的 ROS 闭环。仿真与推理都按真实跑，产物不造假。
"""
import math
from pathlib import Path

import numpy as np

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from mujoco_car_nn_avoidance.config import Config, RESULTS_DIR
from mujoco_car_nn_avoidance.car_env import CarEnv
from mujoco_car_nn_avoidance.sensors import ray_scan

# 障碍布局取自 models/*.xml（body pos 与 geom size），与 _plot 轨迹图标注一致
SCENES = {
    "car_scene_basic.xml":  [((3.2, 0.0), 0.30)],
    "car_scene_slalom.xml": [((3.0, 0.0), 0.30), ((5.4, -0.85), 0.26)],
    "car_scene_dual.xml":   [((3.0, 0.7), 0.30), ((4.8, -0.7), 0.30)],
}


class SimBridge(Node):
    def __init__(self):
        super().__init__("mujoco_car_nn_sim_bridge")
        self.cfg = Config()
        # 参数来自 launch 加载的 params.yaml；缺省时回退到 Config 内置默认值
        self.declare_parameter("scenes", list(self.cfg.scenes))
        self.declare_parameter("goal_x", float(self.cfg.goal_x))
        self.declare_parameter("timeout_s", 30.0)
        self._scenes = list(self.get_parameter("scenes").value)
        self._goal_x = float(self.get_parameter("goal_x").value)
        self._timeout = float(self.get_parameter("timeout_s").value)
        # 场景 XML 随包安装到 share/.../models（见 setup.py data_files）
        share = Path(get_package_share_directory("mujoco_car_nn_avoidance"))
        self._xml_dir = share / "models"
        self._idx = 0
        self._load_scene()
        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._sim_t = 0.0
        self._trajs = {s: [] for s in self._scenes}
        self._timestep_dt = self.cfg.steps_per_control * self.cfg.timestep

        self._scan_pub = self.create_publisher(LaserScan, "scan", 10)
        self._odom_pub = self.create_publisher(Odometry, "odom", 10)
        self._vel_sub = self.create_subscription(Twist, "cmd_vel", self._on_vel, 10)
        self._timer = self.create_timer(self.cfg.dt, self._control_step)

        self.get_logger().info("sim bridge started, scene 1/{}".format(len(self._scenes)))

    def _load_scene(self):
        self.env = CarEnv(self.cfg, self._scenes[self._idx],
                          xml_path=self._xml_dir / self._scenes[self._idx])
        self.env.reset()
        self._sim_t = 0.0

    def _on_vel(self, msg):
        self._cmd_v = float(msg.linear.x)
        self._cmd_w = float(msg.angular.z)

    def _control_step(self):
        # 把 /cmd_vel 还原成 steer/throttle 并驱动小车（差速解算）
        steer = self._cmd_w / self.cfg.omega_max
        throttle = self._cmd_v / self.cfg.v_max
        wl, wr = self.env.desired_wheel_vel(steer, throttle)
        self.env.step_vel(wl, wr)
        self._sim_t += self._timestep_dt

        # 真实射线感知 -> 发布 /scan
        rays = ray_scan(self.env)
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "lidar"
        scan.angle_min = -math.radians(60.0)
        scan.angle_max = math.radians(60.0)
        scan.angle_increment = math.radians(5.0)
        scan.range_min = 0.0
        scan.range_max = float(self.cfg.lidar_range_m)
        scan.ranges = [float(r) for r in rays]
        self._scan_pub.publish(scan)

        # 位姿 -> 发布 /odom（航向反馈，yaw 只在水平面）
        pos = self.env.pos()
        yaw = self.env.yaw()
        odom = Odometry()
        odom.header.stamp = scan.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        self._odom_pub.publish(odom)

        # 记录真实轨迹
        self._trajs[self._scenes[self._idx]].append((float(pos[0]), float(pos[1])))

        if pos[0] > self._goal_x or self._sim_t > self._timeout:
            self.get_logger().info(
                "scene {} done: x={:.2f} t={:.1f}s".format(self._scenes[self._idx],
                                                           float(pos[0]), self._sim_t))
            self._idx += 1
            if self._idx >= len(self._scenes):
                self._finish()
            else:
                self._load_scene()
                self.get_logger().info("sim bridge: scene {}/{}".format(self._idx + 1, len(self._scenes)))

    def _finish(self):
        self.get_logger().info("all scenes done, plotting trajectory")
        try:
            self._plot()
        except Exception as e:
            self.get_logger().error("plot failed: {}".format(e))
        self.get_logger().info("shutdown")
        rclpy.shutdown()

    def _plot(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
        fig, axes = plt.subplots(1, len(self._scenes),
                                 figsize=(6 * len(self._scenes), 5))
        if len(self._scenes) == 1:
            axes = [axes]
        for ax, scene in zip(axes, self._scenes):
            xy = self._trajs[scene]
            xs = [p[0] for p in xy]
            ys = [p[1] for p in xy]
            ax.plot(xs, ys, color="#2060d0", lw=1.6,
                    label="real trajectory (ROS closed loop)")
            for i, ((ox, oy), hw) in enumerate(SCENES[scene]):
                ax.add_patch(Rectangle((ox - hw, oy - hw), 2 * hw, 2 * hw,
                                       facecolor="#cc3322", alpha=0.85,
                                       label="obstacle" if i == 0 else None))
            ax.axvline(self._goal_x, color="#666666", ls="--", lw=1)
            ax.set_title(scene.replace("car_scene_", "").replace(".xml", ""))
            ax.set_xlabel("x / m")
            ax.set_ylabel("y / m")
            ax.axis("equal")
            ax.grid(alpha=0.3)
            ax.legend(fontsize=8)
        fig.tight_layout()
        # 安装模式下结果目录可能只读，依次回退到可写目录
        out = None
        for directory in (RESULTS_DIR, Path.cwd()):
            try:
                directory.mkdir(parents=True, exist_ok=True)
                candidate = directory / "ros_trajectory.png"
                fig.savefig(candidate, dpi=110)
                out = candidate
                break
            except OSError:
                continue
        plt.close(fig)
        print("ROS trajectory ->", out)


def main(args=None):
    rclpy.init(args=args)
    node = SimBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
