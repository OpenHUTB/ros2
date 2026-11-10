"""
sim_node.py —— ROS 2 仿真节点：加载 MJCF，按物理步长推进，发布 Odom/图像/状态。

为什么用 timer 分发布频：
  MuJoCo 物理要 ~500Hz 才稳（1 个仿真 tick 内步进多个 mj_step），
  而 odom 50Hz、相机 10Hz 就够。分三个 timer 避免渲染拖慢物理。
"""
import os

import numpy as np

# 必须在 import mujoco 前设置 OSMesa 后端
os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

from . import common


class SimNode(Node):
    """ROV MuJoCo 仿真节点。"""

    def __init__(self):
        super().__init__("rov_sim_node")
        self._declare_params()

        # 加载 MuJoCo
        import mujoco
        self._mujoco = mujoco
        params = self._read_params()
        self._params = params
        self.model = mujoco.MjModel.from_xml_path(common.find_model_path(params["model_path"]))
        self.data = mujoco.MjData(self.model)
        self.dyn = common.WaterDynamics(params)
        self.rov_id = self.model.body("rov").id
        # 离屏渲染器（OSMesa）
        self.renderer = mujoco.Renderer(self.model,
                                        height=params["image_height"],
                                        width=params["image_width"])

        # 指令订阅
        self.create_subscription(Twist, params["cmd_topic"], self._on_cmd, 10)
        # 发布
        self.pub_odom = self.create_publisher(Odometry, params["odom_topic"], 10)
        # 相机话题：BEST_EFFORT，避免网络/时序导致图像丢包被认为是错误
        qos_best = QoSProfile(depth=2)
        qos_best.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.pub_image = self.create_publisher(Image, params["image_topic"], qos_best)
        self.pub_status = self.create_publisher(String, params["status_topic"], 10)

        self._twist = np.zeros(6)
        self._t = 0.0
        # 物理 timer：频率 = sim_rate
        dt = 1.0 / params["sim_rate"]
        self._n_step_per_tick = max(1, int(round(0.001 / dt)))  # 1ms ROS tick 内步进
        self.create_timer(0.001, self._tick)
        self.create_timer(1.0 / params["odom_rate"], self._pub_odom)
        self.create_timer(1.0 / params["image_rate"], self._pub_image)
        self.create_timer(2.0, self._pub_status)
        self.get_logger().info("rov_sim_node 已启动，等待 /rov_teleop/cmd_vel ...")

    def _declare_params(self):
        d = self.declare_parameter
        d("sim_rate", 500.0)
        d("odom_rate", 50.0)
        d("image_rate", 10.0)
        d("image_width", 160)
        d("image_height", 120)
        d("buoyancy_z", 10.0)
        d("damping_lin", 2.5)
        d("damping_ang", 1.5)
        d("stab_roll", 4.0)
        d("stab_pitch", 4.0)
        d("current_amp_x", 0.15)
        d("current_amp_y", 0.10)
        d("current_w_x", 0.30)
        d("current_w_y", 0.23)
        d("max_force_x", 5.0)
        d("max_force_y", 5.0)
        d("max_torque_yaw", 1.0)
        d("model_path", "")
        d("odom_topic", "/rov_teleop/odom")
        d("image_topic", "/rov_teleop/image_raw")
        d("status_topic", "/rov_teleop/status")
        d("cmd_topic", "/rov_teleop/cmd_vel")

    def _read_params(self):
        g = self.get_parameter
        return dict(
            sim_rate=g("sim_rate").value,
            odom_rate=g("odom_rate").value,
            image_rate=g("image_rate").value,
            image_width=g("image_width").value,
            image_height=g("image_height").value,
            buoyancy_z=g("buoyancy_z").value,
            damping_lin=g("damping_lin").value,
            damping_ang=g("damping_ang").value,
            stab_roll=g("stab_roll").value,
            stab_pitch=g("stab_pitch").value,
            current_amp_x=g("current_amp_x").value,
            current_amp_y=g("current_amp_y").value,
            current_w_x=g("current_w_x").value,
            current_w_y=g("current_w_y").value,
            max_force_x=g("max_force_x").value,
            max_force_y=g("max_force_y").value,
            max_torque_yaw=g("max_torque_yaw").value,
            model_path=g("model_path").value,
            odom_topic=g("odom_topic").value,
            image_topic=g("image_topic").value,
            status_topic=g("status_topic").value,
            cmd_topic=g("cmd_topic").value,
        )

    def _on_cmd(self, msg: Twist):
        self._twist = np.array([
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z,
        ], dtype=float)

    def _tick(self):
        # 每个 ROS tick 推进一步物理（sim_rate 决定步长内子步数）
        for _ in range(self._n_step_per_tick):
            self.dyn.apply(self.data, self._t, self.rov_id, self._twist)
            self._mujoco.mj_step(self.model, self.data)
            self._t += self.model.opt.timestep

    def _pub_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "rov/base_link"
        msg.pose.pose.position.x = float(self.data.qpos[0])
        msg.pose.pose.position.y = float(self.data.qpos[1])
        msg.pose.pose.position.z = float(self.data.qpos[2])
        msg.pose.pose.orientation.w = float(self.data.qpos[3])
        msg.pose.pose.orientation.x = float(self.data.qpos[4])
        msg.pose.pose.orientation.y = float(self.data.qpos[5])
        msg.pose.pose.orientation.z = float(self.data.qpos[6])
        msg.twist.twist.linear.x = float(self.data.qvel[0])
        msg.twist.twist.linear.y = float(self.data.qvel[1])
        msg.twist.twist.linear.z = float(self.data.qvel[2])
        msg.twist.twist.angular.z = float(self.data.qvel[5])
        self.pub_odom.publish(msg)

    def _pub_image(self):
        try:
            self.renderer.update_scene(self.data, camera="forward_cam")
            img = self.renderer.render()  # (H,W,3) uint8
            h, w, _ = img.shape
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "forward_cam"
            msg.height = h
            msg.width = w
            msg.encoding = "rgb8"
            msg.step = w * 3
            msg.data = img.tobytes()
            self.pub_image.publish(msg)
        except Exception as e:  # 渲染失败不能拖垮物理
            self.get_logger().warn(f"相机渲染失败（忽略）: {e}")

    def _pub_status(self):
        msg = String()
        msg.data = (
            f"t={self._t:.1f}s pos=({self.data.qpos[0]:.2f},"
            f"{self.data.qpos[1]:.2f},{self.data.qpos[2]:.2f})"
        )
        self.pub_status.publish(msg)


def main():
    rclpy.init()
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
