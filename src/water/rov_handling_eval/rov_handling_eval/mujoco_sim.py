# -*- coding: utf-8 -*-
"""MuJoCo 物理内核节点：浮力 / 配重 / 各向异性二次阻力 / 三层洋流。

与「水下机器人物理仿真与 6-DOF 键盘运动控制」模块的区别：那个模块回答
"能不能动"，这个模块回答"动得准不准、顶不顶得住流"，所以这里多做了三件事：

1. 把浮心与质心分开施加（产生恢复力矩），而不是把浮力/重力相减成一个力；
2. 洋流真值单独发布到 ``/rov/current``，供评测节点使用，控制器看不到它；
3. 支持 ``/rov/depth_hold`` 开关，用于"有人帮 vs 没人帮"的抗流对比。

坐标系
------
* 世界系 z 向上（MuJoCo 默认，与 ROS ENU 一致），深度 = −z；
* 机体系 FLU（x 前、y 左、z 上）；
* MuJoCo 的 ``xfrc_applied`` 按世界系写入，机体系的力/力矩先乘 R 再写。
"""

import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

from .current_model import CurrentField, CurrentParams
from .dynamics import (VehicleParams, buoyancy_wrench, current_authority,
                       drag_force, drag_torque, velocity_control)
from .model import MJCF, quat_to_matrix


class MujocoSim(Node):
    def __init__(self):
        super().__init__("mujoco_sim_node")

        self.declare_parameter("sim_hz", 500.0)
        self.declare_parameter("pub_hz", 50.0)
        self.declare_parameter("start_depth", 8.0)
        self.declare_parameter("mass", 11.0)
        self.declare_parameter("displacement", 0.0110)
        self.declare_parameter("ballast", 0.0)
        self.declare_parameter("rho", 1025.0)
        self.declare_parameter("drag_linear", [13.20, 35.69, 63.33])
        self.declare_parameter("drag_angular", [0.60, 0.60, 0.40])
        self.declare_parameter("cb_offset", [0.0, 0.0, 0.03])
        self.declare_parameter("max_force", [10.0, 10.0, 10.0])
        self.declare_parameter("max_torque", [2.0, 2.0, 2.0])
        self.declare_parameter("kp_linear", 60.0)
        self.declare_parameter("kp_angular", 8.0)
        self.declare_parameter("depth_hold_kp", 0.8)
        self.declare_parameter("publish_tf", True)
        # 洋流
        self.declare_parameter("current_speed", 0.0)
        self.declare_parameter("current_direction_deg", 90.0)
        self.declare_parameter("current_seed", 0)
        # MuJoCo 的自由关节角速度在机体系；个别版本若有差异可关掉这个开关
        self.declare_parameter("free_joint_angular_in_body", True)

        gp = self.get_parameter
        self.sim_hz = float(gp("sim_hz").value)
        self.pub_hz = float(gp("pub_hz").value)
        self.dt = 1.0 / self.sim_hz
        self.publish_every = max(1, int(round(self.sim_hz / self.pub_hz)))
        self.w_body_local = bool(gp("free_joint_angular_in_body").value)
        self.publish_tf_flag = bool(gp("publish_tf").value)
        self.depth_kp = float(gp("depth_hold_kp").value)
        self.kp_lin = float(gp("kp_linear").value)
        self.kp_ang = float(gp("kp_angular").value)

        self.params = VehicleParams(
            mass=float(gp("mass").value),
            displacement=float(gp("displacement").value),
            ballast=float(gp("ballast").value),
            rho=float(gp("rho").value),
            drag_linear=list(gp("drag_linear").value),
            drag_angular=list(gp("drag_angular").value),
            cb_offset=list(gp("cb_offset").value),
            max_force=list(gp("max_force").value),
            max_torque=list(gp("max_torque").value),
        )
        self.current = CurrentField(CurrentParams(
            mean_speed=float(gp("current_speed").value),
            direction_deg=float(gp("current_direction_deg").value),
            seed=int(gp("current_seed").value),
        ))

        # ---------------- MuJoCo ----------------
        try:
            import mujoco
        except ImportError as exc:      # pragma: no cover - 环境依赖
            raise RuntimeError("未安装 mujoco：pip3 install mujoco") from exc
        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_string(MJCF.format(mass=self.params.mass))
        self.data = mujoco.MjData(self.model)
        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "rov")
        self.data.qpos[2] = -float(gp("start_depth").value)
        self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(self.model, self.data)

        # ---------------- 状态 ----------------
        self.lock = threading.Lock()
        self.cmd = np.zeros(6)
        self.depth_hold = False
        self.depth_target = -float(gp("start_depth").value)
        self.v_body = np.zeros(3)
        self.w_body = np.zeros(3)
        self.R = np.eye(3)
        self.current_world = np.zeros(3)
        self.steps = 0
        self._running = True

        # ---------------- 接口 ----------------
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(Bool, "/rov/depth_hold", self.on_depth_hold, 10)
        self.pub_odom = self.create_publisher(Odometry, "/rov/odom", 10)
        self.pub_depth = self.create_publisher(Float64, "/rov/depth", 10)
        self.pub_current = self.create_publisher(Vector3, "/rov/current", 10)
        self.pub_tf = None
        if self.publish_tf_flag:
            from tf2_ros import TransformBroadcaster
            self.pub_tf = TransformBroadcaster(self)

        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

        self.get_logger().info(
            "mujoco_sim_node 启动：%.0f Hz 物理 / %.0f Hz 发布；净浮力 %+.2f N"
            "（≈ %+.2f kg），不配平时漂移 %.3f m/s"
            % (self.sim_hz, self.pub_hz, self.params.net_buoyancy(),
               self.params.net_buoyancy() / 9.81, self.params.terminal_speed(2)))

    # ---------------------------------------------------------------- 回调
    def on_cmd(self, msg: Twist):
        with self.lock:
            self.cmd = np.array([msg.linear.x, msg.linear.y, msg.linear.z,
                                 msg.angular.x, msg.angular.y, msg.angular.z])

    def on_depth_hold(self, msg: Bool):
        with self.lock:
            self.depth_hold = bool(msg.data)
            if self.depth_hold:
                self.depth_target = float(self.data.qpos[2])   # 以当前深度锁定

    # ---------------------------------------------------------------- 物理
    def _loop(self):
        while self._running and rclpy.ok():
            self._step()
            self.steps += 1
            if self.steps % self.publish_every == 0:
                self._publish()

    def _step(self):
        # 1) 位姿 → 旋转矩阵与机体系速度
        q = self.data.qpos[3:7]
        self.R = quat_to_matrix(q[0], q[1], q[2], q[3])
        v_world = np.asarray(self.data.qvel[0:3], dtype=float)
        w_raw = np.asarray(self.data.qvel[3:6], dtype=float)
        self.v_body = self.R.T @ v_world
        self.w_body = w_raw if self.w_body_local else self.R.T @ w_raw

        # 2) 洋流（世界系）→ 相对流速
        z = float(self.data.qpos[2])
        self.current_world = self.current.step(self.dt, z, distance=0.0)
        v_rel_body = self.v_body - self.R.T @ self.current_world

        # 3) 指令 → 期望力（机体系）
        with self.lock:
            cmd = self.cmd.copy()
            hold = self.depth_hold
            target = self.depth_target
        if hold:
            depth = -z
            cmd[2] = float(np.clip(self.depth_kp * (-target - depth), -0.5, 0.5))

        f_body, tau_body = velocity_control(cmd, self.v_body, self.w_body, self.params,
                                           kp_lin=self.kp_lin, kp_ang=self.kp_ang)

        # 4) 阻力（机体系）
        f_body = f_body + drag_force(v_rel_body, self.params.k())
        tau_body = tau_body + drag_torque(self.w_body, self.params.drag_angular)

        # 5) 转世界系 + 叠加静水浮力（浮心与质心分开 → 有恢复力矩）
        f_world = self.R @ f_body
        tau_world = self.R @ tau_body
        f_buoy, tau_buoy = buoyancy_wrench(self.params, self.R)
        f_world = f_world + f_buoy
        tau_world = tau_world + tau_buoy

        self.data.xfrc_applied[self.body_id, 0:3] = f_world
        self.data.xfrc_applied[self.body_id, 3:6] = tau_world
        self.mj.mj_step(self.model, self.data)

    # ---------------------------------------------------------------- 发布
    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        z = float(self.data.qpos[2])
        q = self.data.qpos[3:7]

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(self.data.qpos[0])
        odom.pose.pose.position.y = float(self.data.qpos[1])
        odom.pose.pose.position.z = float(z)
        odom.pose.pose.orientation.w = float(q[0])
        odom.pose.pose.orientation.x = float(q[1])
        odom.pose.pose.orientation.y = float(q[2])
        odom.pose.pose.orientation.z = float(q[3])
        odom.twist.twist.linear.x = float(self.v_body[0])
        odom.twist.twist.linear.y = float(self.v_body[1])
        odom.twist.twist.linear.z = float(self.v_body[2])
        odom.twist.twist.angular.x = float(self.w_body[0])
        odom.twist.twist.angular.y = float(self.w_body[1])
        odom.twist.twist.angular.z = float(self.w_body[2])
        self.pub_odom.publish(odom)

        d = Float64()
        d.data = float(-z)                      # 深度 = −z（世界系 z 向上）
        self.pub_depth.publish(d)

        cur = Vector3()
        cur.x, cur.y, cur.z = (float(self.current_world[0]),
                               float(self.current_world[1]),
                               float(self.current_world[2]))
        self.pub_current.publish(cur)

        if self.pub_tf is not None:
            from geometry_msgs.msg import TransformStamped
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = float(self.data.qpos[0])
            t.transform.translation.y = float(self.data.qpos[1])
            t.transform.translation.z = float(z)
            t.transform.rotation.w = float(q[0])
            t.transform.rotation.x = float(q[1])
            t.transform.rotation.y = float(q[2])
            t.transform.rotation.z = float(q[3])
            self.pub_tf.sendTransform(t)

    # ---------------------------------------------------------------- 收尾
    def current_authority(self) -> np.ndarray:
        return current_authority(self.params, float(np.linalg.norm(self.current_world[:2])))

    def destroy_node(self):
        self._running = False
        thread = getattr(self, "thread", None)
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MujocoSim()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
