"""
人形机器人平衡仿真节点
====================
核心 ROS2 节点：加载自建双足模型，运行站姿 PD + 踝/髋平衡控制器推物理仿真，
发布质心 / ZMP / 捕获点 / 稳定裕度 / 平衡状态，并接受外部扰动力指令。

话题
----
发布：
  * ``/balance/com``              (PointStamped)  全身质心位置
  * ``/balance/zmp``              (PointStamped)  零矩点（LIPM 近似）
  * ``/balance/capture_point``    (PointStamped)  捕获点
  * ``/balance/stability_margin`` (Float64)       支撑多边形稳定裕度 (m)
  * ``/balance/status``           (String)        stable / recovering / fallen
  * ``/joint_states``             (JointState)    15 个关节位置
  * ``/tf``                       (TFMessage)     world → humanoid/base_link
订阅：
  * ``/balance/push``             (Vector3)       扰动力 (N)，持续时长由参数决定

参数见 config/params.yaml；``visualize`` 控制是否打开 MuJoCo 可视化窗口。
"""

import os

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64, String
from tf2_ros import TransformBroadcaster

try:
    from humanoid_push_recovery.balance import (
        BalanceController, capture_point, stability_margin)
    from humanoid_push_recovery.sim_core import JOINT_NAMES, BalanceSim, open_viewer
except ImportError:                                     # 直接以脚本方式运行时的兜底
    from balance import BalanceController, capture_point, stability_margin
    from sim_core import JOINT_NAMES, BalanceSim, open_viewer


class BalanceSimNode(Node):
    """人形机器人平衡与抗扰恢复仿真节点。"""

    def __init__(self):
        super().__init__('balance_sim_node')

        # ========== 参数 ==========
        for name, default in (
            ('model_path', ''), ('sim_rate', 500.0), ('publish_rate', 30.0),
            ('visualize', True), ('kp', 120.0), ('kv', 6.0), ('kp_roll', 400.0),
            ('torque_limit', 300.0), ('fall_height', 0.85), ('fall_tilt', 0.55),
            # 平衡控制器参数为**力矩量纲**（N·m 及其增益），与 config/params.yaml 一致
            ('kp_ankle_lean', 300.0), ('kd_ankle_lean', 0.0),
            ('kp_ankle', 5400.0), ('kd_ankle', 1440.0), ('ankle_torque_limit', 300.0),
            ('kp_ankle_roll_lean', 270.0), ('kd_ankle_roll_lean', 0.0),
            ('kp_ankle_roll', 4800.0), ('kd_ankle_roll', 600.0),
            ('ankle_roll_torque_limit', 180.0),
            ('hip_enable', True), ('kp_hip', 480.0), ('kd_hip', 72.0),
            ('hip_torque_limit', 54.0), ('kp_hip_roll', 1600.0),
            ('kd_hip_roll', 240.0), ('hip_roll_torque_limit', 180.0),
            ('hip_relax', 0.0),
            ('push_duration', 0.08),
            # 下列两项供 standalone / 遥控节点使用；仿真节点一并声明，
            # 以保证与 params.yaml 的 /balance_sim_node 段参数项一一对应
            ('push_force', 120.0), ('push_direction_deg', 0.0),
        ):
            self.declare_parameter(name, default)

        def val(name):
            return self.get_parameter(name).get_parameter_value()

        params = {
            'kp': val('kp').double_value, 'kv': val('kv').double_value,
            'kp_roll': val('kp_roll').double_value,
            'torque_limit': val('torque_limit').double_value,
            'fall_height': val('fall_height').double_value,
            'fall_tilt': val('fall_tilt').double_value,
            'kp_ankle_lean': val('kp_ankle_lean').double_value,
            'kd_ankle_lean': val('kd_ankle_lean').double_value,
            'kp_ankle': val('kp_ankle').double_value,
            'kd_ankle': val('kd_ankle').double_value,
            'ankle_torque_limit': val('ankle_torque_limit').double_value,
            'kp_ankle_roll_lean': val('kp_ankle_roll_lean').double_value,
            'kd_ankle_roll_lean': val('kd_ankle_roll_lean').double_value,
            'kp_ankle_roll': val('kp_ankle_roll').double_value,
            'kd_ankle_roll': val('kd_ankle_roll').double_value,
            'ankle_roll_torque_limit': val('ankle_roll_torque_limit').double_value,
            'hip_enable': val('hip_enable').bool_value,
            'kp_hip': val('kp_hip').double_value, 'kd_hip': val('kd_hip').double_value,
            'hip_torque_limit': val('hip_torque_limit').double_value,
            'kp_hip_roll': val('kp_hip_roll').double_value,
            'kd_hip_roll': val('kd_hip_roll').double_value,
            'hip_roll_torque_limit': val('hip_roll_torque_limit').double_value,
            'hip_relax': val('hip_relax').double_value,
        }
        self.push_duration = val('push_duration').double_value
        self.visualize = val('visualize').bool_value
        model_path = val('model_path').string_value
        sim_rate = val('sim_rate').double_value or 500.0
        publish_rate = val('publish_rate').double_value or 30.0

        # ========== 仿真核心 ==========
        self.sim = BalanceSim(model_path=model_path or None, params=params)
        self.sim.controller = BalanceController(params)
        self.sim.reset()
        self.com_ref = self.sim.com_position()[:2].copy()
        self.recovering = False

        # ========== 可视化窗口 ==========
        self.viewer = open_viewer(self.sim, self.get_logger()) if self.visualize else None

        # ========== 发布者 ==========
        self.pub_com = self.create_publisher(PointStamped, '/balance/com', 10)
        self.pub_zmp = self.create_publisher(PointStamped, '/balance/zmp', 10)
        self.pub_cp = self.create_publisher(PointStamped, '/balance/capture_point', 10)
        self.pub_margin = self.create_publisher(Float64, '/balance/stability_margin', 10)
        self.pub_status = self.create_publisher(String, '/balance/status', 10)
        self.pub_joint = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ========== 订阅者（独立回调组，确保推扰与复位指令不被定时器阻塞）==========
        self.sub_group = ReentrantCallbackGroup()
        self.sub_push = self.create_subscription(
            Vector3, '/balance/push', self._push_callback, 10,
            callback_group=self.sub_group)
        self.sub_reset = self.create_subscription(
            Empty, '/balance/reset', self._reset_callback, 10,
            callback_group=self.sub_group)

        # ========== 单一主循环定时器（彻底解决 rclpy 多定时器饿死话题订阅问题）==========
        # 将物理仿真积分 (sim_rate)、视窗渲染刷新与话题发布由单一主循环统筹推进，
        # 避免多个高频独立定时器在事件循环中互相抢占造成死锁或订阅饥饿。
        self.substeps = max(1, int(round(sim_rate / publish_rate)))
        self.step_count = 0
        self.steps_per_publish = self.substeps
        self.loop_timer = self.create_timer(1.0 / publish_rate, self._loop_callback)

        self.get_logger().info(
            f'平衡仿真节点已启动 | 物理 {sim_rate:.0f} Hz (主循环 {publish_rate:.0f} Hz × {self.substeps} 步) | '
            f'发布 {publish_rate:.0f} Hz | '
            f'髋策略 {"开" if params.get("hip_enable") else "关"} | '
            f'模型 {model_path or "(包内默认)"}')

    # ------------------------------------------------------------------
    def _push_callback(self, msg: Vector3):
        """接收扰动力指令（N），持续 push_duration 秒。"""
        self.sim.apply_push(force=(msg.x, msg.y, msg.z), duration=self.push_duration)
        self.recovering = True
        self.get_logger().info(
            f'施加扰动 {np.linalg.norm([msg.x, msg.y, msg.z]):.1f} N '
            f'(冲量 {np.linalg.norm([msg.x, msg.y, msg.z]) * self.push_duration:.2f} N·s)')

    def _reset_callback(self, _msg: Empty):
        """复位仿真并把当前质心位置重新作为平衡参考。"""
        self.sim.reset()
        self.com_ref = self.sim.com_position()[:2].copy()
        self.recovering = False
        self.get_logger().info('仿真已复位，平衡参考已更新。')

    def _loop_callback(self):
        """主仿真步进、视窗刷新与发布循环。"""
        # 1. 批量推进物理步
        for _ in range(self.substeps):
            self.sim.step()
            self.step_count += 1

        # 2. 刷新 3D 可视化窗口
        if self.viewer is not None:
            try:
                self.viewer.sync()
                if not self.viewer.is_running():
                    self.get_logger().info('可视化窗口已关闭，仿真继续在后台运行。')
                    self.close_viewer()
            except Exception as exc:                              # noqa: BLE001
                self.get_logger().warning(f'可视化窗口异常，已停止刷新：{exc}')
                self.close_viewer()

        # 3. 发布话题状态
        self._publish_callback()

    def close_viewer(self):
        if self.viewer is not None:
            try:
                self.viewer.close()
            except Exception:                                     # noqa: BLE001
                pass
            self.viewer = None

    # ------------------------------------------------------------------
    def _publish_callback(self):
        """发布质心 / ZMP / 捕获点 / 稳定裕度 / 状态 / 关节状态 / TF。"""
        now = self.get_clock().now().to_msg()
        # with_margin=True：额外计算支撑多边形与稳定裕度（控制回路内不需要）
        state = self.sim.read_state(with_margin=True)
        com, vel = state["com"], state["com_vel"]

        # --- 质心 ---
        msg = PointStamped()
        msg.header.stamp = now
        msg.header.frame_id = 'world'
        msg.point.x, msg.point.y, msg.point.z = float(com[0]), float(com[1]), float(com[2])
        self.pub_com.publish(msg)

        # --- ZMP ---
        zmp = state["zmp"]
        msg = PointStamped()
        msg.header.stamp = now
        msg.header.frame_id = 'world'
        msg.point.x, msg.point.y, msg.point.z = float(zmp[0]), float(zmp[1]), 0.0
        self.pub_zmp.publish(msg)

        # --- 捕获点 ---
        xi = capture_point(com, vel, com[2])
        msg = PointStamped()
        msg.header.stamp = now
        msg.header.frame_id = 'world'
        msg.point.x, msg.point.y, msg.point.z = float(xi[0]), float(xi[1]), 0.0
        self.pub_cp.publish(msg)

        # --- 稳定裕度（捕获点到支撑多边形边界的距离，见 balance.stability_margin）---
        margin = state["stability_margin"]
        self.pub_margin.publish(Float64(data=float(margin)))

        # --- 平衡状态 ---
        fallen = state["fallen"]
        offset = float(np.linalg.norm(com[:2] - self.com_ref))
        if fallen:
            status = 'fallen'
        elif offset > 0.008:
            status = 'recovering'
        else:
            status = 'stable'
            self.recovering = False
        self.pub_status.publish(String(data=status))

        # --- TF: world -> humanoid/base_link ---
        quat = self.sim.data.qpos[self.sim.root_qadr + 3: self.sim.root_qadr + 7]
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id = 'humanoid/base_link'
        tf_msg.transform.translation.x = float(com[0])
        tf_msg.transform.translation.y = float(com[1])
        tf_msg.transform.translation.z = float(self.sim.root_height())
        tf_msg.transform.rotation.w = float(quat[0])
        tf_msg.transform.rotation.x = float(quat[1])
        tf_msg.transform.rotation.y = float(quat[2])
        tf_msg.transform.rotation.z = float(quat[3])
        self.tf_broadcaster.sendTransform(tf_msg)

        # --- 关节状态 ---
        jstate = JointState()
        jstate.header.stamp = now
        jstate.name = list(JOINT_NAMES)
        jstate.position = [float(self.sim.data.qpos[a]) for a in self.sim.joint_qadr]
        jstate.velocity = [float(self.sim.data.qvel[a]) for a in self.sim.joint_dadr]
        jstate.effort = [float(v) for v in self.sim.data.ctrl[:len(JOINT_NAMES)]]
        self.pub_joint.publish(jstate)

        # --- 状态打印（1 Hz）---
        if self.step_count % (self.steps_per_publish * 30) == 0:
            self.get_logger().info(
                f'status={status:<10s} 偏移={offset * 1000:6.1f} mm '
                f'倾角={np.degrees(state["tilt"]):5.1f}° 根高={state["root_height"]:.3f} m '
                f'裕度={margin * 1000:+7.1f} mm')


def main(args=None):
    rclpy.init(args=args)
    node = BalanceSimNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close_viewer()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
