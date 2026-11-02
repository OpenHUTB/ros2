# -*- coding: utf-8 -*-
"""controller_node.py —— 控制节点：检测结果(+位姿) -> 速度指令。

三种模式（参数 mode）：
  expert     几何视觉伺服，不使用网络，作为 ROS 侧的对照基线
  two_stage  感知节点给的像素 -> 视觉伺服特征 -> 控制网络 -> 速度
  e2e        直接从 /uav/image 抽特征 -> 端到端网络 -> 速度

输出 /uav/cmd_vel (geometry_msgs/Twist)：
  linear.x/y  机体前向/右向速度(m/s)，linear.z 世界系垂直速度(m/s)，
  angular.z   偏航角速度(rad/s)。偏航律与离线实验完全一致（机头指向目标），
  只是这里被写成角速度指令，交给仿真节点的一阶模型去跟踪。

**为什么在订阅回调里直接下发指令，而不是挂在定时器上**：
初版把控制律放在 20 Hz 的定时器里，结果闭环误差比离线解析式仿真明显偏大
（圆周场景 0.149 对 0.136）。原因是定时器与图像发布同频但相位随机——有相当
比例的周期里，控制器比检测结果早执行几十毫秒，用的是**上一帧**的像素，于是
执行延迟在 1~2 个周期之间抖动。改成"检测一到就算、算完立刻发"之后延迟固定为
1 个周期（图像经仿真节点要等定时器回调返回才会被消费，这一步消不掉），
与离线评估的差异就只剩这一个常数项。

定时器保留为**看门狗**：连续 3 个周期没有新输入（感知节点没起、图像断流、
目标长时间丢失）就下发悬停指令，避免仿真节点一直沿用最后一条旧指令飞出去。
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image

from . import params as P
from .controllers import (ControllerNet, E2ENet, ServoState, expert_command,
                          pixel_errors, pointing_yaw)
from .detector import DetectorNet
from .model_io import model_path

# 多少个周期没有新输入就判定为断流
WATCHDOG_TICKS = 3


class ControllerNode(Node):
    def __init__(self):
        super().__init__('uav_controller')
        self.declare_parameter('mode', 'two_stage')
        self.declare_parameter('control_model', model_path('control.npz'))
        self.declare_parameter('e2e_model', model_path('e2e.npz'))

        self.mode = str(self.get_parameter('mode').value)
        if self.mode not in ('expert', 'two_stage', 'e2e'):
            raise ValueError('mode 只能是 expert / two_stage / e2e')

        self.control = None
        self.e2e = None
        self.detector = None
        if self.mode == 'two_stage':
            self.control = ControllerNet.load(
                str(self.get_parameter('control_model').value))
        elif self.mode == 'e2e':
            self.e2e = E2ENet.load(str(self.get_parameter('e2e_model').value))
            self.detector = DetectorNet()      # 只借用特征提取(无权重)

        self.servo = ServoState()
        self.det = None
        self.img = None
        self.yaw = 0.0
        self.z = P.H_DES
        self.v_prev = np.zeros(3)
        self.lost_steps = 0
        self.ex = 0.0
        self.steps = 0          # 看门狗心跳
        self.fresh_step = -1    # 最近一次收到有效输入的节拍

        self.pub = self.create_publisher(Twist, '/uav/cmd_vel', 10)
        self.create_subscription(PointStamped, '/uav/detection',
                                 self._on_det, 10)
        self.create_subscription(Odometry, '/uav/odom', self._on_odom, 10)
        self.create_subscription(Image, '/uav/image', self._on_image, 1)
        self.timer = self.create_timer(P.DT, self._watchdog)
        self.get_logger().info('控制节点模式：%s（反应式下发 + 看门狗）' % self.mode)

    # ---------- 输入 ----------
    def _on_det(self, msg):
        self.det = {'u': msg.point.x, 'v': msg.point.y, 'vis': msg.point.z}
        if self.mode == 'e2e':
            return                  # 端到端模式只吃图像
        self.fresh_step = self.steps
        self._publish_cmd(self._compute())

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.z = msg.pose.pose.position.z

    def _on_image(self, msg):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        img = buf.reshape(msg.height, msg.width, 3).astype(np.float32)
        if msg.encoding.lower() == 'bgr8':
            img = img[:, :, ::-1]
        self.img = img / 255.0
        if self.mode != 'e2e':
            return                  # 两阶段/专家模式用感知节点的结果
        self.fresh_step = self.steps
        self._publish_cmd(self._compute())

    # ---------- 看门狗 ----------
    def _watchdog(self):
        self.steps += 1
        if self.steps - self.fresh_step >= WATCHDOG_TICKS:
            self.get_logger().debug(
                '%d 个周期没有新输入，下发悬停指令' % WATCHDOG_TICKS)
            self._publish_cmd(np.zeros(3))
            self.fresh_step = self.steps     # 避免每周期重复刷日志

    # ---------- 控制律 ----------
    def _compute(self):
        if self.mode == 'e2e':
            return self._step_e2e()
        if self.det is None:
            return np.zeros(3)
        if self.mode == 'expert':
            return self._step_expert()
        return self._step_two_stage()

    def _features(self, ex, ey, visible):
        return self.servo.features(ex, ey, self.z,
                                   self.v_prev[0], self.v_prev[1], visible)

    def _step_expert(self):
        ex, ey = pixel_errors(self.det)
        self.ex = ex
        # 与离线 ExpertController.step 保持一致：无条件下更新（update 内部
        # 已按 I_BAND 限制积分区间），否则两边就不是同一条控制律了
        self.servo.update(ey, P.DT)
        cmd, lost = expert_command(self._features(ex, ey, self.det['vis'] >= 0.5),
                                   self.det['vis'] >= 0.5)
        self.lost_steps = self.lost_steps + 1 if lost else 0
        return cmd

    def _step_two_stage(self):
        ex, ey = pixel_errors(self.det)
        self.ex = ex
        self.servo.update(ey, P.DT)          # 与离线 TwoStageController.step 一致
        if self.det['vis'] < 0.5:
            self.lost_steps += 1
            return np.zeros(3)
        self.lost_steps = 0
        return self.control.predict(self._features(ex, ey, True))

    def _step_e2e(self):
        if self.img is None:
            return np.zeros(3)
        feat = self.detector.features_e2e(self.img)
        if P.E2E_CMD_FEEDBACK:
            feat = np.concatenate([feat, self.v_prev
                                   / np.array([P.V_MAX, P.V_MAX, P.VZ_MAX])])
        cmd = self.e2e.predict_img(feat)
        if float(np.abs(cmd).sum()) < 1e-3:
            self.lost_steps += 1
        else:
            self.lost_steps = 0
        self.ex = 0.0
        return cmd

    # ---------- 输出 ----------
    def _publish_cmd(self, cmd):
        cmd = np.asarray(cmd, dtype=float)
        self.ex = float(self.ex)
        yaw_des = pointing_yaw(self.yaw, self.ex, self.lost_steps)
        yaw_rate = float(np.clip((yaw_des - self.yaw) / P.DT,
                                 -P.YAW_RATE_MAX, P.YAW_RATE_MAX))

        msg = Twist()
        msg.linear.x = float(cmd[0])
        msg.linear.y = float(cmd[1])
        msg.linear.z = float(cmd[2])
        msg.angular.z = yaw_rate
        self.pub.publish(msg)
        self.v_prev = cmd


def main(argv=None):
    rclpy.init(args=argv)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
