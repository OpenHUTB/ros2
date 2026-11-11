#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轨迹识别节点：订阅速度指令，识别飞行状态并记录轨迹.

在原有“直线 / 旋转 / 悬停”识别与 matplotlib 实时绘图的基础上，新增：

* 状态细分：前进后退 × 左转右转 共 9 种状态（见 :mod:`flight_state`）；
* 状态广播：把当前状态发布到 ``/drone/flight_state``（std_msgs/String）；
* 记录导出：退出时（或每 N 个采样点）导出 CSV 与 JSON 汇总；
* 参数化：步长、转角、死区、绘图开关、导出目录等都可以通过 ROS 参数配置。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from .flight_state import PoseIntegrator, TrajectoryRecorder, classify_state


class TrajectoryAnalyzer(Node):
    def __init__(self):
        super().__init__('trajectory_analyzer')

        # ---------------------------------------------------------- 参数
        self.cmd_vel_topic = str(self.declare_parameter('cmd_vel_topic', '/drone/cmd_vel').value)
        self.state_topic = str(self.declare_parameter('state_topic', '/drone/flight_state').value)
        self.step_length = float(self.declare_parameter('step_length', 0.5).value)
        self.turn_step = float(self.declare_parameter('turn_step', 0.2).value)
        self.linear_deadband = float(self.declare_parameter('linear_deadband', 0.01).value)
        self.angular_deadband = float(self.declare_parameter('angular_deadband', 0.01).value)
        self.enable_plot = bool(self.declare_parameter('enable_plot', True).value)
        self.export_on_shutdown = bool(self.declare_parameter('export_on_shutdown', True).value)
        self.export_every = int(self.declare_parameter('export_every', 0).value)
        self.output_dir = str(self.declare_parameter('output_dir', '').value)
        self.csv_name = str(self.declare_parameter('csv_name', 'drone_trajectory.csv').value)
        self.json_name = str(self.declare_parameter(
            'json_name', 'drone_trajectory_summary.json').value)

        # ---------------------------------------------------------- 记录器
        self.integrator = PoseIntegrator(
            step_length=self.step_length,
            turn_step=self.turn_step,
            linear_deadband=self.linear_deadband,
            angular_deadband=self.angular_deadband,
        )
        self.recorder = TrajectoryRecorder(
            dt=1.0,   # 仅在未提供时间戳时作为回退；节点始终传真实时间戳
            output_dir=self.output_dir or None,
            csv_name=self.csv_name,
            json_name=self.json_name,
        )

        # ---------------------------------------------------------- 通信
        self.subscription = self.create_subscription(
            Twist, self.cmd_vel_topic, self.listener_callback, 10)
        self.state_publisher = self.create_publisher(String, self.state_topic, 10)
        self.get_logger().info('轨迹识别节点已启动，等待飞行数据...')

        # ---------------------------------------------------------- 绘图
        self.plot = None
        if self.enable_plot:
            self._init_plot()

    # -------------------------------------------------------------- 回调
    def _now(self):
        """节点时钟（秒）；使用仿真时间时会跟随 /clock。"""
        return self.get_clock().now().nanoseconds / 1e9

    def listener_callback(self, msg):
        state = classify_state(msg.linear.x, msg.angular.z,
                               self.linear_deadband, self.angular_deadband)
        x, y, theta = self.integrator.step(msg.linear.x, msg.angular.z)
        # 用真实时间戳记录：发布频率变化时，duration_s / 各状态时长不会被算错。
        self.recorder.add(state, x, y, theta, msg.linear.x, msg.angular.z,
                          timestamp=self._now())

        self.state_publisher.publish(String(data=state))
        self.get_logger().info(
            '【轨迹识别】当前飞行状态：%s（x=%.2f, y=%.2f, θ=%.2f rad）' % (state, x, y, theta))

        if self.plot is not None:
            self._update_plot()
        if self.export_every > 0 and len(self.recorder.samples) % self.export_every == 0:
            self.export_records()

    # -------------------------------------------------------------- 导出
    def export_records(self):
        """把当前记录导出为 CSV + JSON，返回文件路径字典。"""
        params = {
            'step_length': self.step_length,
            'turn_step': self.turn_step,
            'linear_deadband': self.linear_deadband,
            'angular_deadband': self.angular_deadband,
            'cmd_vel_topic': self.cmd_vel_topic,
            'time_source': 'node_clock',
        }
        paths = self.recorder.export(params=params)
        self.get_logger().info('轨迹记录已导出：%s / %s' % (paths['csv'], paths['json']))
        return paths

    def shutdown(self):
        """退出前按需落盘，并关闭绘图窗口。"""
        if self.export_on_shutdown and self.recorder.samples:
            try:
                self.export_records()
            except Exception as exc:  # pragma: no cover - 只在异常环境触发
                self.get_logger().error('轨迹记录导出失败：%s' % exc)
        if self.plot is not None:
            try:
                import matplotlib.pyplot as plt
                plt.close(self.plot['figure'])
            except Exception:  # pragma: no cover
                pass
            self.plot = None

    # -------------------------------------------------------------- 绘图
    def _init_plot(self):
        """延迟导入 matplotlib；没装或没有显示器时只告警、不退出。"""
        try:
            import matplotlib.pyplot as plt
            plt.ion()
            figure, axes = plt.subplots()
        except Exception as exc:  # ImportError / TclError / 无 DISPLAY 等
            self.get_logger().warn(
                '无法初始化绘图窗口（%s），已关闭实时绘图（记录照常进行）' % exc)
            self.enable_plot = False
            self.plot = None
            return
        line, = axes.plot([], [], 'b-', label='Drone Path')
        axes.set_xlim(-10, 10)
        axes.set_ylim(-10, 10)
        axes.set_title('UAV Flight Trajectory')
        axes.set_xlabel('X')
        axes.set_ylabel('Y')
        axes.legend()
        axes.grid(True)
        self.plot = {'figure': figure, 'axes': axes, 'line': line}

    def _update_plot(self):
        xs = [sample['x'] for sample in self.recorder.samples]
        ys = [sample['y'] for sample in self.recorder.samples]
        self.plot['line'].set_data(xs, ys)
        self.plot['axes'].relim()
        self.plot['axes'].autoscale_view()
        self.plot['figure'].canvas.draw()
        self.plot['figure'].canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
