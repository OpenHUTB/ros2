#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 1 (Noetic) 节点：在线跑横向控制对比实验。

节点内部积分车辆模型并运行选定的横向控制器，通过话题把状态与累积指标发出来，
便于用 ``rostopic echo`` / ``rqt_plot`` / RViz 观察：

发布
----
* ``/lateral_control/odom``      nav_msgs/Odometry           模拟车辆位姿与速度
* ``/lateral_control/cmd``       geometry_msgs/Twist         由前轮转角折算的角速度指令
* ``/lateral_control/styles``    std_msgs/Float64MultiArray  [横向偏差, 航向偏差, 前轮转角, 车速]
* ``/lateral_control/metrics``   std_msgs/Float64MultiArray  累积指标 [mean|e|, max|e|, RMS, 步数]
* ``/lateral_control/path``      nav_msgs/Path               参考路径（latched，便于 RViz 观察）

参数见 ``config/params.yaml``：controller / scenario / cruise_speed / model /
initial_offset / initial_heading_error / rate_hz。

设计说明：本节点**不依赖** Gazebo / CARLA，车辆模型在节点内积分，
因此 ``roslaunch lateral_control_benchmark main.launch`` 一条命令即可看到完整闭环，
这在没有 GPU 或没有仿真器的机器上也能复现。
"""

from __future__ import annotations

import math

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64MultiArray

from .controllers import BaseController, build_controller
from .geometry import build_scenario
from .simulation import SimulationConfig, Simulator, SpeedProfile, track_speed
from .vehicle_model import DynamicBicycle, KinematicBicycle, VehicleParams


def yaw_to_quaternion(yaw: float):
    """偏航角转四元数（只绕 z 轴）。"""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class LateralControlNode:
    """横向控制对比节点。"""

    def __init__(self) -> None:
        rospy.init_node("lateral_control_node")

        self.controller_name = str(rospy.get_param("~controller", "stanley"))
        self.scenario_name = str(rospy.get_param("~scenario", "sine"))
        self.cruise_speed = float(rospy.get_param("~cruise_speed", 8.0))
        self.model_name = str(rospy.get_param("~model", "kinematic"))
        self.rate_hz = float(rospy.get_param("~rate_hz", 50.0))
        self.initial_offset = float(rospy.get_param("~initial_offset", 1.2))
        self.initial_heading_error = float(rospy.get_param("~initial_heading_error", 0.10))

        params = VehicleParams()
        self.controller: BaseController = build_controller(self.controller_name, params)
        self.path = build_scenario(self.scenario_name)

        config = SimulationConfig(
            dt=1.0 / max(self.rate_hz, 1.0),
            model=self.model_name,
            initial_offset=self.initial_offset,
            initial_heading_error=self.initial_heading_error,
            speed=SpeedProfile(cruise_speed=self.cruise_speed),
            params=params,
        )
        self.sim = Simulator(self.path, config)
        self.state = self.sim.initial_state()
        self.memory = {"vy": 0.0, "r": 0.0}
        self.dt = config.dt
        self.errors = []
        self.loop_path = bool(rospy.get_param("~loop_path", True))
        self.laps = 0

        self.pub_odom = rospy.Publisher("/lateral_control/odom", Odometry, queue_size=1)
        self.pub_cmd = rospy.Publisher("/lateral_control/cmd", Twist, queue_size=1)
        self.pub_styles = rospy.Publisher("/lateral_control/styles", Float64MultiArray, queue_size=1)
        self.pub_metrics = rospy.Publisher("/lateral_control/metrics", Float64MultiArray, queue_size=1)
        self.pub_path = rospy.Publisher("/lateral_control/path", Path, queue_size=1, latch=True)

        self.publish_path()
        rospy.loginfo("lateral_control_node 已启动: controller=%s scenario=%s model=%s",
                      self.controller_name, self.scenario_name, self.model_name)
        rospy.Timer(rospy.Duration(self.dt), self.on_timer)
        rospy.loginfo("Ctrl+C 退出；用 rostopic echo /lateral_control/styles 观察横向偏差")

    def publish_path(self) -> None:
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        for point in self.path.sample(400):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            msg.poses.append(pose)
        self.pub_path.publish(msg)

    def on_timer(self, _event) -> None:
        projection = self.path.project(self.state[:2])
        v_target = self.sim.cfg.speed.target(self.path, projection.s)
        accel = track_speed(v_target, self.state[3], self.dt)

        result = self.controller.compute(self.state, self.state[3], self.path,
                                         projection, self.dt)
        delta = result.delta
        yaw_rate = self.state[3] * math.tan(delta) / self.sim.cfg.params.wheelbase

        self.errors.append(float(projection.signed_distance))
        path_yaw = math.atan2(projection.tangent[1], projection.tangent[0])
        psi = float((self.state[2] - path_yaw + math.pi) % (2 * math.pi) - math.pi)

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(self.state[0])
        odom.pose.pose.position.y = float(self.state[1])
        qx, qy, qz, qw = yaw_to_quaternion(float(self.state[2]))
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(self.state[3])
        odom.twist.twist.angular.z = float(yaw_rate)
        self.pub_odom.publish(odom)

        cmd = Twist()
        cmd.linear.x = float(self.state[3])
        cmd.angular.z = float(yaw_rate)
        self.pub_cmd.publish(cmd)

        styles = Float64MultiArray()
        styles.data = [float(projection.signed_distance), psi, float(delta), float(self.state[3])]
        self.pub_styles.publish(styles)

        arr = np.asarray(self.errors)
        metrics = Float64MultiArray()
        metrics.data = [float(np.mean(np.abs(arr))), float(np.max(np.abs(arr))),
                        float(np.sqrt(np.mean(arr ** 2))), float(len(arr))]
        self.pub_metrics.publish(metrics)

        if isinstance(self.sim.model, DynamicBicycle):
            self.state, self.memory = self.sim.model.step(
                self.state, delta, accel, self.dt, self.memory)
        else:
            self.state = self.sim.model.step(self.state, delta, accel, self.dt)

        # 到达非闭合轨迹终点：默认重置回起点，让演示可以持续观察
        # （否则车辆会沿末段切向一直直行，横向偏差冻结在某个常数值，看起来像"卡住"）
        if self._reached_end(projection.s):
            if self.loop_path:
                self.laps += 1
                self.state = self.sim.initial_state()
                self.memory = {"vy": 0.0, "r": 0.0}
                self.controller.reset()
                self.errors.clear()
                rospy.loginfo("已到达轨迹终点，重置回起点（第 %d 次）", self.laps)
            else:
                rospy.loginfo_throttle(5.0, "已到达轨迹终点（loop_path=false，继续直行）")

    def _reached_end(self, s: float) -> bool:
        if self.path.closed:
            return False
        return s >= self.path.total_length - 1.0


def main() -> None:
    LateralControlNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
