#!/usr/bin/env python3
"""ROS Noetic 节点：把仿真核心包成标准话题接口。

订阅：
    /cmd_vel        geometry_msgs/Twist   linear.x=前进速度(m/s), angular.z=偏航角速度(rad/s)
发布：
    /odom           nav_msgs/Odometry     里程计（世界系位姿 + 车体系速度）
    /imu            sensor_msgs/Imu       陀螺仪/加速度计/姿态四元数
    /suspension     sensor_msgs/JointState  四个悬架滑柱行程（米）

两种模式：
    ~auto_target=true（默认）  节点按目标点自主行驶（与无头演示同一套控制律）
    ~auto_target=false         完全听 /cmd_vel，差速由调用方决定

启动：
    roslaunch mujoco_offroad_suspension_robot_noetic offroad_sim.launch
    roslaunch mujoco_offroad_suspension_robot_noetic offroad_sim.launch gui:=false
"""
import os
import sys
import threading

import numpy as np
import rospy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402  MuJoCo >=2.2 必须显式导入 viewer 子模块
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from tf.transformations import quaternion_from_euler  # noqa: E402

from offroad_sim_core import OffroadSim, SUS_JOINTS  # noqa: E402


class MujocoOffroadNode:
    def __init__(self):
        rospy.init_node("mujoco_offroad_node")

        target = rospy.get_param("~target", [18.0, 2.0])
        start = rospy.get_param("~start", [-2.0, -0.5])
        self.auto_target = bool(rospy.get_param("~auto_target", True))
        self.gui = bool(rospy.get_param("~gui", True))
        self.rate_hz = int(rospy.get_param("~rate", 50))
        self.wheel_radius = float(rospy.get_param("~wheel_radius", 0.145))
        self.track = float(rospy.get_param("~track_width", 0.54))

        self.sim = OffroadSim(target=tuple(target), start=tuple(start))
        self.sim.ctrl.track = self.track
        self.sim.ctrl.r = self.wheel_radius

        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = rospy.Time(0)
        self.mode = None          # "auto" / "remote"，用于模式切换时清理控制器状态

        rospy.Subscriber("/cmd_vel", Twist, self.on_cmd, queue_size=1)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher("/imu", Imu, queue_size=10)
        self.sus_pub = rospy.Publisher("/suspension", JointState, queue_size=10)

        self.model = self.sim.model
        self.data = self.sim.data

        # 每个发布周期推进多少物理步：保证仿真按 1x 实时速度运行
        self.steps_per_cycle = max(
            1, int(round(1.0 / (self.rate_hz * float(self.model.opt.timestep))))
        )
        self.lock = threading.Lock()

        rospy.loginfo("terrain: n=%dx%d cell=%.2fm elev[%.2f, %.2f] m",
                      self.sim.terrain_info["nrow"], self.sim.terrain_info["ncol"],
                      self.sim.terrain_info["cell_m"],
                      self.sim.terrain_info["elev_min"], self.sim.terrain_info["elev_max"])

    # ------------------------------------------------------------------
    def on_cmd(self, msg: Twist):
        self.cmd_v = float(msg.linear.x)
        self.cmd_w = float(msg.angular.z)
        self.last_cmd_time = rospy.Time.now()

    # ------------------------------------------------------------------
    def publish(self, stamp):
        s = self.sim.state()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = s["x"]
        odom.pose.pose.position.y = s["y"]
        odom.pose.pose.position.z = s["z"]
        q = quaternion_from_euler(s["roll"], s["pitch"], s["yaw"])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = s["v_body"]
        odom.twist.twist.angular.z = s["gyro"][2]
        self.odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "base_link"
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = s["gyro"]
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = s["accel"]
        # 协方差：orientation 未建模噪声，按 ROS 惯例首元素填 -1 表示未知；
        # 角速度/线加速度按 IMU 典型噪声量级填写（0.02 rad/s、0.1 m/s²），不再留全 0 的「完全精确」。
        imu.orientation_covariance[0] = -1.0
        for i in (0, 4, 8):
            imu.angular_velocity_covariance[i] = 0.02 ** 2
            imu.linear_acceleration_covariance[i] = 0.1 ** 2
        self.imu_pub.publish(imu)

        js = JointState()
        js.header.stamp = stamp
        js.name = list(SUS_JOINTS)
        js.position = list(s["sus"])
        self.sus_pub.publish(js)

    # ------------------------------------------------------------------
    def step_once(self):
        """一次物理步：自动目标 or 遥控指令。"""
        auto = self.auto_target and (rospy.Time.now() - self.last_cmd_time).to_sec() > 0.3
        if auto:
            if self.mode != "auto":
                # 遥控期间控制器不更新 v_ref，切回自动的第一拍会带着旧速度起步 —— 这里清零
                self.sim.ctrl.v_ref = 0.0
                self.mode = "auto"
            self.sim.step()
            return
        self.mode = "remote"
        # 遥控模式：把 /cmd_vel 直接映射成左右轮速
        v = float(np.clip(self.cmd_v, -2.0, 2.0))
        w = float(np.clip(self.cmd_w, -1.5, 1.5))
        wl = (v - w * self.track / 2.0) / self.wheel_radius
        wr = (v + w * self.track / 2.0) / self.wheel_radius
        self.data.ctrl[0] = float(np.clip(wl, -9, 9))
        self.data.ctrl[1] = float(np.clip(wr, -9, 9))
        self.data.ctrl[2] = self.data.ctrl[0]
        self.data.ctrl[3] = self.data.ctrl[1]
        mujoco.mj_step(self.model, self.data)

    # ------------------------------------------------------------------
    def run(self):
        rate = rospy.Rate(self.rate_hz)
        if self.gui:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                viewer.cam.lookat[:] = [self.data.qpos[0], self.data.qpos[1], 0.3]
                viewer.cam.distance = 3.4
                viewer.cam.elevation = -18
                viewer.cam.azimuth = 140
                while not rospy.is_shutdown() and viewer.is_running():
                    for _ in range(self.steps_per_cycle):
                        self.step_once()
                    viewer.cam.lookat[:] = [self.data.qpos[0], self.data.qpos[1],
                                            self.data.qpos[2] + 0.05]
                    viewer.sync()          # 被动 viewer 必须每周期 sync，否则画面/交互卡死
                    self.publish(rospy.Time.now())
                    rate.sleep()
        else:
            while not rospy.is_shutdown():
                for _ in range(self.steps_per_cycle):
                    self.step_once()
                self.publish(rospy.Time.now())
                rate.sleep()


if __name__ == "__main__":
    try:
        node = MujocoOffroadNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
