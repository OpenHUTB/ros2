#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""动态交通避障部署节点：订阅里程计 + 激光 + 车辆流，发布世界系速度指令.

输入
----
* ``/uav/odom`` (nav_msgs/Odometry)：世界系 ENU 位姿与速度
* ``/lidar/points`` (sensor_msgs/PointCloud2)：世界系点云（静态障碍）
* ``/traffic/vehicles`` (std_msgs/String)：JSON 车辆列表
  ``[{"x":..,"y":..,"vx":..,"vy":..}, ...]``，世界系 ENU

输出
----
* ``/uav/cmd_vel`` (geometry_msgs/Twist)：**世界系**速度（东/北/天），
  桥接需以 ``body_frame:=false`` 启动（与 ``uav_ppo_nav`` 相同）。

车辆来源可用 ``traffic_nav/traffic_feed.launch`` 里的 ``traffic_feed_node``
从 CARLA 的 ``live.json`` 转换而来（支持 calibration offset 的坐标对齐）。

策略用训练导出的 MLP 权重做纯 numpy 推理，部署侧不需要 torch/sb3。
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from policy import (MlpPolicy, N_SECTORS, action_to_velocity,  # noqa: E402
                    build_observation, lidar_points_to_histogram,
                    traffic_features)


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    pts = np.frombuffer(msg.data, dtype=np.float32)
    return pts.reshape(-1, msg.point_step // 4)[:, :3].astype(np.float64)


def parse_goal(value) -> np.ndarray:
    if isinstance(value, str):
        value = [float(x) for x in value.strip().strip("[]").replace(",", " ").split()]
    return np.asarray(value, dtype=np.float64)


def parse_vehicles(payload: str):
    """JSON -> [(x, y, vx, vy), ...]；坏数据返回空列表而不是抛异常."""
    try:
        items = json.loads(payload)
    except (TypeError, ValueError):
        return []
    cars = []
    for item in items if isinstance(items, list) else []:
        try:
            cars.append((float(item["x"]), float(item["y"]),
                         float(item.get("vx", 0.0)), float(item.get("vy", 0.0))))
        except (KeyError, TypeError, ValueError):
            continue
    return cars


class TrafficNavNode(object):
    def __init__(self):
        weights = rospy.get_param("traffic_nav/weights", "")
        if not weights:
            import rospkg  # noqa: WPS433
            weights = os.path.join(rospkg.RosPack().get_path("uav_traffic_nav"),
                                   "models", "bc_policy_s42.npz")
        data = np.load(weights)
        self.include_traffic = bool(int(data["include_traffic"])) \
            if "include_traffic" in data else True
        self.policy = MlpPolicy(data)
        rospy.loginfo("traffic_nav: 策略 %s（交通特征=%s）", weights, self.include_traffic)

        self.goal = parse_goal(rospy.get_param("traffic_nav/goal", [8.0, 0.0, 1.2]))
        self.publish_hz = float(rospy.get_param("rate/publish_hz", 10.0))
        self.cmd_topic = rospy.get_param("topic/cmd_vel", "/uav/cmd_vel")
        self.traffic_topic = rospy.get_param("topic/traffic", "/traffic/vehicles")
        self.safety_stop = float(rospy.get_param("traffic_nav/safety_stop_m", 1.6))
        rospy.loginfo("traffic_nav: 目标点 ENU %s", self.goal)

        bridge_bf = rospy.get_param("control/body_frame", None)
        if bridge_bf is None:
            rospy.logwarn("traffic_nav: 未读到桥接 control/body_frame，"
                          "请确认桥接以 body_frame:=false 启动")
        elif bool(bridge_bf):
            rospy.logerr("traffic_nav: 桥接 body_frame=true，但本节点发布世界系速度！"
                         "请以 body_frame:=false 重新启动桥接。")

        self.latest_odom = None
        self.latest_cloud = None
        self.cars = []
        self.pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        rospy.Subscriber("/uav/odom", Odometry, self._cb_odom, queue_size=1)
        rospy.Subscriber("/lidar/points", PointCloud2, self._cb_cloud, queue_size=1)
        rospy.Subscriber(self.traffic_topic, String, self._cb_traffic, queue_size=1)
        self.rate = rospy.Rate(self.publish_hz)

    def _cb_odom(self, msg):
        self.latest_odom = msg

    def _cb_cloud(self, msg):
        self.latest_cloud = msg

    def _cb_traffic(self, msg):
        self.cars = parse_vehicles(msg.data)

    # ------------------------------------------------------------------ 观测
    def build_obs(self) -> np.ndarray:
        odom = self.latest_odom
        pos = np.array([odom.pose.pose.position.x,
                        odom.pose.pose.position.y,
                        odom.pose.pose.position.z], dtype=np.float64)
        vel = np.array([odom.twist.twist.linear.x,
                        odom.twist.twist.linear.y,
                        odom.twist.twist.linear.z], dtype=np.float64)
        if self.latest_cloud is not None:
            pts = cloud_to_xyz(self.latest_cloud)
            hist = lidar_points_to_histogram(pts, drone_xy=pos[:2], drone_z=pos[2])
        else:
            hist = np.ones(N_SECTORS, dtype=np.float32)
        traffic = traffic_features(self.cars, drone_xy=(pos[0], pos[1]),
                                   drone_vel=(vel[0], vel[1]))
        return build_observation(hist, self.goal - pos, vel, traffic=traffic,
                                 include_traffic=self.include_traffic), pos

    def emergency_stop(self) -> bool:
        """几何安全兜底：任何车辆进入安全半径内立即悬停（与策略无关）."""
        if self.latest_odom is None:
            return False
        pos = self.latest_odom.pose.pose.position
        for (x, y, _vx, _vy) in self.cars:
            if math.hypot(x - pos.x, y - pos.y) < self.safety_stop:
                return True
        return False

    # ------------------------------------------------------------------ 主循环
    def step_once(self):
        obs, _pos = self.build_obs()
        if self.emergency_stop():
            action = np.zeros(4, dtype=np.float32)
        else:
            action = self.policy.forward(obs)
        vx, vy, vz, wz = action_to_velocity(action)
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = vx, vy, vz
        t.angular.z = wz
        self.pub.publish(t)
        return obs, action

    def spin(self):
        while not rospy.is_shutdown():
            if self.latest_odom is None:
                rospy.loginfo_throttle(3.0, "traffic_nav: 等待 /uav/odom ...")
                self.rate.sleep()
                continue
            try:
                self.step_once()
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn_throttle(3.0, "traffic_nav: %s", exc)
            self.rate.sleep()


def main():
    rospy.init_node("uav_traffic_nav", anonymous=False)
    TrafficNavNode().spin()


if __name__ == "__main__":
    main()
