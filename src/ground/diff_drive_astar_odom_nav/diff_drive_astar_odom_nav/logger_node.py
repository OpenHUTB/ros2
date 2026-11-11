#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验数据记录节点：把一次真实运行中的里程计、Gazebo 真值、指令与
横向误差逐行写入 CSV；同时保存全局路径点列，供离线画图与指标统计。"""

import csv
import json
import math
import os
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from diff_drive_astar_odom_nav.map_utils import point_to_polyline_distance


def pose_from_odom(msg):
    p = msg.pose.pose
    q = p.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return (p.position.x, p.position.y, yaw)


class LoggerNode(Node):
    def __init__(self):
        super().__init__("experiment_logger")
        self.declare_parameter("output_dir", "results/latest")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("truth_topic", "/ground_truth")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/nav_status")
        self.declare_parameter("sample_rate", 20.0)

        out = self.get_parameter("output_dir").value
        os.makedirs(out, exist_ok=True)
        self.traj_path = os.path.join(out, "trajectory.csv")
        self.plan_path = os.path.join(out, "plan.csv")
        self.info_path = os.path.join(out, "run_info.json")
        self.traj_file = open(self.traj_path, "w", newline="", encoding="utf-8")
        self.traj_writer = csv.writer(self.traj_file)
        self.traj_writer.writerow([
            "t", "odom_x", "odom_y", "odom_yaw",
            "truth_x", "truth_y", "truth_yaw",
            "cmd_v", "cmd_w", "ref_x", "ref_y", "cross_track",
            "odom_cross_track", "progress_index",
        ])
        self.traj_file.flush()

        self.odom = None
        self.truth = None
        self.odom_history = deque(maxlen=200)
        self.truth_history = deque(maxlen=200)
        self.cmd = (0.0, 0.0)
        self.plan = []
        self.plan_saved = False
        self.status = "IDLE"
        self.rows = 0
        self.started = time.time()

        plan_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value,
                                 self.on_odom, 20)
        self.create_subscription(Odometry, self.get_parameter("truth_topic").value,
                                 self.on_truth, 20)
        self.create_subscription(Path, self.get_parameter("plan_topic").value,
                                 self.on_plan, plan_qos)
        self.create_subscription(Twist, self.get_parameter("cmd_topic").value,
                                 self.on_cmd, 20)
        self.create_subscription(String, self.get_parameter("status_topic").value,
                                 self.on_status, 10)
        rate = max(5.0, float(self.get_parameter("sample_rate").value))
        self.create_timer(1.0 / rate, self.sample)
        self.get_logger().info("实验数据将写入 %s" % out)

    def on_odom(self, msg):
        self.odom = pose_from_odom(msg)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_history.append((stamp, self.odom))

    def on_truth(self, msg):
        self.truth = pose_from_odom(msg)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.truth_history.append((stamp, self.truth))

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)

    def on_status(self, msg):
        self.status = msg.data

    def on_plan(self, msg):
        self.plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not self.plan_saved and self.plan:
            with open(self.plan_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["x", "y"])
                for x, y in self.plan:
                    writer.writerow([round(x, 4), round(y, 4)])
            self.plan_saved = True

    def sample(self):
        if self.odom is None:
            return
        t = time.time() - self.started
        if self.truth_history:
            # 里程计和真值来自不同回调，直接用“最新值”会把一帧的时间差算成漂移，
            # 这里按消息时间戳配对，误差不超过一个发布周期。
            truth_stamp, truth = self.truth_history[-1]
            odom = min(self.odom_history,
                       key=lambda item: abs(item[0] - truth_stamp))[1] \
                if self.odom_history else self.odom
        else:
            truth, odom = self.odom, self.odom
        if self.plan:
            ref_distance = point_to_polyline_distance(truth[:2], self.plan)
            odom_distance = point_to_polyline_distance(odom[:2], self.plan)
            nearest_index = min(
                range(len(self.plan)),
                key=lambda i: (self.plan[i][0] - truth[0]) ** 2 + (self.plan[i][1] - truth[1]) ** 2,
            )
            ref = self.plan[nearest_index]
            progress = nearest_index
        else:
            ref_distance = float("nan")
            odom_distance = float("nan")
            ref = (float("nan"), float("nan"))
            progress = -1
        self.traj_writer.writerow([
            round(t, 3),
            round(odom[0], 4), round(odom[1], 4), round(odom[2], 4),
            round(truth[0], 4), round(truth[1], 4), round(truth[2], 4),
            round(self.cmd[0], 4), round(self.cmd[1], 4),
            round(ref[0], 4), round(ref[1], 4), round(ref_distance, 4),
            round(odom_distance, 4), progress,
        ])
        self.rows += 1
        if self.rows % 10 == 0:
            self.traj_file.flush()

    def finish(self):
        self.traj_file.flush()
        self.traj_file.close()
        info = {
            "status": self.status,
            "rows": self.rows,
            "duration_s": round(time.time() - self.started, 2),
            "plan_points": len(self.plan),
        }
        with open(self.info_path, "w", encoding="utf-8") as f:
            json.dump(info, f, ensure_ascii=False, indent=2)


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
