"""ROS message helpers shared by the nodes."""

from __future__ import annotations

import math
from typing import Iterable, Optional, Sequence

import numpy as np

from geometry_msgs.msg import (Pose, PoseStamped, Quaternion, TransformStamped,
                               Twist)
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage


def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quat_to_yaw(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def header(frame_id: str, stamp) -> Header:
    h = Header()
    h.stamp = stamp
    h.frame_id = frame_id
    return h


def pose_stamped(pose: Sequence[float], frame_id: str = "map", stamp=None) -> PoseStamped:
    msg = PoseStamped()
    msg.header = header(frame_id, stamp)
    msg.pose.position.x = float(pose[0])
    msg.pose.position.y = float(pose[1])
    msg.pose.position.z = 0.0
    msg.pose.orientation = quat_from_yaw(float(pose[2]))
    return msg


def odometry(pose: Sequence[float], v: float, w: float, stamp,
             frame_id: str = "odom", child: str = "base_link") -> Odometry:
    msg = Odometry()
    msg.header = header(frame_id, stamp)
    msg.child_frame_id = child
    msg.pose.pose.position.x = float(pose[0])
    msg.pose.pose.position.y = float(pose[1])
    msg.pose.pose.orientation = quat_from_yaw(float(pose[2]))
    msg.twist.twist.linear.x = float(v)
    msg.twist.twist.angular.z = float(w)
    for i in (0, 7, 14, 21, 28, 35):
        msg.pose.covariance[i] = 0.02
        msg.twist.covariance[i] = 0.02
    return msg


def laser_scan(ranges: Sequence[float], stamp, frame_id: str = "laser",
               angle_min: float = 0.0, angle_increment: float = 2.0 * math.pi / 360.0,
               range_min: float = 0.08, range_max: float = 9.0) -> LaserScan:
    msg = LaserScan()
    msg.header = header(frame_id, stamp)
    msg.angle_min = float(angle_min)
    msg.angle_increment = float(angle_increment)
    msg.angle_max = float(angle_min + (len(ranges) - 1) * angle_increment)
    msg.time_increment = 0.0
    msg.scan_time = 0.1
    msg.range_min = float(range_min)
    msg.range_max = float(range_max)
    msg.ranges = [float(r) for r in ranges]
    return msg


def image(msg_rgb: np.ndarray, stamp, frame_id: str) -> Image:
    arr = np.ascontiguousarray(msg_rgb.astype(np.uint8))
    msg = Image()
    msg.header = header(frame_id, stamp)
    msg.height, msg.width = arr.shape[:2]
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = arr.tobytes()
    return msg


def depth_image(depth: np.ndarray, stamp, frame_id: str) -> Image:
    arr = np.ascontiguousarray(depth.astype(np.float32))
    msg = Image()
    msg.header = header(frame_id, stamp)
    msg.height, msg.width = arr.shape[:2]
    msg.encoding = "32FC1"
    msg.is_bigendian = 0
    msg.step = msg.width * 4
    msg.data = arr.tobytes()
    return msg


def occupancy_grid(data: np.ndarray, resolution: float, stamp,
                   frame_id: str = "map") -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header = header(frame_id, stamp)
    msg.info.resolution = float(resolution)
    msg.info.width = int(data.shape[1])
    msg.info.height = int(data.shape[0])
    msg.info.origin.position.x = 0.0
    msg.info.origin.position.y = 0.0
    msg.info.origin.orientation.w = 1.0
    msg.data = np.asarray(data, dtype=np.int8).ravel().tolist()
    return msg


def path(points: Iterable[Sequence[float]], stamp, frame_id: str = "map") -> Path:
    msg = Path()
    msg.header = header(frame_id, stamp)
    for p in points:
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = float(p[0])
        ps.pose.position.y = float(p[1])
        ps.pose.orientation = quat_from_yaw(float(p[2]) if len(p) > 2 else 0.0)
        msg.poses.append(ps)
    return msg


def transform(parent: str, child: str, x: float, y: float, yaw: float, stamp,
              z: float = 0.0) -> TransformStamped:
    t = TransformStamped()
    t.header = header(parent, stamp)
    t.child_frame_id = child
    t.transform.translation.x = float(x)
    t.transform.translation.y = float(y)
    t.transform.translation.z = float(z)
    t.transform.rotation = quat_from_yaw(float(yaw))
    return t


def tf_message(transforms: Sequence[TransformStamped]) -> TFMessage:
    return TFMessage(transforms=list(transforms))


def imu_message(yaw: float, yaw_rate: float, stamp,
                frame_id: str = "base_link") -> Imu:
    msg = Imu()
    msg.header = header(frame_id, stamp)
    msg.orientation = quat_from_yaw(yaw)
    msg.angular_velocity.z = float(yaw_rate)
    return msg
