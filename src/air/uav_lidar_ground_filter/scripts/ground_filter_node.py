#!/usr/bin/env python3
"""Split a world-frame LiDAR PointCloud2 into ground and obstacle clouds."""
from __future__ import annotations

import os
import sys

import numpy as np
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ground_filter import fit_plane, plane_metrics, split_cloud  # noqa: E402


def make_cloud(points, frame_id, stamp, seq=0):
    fields = [PointField("x", 0, PointField.FLOAT32, 1),
              PointField("y", 4, PointField.FLOAT32, 1),
              PointField("z", 8, PointField.FLOAT32, 1)]
    msg = point_cloud2.create_cloud(PointCloud2().header, fields,
                                    [tuple(map(float, p)) for p in points])
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.header.seq = seq
    return msg


class GroundFilterNode(object):
    def __init__(self):
        self.threshold = float(rospy.get_param("~distance_threshold", 0.2))
        self.iterations = int(rospy.get_param("~max_iterations", 120))
        self.min_range = float(rospy.get_param("~min_range", 0.5))
        self.max_range = float(rospy.get_param("~max_range", 60.0))
        self.pub_ground = rospy.Publisher("/lidar/ground", PointCloud2, queue_size=2)
        self.pub_obstacles = rospy.Publisher("/lidar/obstacles", PointCloud2, queue_size=2)
        rospy.Subscriber("/lidar/points", PointCloud2, self.on_cloud, queue_size=1)
        rospy.loginfo("ground_filter: threshold=%.2f range=[%.1f, %.1f]",
                      self.threshold, self.min_range, self.max_range)

    def on_cloud(self, msg):
        pts = np.asarray(list(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float64)
        if len(pts) < 3:
            return
        r = np.linalg.norm(pts[:, :2], axis=1)
        pts = pts[(r >= self.min_range) & (r <= self.max_range)]
        if len(pts) < 3:
            return
        normal, d = fit_plane(pts, self.threshold, self.iterations)
        ground_mask, obstacle_mask = split_cloud(pts, normal, d, self.threshold)
        frame = msg.header.frame_id or "world"
        stamp = msg.header.stamp
        self.pub_ground.publish(make_cloud(pts[ground_mask], frame, stamp, msg.header.seq))
        self.pub_obstacles.publish(make_cloud(pts[obstacle_mask], frame, stamp,
                                              msg.header.seq))
        m = plane_metrics(pts, ground_mask)
        rospy.loginfo_throttle(
            2.0, "ground_filter: %d pts, ground %.1f%%, plane n=[%.2f %.2f %.2f]",
            m["points"], 100.0 * m["ground_ratio"], normal[0], normal[1], normal[2])


def main():
    rospy.init_node("uav_lidar_ground_filter", anonymous=False)
    GroundFilterNode()
    rospy.spin()


if __name__ == "__main__":
    main()
