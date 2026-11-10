#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 1 航点任务飞行前校验节点。"""

import json
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from preflight_validator import validate_mission_file, write_report


def _color(marker, rgba):
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba


def _base_marker(frame_id, marker_type, namespace, marker_id, rgba, scale):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x, marker.scale.y, marker.scale.z = scale
    _color(marker, rgba)
    return marker


def build_markers(report, frame_id):
    """把报告转换为 RViz MarkerArray。"""
    markers = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    markers.markers.append(clear)

    next_id = 1
    for segment in report["segments"]:
        status = segment["status"]
        rgba = (0.15, 0.85, 0.25, 1.0) if status == "VALID" else (0.95, 0.15, 0.15, 1.0)
        marker = _base_marker(
            frame_id, Marker.LINE_STRIP, "preflight_path", next_id, rgba,
            (0.045, 0.045, 0.045),
        )
        marker.points = [
            Point(x=segment["start"][0], y=segment["start"][1], z=segment["start"][2]),
            Point(x=segment["end"][0], y=segment["end"][1], z=segment["end"][2]),
        ]
        markers.markers.append(marker)
        next_id += 1

    for waypoint in report["waypoints"]:
        if "point" not in waypoint:
            continue
        if waypoint["status"] == "VALID":
            rgba = (0.15, 0.85, 0.25, 1.0)
        elif waypoint["status"] == "WARNING":
            rgba = (1.0, 0.72, 0.05, 1.0)
        else:
            rgba = (0.95, 0.15, 0.15, 1.0)
        marker = _base_marker(
            frame_id, Marker.SPHERE, "preflight_waypoints", next_id, rgba,
            (0.20, 0.20, 0.20),
        )
        marker.pose.position = Point(
            x=waypoint["point"][0],
            y=waypoint["point"][1],
            z=waypoint["point"][2],
        )
        markers.markers.append(marker)
        next_id += 1

        text = _base_marker(
            frame_id, Marker.TEXT_VIEW_FACING, "preflight_labels", next_id, rgba,
            (0.0, 0.0, 0.20),
        )
        text.pose.position = Point(
            x=waypoint["point"][0],
            y=waypoint["point"][1],
            z=waypoint["point"][2] + 0.25,
        )
        text.text = "WP%d" % waypoint["index"]
        markers.markers.append(text)
        next_id += 1
    return markers


def main():
    rospy.init_node("uav_waypoint_preflight_validator", anonymous=False)
    mission_file = rospy.get_param("~mission_file", "")
    output_json = rospy.get_param("~output_json", "")
    marker_frame = rospy.get_param("~marker_frame", "world")
    publish_markers = bool(rospy.get_param("~publish_markers", True))
    exit_after_publish = bool(rospy.get_param("~exit_after_publish", True))

    status_pub = rospy.Publisher(
        rospy.get_param("~status_topic", "/uav_waypoint_preflight/status"),
        String,
        queue_size=1,
        latch=True,
    )
    valid_pub = rospy.Publisher(
        rospy.get_param("~valid_topic", "/uav_waypoint_preflight/valid"),
        Bool,
        queue_size=1,
        latch=True,
    )
    markers_pub = rospy.Publisher(
        rospy.get_param("~markers_topic", "/uav_waypoint_preflight/markers"),
        MarkerArray,
        queue_size=1,
        latch=True,
    )

    report = validate_mission_file(mission_file)
    status_pub.publish(String(data=json.dumps(report, ensure_ascii=False)))
    valid_pub.publish(Bool(data=bool(report["valid"])))
    if publish_markers:
        markers_pub.publish(build_markers(report, marker_frame))

    if output_json:
        try:
            write_report(os.path.expanduser(output_json), report)
        except OSError as exc:
            rospy.logwarn("飞行前报告写入失败: %s", exc)

    for issue in report["errors"]:
        rospy.logerr("%s: %s", issue["code"], issue["message"])
    for issue in report["warnings"]:
        rospy.logwarn("%s: %s", issue["code"], issue["message"])
    rospy.loginfo(
        "航点校验完成: status=%s waypoints=%d distance=%.2f m duration=%.2f s",
        report["status"],
        report["summary"]["waypoint_count"],
        report["summary"]["total_distance_m"],
        report["summary"]["estimated_duration_s"],
    )

    rospy.sleep(0.2)
    if exit_after_publish:
        rospy.signal_shutdown("validation published")
        return 0 if report["valid"] else 2
    else:
        rospy.spin()
        return 0


if __name__ == "__main__":
    sys.exit(main())
