#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 CARLA 交通遥测 ``live.json`` 转成世界系车辆流话题.

配套 ``openhutb_yolo_mlp_control/tools/traffic``（车辆生成 / 记录工具）：
``main.py record ...`` 持续写 ``live.json``，本节点把它转换成
``/traffic/vehicles`` (std_msgs/String, JSON)，供 ``traffic_nav_node`` 使用。

坐标对齐：``calibration.json`` 里的 ``offset`` 把 CARLA 世界坐标平移到
AirSim/ENU 坐标（``x + offset[0], y + offset[1]``），与跟随工具的标定一致。

安全性：读文件遇到 Windows 共享冲突（PermissionError）会重试；
数据超过 ``max_age_s`` 视为过期，发布空列表而不是让策略用过时车辆位置。
"""
from __future__ import annotations

import json
import os
import sys

import rospy
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from feed_utils import load_offset, read_live, to_vehicles  # noqa: E402


def main():
    rospy.init_node("traffic_feed", anonymous=False)
    live_path = rospy.get_param("traffic_feed/live_json", "")
    calibration = rospy.get_param("traffic_feed/calibration_json", "")
    topic = rospy.get_param("traffic_feed/topic", "/traffic/vehicles")
    rate_hz = float(rospy.get_param("traffic_feed/rate_hz", 10.0))
    max_age = float(rospy.get_param("traffic_feed/max_age_s", 1.0))
    if not live_path:
        rospy.logerr("traffic_feed: 必须设置 traffic_feed/live_json")
        return
    if not os.path.isfile(live_path):
        rospy.logwarn("traffic_feed: live.json 还不存在：%s", live_path)
    offset = load_offset(calibration)
    rospy.loginfo("traffic_feed: %s -> %s，offset=(%.2f, %.2f)",
                  live_path, topic, offset[0], offset[1])

    pub = rospy.Publisher(topic, String, queue_size=1)
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        live = read_live(live_path)
        vehicles = to_vehicles(live, offset, max_age)
        pub.publish(String(data=json.dumps(vehicles)))
        rate.sleep()


if __name__ == "__main__":
    main()
