#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
map_republisher.py -- 把一次性发布的地图变成常驻（latched）话题

为什么需要它
============
`structure_map` 节点的 `/structure_map/global_gridmap` 有两个问题：

1. **不是 latched 话题** —— 点云只在生成时发布一次，之后不再重发。
   任何后启动的订阅者（尤其是 RViz，通常比地图节点晚几秒）都会
   **永久错过**地图，表现为 RViz 的 By topic 列表里看不到地图，
   或者加进来了也是空的。
2. 若用 `/structure_map/change_map` 去"催"它重发，会**换一张新地图**
   并重新随机化障碍比例（见 `structure_map.cpp` 的 `genMapCallback`），
   把地图越堆越密，最终任何航点都不可规划。

本节点把地图"接住"再转发：
    /structure_map/global_gridmap (非 latched, 一次性)
        --> /map/global_gridmap    (latched + 周期重发)
        --> /map/global_gridmap_vis (降采样，供 RViz 流畅显示)

RViz 只需订阅 `/map/global_gridmap` 就能任何时刻看到完整地图，
且地图内容与规划器所见完全一致（同一个点云，不做任何修改）。

另外，本节点启动时会主动发一次 `change_res`（相同分辨率，
只重置不换图），确保能拿到地图 —— 因为地图可能在本节点启动前就发完了。
"""

import sys

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, Header

IN_TOPIC = "/structure_map/global_gridmap"
OUT_TOPIC = "/map/global_gridmap"
OUT_VIS_TOPIC = "/map/global_gridmap_vis"
RES_TOPIC = "/structure_map/change_res"


class MapRepublisher(object):
    def __init__(self):
        # 订阅哪个地图话题（默认与 launch 的 $(arg cloud) 一致）
        self.in_topic = rospy.get_param("~input_topic", IN_TOPIC)
        # 重发周期：RViz 后启动也能很快拿到
        self.period = rospy.get_param("~period", 2.0)
        # 用于催 map 的请求间隔（只发有限次）
        self.nudge_interval = rospy.get_param("~nudge_interval", 1.0)
        self.nudge_max = rospy.get_param("~nudge_max", 3)
        # 是否额外发布降采样版本供 RViz 使用
        self.publish_vis = rospy.get_param("~publish_vis", True)
        self.vis_step = rospy.get_param("~vis_step", 4)

        self.latest = None
        self.stamp = None
        self.sent_vis = False

        self.pub = rospy.Publisher(OUT_TOPIC, PointCloud2,
                                   queue_size=1, latch=True)
        self.pub_vis = rospy.Publisher(OUT_VIS_TOPIC, PointCloud2,
                                       queue_size=1, latch=True)
        self.pub_res = rospy.Publisher(RES_TOPIC, Float32,
                                       queue_size=1, latch=True)

        rospy.Subscriber(self.in_topic, PointCloud2, self.on_cloud, queue_size=1)
        rospy.Timer(rospy.Duration(self.period), self.on_timer)

        rospy.loginfo("[map_repub] %s -> %s (latched)", self.in_topic, OUT_TOPIC)
        self.nudge()

    # ------------------------------------------------------------------
    def nudge(self):
        """发 change_res 让 structure_map 重新发布当前地图。

        用相同分辨率 -> resCallback 走 changeRes + resetMap，
        **不改变障碍内容**，这与 change_map 的"换图"语义有本质区别。
        """
        if self.latest is not None:
            return
        rospy.loginfo("[map_repub] 请求地图重发 (change_res, 相同分辨率)")
        self.pub_res.publish(Float32(data=0.1))

    def on_cloud(self, msg):
        if self.latest is None:
            rospy.loginfo("[map_repub] 收到地图: %dx%d, %d 点"
                          % (msg.width, msg.height, msg.width * msg.height))
        self.latest = msg

    def on_timer(self, _evt):
        # 还没拿到地图就继续催（有上限，避免反复重置）
        if self.latest is None:
            if self.nudge_max > 0:
                self.nudge_max -= 1
                self.nudge()
            return

        # 转发原始点云（latched，新订阅者立刻能拿到）
        self.pub.publish(self.latest)

        # 降采样版本：原始地图在 20x20x5m/0.1m 下约 30 万点，
        # RViz 用 Squares 渲染会明显卡顿。降采样只影响显示，不影响规划。
        if self.publish_vis and not self.sent_vis:
            vis = self.downsample(self.latest, self.vis_step)
            if vis is not None:
                self.pub_vis.publish(vis)
                self.sent_vis = True

    def downsample(self, msg, step):
        """按固定步长抽稀点云，保持字段结构不变。"""
        if step <= 1:
            return msg
        try:
            pts = list(pc2.read_points(msg, field_names=None, skip_nans=True))
        except Exception as e:                       # noqa: BLE001
            rospy.logwarn("[map_repub] 点云解析失败，跳过降采样: %s" % e)
            return None

        names = [f.name for f in msg.fields]
        if not names:
            return None

        out = []
        for i in range(0, len(pts), step):
            p = pts[i]
            out.append(tuple(p[:len(names)]))
        try:
            return pc2.create_cloud(Header(stamp=msg.header.stamp,
                                           frame_id=msg.header.frame_id),
                                    msg.fields, out)
        except Exception as e:                       # noqa: BLE001
            rospy.logwarn("[map_repub] 构造点云失败: %s" % e)
            return None


def main():
    rospy.init_node("map_republisher", anonymous=False)
    MapRepublisher()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:                          # noqa: BLE001
        rospy.logerr("[map_repub] 异常退出: %s" % exc)
        sys.exit(1)
