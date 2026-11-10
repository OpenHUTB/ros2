#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线点云模拟发布器（**不需要模拟器**）

用途：把"点云 → 八叉树 → A* 寻路 → RViz"整条链路单独拎出来验证。
      排障时用它就能分清问题出在：
        · airsim_bridge 取数/坐标转换（用本脚本时不参与，直接排除）
        · pointcloud_to_octomap 建图（本脚本照常考验它）
        · astar_planner 寻路（本脚本照常考验它）
        · RViz 配置与 Fixed Frame（本脚本照常考验它）

场景：20 x 20 x 6 m 的封闭房间（地面 + 四壁），房间内立着若干**满高的方柱**
      （从地面一直顶到天花板，模拟城市里的建筑），其中最大的一根在正中央，
      专门挡住"左上角 → 右下角"的对角直线。**路径只能左右绕行** ——
      城市里的建筑是实体，不存在"从下方穿过"这回事。

**为什么传感器要巡游**：激光雷达是单视角的，障碍物背后会留下"阴影锥" ——
那些区域永远扫不到，在地图里是"未知"。而规划器的策略是**未知一律当障碍**，
所以传感器固定不动的话，从角落到角落的路径根本规划不出来。

因此本脚本让传感器依次驻留房间的四个角（各 10 秒），
地图会被从四个视角填满，阴影基本消失。

用法：
    rosrun octree_uav_3d_pathfinding mock_cloud_publisher.py

然后另开终端：
    roslaunch octree_uav_3d_pathfinding main.launch bridge:=false check:=false body_frame:=lidar_link
    # 等约 40 秒让地图从四个角都扫一遍，再下发目标点：
    rostopic pub -1 /uav/goal geometry_msgs/Point "{x: 7.5, y: 7.5, z: 0.0}"

**期望结果**：RViz 里出现彩色的空心房间和几根障碍柱，
绿色路径绕开障碍到达目标点。
"""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


ROOM_L = 20.0     # 房间长（x）
ROOM_W = 20.0     # 房间宽（y）
ROOM_H = 6.0      # 房间高（z）
STEP = 0.2        # 房间表面的采样间距（m）

# 采样间距必须**明显小于八叉树分辨率**，原因和障碍物一样：
# 采样点之间的缝如果比体素还大，画出来的地面/墙面就是一张"有洞的网"，
# 视觉上像是有缺口，地图上也确实会缺少占用体素。
#
# 取 0.2 m，并且建图节点的 point_stride 用 1（见 config/params.yaml）——
# 等效间距就是 0.2 m，稳定小于 0.3 m 的分辨率。
# （用 stride=2 的话等效 0.4 m 就超了，不行。）

# 障碍物表面的采样间距同样要小于分辨率。
#
# 原因：射线是从一个点向外射的，采样点之间留有缝。如果缝比体素还大，
# 射线就会从缝里钻进去、穿到障碍物对面，把**障碍物内部也标成空闲** ——
# 于是规划器会认为可以直穿过去，画出一条"穿墙"的路径。
#
# 和房间表面保持一致，都用 0.2 m（配合 point_stride=1 使用）。
BOX_STEP = 0.2

# 障碍：**从地面一直顶到天花板的柱子和墙**，(中心 x, 中心 y, 沿 x 半宽, 沿 y 半宽)
#
# 为什么做成满高：城市里的建筑是从地面长到顶的实体，**不存在"从建筑下方穿过"**。
# 悬空的障碍会让路径从下面钻过去，虽然算法上没错，但不符合真实场景。
#
# 为什么要有墙：房间太空旷时，octomap 会把大片连通的自由空间**塌缩成一个粗节点**，
# 起点和终点一旦落进同一个节点，规划就退化成"平凡解"（代价 0），演示不出寻路能力。
# 用错开的墙把空间切成走廊，既避免这个问题，又让路径走出明显的 S 形。
#
# 布局意图（起点 (-7.5, 7.5) → 终点 (7.5, -7.5)）：
#   墙 1 在 y=+4，x ∈ [-10, 2]   → 左端**顶到房间左墙**，只能从右侧缺口过去
#   墙 2 在 y=-4，x ∈ [2, 10]    → 右端**顶到房间右墙**，只能从左侧缺口回来
#   于是路径必然形成一个 S 形
#
# ⚠️ 墙必须**和房间的墙连在一起**（不留缝）。否则路径会从墙根的缝里绕过去，
#    根本不走中间的缺口，演示效果就没了。
OBSTACLES = [
    (-4.0, 4.0, 6.0, 0.5),      # 墙 1：x ∈ [-10, 2]，左端与房间左墙相接
    (6.0, -4.0, 4.0, 0.5),      # 墙 2：x ∈ [2, 10]，右端与房间右墙相接
    (0.0, 0.0, 1.5, 1.5),       # 中间柱子
    (-4.0, -1.0, 1.0, 1.0),     # 把通道挤窄的柱子
    (4.0, 1.0, 1.0, 1.0),
]

# 传感器巡游的驻留点：房间四个角。每个点停 tour_hold 秒。
# 四个视角足以把中央柱子背后的阴影基本填掉。
TOUR = [(-7.5, -7.5, 0.0), (7.5, -7.5, 0.0), (7.5, 7.5, 0.0), (-7.5, 7.5, 0.0)]
TOUR_HOLD = 10.0  # 每个驻留点停留秒数（可用参数 ~tour_hold 覆盖）


def build_scene():
    """生成房间表面 + 全部障碍表面的采样点（世界系坐标）。"""
    hx, hy = ROOM_L / 2.0, ROOM_W / 2.0
    z0 = -ROOM_H / 2.0
    z1 = z0 + ROOM_H
    pts = []

    def grid(u0, u1, v0, v1, fn, step=STEP):
        u = u0
        while u <= u1 + 1e-6:
            v = v0
            while v <= v1 + 1e-6:
                pts.append(fn(u, v))
                v += step
            u += step

    # 地面
    grid(-hx, hx, -hy, hy, lambda x, y: (x, y, z0))
    # 四面墙
    grid(-hx, hx, z0, z1, lambda x, z: (x, -hy, z))
    grid(-hx, hx, z0, z1, lambda x, z: (x, hy, z))
    grid(-hy, hy, z0, z1, lambda y, z: (-hx, y, z))
    grid(-hy, hy, z0, z1, lambda y, z: (hx, y, z))

    # 每根柱子的四个侧面，**从地面一直延伸到天花板**。
    # 用 BOX_STEP —— 比房间密得多，否则射线会从采样点的缝里穿过去，
    # 把柱子内部误标成空闲。
    for (cx, cy, hx_, hy_) in OBSTACLES:
        # 四个侧面（竖直方向铺满整个房间高度）
        grid(cx - hx_, cx + hx_, z0, z1, lambda x, z: (x, cy - hy_, z), BOX_STEP)
        grid(cx - hx_, cx + hx_, z0, z1, lambda x, z: (x, cy + hy_, z), BOX_STEP)
        grid(cy - hy_, cy + hy_, z0, z1, lambda y, z: (cx - hx_, y, z), BOX_STEP)
        grid(cy - hy_, cy + hy_, z0, z1, lambda y, z: (cx + hx_, y, z), BOX_STEP)
        # 顶面与天花板齐平；底面与地面齐平（地面本身已经采样过，不必重复）。
        # 补顶面是为了让柱子和天花板之间不留缝。
        grid(cx - hx_, cx + hx_, cy - hy_, cy + hy_,
             lambda x, y: (x, y, z1), BOX_STEP)

    return pts


class MockCloudPublisher(object):
    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'lidar_link')
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.rate = rospy.get_param('~rate', 10.0)
        # 巡游开关：设为 false 时传感器固定在第一个驻留点不动
        self.tour_enabled = rospy.get_param('~tour', True)
        self.tour_hold = rospy.get_param('~tour_hold', TOUR_HOLD)
        # 巡游几轮之后停下（停在最后一个驻留点）。
        # 这样既保证地图被多视角扫过、阴影填掉，又让"起点"稳定在一个角上，
        # 便于做"从一个角到对角"的寻路演示。
        self.tour_cycles = rospy.get_param('~tour_cycles', 1)
        self.start_time = rospy.Time.now()

        # 世界系下的场景点（固定不变的"真值"）
        self.world_points = build_scene()
        rospy.loginfo('离线点云模拟器：生成 %d 个点（房间 %.0fx%.0fx%.0f m + %d 个障碍）',
                      len(self.world_points), ROOM_L, ROOM_W, ROOM_H, len(OBSTACLES))
        rospy.loginfo('传感器巡游：%s，每点停留 %.0f 秒',
                      '开启' if self.tour_enabled else '关闭', self.tour_hold)
        for i, p in enumerate(TOUR):
            rospy.loginfo('  驻留点 %d: (%.1f, %.1f, %.1f)', i, p[0], p[1], p[2])

        self.pub = rospy.Publisher('/cloud_in', PointCloud2, queue_size=1)
        # 传感器会移动，所以用动态 TF 广播器（不是 StaticTransformBroadcaster）
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self.publish_cb)
        rospy.loginfo('已开始发布 /cloud_in（frame_id=%s，%.0f Hz）',
                      self.frame_id, self.rate)
        rospy.loginfo('传感器将巡游 %.0f 秒填满地图，之后**停在最后一个驻留点** '
                      '(%.1f, %.1f, %.1f)',
                      self.tour_hold * len(TOUR) * max(1, self.tour_cycles),
                      TOUR[-1][0], TOUR[-1][1], TOUR[-1][2])
        last = TOUR[-1]
        rospy.loginfo('然后就下发目标点（与最后一个驻留点成对角）：'
                      'rostopic pub -1 /uav/goal geometry_msgs/Point '
                      '"{x: %.1f, y: %.1f, z: %.1f}"',
                      -last[0], -last[1], last[2])

    def current_pose(self, now):
        """返回当前传感器位置（世界系）。"""
        if not self.tour_enabled:
            return TOUR[0]
        total = self.tour_hold * len(TOUR)
        elapsed = (now - self.start_time).to_sec()
        if elapsed >= total * max(1, self.tour_cycles):
            # 巡游结束 —— 停在最后一个驻留点，让起点稳定下来
            return TOUR[-1]
        return TOUR[int((elapsed % total) / self.tour_hold)]

    def publish_cb(self, _event):
        now = rospy.Time.now()
        sx, sy, sz = self.current_pose(now)

        # 世界系场景点 → 传感器坐标系（传感器水平放置，只差一个平移）
        cloud_pts = [(x - sx, y - sy, z - sz) for (x, y, z) in self.world_points]

        header = Header()
        header.stamp = now
        header.frame_id = self.frame_id
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, cloud_pts))

        # 广播 world -> lidar_link：建图节点靠它把点云变换回世界系，
        # 规划节点靠它取起点（所以巡游时起点会跟着变）
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.frame_id
        t.transform.translation.x = sx
        t.transform.translation.y = sy
        t.transform.translation.z = sz
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)


def main():
    rospy.init_node('mock_cloud_publisher')
    MockCloudPublisher()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
