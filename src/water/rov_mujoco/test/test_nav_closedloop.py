#!/usr/bin/env python3
"""
导航闭环测试：/map → 全局规划 → 局部控制 → 路径话题
====================================================
验证 autonomous_navigation_node：
  1. 收到 /map 后真正调用全局规划器，生成绕障路径；
  2. 发布 /rov/global_path 与 /rov/local_path；
  3. 避障计数按“事件”边沿触发，而不是每个 50 Hz tick 累加；
  4. 局部控制输出非零并与航点方向一致。

运行：
    ./main.sh --test-nav
"""

import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Path, Odometry  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import Int32  # noqa: E402


def make_map():
    """20m x 20m, 0.05m 栅格；在起点与 WP1 之间放一堵墙，迫使全局规划绕行"""
    w = h = 400
    grid = np.full((h, w), -1, dtype=np.int8)
    ox = oy = -10.0
    res = 0.05

    # 已知自由区域：整个地图
    grid[:, :] = 0

    # 墙: x = -1.8m, y ∈ [-0.9, 0.9]
    wx = int(round((-1.8 - ox) / res))
    y0 = int(round((-0.9 - oy) / res))
    y1 = int(round((0.9 - oy) / res))
    grid[y0:y1, wx - 1:wx + 2] = 100

    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = res
    msg.info.width = w
    msg.info.height = h
    msg.info.origin.position.x = ox
    msg.info.origin.position.y = oy
    msg.info.origin.orientation.w = 1.0
    msg.data = grid.flatten().tolist()
    return msg, grid, (ox, oy, res)


def make_odom(x, y, z, yaw):
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


def make_scan(front_dist):
    msg = LaserScan()
    msg.angle_min = -math.pi / 3.0
    msg.angle_max = math.pi / 3.0
    msg.angle_increment = (2.0 * math.pi / 3.0) / 71.0
    msg.range_min = 0.2
    msg.range_max = 15.0
    msg.ranges = [15.0] * 72
    if front_dist is not None:
        for b in range(32, 40):        # 正前方 8 束
            msg.ranges[b] = front_dist
    return msg


def run_test():
    print("=" * 64)
    print("  导航闭环测试 (map → global path → local path → cmd_vel)")
    print("=" * 64)

    from rov_mujoco.autonomous_navigation_node import AutonomousNavigationNode

    rclpy.init()
    node = AutonomousNavigationNode()

    got = {'global': [], 'local': [], 'avoid': [], 'goal': []}
    node.create_subscription(Path, '/rov/global_path', lambda m: got['global'].append(m), 10)
    node.create_subscription(Path, '/rov/local_path', lambda m: got['local'].append(m), 10)
    node.create_subscription(Int32, '/rov/avoidance_count', lambda m: got['avoid'].append(m), 10)

    map_msg, raw_grid, (ox, oy, res) = make_map()

    # 1. 无里程计时不发控制
    node._navigation_step()
    print(f"[1] 未收到里程计时 has_odom={node.has_odom}")

    # 2. 注入位姿、地图、声呐
    node._odom_callback(make_odom(-2.5, 0.0, -1.5, 0.0))
    node._map_callback(map_msg)
    node._scan_callback(make_scan(None))

    for _ in range(5):
        node._navigation_step()
        rclpy.spin_once(node, timeout_sec=0.01)

    assert got['global'], "未收到 /rov/global_path！"
    gp = got['global'][-1]
    pts = [(p.pose.position.x, p.pose.position.y) for p in gp.poses]
    print(f"[2] 全局路径: {len(pts)} 个点 | 起点={pts[0]} 终点={pts[-1]}")

    start_err = math.hypot(pts[0][0] + 2.5, pts[0][1] - 0.0)
    goal_err = math.hypot(pts[-1][0] - (-1.2), pts[-1][1] - 0.0)
    assert start_err < 0.5, f"全局路径起点误差过大: {start_err:.2f} m"
    assert goal_err < 0.5, f"全局路径终点误差过大: {goal_err:.2f} m"

    # 路径必须绕开 x=-1.8, y∈[-0.9,0.9] 的墙
    min_clear = min(
        abs(x - (-1.8)) if -0.9 <= y <= 0.9 else math.hypot(x + 1.8, y - max(-0.9, min(0.9, y)))
        for x, y in pts
    )
    print(f"[3] 路径距墙体最近距离: {min_clear:.3f} m")
    assert min_clear > 0.25, f"全局路径穿过障碍物！最近距离 {min_clear:.2f} m"

    # 3. 局部路径与速度指令
    assert got['local'], "未收到 /rov/local_path！"
    lp = got['local'][-1]
    lp_pts = [(p.pose.position.x, p.pose.position.y) for p in lp.poses]
    print(f"[4] 局部预测轨迹: {len(lp_pts)} 个点 | 末端={lp_pts[-1]}")
    assert len(lp_pts) >= 5, "局部预测轨迹点数不足"
    assert math.hypot(lp_pts[-1][0] - lp_pts[0][0], lp_pts[-1][1] - lp_pts[0][1]) > 0.05, \
        "局部轨迹没有前进，控制指令可能为零"

    # 4. 避障计数边沿触发
    for _ in range(20):
        node._scan_callback(make_scan(0.45))
        node._navigation_step()
        rclpy.spin_once(node, timeout_sec=0.001)
    count_after_20_ticks = node.core.avoidance_count
    print(f"[5] 连续 20 tick 近障后 avoidance_count = {count_after_20_ticks}")
    assert count_after_20_ticks == 1, \
        f"避障计数应按事件计数 (期望 1)，实际 {count_after_20_ticks}"

    # 脱离避障 >1 s 后再次触发，应计为第 2 次事件
    node._scan_callback(make_scan(None))
    node._navigation_step()
    time.sleep(1.05)
    node._scan_callback(make_scan(0.45))
    node._navigation_step()
    print(f"[6] 第二次避障事件后 avoidance_count = {node.core.avoidance_count}")
    assert node.core.avoidance_count == 2, "第二次避障事件未被正确计数"

    # 5. 控制指令与目标方向一致（目标在 +X 侧，应产生正向前进）
    node._scan_callback(make_scan(None))
    res = node.core.update(node.current_pos, node.current_yaw, node.current_vel,
                           node.sonar_ranges, now=time.monotonic())
    print(f"[7] 局部控制输出: vx={res['vx']:.3f} vy={res['vy']:.3f} wz={res['wz']:.3f}")
    assert res['vx'] > 0.0, "目标在前方时局部策略应给出前进速度"

    print("\n✅ 导航闭环测试全部通过！")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    run_test()
