#!/usr/bin/env python3
"""Mbot 激光雷达阈值避障节点（ROS 1 Noetic + Gazebo 11）。

订阅 ``/scan``，按扇区最小距离 + 滞回状态机向 ``/cmd_vel`` 发布速度指令：

- 巡航 ``cruise``：前方安全 -> 直行；侧向过近时朝更宽一侧微调
- 避障 ``avoid``：前方进入 ``safe_distance`` -> 减速并朝更宽一侧转向
- 脱困 ``escape``：前方进入 ``danger_distance`` -> 倒车并朝更宽一侧转；
  后方也被堵时改为原地旋转，避免在墙角/死角停死

扇区边界按 ``angle_min`` / ``angle_increment`` 实时换算成角度，
不假定束数与 ``angle_min``，因此 360 束、720 束以及 ``angle_min = -π``
的雷达均可正常工作。

全部参数在节点启动时通过 rosparam 读取（见 ``config/lidar_avoid.yaml``）；
修改后需重启节点（或重新 roslaunch）才生效，本节点不支持运行时动态重配置。

超时保护使用**墙钟**（``time.time()``）+ 独立线程，与 ``/clock`` 解耦：
即使 Gazebo 暂停物理、仿真时钟停走或 gzserver 崩溃，节点仍会在
``scan_timeout`` 秒后发布零速度，不会让最后一条指令滞留。
"""

import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# 默认参数；launch/rosparam 未提供时使用这些值
DEFAULT_PARAMS = {
    'cruise_speed': 0.30,          # 巡航线速度 (m/s)
    'avoid_speed': 0.10,           # 避障线速度 (m/s)
    'reverse_speed': -0.12,        # 脱困倒车速度 (m/s)
    'turn_rate': 1.20,             # 避障转向角速度 (rad/s)
    'rotate_rate': 0.90,           # 脱困旋转角速度 (rad/s)
    'safe_distance': 0.80,         # 进入避障阈值 (m)
    'exit_distance': 1.00,         # 退出避障阈值 (m)，与进入阈值形成滞回
    'danger_distance': 0.30,       # 进入脱困阈值 (m)
    'danger_exit_distance': 0.50,  # 退出脱困阈值 (m)
    'side_distance': 0.45,         # 侧向障碍影响转向的阈值 (m)
    'rear_block_distance': 0.35,   # 后方受阻阈值，决定倒车还是原地旋转 (m)
    'front_half_deg': 20.0,        # 前扇区半角 (°)
    'side_deg': 60.0,              # 侧扇区外边界 (°)
    'rear_deg': 150.0,             # 后扇区外边界 (°)
    'scan_timeout': 1.5,           # /scan 超时停车的秒数
    'range_max': 3.5,              # 距离上限裁剪 (m)
    'control_rate': 10.0,          # watchdog 频率 (Hz)
}


class LidarAvoidNode:
    """ROS 1 激光雷达避障节点。"""

    def __init__(self):
        rospy.init_node('mbot_lidar_avoid_node', anonymous=False)

        # —— 参数全部走 rosparam，未配置时回落到默认值 ——
        cfg = {}
        for key, default in DEFAULT_PARAMS.items():
            cfg[key] = float(rospy.get_param('~' + key, default))
        self.cfg = cfg

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self._scan_callback)

        self.state = 'cruise'         # cruise / avoid / escape
        self._last_scan_time = None   # 最近一次收到有效 /scan 的墙钟时间

        # 状态机跨两个线程：rospy 订阅线程跑 _scan_callback，本类的 watchdog
        # 线程跑 _tick。两者都会改 self.state 并发布 /cmd_vel，若不加锁，可能
        # 出现 callback 刚把 escape 改成 avoid、watchdog 又 _reset_state() 改回
        # cruise 的交错，使上一帧的脱困指令被下一帧的巡航指令覆盖。
        # 用 RLock（可重入）：callback 内部还会再调用 _decide / _reset_state。
        self._state_lock = threading.RLock()

        # 墙钟 watchdog：与 /clock 无关，仿真暂停/时钟停走时仍能触发停车
        self._stop_event = threading.Event()
        self._watchdog_thread = None

        # Ctrl+C / rosnode kill 时保证停车
        rospy.on_shutdown(self._on_shutdown)

    # ------------------------------------------------------------------
    # 基础工具
    # ------------------------------------------------------------------
    def start(self):
        """启动墙钟 watchdog 线程（不依赖 rospy.Timer，因此不受仿真时钟影响）。"""
        if self._watchdog_thread is not None:
            return
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, name='scan_watchdog')
        self._watchdog_thread.daemon = True
        self._watchdog_thread.start()

    def _watchdog_loop(self):
        period = 1.0 / max(self.cfg['control_rate'], 1.0)
        while not self._stop_event.is_set():
            # 用 Event.wait 而不是 rospy.sleep：后者受 /clock 影响
            if self._stop_event.wait(period):
                return
            if rospy.is_shutdown():
                return
            self._tick()

    @staticmethod
    def _zero_twist():
        t = Twist()
        t.linear.x = 0.0
        t.linear.y = 0.0
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = 0.0
        return t

    def _publish_zero(self):
        self.pub.publish(self._zero_twist())

    def _reset_state(self):
        with self._state_lock:
            self.state = 'cruise'

    # ------------------------------------------------------------------
    # 扇区计算（不依赖固定束数与 angle_min）
    # ------------------------------------------------------------------
    def _sector_mins(self, msg):
        """按角度把扫描点归入前 / 左 / 右 / 左后 / 右后五个扇区。

        返回各扇区最小合法距离（无数据记 range_max），以及合法点数量。
        数据不可用（空、angle_increment 非法）时返回 None。
        """
        ranges = msg.ranges
        if ranges is None or len(ranges) == 0:
            return None

        inc = msg.angle_increment
        if not math.isfinite(inc) or inc <= 0.0:
            return None

        cap = self.cfg['range_max']
        buckets = {k: [] for k in
                   ('front', 'left', 'right', 'rear_left', 'rear_right')}
        valid_total = 0

        for i, value in enumerate(ranges):
            if value is None:
                continue
            try:
                value = float(value)
            except (TypeError, ValueError):
                continue
            # NaN 与非正值（含 -inf）视为无效点；+inf 是“无回波”，
            # 按 range_max 处理，否则空旷场地会整帧被判成数据不可用而一直停车
            if math.isnan(value) or value <= 0.0:
                continue
            if math.isinf(value):
                value = cap
            valid_total += 1

            # 用索引换算真实角度，天然兼容任意束数与任意 angle_min
            angle = msg.angle_min + i * inc
            # 归一化到 [-π, π]，避免多圈角度溢出
            angle = math.atan2(math.sin(angle), math.cos(angle))
            deg = abs(math.degrees(angle))

            if deg <= self.cfg['front_half_deg']:
                buckets['front'].append(value)
            elif deg <= self.cfg['side_deg']:
                if angle > 0.0:
                    buckets['left'].append(value)
                else:
                    buckets['right'].append(value)
            elif deg <= self.cfg['rear_deg']:
                if angle > 0.0:
                    buckets['rear_left'].append(value)
                else:
                    buckets['rear_right'].append(value)

        result = {}
        for name, values in buckets.items():
            result[name] = min(min(values), cap) if values else cap
        result['valid_count'] = valid_total
        return result

    # ------------------------------------------------------------------
    # 扫描回调
    # ------------------------------------------------------------------
    def _scan_callback(self, msg):
        with self._state_lock:
            sec = self._sector_mins(msg)

            if sec is None or sec['valid_count'] == 0:
                rospy.logwarn_throttle(
                    2.0,
                    '[mbot_gazebo_lidar_avoid] 扫描数据不可用'
                    '（空 / angle_increment 非法 / 全部 NaN 或非正值），停车保护')
                self._last_scan_time = None
                self._reset_state()
                self._publish_zero()
                return

            self._last_scan_time = time.time()
            self.pub.publish(self._decide(
                front=sec['front'],
                left=sec['left'],
                right=sec['right'],
                rear_left=sec['rear_left'],
                rear_right=sec['rear_right']))

    # ------------------------------------------------------------------
    # 决策（带滞回与脱困分支）
    # ------------------------------------------------------------------
    def _decide(self, front, left, right, rear_left, rear_right):
        cfg = self.cfg
        twist = Twist()

        with self._state_lock:
            # 脱困状态：前方恢复到 danger_exit_distance 以上才退出
            if self.state == 'escape':
                if front > cfg['danger_exit_distance']:
                    self.state = 'avoid'
                else:
                    return self._escape_twist(left, right, rear_left, rear_right)

            # 进入脱困：前方过近
            if front < cfg['danger_distance']:
                self.state = 'escape'
                return self._escape_twist(left, right, rear_left, rear_right)

            # 滞回：avoid -> cruise 需要前方大于 exit_distance，
            # 避免在阈值附近反复切换造成转向抖动
            if self.state == 'avoid':
                if front > cfg['exit_distance']:
                    self.state = 'cruise'
            elif front < cfg['safe_distance']:
                self.state = 'avoid'

            if self.state == 'avoid':
                twist.linear.x = cfg['avoid_speed']
                # 朝空间更宽的一侧转向；若某侧过近则强制避开该侧
                if left >= cfg['side_distance'] and right < cfg['side_distance']:
                    twist.angular.z = cfg['turn_rate']
                elif right >= cfg['side_distance'] and left < cfg['side_distance']:
                    twist.angular.z = -cfg['turn_rate']
                else:
                    twist.angular.z = (
                        cfg['turn_rate'] if left >= right else -cfg['turn_rate'])
                return twist

        # 巡航：前方安全
        twist.linear.x = cfg['cruise_speed']
        twist.angular.z = 0.0

        # 侧向贴得太近时轻微转向让开，避免擦碰侧向障碍
        if left < cfg['side_distance'] and right > left:
            twist.angular.z = -cfg['rotate_rate']
        elif right < cfg['side_distance'] and left > right:
            twist.angular.z = cfg['rotate_rate']
        return twist

    def _escape_twist(self, left, right, rear_left, rear_right):
        """脱困：优先倒车并转向更宽侧；后方被堵则原地旋转。"""
        cfg = self.cfg
        twist = Twist()
        block = cfg['rear_block_distance']

        if rear_left < block and rear_right < block:
            # 后方两侧都堵：原地旋转，朝前后合计空间更大的一侧
            twist.linear.x = 0.0
            if (left + rear_left) >= (right + rear_right):
                twist.angular.z = cfg['rotate_rate']
            else:
                twist.angular.z = -cfg['rotate_rate']
            return twist

        # 正常脱困：倒车，同时转向远离更近的一侧
        twist.linear.x = cfg['reverse_speed']
        if left < cfg['side_distance']:
            twist.angular.z = -cfg['rotate_rate']      # 左侧近 -> 向右转
        elif right < cfg['side_distance']:
            twist.angular.z = cfg['rotate_rate']       # 右侧近 -> 向左转
        else:
            twist.angular.z = 0.0
        return twist

    # ------------------------------------------------------------------
    # 超时与退出保护
    # ------------------------------------------------------------------
    def _tick(self):
        """watchdog：从未收到扫描或超时未更新 -> 停车（按墙钟计时）。"""
        with self._state_lock:
            if self._last_scan_time is None:
                self._publish_zero()
                return
            elapsed = time.time() - self._last_scan_time
            if elapsed > self.cfg['scan_timeout']:
                rospy.logwarn_throttle(
                    2.0,
                    '[mbot_gazebo_lidar_avoid] %.1fs 未收到 /scan，停车保护', elapsed)
                self._reset_state()
                self._publish_zero()

    def _on_shutdown(self):
        rospy.loginfo('[mbot_gazebo_lidar_avoid] 节点退出，发布零速度')
        self._stop_event.set()
        try:
            with self._state_lock:
                self._publish_zero()
        except Exception:
            pass  # 退出时 Publisher 可能已失效，忽略


def main():
    node = LidarAvoidNode()
    node.start()
    rospy.spin()


if __name__ == '__main__':
    main()
