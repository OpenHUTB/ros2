#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
laser_avoid.py —— 基于 2D 激光雷达的自主避障
==============================================

【任务书对应】任务 (2)：
    「实现传感器感知及运动控制，要求使用传感器获取的数据进行感知
      （如：雷达、摄像头、深度相机等）对机器人进行运动控制
      （5*2：感知、给定轨迹导航与运动控制）」

【功能说明】
    订阅激光雷达 /scan 话题，将 360° 扫描数据划分为若干扇形区域，
    基于「人工势场 + 有限状态机」策略实时生成速度指令，
    使无人车在未知环境中自主避障行驶。

【算法原理】

  一、感知层：极坐标扇区划分
      ------------------------------------------------------
      激光雷达返回 N 个距离量测 r[i]，对应角度
          θ[i] = angle_min + i · angle_increment
      剔除非有限值（inf / nan，表示超出量程或无回波）后，
      将 [-π, π) 划分为 5 个扇区，每扇区取最小距离作为该方向的可通行度：

          扇区       角度范围          含义
          ------------------------------------------
          front     [-30°,  30°]      正前方
          left      [ 30°,  90°]      左前方
          right     [-90°, -30°]      右前方
          left_bk   [ 90°, 180°]      左后方
          right_bk  [-180°,-90°]      右后方

      取最小值而非均值，是因为避障决策关心的是「最近威胁」而非平均距离。
      均值会被远处的空旷区域稀释，导致对近处障碍不敏感。

  二、决策层：有限状态机 (FSM)
      ------------------------------------------------------
      根据前方距离 d_front 与安全阈值的关系，状态机在三种行为间切换：

        ┌─────────────┐  d_front > d_safe
        │   CRUISE    │ ─────────────────┐
        │  （巡航）    │                  │
        └─────────────┘                  ▼
                ▲                  ┌───────────┐
                │  障碍消失         │  d_front  │
                └──────────────────│  判定      │
                                   └───────────┘
                                         │ d_front < d_safe
                                         ▼
                                   ┌─────────────┐
                                   │   AVOID     │
                                   │ （转向避障） │
                                   └─────────────┘
                                         │ d_front < d_danger
                                         ▼
                                   ┌─────────────┐
                                   │  EMERGENCY  │
                                   │ （紧急后退） │
                                   └─────────────┘

        CRUISE    : 前方安全，全速直行
        AVOID     : 前方有障碍，减速并转向较空旷一侧
        EMERGENCY : 障碍极近，先停车后退脱困

  三、控制层：势场法转向选择
      ------------------------------------------------------
      转向方向由左右两侧可通行度之差决定：

          Δ = d_left - d_right

      若 Δ > 0，说明左侧更空旷，应左转；反之右转。
      角速度大小与 Δ 成正比（带饱和），使转向平缓自然：

          ω = k_ω · clamp(Δ, -Δ_max, Δ_max) · sign（避障方向）

      同时线速度随前方障碍接近程度线性衰减：

          v = v_cruise · clamp((d_front - d_stop) / (d_safe - d_stop), 0, 1)

      这一设计保证了「越接近障碍开得越慢」，为转向留出时间裕度。

  四、安全层：抖动抑制
      ------------------------------------------------------
      为防止在阈值附近反复切换状态引起转向抖动，引入滞回 (hysteresis)：
      进入 AVOID 的阈值 d_safe 高于退出 AVOID 的阈值 d_clear。
      只有当 d_front > d_clear 时才恢复巡航，避免临界振荡。

【话题接口】
    订阅：/scan         (sensor_msgs/LaserScan)
    发布：/cmd_vel      (geometry_msgs/Twist)

【作者】OpenHUTB Car Team
"""

from __future__ import print_function

import math
import threading

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# ============================================================================
# 算法参数
# ============================================================================

#: 障碍物进入「需避障」状态的距离阈值 (m)
D_SAFE = 1.0

#: 退出避障、恢复巡航的距离阈值 (m)。大于 D_SAFE 以形成滞回，抑制抖动。
D_CLEAR = 1.4

#: 进入「紧急」状态的距离阈值 (m)
D_DANGER = 0.45

#: 巡航线速度 (m/s)
V_CRUISE = 0.35

#: 避障时的最大线速度 (m/s)
V_AVOID = 0.18

#: 紧急后退速度 (m/s)
V_BACK = -0.12

#: 最小角速度（保证转向有效）(rad/s)
W_MIN = 0.45

#: 最大角速度 (rad/s)
W_MAX = 1.3

#: 侧向距离差到角速度的比例增益
K_OMEGA = 1.8

#: 紧急状态持续的最小时长 (s)，避免后退不足即再次前冲
EMERGENCY_HOLD = 0.8

#: 自遮挡过滤半径 (m)。
#: 激光雷达即使安装在车体轮廓外，仍可能扫到车体边缘、天线或线缆，
#: 在某个固定角度上产生恒定的近距离回波。这类「虚假障碍」会让车辆
#: 误判为正前方被堵死而原地不动。
#: 处理策略：将小于该阈值的回波视为可疑，仅在数量超过
#: MIN_CLOSE_HITS 时才认定为真实障碍（真实的近距障碍会覆盖连续多束）。
SELF_OCCLUSION_RANGE = 0.30

#: 判定近距离回波为真实障碍所需的最少连续光束数
MIN_CLOSE_HITS = 5


class CarState(object):
    """有限状态机的状态枚举。"""
    CRUISE = 0     # 巡航
    AVOID = 1      # 避障转向
    EMERGENCY = 2  # 紧急脱困


class LaserAvoidNode(object):
    """激光雷达避障节点。"""

    def __init__(self, cmd_pub, watchdog):
        """
        :param cmd_pub:  /cmd_vel 发布者
        :param watchdog: 安全看门狗
        """
        self._cmd_pub = cmd_pub
        self._watchdog = watchdog

        # 感知数据
        self._scan = None
        self._scan_lock = threading.Lock()

        # 状态机
        self._state = CarState.CRUISE
        self._emergency_since = None

        # 统计信息（用于文档与性能评价）
        self.stats = {
            "scans": 0,
            "avoid_count": 0,
            "emergency_count": 0,
            "min_front": float("inf"),
        }

        # 订阅激光雷达
        self._sub = rospy.Subscriber(
            "/scan", LaserScan, self._scan_callback, queue_size=1
        )

        rospy.loginfo("[laser_avoid] 已订阅 /scan，等待激光雷达数据……")

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def start(self):
        rospy.loginfo("[laser_avoid] 避障节点启动 "
                      "(D_SAFE=%.2fm, D_DANGER=%.2fm)",
                      D_SAFE, D_DANGER)

    def stop(self):
        self._cmd_pub.publish(Twist())
        self._sub.unregister()
        rospy.loginfo("[laser_avoid] 统计：扫描 %d 帧，避障 %d 次，"
                      "紧急 %d 次，最近前方距离 %.2f m",
                      self.stats["scans"],
                      self.stats["avoid_count"],
                      self.stats["emergency_count"],
                      self.stats["min_front"])

    # ------------------------------------------------------------------
    # 感知层
    # ------------------------------------------------------------------

    def _scan_callback(self, msg):
        """缓存最新一帧激光数据。"""
        with self._scan_lock:
            self._scan = msg

    def _extract_sectors(self, scan):
        """将激光扫描划分为 5 个扇区，返回各扇区最小距离。

        :param scan: sensor_msgs/LaserScan
        :return: dict，键为扇区名，值为该扇区最小有效距离 (m)
                 若某扇区全无有效回波，则该方向记为量程上限。
        """
        ranges = scan.ranges
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        range_max = scan.range_max

        # ---- 第一步：逐束有效性过滤，得到 (角度, 距离) 有效量测序列 ----
        valid_beams = []
        for i, r in enumerate(ranges):
            # 1) 非有限值：inf 表示超量程，nan 表示无回波
            if not (r == r) or r == float("inf") or r == float("-inf"):
                continue
            # 2) 小于最小量程：传感器噪声
            if r < scan.range_min:
                continue

            theta = angle_min + i * angle_inc
            # 将角度规范化到 [-π, π)
            while theta >= math.pi:
                theta -= 2.0 * math.pi
            while theta < -math.pi:
                theta += 2.0 * math.pi

            valid_beams.append((theta, r))

        # ---- 第二步：自遮挡过滤 ----
        # 真实障碍会连续遮挡多束激光，而车体自遮挡通常只影响少数几束。
        # 若某束距离过近，检查其邻域内是否也有同样近的回波；
        # 若无，则判定为自遮挡噪声并剔除。
        filtered = []
        n = len(valid_beams)
        for idx, (theta, r) in enumerate(valid_beams):
            if r >= SELF_OCCLUSION_RANGE:
                filtered.append((theta, r))
                continue

            # 统计邻域内同样近的回波数量
            hits = 0
            for off in range(-MIN_CLOSE_HITS, MIN_CLOSE_HITS + 1):
                j = idx + off
                if 0 <= j < n and valid_beams[j][1] < SELF_OCCLUSION_RANGE:
                    hits += 1

            if hits >= MIN_CLOSE_HITS:
                # 连续多束都近，判定为真实障碍，保留
                filtered.append((theta, r))
            # 否则丢弃（自遮挡噪声）

        # ---- 第三步：扇区划分 ----
        sectors = {
            "front":    (-math.radians(30.0),  math.radians(30.0)),
            "left":     (math.radians(30.0),   math.radians(90.0)),
            "right":    (-math.radians(90.0),  -math.radians(30.0)),
            "left_bk":  (math.radians(90.0),   math.pi),
            "right_bk": (-math.pi,             -math.radians(90.0)),
        }

        result = {k: range_max for k in sectors}

        for theta, r in filtered:
            for name, (lo, hi) in sectors.items():
                if lo <= theta < hi:
                    if r < result[name]:
                        result[name] = r
                    break

        return result

    # ------------------------------------------------------------------
    # 决策层
    # ------------------------------------------------------------------

    def _decide(self, sectors):
        """有限状态机：根据扇区距离决定当前状态。

        :return: (state, sectors)
        """
        d_front = sectors["front"]
        now = rospy.Time.now()

        # 记录最小前方距离（用于统计）
        if d_front < self.stats["min_front"]:
            self.stats["min_front"] = d_front

        # --- 紧急状态处理 ---
        if self._state == CarState.EMERGENCY:
            elapsed = (now - self._emergency_since).to_sec()
            if elapsed < EMERGENCY_HOLD:
                # 保持后退，给车辆脱离的时间
                return CarState.EMERGENCY, sectors
            # 后退结束后若前方已清空，恢复巡航，否则转入避障转向
            if d_front > D_CLEAR:
                return CarState.CRUISE, sectors
            return CarState.AVOID, sectors

        # --- 紧急触发 ---
        if d_front < D_DANGER:
            self._state = CarState.EMERGENCY
            self._emergency_since = now
            self.stats["emergency_count"] += 1
            rospy.logwarn("[laser_avoid] 紧急！前方 %.2f m < %.2f m，后退脱困",
                          d_front, D_DANGER)
            return CarState.EMERGENCY, sectors

        # --- 滞回切换 ---
        if self._state == CarState.CRUISE:
            if d_front < D_SAFE:
                self._state = CarState.AVOID
                self.stats["avoid_count"] += 1
                rospy.loginfo("[laser_avoid] 前方 %.2f m 有障碍，开始避障转向",
                              d_front)
        else:  # AVOID
            if d_front > D_CLEAR:
                self._state = CarState.CRUISE
                rospy.loginfo("[laser_avoid] 前方已清空 (%.2f m)，恢复巡航",
                              d_front)

        return self._state, sectors

    # ------------------------------------------------------------------
    # 控制层
    # ------------------------------------------------------------------

    def _compute_command(self, state, sectors):
        """由状态与扇区数据计算速度指令。

        :return: geometry_msgs/Twist
        """
        cmd = Twist()

        if state == CarState.CRUISE:
            # ---- 巡航：全速直行 ----
            cmd.linear.x = V_CRUISE
            cmd.angular.z = 0.0

        elif state == CarState.AVOID:
            # ---- 避障：减速 + 朝空旷侧转向 ----
            d_front = sectors["front"]
            d_left = min(sectors["left"], sectors["left_bk"])
            d_right = min(sectors["right"], sectors["right_bk"])

            # 1) 速度随障碍接近线性衰减
            #    分母保证不为 0；D_SAFE > D_DANGER 恒成立
            ratio = (d_front - D_DANGER) / max(D_SAFE - D_DANGER, 1e-6)
            ratio = max(0.0, min(1.0, ratio))
            cmd.linear.x = V_AVOID * ratio

            # 2) 转向方向：哪边远就往哪边转
            delta = d_left - d_right
            # 归一化到 [-1, 1]，避免远距离处增益过大
            delta_norm = max(-1.0, min(1.0, delta / 2.0))
            omega = K_OMEGA * delta_norm

            # 3) 保证最小转向力度，否则可能贴着障碍面平移
            if abs(omega) < W_MIN:
                omega = W_MIN if delta >= 0 else -W_MIN

            # 4) 若两侧都极窄（死胡同），原地大角度转向
            if max(d_left, d_right) < D_SAFE:
                omega = W_MAX if d_left >= d_right else -W_MAX
                cmd.linear.x = 0.0

            cmd.angular.z = max(-W_MAX, min(omega, W_MAX))

        else:  # EMERGENCY
            # ---- 紧急：后退 + 反向转向 ----
            cmd.linear.x = V_BACK
            d_left = min(sectors["left"], sectors["left_bk"])
            d_right = min(sectors["right"], sectors["right_bk"])
            cmd.angular.z = W_MAX if d_left >= d_right else -W_MAX

        return cmd

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------

    def update(self):
        """主循环调用：感知 → 决策 → 控制。"""
        with self._scan_lock:
            scan = self._scan

        if scan is None:
            # 尚未收到雷达数据，保持静止等待
            self._watchdog.feed(Twist())
            return

        self.stats["scans"] += 1

        # 1) 感知：扇区划分
        sectors = self._extract_sectors(scan)

        # 2) 决策：状态机
        state, sectors = self._decide(sectors)

        # 3) 控制：生成速度指令
        cmd = self._compute_command(state, sectors)

        # 4) 输出（经看门狗统一发布）
        self._watchdog.feed(cmd)
