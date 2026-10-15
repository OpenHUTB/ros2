#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
depth_camera_avoid.py —— 基于 RGB-D 深度相机的视觉避障
========================================================

【任务书对应】任务 (2)：
    「实现传感器感知及运动控制，要求使用传感器获取的数据进行感知
      （如：雷达、摄像头、深度相机等）对机器人进行运动控制
      （5*2：感知、给定轨迹导航与运动控制）」

    本节点使用【深度相机】这一与激光雷达原理完全不同的传感器，
    独立实现第二条感知→控制闭环，满足任务书对不同传感器类型的要求。

【功能说明】
    订阅深度相机的深度图像 (sensor_msgs/Image, 32FC1 编码)，
    在图像上划分若干感兴趣区域 (ROI)，计算各区域的深度统计量，
    据此判断前方是否有障碍并生成转向指令。

【算法原理】

  一、传感器模型
      ------------------------------------------------------
      深度相机通过结构光或双目视差获得每个像素的深度值：

          Z(u, v) = 该像素对应空间点距相机光心的距离

      输出为 640x480 的 32 位浮点图像，单位米。
      无效值（无回波、超出量程）以 0 或 inf 表示，需过滤。

  二、感知层：图像分区与深度统计
      ------------------------------------------------------
      将深度图像在水平方向划分为三个竖直条带（模仿人眼视野划分）：

          ┌──────────┬──────────┬──────────┐
          │          │          │          │
          │   左区    │   中区    │   右区    │
          │  (0~25%) │ (37~62%) │ (75~100%)│
          │          │          │          │
          └──────────┴──────────┴──────────┘
                忽略边缘畸变区，仅取中间 50% 高度

      对每个条带，使用 **第 15 百分位数** 而非最小值或均值：

        - 用最小值：单个噪声像素即可触发误判，鲁棒性差；
        - 用均值：远处空旷区域会稀释近处障碍，导致"看不见"障碍；
        - 用低百分位数：既能反映"最近威胁"，又能容忍少量噪声像素。

      这是本算法与激光雷达版本在感知层的关键差异。

  三、决策层：基于视觉的势场法
      ------------------------------------------------------
          d_mid  = 中区深度  (对应正前方)
          d_left = 左区深度
          d_right= 右区深度

      状态判定与激光版本一致的三态机，但阈值按相机视场角换算：
      相机水平视角 80°，中区覆盖约 ±15°，其量程参考值与雷达一致。

  四、控制层：转向选择
      ------------------------------------------------------
      与激光版本同理，取 Δ = d_left - d_right 决定转向方向，
      线速度随 d_mid 接近程度衰减。

  五、与激光雷达方案的对比（任务书"多算法/多传感器对比"要求）
      ------------------------------------------------------
      ┌──────────┬──────────────────┬──────────────────────┐
      │   维度    │    激光雷达       │     深度相机          │
      ├──────────┼──────────────────┼──────────────────────┤
      │ 数据形式  │ 1D 极坐标距离     │ 2D 稠密深度图         │
      │ 视角      │ 360° 全向        │ 80° 前向              │
      │ 盲区      │ 车体自身遮挡       │ 视场外全部不可见       │
      │ 计算量    │ 低（360 点）      │ 高（30 万像素）        │
      │ 优势      │ 全向、精确测距     │ 可识别障碍形状与纹理    │
      │ 劣势      │ 无法感知低于扫描面  │ 视角窄、强光/反光敏感  │
      │          │ 的障碍（如台阶）   │                      │
      └──────────┴──────────────────┴──────────────────────┘

      在本任务中，两者都实现了完整的感知→控制闭环，
      但激光方案在避障任务上更可靠（全向视角），
      深度相机方案更适合需要"理解"障碍物形状的场景。

【话题接口】
    订阅：/camera/depth/image_raw    (sensor_msgs/Image, 32FC1)
    发布：/cmd_vel                   (geometry_msgs/Twist)

【作者】OpenHUTB Car Team
"""

from __future__ import print_function

import threading

import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


# ============================================================================
# 算法参数
# ============================================================================

#: 中区进入避障的距离阈值 (m)
D_SAFE = 1.2

#: 退出避障的距离阈值 (m)，形成滞回
D_CLEAR = 1.6

#: 紧急状态阈值 (m)
D_DANGER = 0.5

#: 巡航线速度 (m/s)
V_CRUISE = 0.30

#: 避障最大线速度 (m/s)
V_AVOID = 0.15

#: 紧急后退速度 (m/s)
V_BACK = -0.10

#: 最小/最大角速度 (rad/s)
W_MIN = 0.40
W_MAX = 1.2

#: 侧向深度差到角速度的比例增益
K_OMEGA = 1.6

#: 深度统计所用百分位数。
#:
#: 选值依据（实测调参）：
#:   Gazebo 深度相机对「无回波」区域（看向场景外或超出量程的部分）
#:   会输出一个固定的地板值（实测约 1.000 m），该区域在画面中占比可能
#:   超过 40%。若直接对其做百分位统计，结果会被这个假值锁定为常数，
#:   导致算法误判前方恒有障碍。
#:   因此本节点先做地板值剔除（见 _roi_stats），再用百分位数提取
#:   近处威胁。取 25% 可在「对近障碍敏感」与「容忍少量噪声像素」之间平衡。
DEPTH_PERCENTILE = 25.0

#: 深度地板值（无效标记）剔除参数。
#:
#: Gazebo 深度相机输出的无效区域并非 0 或 inf，而是一个固定常数。
#: 检测方法：对有效像素做直方图统计，若某一深度值的像素占比超过
#: FLOOR_VALUE_RATIO，且其方差极小，则判定为无效标记并整体剔除。
FLOOR_VALUE_RATIO = 0.30

#: 直方图分箱宽度 (m)，用于识别地板值
FLOOR_BIN_WIDTH = 0.01

#: 图像分区比例（左区起点/终点，中区起点/终点，右区起点/终点）
ROI_LEFT_START, ROI_LEFT_END = 0.00, 0.25
ROI_MID_START, ROI_MID_END = 0.37, 0.62
ROI_RIGHT_START, ROI_RIGHT_END = 0.75, 1.00

#: 竖直方向取样范围（避开地面与天花板）
ROI_ROW_START, ROI_ROW_END = 0.25, 0.75

#: 紧急状态持续时长 (s)
EMERGENCY_HOLD = 0.8

#: 原地点转的最大持续时长 (s)。
#: 当左右两侧都被判定为阻塞时会原地转向寻找出路；若因传感器异常
#: 导致该状态持续过久，应主动退出并恢复巡航，避免车辆无限打转。
MAX_SPIN_DURATION = 1.5


class CarState(object):
    CRUISE = 0
    AVOID = 1
    EMERGENCY = 2


class DepthCameraAvoidNode(object):
    """深度相机视觉避障节点。"""

    def __init__(self, cmd_pub, watchdog):
        self._cmd_pub = cmd_pub
        self._watchdog = watchdog

        self._depth = None
        self._depth_lock = threading.Lock()

        self._state = CarState.CRUISE
        self._emergency_since = None
        self._spin_since = None

        self.stats = {
            "frames": 0,
            "avoid_count": 0,
            "emergency_count": 0,
            "min_mid": float("inf"),
        }

        self._sub = rospy.Subscriber(
            "/camera/depth/image_raw", Image,
            self._depth_callback, queue_size=1
        )

        rospy.loginfo("[depth_camera] 已订阅 /camera/depth/image_raw，"
                      "等待深度图像……")

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def start(self):
        rospy.loginfo("[depth_camera] 视觉避障节点启动 "
                      "(D_SAFE=%.2fm, 百分位=%.0f%%)",
                      D_SAFE, DEPTH_PERCENTILE)

    def stop(self):
        self._cmd_pub.publish(Twist())
        self._sub.unregister()
        rospy.loginfo("[depth_camera] 统计：处理 %d 帧，避障 %d 次，"
                      "紧急 %d 次，最近中区深度 %.2f m",
                      self.stats["frames"],
                      self.stats["avoid_count"],
                      self.stats["emergency_count"],
                      self.stats["min_mid"])

    # ------------------------------------------------------------------
    # 感知层
    # ------------------------------------------------------------------

    def _depth_callback(self, msg):
        with self._depth_lock:
            self._depth = msg

    def _image_to_array(self, msg):
        """将 sensor_msgs/Image 转换为 numpy 数组。

        支持 32FC1（浮点深度，单位米）与 16UC1（毫米整数，少见）。
        """
        try:
            if msg.encoding == "32FC1":
                dtype = np.dtype(np.float32)
                scale = 1.0
            elif msg.encoding == "16UC1":
                dtype = np.dtype(np.uint16)
                scale = 0.001  # 毫米 → 米
            else:
                rospy.logwarn_throttle(
                    5.0, "[depth_camera] 不支持的编码格式：%s", msg.encoding
                )
                return None

            img = np.frombuffer(msg.data, dtype=dtype)
            img = img.reshape((msg.height, msg.width)).astype(np.float32)
            if scale != 1.0:
                img = img * scale
            return img
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn_throttle(5.0, "[depth_camera] 图像解析失败：%s", exc)
            return None

    def _roi_stats(self, img, col_start, col_end):
        """计算指定竖直条带的深度统计量。

        处理流程：
          1) 裁取感兴趣区域 (ROI)；
          2) 剔除 0 / inf / nan 等明显无效值；
          3) 识别并剔除「深度地板值」——Gazebo 深度相机对无回波区域
             输出的固定常数（实测 1.000 m），该值占比可能很高，
             若不清除会完全主导统计结果；
          4) 对剩余有效像素取低百分位数，作为该方向的通近程度。

        :param img:       (H, W) 深度数组，单位米
        :param col_start: 条带起始列比例 [0,1]
        :param col_end:   条带结束列比例 [0,1]
        :return: (percentile, valid_count)
        """
        h, w = img.shape
        c0 = int(col_start * w)
        c1 = int(col_end * w)
        r0 = int(ROI_ROW_START * h)
        r1 = int(ROI_ROW_END * h)

        patch = img[r0:r1, c0:c1]

        # --- 步骤 2：剔除 0 / inf / nan ---
        valid = patch[np.isfinite(patch) & (patch > 0.01)]
        if valid.size == 0:
            return float("inf"), 0

        # --- 步骤 3：剔除深度地板值 ---
        floor_value = self._detect_floor_value(valid)
        if floor_value is not None:
            # 保留与地板值差异超过半个分箱宽度的像素
            keep = np.abs(valid - floor_value) > (FLOOR_BIN_WIDTH * 0.5)
            filtered = valid[keep]
            if filtered.size >= 30:
                # 剔除后仍留有足够像素，采用剔除结果
                valid = filtered
            # 若剔除后所剩无几，说明画面几乎全是无效区域，
            # 此时视为「前方无有效信息」，交由上层按开阔处理

        if valid.size == 0:
            return float("inf"), 0

        return float(np.percentile(valid, DEPTH_PERCENTILE)), int(valid.size)

    @staticmethod
    def _detect_floor_value(values):
        """检测深度图中的固定地板值（无效标记）。

        :param values: 一维有效深度数组
        :return: 地板值 (float)；若未检测到则返回 None
        """
        if values.size < 100:
            return None

        # 直方图统计
        vmin = float(values.min())
        vmax = float(values.max())
        if vmax - vmin < FLOOR_BIN_WIDTH:
            return None

        n_bins = int((vmax - vmin) / FLOOR_BIN_WIDTH) + 1
        n_bins = min(n_bins, 4096)  # 限制分箱数量，避免内存开销

        hist, edges = np.histogram(values, bins=n_bins)

        peak_idx = int(np.argmax(hist))
        peak_ratio = float(hist[peak_idx]) / float(values.size)

        if peak_ratio >= FLOOR_VALUE_RATIO:
            # 直方图峰值对应深度即为地板值
            return float((edges[peak_idx] + edges[peak_idx + 1]) / 2.0)

        return None

    def _roi_percentile(self, img, col_start, col_end):
        """兼容接口：仅返回百分位数。"""
        value, _ = self._roi_stats(img, col_start, col_end)
        return value

    # ------------------------------------------------------------------
    # 决策层与控制层（与激光版本结构一致，便于对比）
    # ------------------------------------------------------------------

    def _decide(self, d_mid):
        """三态有限状态机。"""
        now = rospy.Time.now()

        if d_mid < self.stats["min_mid"]:
            self.stats["min_mid"] = d_mid

        if self._state == CarState.EMERGENCY:
            elapsed = (now - self._emergency_since).to_sec()
            if elapsed < EMERGENCY_HOLD:
                return CarState.EMERGENCY
            if d_mid > D_CLEAR:
                return CarState.CRUISE
            return CarState.AVOID

        if d_mid < D_DANGER:
            self._state = CarState.EMERGENCY
            self._emergency_since = now
            self.stats["emergency_count"] += 1
            rospy.logwarn("[depth_camera] 紧急！中区深度 %.2f m，后退脱困", d_mid)
            return CarState.EMERGENCY

        if self._state == CarState.CRUISE:
            if d_mid < D_SAFE:
                self._state = CarState.AVOID
                self.stats["avoid_count"] += 1
                rospy.loginfo("[depth_camera] 中区深度 %.2f m，开始转向避障",
                              d_mid)
        else:
            if d_mid > D_CLEAR:
                self._state = CarState.CRUISE
                rospy.loginfo("[depth_camera] 中区已清空 (%.2f m)，恢复巡航",
                              d_mid)

        return self._state

    def _compute_command(self, state, d_left, d_mid, d_right):
        """生成速度指令。"""
        cmd = Twist()

        # 两侧深度做有限值保护，inf 视为"极远"，利于转向选择
        dl = d_left if np.isfinite(d_left) else D_CLEAR
        dr = d_right if np.isfinite(d_right) else D_CLEAR

        if state == CarState.CRUISE:
            cmd.linear.x = V_CRUISE
            cmd.angular.z = 0.0
            return cmd

        if state == CarState.AVOID:
            # 速度随中区深度衰减
            ratio = (d_mid - D_DANGER) / max(D_SAFE - D_DANGER, 1e-6)
            ratio = max(0.0, min(1.0, ratio))
            cmd.linear.x = V_AVOID * ratio

            delta = dl - dr
            delta_norm = max(-1.0, min(1.0, delta / 2.0))
            omega = K_OMEGA * delta_norm
            if abs(omega) < W_MIN:
                omega = W_MIN if delta >= 0 else -W_MIN

            # 两侧都被堵住 → 原地转向找出路
            if max(dl, dr) < D_SAFE:
                omega = W_MAX if dl >= dr else -W_MAX
                cmd.linear.x = 0.0

                # 原地转向超时保护：若长时间无法脱困（例如传感器读数异常），
                # 主动恢复巡航，避免车辆无限原地打转。
                now = rospy.Time.now()
                if self._spin_since is None:
                    self._spin_since = now
                elif (now - self._spin_since).to_sec() > MAX_SPIN_DURATION:
                    rospy.logwarn(
                        "[depth_camera] 原地转向已持续 %.1fs 仍未脱困，"
                        "恢复正常行驶", MAX_SPIN_DURATION
                    )
                    self._spin_since = None
                    self._state = CarState.CRUISE
                    cmd.linear.x = V_CRUISE
                    cmd.angular.z = 0.0
                    return cmd
            else:
                self._spin_since = None

            cmd.angular.z = max(-W_MAX, min(omega, W_MAX))
            return cmd

        # EMERGENCY
        cmd.linear.x = V_BACK
        cmd.angular.z = W_MAX if dl >= dr else -W_MAX
        return cmd

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------

    def update(self):
        with self._depth_lock:
            msg = self._depth

        if msg is None:
            self._watchdog.feed(Twist())
            return

        img = self._image_to_array(msg)
        if img is None:
            self._watchdog.feed(Twist())
            return

        self.stats["frames"] += 1

        # 1) 感知：三区深度统计
        d_left = self._roi_percentile(img, ROI_LEFT_START, ROI_LEFT_END)
        d_mid = self._roi_percentile(img, ROI_MID_START, ROI_MID_END)
        d_right = self._roi_percentile(img, ROI_RIGHT_START, ROI_RIGHT_END)

        # 中区无有效回波视为前方开阔
        if not np.isfinite(d_mid):
            d_mid = D_CLEAR * 2.0

        # 2) 决策
        state = self._decide(d_mid)

        # 3) 控制
        cmd = self._compute_command(state, d_left, d_mid, d_right)

        # 4) 输出
        self._watchdog.feed(cmd)
