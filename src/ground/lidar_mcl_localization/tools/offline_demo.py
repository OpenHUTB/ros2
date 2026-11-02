# -*- coding: utf-8 -*-
"""本地闭环自检：不起 ROS/Gazebo，用地图射线投射合成激光，验证 MCL 是否收敛。

流程：
    真值机器人沿一条弧线走 → 里程计按"标定误差 + 噪声"给出漂移增量
    → 粒子滤波预测/更新 → 与真值、纯里程计对比
"""

import math
import os
import sys

import numpy as np

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(PKG, "src"))
sys.path.insert(0, os.path.join(PKG, "config"))

from lidar_mcl_localization.geometry import compose, pose_error, wrap_angle   # noqa: E402
from lidar_mcl_localization.map_model import OccupancyMap                    # noqa: E402
from lidar_mcl_localization.motion_model import MotionModel, odometry_delta   # noqa: E402
from lidar_mcl_localization.particle_filter import ParticleFilter             # noqa: E402
from lidar_mcl_localization.sensor_model import LikelihoodFieldModel          # noqa: E402
from scene_layout import LIDAR_SAMPLES, SPAWN                                  # noqa: E402

PKG_ROOT = os.path.dirname(os.path.abspath(__file__))
PGM = os.path.join(PKG_ROOT, "..", "maps", "mcl_room.pgm")

MAX_RANGE = 8.0


def main():
    occ = OccupancyMap.from_files(PGM)
    print("[地图] %s" % occ)
    print("[地图] 占据像素 %d / 空闲 %d" % (int(occ.occupied.sum()),
                                            int((occ.grid == 254).sum())))

    angles = np.linspace(-math.pi, math.pi, LIDAR_SAMPLES, endpoint=False)

    # ---- 真值机器人：以 (v, w) 沿弧线走，里程计带 1.5% 轮距误差 + 噪声 ----
    truth = (SPAWN["x"], SPAWN["y"], SPAWN["yaw"])
    truth0 = truth
    odom = (0.0, 0.0, 0.0)
    rng = np.random.RandomState(7)

    def to_odom_frame(pose, origin):
        """把世界系位姿换算到"以 origin 为原点"的里程计坐标系。

        这一步很关键：里程计从 (0,0,0) 开始，而真值从出生点开始，
        直接相减会得到一个固定的 3.5 m 偏差——那是坐标原点不同，不是漂移。
        """
        dx = pose[0] - origin[0]
        dy = pose[1] - origin[1]
        c, s = math.cos(-origin[2]), math.sin(-origin[2])
        return (c * dx - s * dy, s * dx + c * dy, wrap_angle(pose[2] - origin[2]))

    pf = ParticleFilter(num_particles=600, init_pose=truth,
                        init_std=(0.25, 0.25, 0.10), rng=rng)
    motion = MotionModel(alpha=(0.12, 0.03, 0.12, 0.03))
    sensor = LikelihoodFieldModel(sigma=0.18, beam_step=8)

    # 方形循环：直行 6 s + 左转 2 s，重复 4 轮（≈ 48 s，绕房间一圈多）。
    # 之所以走"回到原点"的闭环，是因为系统性标定误差跑得越久累积越多，
    # MCL 的优势正是在这种场景下体现（有界 vs 无界）。
    segments = [(0.40, 0.0, 6.0), (0.0, 0.80, 2.0)] * 4
    plan = []
    for v, w, dur in segments:
        plan.extend([(v, w)] * int(round(dur / 0.1)))
    n_steps = len(plan)

    dt = 0.1
    errors_pf, errors_od, spreads = [], [], []

    for k in range(n_steps):
        v, w = plan[k]
        truth = compose(truth, (v * dt, w * dt))

        # 里程计的系统性误差：走了 1.03 倍（轮径偏大）、转角只报 0.97 倍（轮距偏大）
        scale_s, scale_th = 1.03, 1.0 / 1.03
        odom = compose(odom, (v * dt * scale_s * (1.0 + rng.normal(0, 0.01)),
                              w * dt * scale_th + rng.normal(0, 0.004)))
        delta = odometry_delta((0.0, 0.0, 0.0), (v * dt, 0.0, w * dt))

        # 用真值位姿合成激光（加 1 cm 噪声）
        ranges = occ.raycast(truth, angles, MAX_RANGE)
        ranges = ranges + rng.normal(0, 0.01, ranges.size)
        ranges = np.clip(ranges, 0.12, MAX_RANGE)

        pf.predict(motion, delta)
        a_valid, r_valid = sensor.subsample(angles, ranges, MAX_RANGE)
        pf.update(sensor, a_valid, r_valid, occ, MAX_RANGE)
        pf.maybe_resample(0.5)

        est = pf.estimate()
        e_pf, yaw_pf = pose_error(est, truth)
        # 里程计误差：与"真值换算到里程计坐标系"比，两者起点一致
        e_od, yaw_od = pose_error(odom, to_odom_frame(truth, truth0))
        sp_pos, sp_yaw = pf.spread()
        errors_pf.append((e_pf, yaw_pf))
        errors_od.append((e_od, yaw_od))
        spreads.append((sp_pos, sp_yaw))

        if k % 30 == 0 or k == n_steps - 1:
            print("  t=%4.1fs  PF 误差 %6.3f m / %5.2f°   里程计 %6.3f m / %5.2f°   "
                  "粒子散布 %.3f m / %.2f°"
                  % (k * dt, e_pf, math.degrees(yaw_pf), e_od, math.degrees(yaw_od),
                     sp_pos, math.degrees(sp_yaw)))

    pf_pos = np.array([e[0] for e in errors_pf])
    od_pos = np.array([e[0] for e in errors_od])
    pf_yaw = np.array([math.degrees(e[1]) for e in errors_pf])
    od_yaw = np.array([math.degrees(e[1]) for e in errors_od])

    print()
    print("== 统计（%d 步，%.1f 秒）==" % (n_steps, n_steps * dt))
    print("  位置误差  MCL 均值 %.3f m / 最大 %.3f m   里程计 均值 %.3f m / 最大 %.3f m"
          % (pf_pos.mean(), pf_pos.max(), od_pos.mean(), od_pos.max()))
    print("  航向误差  MCL 均值 %.2f°  / 最大 %.2f°    里程计 均值 %.2f°  / 最大 %.2f°"
          % (pf_yaw.mean(), pf_yaw.max(), od_yaw.mean(), od_yaw.max()))
    print("  重采样次数 %d" % pf.resample_count)

    ok = pf_pos.mean() < od_pos.mean() * 0.5
    print()
    print("  MCL 是否明显优于纯里程计: %s" % ok)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
