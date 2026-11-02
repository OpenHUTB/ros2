# -*- coding: utf-8 -*-
"""MCL 模块的单元测试：全部离线、不需要 ROS / Gazebo。

运行方式（两种都支持）：

    python3 -m pytest test/ -q
    python3 test/test_mcl.py          # 不装 pytest 也能跑，自带断言计数

覆盖的内容都是"容易悄悄坏掉、坏了又很难查"的地方：
    * 角度归一化与位姿复合（所有估计量的基础）
    * 地图加载 / 坐标变换往返（写错一格就整体偏半个像素）
    * 射线投射（合成测试数据的地基，也用来核对地图与场景一致）
    * 似然场是否真的"离障碍越近值越小"
    * 观测模型：正确位姿的权重必须显著高于偏移位姿（这是滤波能不能收敛的关键）
    * 粒子滤波：从带噪声的初始分布能否收敛、权重是否归一化
    * 重采样：不改变加权均值、Neff 计算是否正确
    * 神经网络：特征有界、修正量被截断、反向传播能让 loss 下降
"""

import math
import os
import sys

import numpy as np

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PKG_ROOT, "src"))
sys.path.insert(0, os.path.join(PKG_ROOT, "config"))

from lidar_mcl_localization import nn_motion                       # noqa: E402
from lidar_mcl_localization.geometry import (compose, pose_error,  # noqa: E402
                                             relative_pose, weighted_pose, wrap_angle)
from lidar_mcl_localization.map_model import OccupancyMap, load_pgm  # noqa: E402
from lidar_mcl_localization.motion_model import MotionModel, odometry_delta  # noqa: E402
from lidar_mcl_localization.nn_motion import (MotionMLP, NNMotionCorrector,  # noqa: E402
                                              StandardScaler, correction_from_output,
                                              features_from_delta)
from lidar_mcl_localization.particle_filter import ParticleFilter    # noqa: E402
from lidar_mcl_localization.sensor_model import LikelihoodFieldModel, ScanMatcher  # noqa: E402
from scene_layout import LIDAR_SAMPLES, SPAWN                        # noqa: E402

MAP_PGM = os.path.join(PKG_ROOT, "maps", "mcl_room.pgm")
MAX_RANGE = 8.0

PASS = 0
FAIL = 0


def check(cond, msg):
    global PASS, FAIL
    if cond:
        PASS += 1
        print("  [PASS] %s" % msg)
    else:
        FAIL += 1
        print("  [FAIL] %s" % msg)


def close(a, b, tol, msg):
    check(abs(a - b) <= tol, "%s（%.6f ≈ %.6f，容差 %.0e）" % (msg, a, b, tol))


# ------------------------------------------------------------------ 1. 几何
def test_geometry():
    print("\n== 1. 几何 ==")
    close(wrap_angle(0.0), 0.0, 1e-12, "wrap(0)=0")
    close(wrap_angle(math.pi), -math.pi, 1e-9, "wrap(+pi) 归一化到 -pi")
    close(wrap_angle(-math.pi), -math.pi, 1e-9, "wrap(-pi)=-pi")
    close(wrap_angle(3 * math.pi), -math.pi, 1e-9, "wrap(3pi) 同余")
    for a in np.linspace(-20, 20, 401):
        if not (-math.pi - 1e-9 <= wrap_angle(a) <= math.pi + 1e-9):
            check(False, "wrap 值域越界：%.3f" % a)
            break
    else:
        check(True, "wrap 在 [-20, 20] 上值域都在 [-pi, pi]")

    # 复合的往返：先走再反推
    p0 = (1.0, -2.0, 0.3)
    p1 = compose(p0, (0.5, 0.2))
    ds, dth = relative_pose(p0, p1)
    close(ds, 0.5, 1e-9, "relative_pose 还原位移")
    close(dth, 0.2, 1e-9, "relative_pose 还原转角")

    # 加权位姿的圆周平均：两个相差 2pi 的角应给出同一个朝向
    parts = np.array([[0.0, 0.0, math.pi - 0.05],
                      [0.0, 0.0, -math.pi + 0.05]])
    w = np.array([0.5, 0.5])
    _x, _y, yaw = weighted_pose(parts, w)
    close(abs(yaw), math.pi, 1e-6, "圆周平均在 ±pi 边界不跳变")

    e_pos, e_yaw = pose_error((1.0, 0.0, 0.0), (0.0, 0.0, math.pi - 0.01))
    close(e_pos, 1.0, 1e-9, "pose_error 位置")
    close(e_yaw, math.pi - 0.01, 1e-9, "pose_error 航向取绝对值的短边")


# -------------------------------------------------------------------- 2. 地图
def test_map():
    print("\n== 2. 地图 ==")
    grid = load_pgm(MAP_PGM)
    check(grid.dtype == np.uint8, "load_pgm 返回 uint8")
    check(grid.ndim == 2 and grid.shape[0] > 10 and grid.shape[1] > 10,
          "地图尺寸合理：%dx%d" % (grid.shape[1], grid.shape[0]))
    check(int(np.sum(grid == 0)) > 1000, "地图里有足够多的占据格（%d）"
          % int(np.sum(grid == 0)))

    occ = OccupancyMap.from_files(MAP_PGM)
    check(abs(occ.resolution - 0.05) < 1e-9, "分辨率 0.05 m/px")
    check(occ.origin[0] < 0 and occ.origin[1] < 0, "原点在负半轴（地图包住场景）")

    # 世界坐标 <-> 栅格坐标 往返
    for (x, y) in ((SPAWN["x"], SPAWN["y"]), (0.0, 0.0), (2.5, -3.0)):
        col, row = occ.world_to_grid(x, y)
        x2, y2 = occ.grid_to_world(round(float(col)), round(float(row)))
        check(abs(x2 - x) <= occ.resolution and abs(y2 - y) <= occ.resolution,
              "坐标往返误差小于一格 (%.2f, %.2f)" % (x, y))

    # 似然场：墙上是 0，走廊中间到侧墙约 0.9 m（走廊净宽 1.8 m）
    field = occ.likelihood_field()
    check(field.shape == occ.grid.shape, "似然场与地图同尺寸")
    close(float(field.min()), 0.0, 1e-9, "似然场最小值为 0（墙格）")
    corridor = float(occ.likelihood_at_world(0.0, 0.0))
    check(0.7 < corridor < 1.1,
          "走廊中心到最近墙约 0.9 m（实测 %.2f m）" % corridor)

    # 射线投射：从出生点正前方（+x）打一束，沿走廊一直打到东墙
    angles = np.array([0.0])
    d = occ.raycast((SPAWN["x"], SPAWN["y"], 0.0), angles, MAX_RANGE)[0]
    check(2.0 < d <= MAX_RANGE,
          "正前方射线沿走廊命中（%.2f m，走廊长 8.5 m，超出 8 m 量程即返回量程）" % d)

    # 朝 +y 打：走廊北墙在 y≈+0.9，距离约 0.9 m
    d_up = occ.raycast((0.0, 0.0, math.pi / 2), angles, MAX_RANGE)[0]
    check(0.5 < d_up < 1.5, "朝北打到走廊侧墙（%.2f m）" % d_up)


# ---------------------------------------------------------------- 3. 观测模型
def test_sensor_model():
    print("\n== 3. 观测模型 ==")
    occ = OccupancyMap.from_files(MAP_PGM)
    angles = np.linspace(-math.pi, math.pi, LIDAR_SAMPLES, endpoint=False)

    truth = (SPAWN["x"], SPAWN["y"], SPAWN["yaw"])
    ranges = occ.raycast(truth, angles, MAX_RANGE)

    sensor = LikelihoodFieldModel(sigma=0.20, beam_step=8)
    a_v, r_v = sensor.subsample(angles, ranges, MAX_RANGE)
    check(a_v.size > 10, "抽样后仍有 %d 束有效激光" % a_v.size)

    # 关键断言：正确位姿的权重必须远高于偏移位姿
    good = np.array([truth, truth, truth])
    bad = np.array([(truth[0] + 1.5, truth[1], truth[2]),
                    (truth[0], truth[1] + 1.5, truth[2]),
                    (truth[0], truth[1], truth[2] + 0.8)])
    w_good = sensor.log_weights(good, a_v, r_v, occ, MAX_RANGE)
    w_bad = sensor.log_weights(bad, a_v, r_v, occ, MAX_RANGE)
    check(np.all(w_good > w_bad),
          "正确位姿的对数权重高于平移 1.5 m / 转 0.8 rad 的位姿"
          "（%.1f vs %.1f）" % (float(w_good[0]), float(w_bad.max())))

    # 权重归一化
    w = sensor.weights(good, a_v, r_v, occ, MAX_RANGE)
    close(float(w.sum()), 1.0, 1e-9, "权重之和为 1")

    # 空扫描不应崩溃，权重退化为均匀
    w_empty = sensor.weights(good, np.empty(0), np.empty(0), occ, MAX_RANGE)
    close(float(w_empty.sum()), 1.0, 1e-9, "空扫描时权重仍是归一化的")

    # 粗配准：从偏移 0.15 m 的先验出发，应回到真值附近
    matcher = ScanMatcher(search_xy=0.25, search_th=0.10)
    prior = (truth[0] + 0.15, truth[1] - 0.10, truth[2] + 0.05)
    best, score = matcher.match(occ, prior, a_v, r_v, MAX_RANGE)
    err = math.hypot(best[0] - truth[0], best[1] - truth[1])
    check(err < 0.12, "扫描匹配把 0.18 m 的先验误差收敛到 %.3f m" % err)
    check(score < 0, "扫描匹配得分为负（平方距离之和）")


# ---------------------------------------------------------------- 4. 运动模型
def test_motion_model():
    print("\n== 4. 运动模型 ==")
    rng = np.random.RandomState(0)
    motion = MotionModel(alpha=(0.10, 0.02, 0.10, 0.02))

    zero = np.zeros((100, 3))
    out = motion.sample(zero, (0.0, 0.0), rng)
    check(np.all(np.abs(out) < 1e-3), "零增量时粒子几乎不动（噪声下限被限制）")

    # 采样均值应接近真值增量
    parts = np.zeros((20000, 3))
    out = motion.sample(parts, (0.5, 0.2), rng)
    close(float(np.mean(out[:, 0])), 0.5, 0.02, "ds 的采样均值接近 0.5 m")
    close(float(np.mean(out[:, 2])), 0.2, 0.02, "dθ 的采样均值接近 0.2 rad")
    check(float(np.std(out[:, 2])) > 1e-3, "粒子确实有散布（不是全部重合）")

    # 里程计增量换算
    ds, dth = odometry_delta((0.0, 0.0, 0.0), (1.0, 0.0, 0.5))
    close(ds, 1.0, 1e-9, "odometry_delta 位移")
    close(dth, 0.5, 1e-9, "odometry_delta 转角")


# -------------------------------------------------------------- 5. 粒子滤波
def test_particle_filter():
    print("\n== 5. 粒子滤波 ==")
    occ = OccupancyMap.from_files(MAP_PGM)
    angles = np.linspace(-math.pi, math.pi, LIDAR_SAMPLES, endpoint=False)
    sensor = LikelihoodFieldModel(sigma=0.20, beam_step=8)
    motion = MotionModel(alpha=(0.08, 0.02, 0.08, 0.02))

    truth = (SPAWN["x"], SPAWN["y"], SPAWN["yaw"])
    rng = np.random.RandomState(3)
    pf = ParticleFilter(num_particles=500, init_pose=(truth[0] + 0.30, truth[1], truth[2]),
                        init_std=(0.30, 0.30, 0.15), rng=rng)

    e0 = pose_error(pf.estimate(), truth)[0]
    check(e0 > 0.05, "初始估计确实有偏差（%.3f m）" % e0)

    # 静止状态下反复观测，应逐渐收敛到真值
    for _ in range(12):
        ranges = occ.raycast(truth, angles, MAX_RANGE)
        a_v, r_v = sensor.subsample(angles, ranges, MAX_RANGE)
        pf.update(sensor, a_v, r_v, occ, MAX_RANGE)
        pf.maybe_resample(0.5)

    e1 = pose_error(pf.estimate(), truth)
    check(e1[0] < e0, "重复观测后位置误差下降（%.3f → %.3f m）" % (e0, e1[0]))
    check(e1[0] < 0.15, "收敛到 0.15 m 以内（%.3f m）" % e1[0])
    check(e1[1] < 0.15, "航向收敛到 0.15 rad 以内（%.3f rad）" % e1[1])

    # 权重与 Neff
    close(float(pf.weights.sum()), 1.0, 1e-9, "权重归一化")
    check(0 < pf.neff() <= pf.num_particles, "Neff 落在 (0, N]")

    # 低方差重采样：加权均值不应被破坏
    before = weighted_pose(pf.particles, pf.weights)
    pf.resample()
    after = weighted_pose(pf.particles, pf.weights)
    check(abs(after[0] - before[0]) < 0.10 and abs(after[1] - before[1]) < 0.10,
          "重采样后加权均值基本不变（%.3f → %.3f m）"
          % (math.hypot(before[0], before[1]), math.hypot(after[0], after[1])))
    close(float(pf.weights.sum()), 1.0, 1e-9, "重采样后权重仍归一化")

    # 散布与协方差
    pos_std, yaw_std = pf.spread()
    check(pos_std >= 0 and yaw_std >= 0, "散布非负")
    cov = pf.covariance()
    check(cov.shape == (2, 2) and abs(cov[0, 1] - cov[1, 0]) < 1e-12, "协方差对称")


# ---------------------------------------------------------- 6. 神经网络修正
def test_nn_motion():
    print("\n== 6. 神经网络运动模型 ==")
    f = features_from_delta(0.04, 0.08, dt=0.1)
    close(f[0], 0.4, 1e-9, "特征第 1 维 = 线速度")
    close(f[1], 0.8, 1e-9, "特征第 2 维 = 角速度")
    close(f[2], 0.8, 1e-9, "特征第 3 维 = |角速度|")
    check(f[3] <= 0.4 * 0.8 + 1e-9, "特征第 4 维 = v·|ω|（有界）")
    check(np.all(np.abs(f) < 5.0), "所有特征都在有界范围内")

    # 截断：网络输出再离谱，修正量也只在 [0.8, 1.2]
    k_s, k_th, noise = correction_from_output([100.0, -100.0, 99.0])
    close(k_s, 1.20, 1e-9, "k_s 上截断到 1.2")
    close(k_th, 0.80, 1e-9, "k_th 下截断到 0.8")
    check(0.5 <= noise <= 4.0, "noise 落在 [0.5, 4.0]")

    # 反向传播：常数标签下 loss 必须显著下降
    rng = np.random.RandomState(0)
    X = np.c_[rng.uniform(0.1, 0.6, 200),
              rng.uniform(-1.2, 1.2, 200)]
    X = np.c_[X, np.abs(X[:, 1]), X[:, 0] * np.abs(X[:, 1])]
    Y = np.tile(np.array([0.95, 0.94, 2.0]), (len(X), 1))
    scaler = StandardScaler().fit(X)
    net = MotionMLP(seed=0)
    xs = scaler.transform(X)
    first = net.backward(xs, Y, lr=1e-2)
    for _ in range(400):
        last = net.backward(xs, Y, lr=1e-2)
    check(last < first * 0.1, "反向传播让 loss 下降一个数量级（%.4f → %.4f）"
          % (first, last))

    # corrector 在无网络时必须退化为"原样返回"
    plain = NNMotionCorrector(None, None)
    ds, dth, ns = plain((0.5, 0.2))
    close(ds, 0.5, 1e-12, "无网络时 ds 原样返回")
    close(dth, 0.2, 1e-12, "无网络时 dθ 原样返回")
    close(ns, 1.0, 1e-12, "无网络时噪声标度为 1")

    # corrector 有网络时输出应被限制在合理范围
    trained = NNMotionCorrector(net, scaler)
    trained.push(0.04, 0.08)
    ds2, dth2, ns2 = trained((0.04, 0.08))
    check(0.8 * 0.04 - 1e-9 <= ds2 <= 1.2 * 0.04 + 1e-9,
          "有网络时 ds 修正量被限制在 ±20%% 内（%.5f）" % ds2)
    check(0.5 <= ns2 <= 4.0, "噪声标度在 [0.5, 4.0]")


def main():
    print("=" * 72)
    print("MCL 定位模块 —— 离线单元测试（不需要 ROS / Gazebo）")
    print("=" * 72)
    test_geometry()
    test_map()
    test_sensor_model()
    test_motion_model()
    test_particle_filter()
    test_nn_motion()
    print("\n" + "=" * 72)
    print("通过 %d 项，失败 %d 项" % (PASS, FAIL))
    print("=" * 72)
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
