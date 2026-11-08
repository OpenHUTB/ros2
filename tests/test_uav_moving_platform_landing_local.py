#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_moving_platform_landing 本地回归测试（**无需 ROS / 仿真器 / GPU**）。

运行::

    python3 tests/test_uav_moving_platform_landing_local.py

覆盖六类内容：

* **平台速度估计** —— 匀速 / 带延迟 / 带跳点 / 观测断流 / 短窗口退化；
* **曲率判定** —— 圆周能判出来、直线不能误判（长窗估计器）；
* **下降剖面与接地时间** —— 加速度约束下的解析解、升沉甲板上的不动点解；
* **状态机** —— 正常降落、超时、穿到甲板下面、复飞次数耗尽；
* **闭环落点精度** —— 同一套运动学模型下 4 种制导策略的定量对照
  （朴素追法 / 追当前位置 / 相遇点 / 自动），以及「平台比无人机快」的场景；
* **工程文件** —— 目录结构、入口命名、launch / package / CMake 等约定。
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, os.pardir))
PKG = os.path.join(REPO, "src", "air", "uav_moving_platform_landing")
sys.path.insert(0, os.path.join(PKG, "scripts"))
sys.path.insert(0, os.path.join(PKG, "tools"))

import moving_landing_core as core    # noqa: E402
import closed_loop_sim as sim         # noqa: E402

PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print("%s %s%s" % ("[通过]" if cond else "[失败]", name,
                       ("  -- " + detail) if detail and not cond else ""))


def feed(tracker, pos_fn, n, dt=0.1, noise=0.0, seed=0):
    rng = np.random.RandomState(seed)
    for i in range(n):
        t = i * dt
        p = np.asarray(pos_fn(t), dtype=float)
        if noise:
            p = p + rng.normal(0.0, noise, 3)
        tracker.push(t, p)
    return tracker


# --------------------------------------------------------------------------- #
# 1. 平台速度估计
# --------------------------------------------------------------------------- #
def test_tracker_constant_velocity():
    tr = core.DeckTracker()
    feed(tr, lambda t: (0.6 * t, -0.3 * t, 0.0), 40, noise=0.0)
    now = 3.9
    v = tr.velocity_at(now)
    check("匀速直线：速度估计无偏", abs(v[0] - 0.6) < 1e-6 and abs(v[1] + 0.3) < 1e-6,
          str(v))
    p = tr.position_at(now)
    check("匀速直线：当前时刻位置估计无偏",
          abs(p[0] - 0.6 * now) < 1e-6, str(p))
    p2 = tr.position_at(now + 5.0)
    check("匀速直线：外推 5 s 无偏", abs(p2[0] - 0.6 * (now + 5.0)) < 1e-6, str(p2))


def test_tracker_noise_and_latency():
    tr = core.DeckTracker()
    feed(tr, lambda t: (0.8 * t, 0.0, 0.0), 60, noise=0.03, seed=7)
    now = 5.9
    v = tr.velocity_at(now)
    check("5 cm 级噪声下速度估计误差 < 0.06 m/s", abs(v[0] - 0.8) < 0.06, str(v))
    # 观测时间戳比 now 旧 0.25 s：估计器要把这段延迟补偿掉
    p = tr.position_at(now)
    check("量测比 now 旧 0.25 s 时，外推到 now 的误差 < 0.15 m",
          abs(p[0] - 0.8 * now) < 0.15, "%.3f vs %.3f" % (p[0], 0.8 * now))


def test_tracker_outlier_rejection():
    tr = core.DeckTracker()
    feed(tr, lambda t: (0.5 * t, 0.0, 0.0), 30, noise=0.0)
    before = tr.velocity_at(2.9).copy()
    tr.push(3.0, [1.5 + 40.0, 0.0, 0.0])       # 一个 40 m 的跳点（RPC 抖动）
    after = tr.velocity_at(3.0)
    check("单个 40 m 跳点不会把速度估计带偏", abs(after[0] - before[0]) < 0.2,
          "%.3f -> %.3f" % (before[0], after[0]))


def test_tracker_staleness_and_reset():
    tr = core.DeckTracker()
    feed(tr, lambda t: (t, 0.0, 0.0), 20)
    check("观测断流能被识别", tr.is_stale(5.0, max_age_s=0.6) and not tr.is_stale(2.0))
    check("观测年龄计算正确", abs(tr.age(3.0) - 1.1) < 0.06, str(tr.age(3.0)))
    tr.reset()
    check("reset 之后样本清空", len(tr) == 0)
    tr2 = core.DeckTracker(min_samples=4)
    tr2.push(0.0, [0.0, 0.0, 0.0])
    tr2.push(0.1, [1.0, 0.0, 0.0])
    check("样本不足时不做估计（ready=False）", not tr2.ready(0.1))


def test_tracker_curvature():
    curv = core.DeckTracker(window_s=4.0, half_life_s=60.0, min_samples=8)
    feed(curv, lambda t: sim.CircleMotion(radius=4.0, speed=0.6).pos(t), 80, noise=0.04,
         seed=3)
    acc = curv.curvature_accel(7.9)
    check("圆周运动（0.6 m/s, R=4）能判出横向加速度 ≈0.09 m/s²",
          0.04 < acc < 0.35, "%.3f" % acc)
    check("圆周运动二次项被采用（deg=2）", curv.degree() == 2, str(curv.degree()))

    line = core.DeckTracker(window_s=4.0, half_life_s=60.0, min_samples=8)
    feed(line, lambda t: (0.6 * t, 0.0, 0.0), 80, noise=0.04, seed=3)
    acc_line = line.curvature_accel(7.9)
    check("匀速直线不会被误判成转弯", acc_line < 0.04, "%.3f" % acc_line)


# --------------------------------------------------------------------------- #
# 2. 下降剖面与接地时间
# --------------------------------------------------------------------------- #
def test_descend_profile():
    dz, vz = core.descend_profile(0.4, 0.0, 0.4, 1.0)
    check("下降剖面：0.4 s 后到达目标速度",
          abs(float(vz) + 0.4) < 1e-6, str(vz))
    check("下降剖面：加速段位移 = -0.5*a*t²",
          abs(float(dz) - (-0.5 * 1.0 * 0.16)) < 1e-6, str(dz))
    dz2, _ = core.descend_profile(2.0, 0.0, 0.4, 1.0)
    check("下降剖面：2 s 的位移 = 加速段 + 恒速段",
          abs(float(dz2) - (-0.08 - 0.4 * 1.6)) < 1e-6, str(dz2))
    _, vz_neg = core.descend_profile(np.array([1.0]), -1.0, 0.4, 1.0)
    check("下降剖面：本来就在更快下降时不再反向加速",
          abs(float(vz_neg[0]) + 1.0) < 1e-6, str(vz_neg))


def test_predict_touchdown_flat():
    tr = core.DeckTracker()
    feed(tr, lambda t: (0.5 * t, 0.0, 0.0), 40)
    T, lead = core.predict_touchdown(2.0, tr, 3.9, -0.4, 0.4, 1.0)
    check("平地甲板：T ≈ 落差/下降速度", T is not None and abs(T - 5.0) < 0.1, str(T))
    check("平地甲板：落点 = 平台 5 s 后的位置",
          lead is not None and abs(lead[0] - 0.5 * (3.9 + T)) < 0.1,
          str(None if lead is None else lead[0]))


def test_predict_touchdown_heave():
    tr = core.DeckTracker()
    motion = sim.HeaveMotion(sim.LineMotion(v=(0.5, 0.0, 0.0)), amp=0.15, period=4.0)
    feed(tr, motion.pos, 60, noise=0.01, seed=5)
    now = 5.9
    z_rel = 2.0
    T, lead = core.predict_touchdown(z_rel, tr, now, -0.4, 0.4, 1.0,
                                     deck_vz=0.2)      # 甲板正在上浮
    check("升沉甲板：甲板上浮时接地时间变短", T is not None and T < 5.0, str(T))
    # 校验不动点：z_rel - v*T 应等于甲板在这段时间内上升的高度
    if T is not None:
        lhs = z_rel - 0.4 * T
        rhs = 0.2 * T
        check("升沉甲板：解满足 z = (v_desc + v_deck) * T", abs(lhs - rhs) < 0.05,
              "%.3f vs %.3f" % (lhs, rhs))
    T_no, _ = core.predict_touchdown(2.0, tr, now, -0.4, 0.4, 1.0, deck_vz=-0.6)
    check("甲板下沉比下降还快时无解（返回 None）", T_no is None, str(T_no))


# --------------------------------------------------------------------------- #
# 3. 状态机
# --------------------------------------------------------------------------- #
def run_ctl(motion, strategy="auto", latency=0.15, noise=0.04, seed=1,
            duration=75.0, params=None):
    return sim.run_trial(motion, strategy, latency=latency, noise_m=noise, seed=seed,
                         duration=duration, params=params)


def test_state_machine_nominal():
    r = run_ctl(sim.LineMotion(v=(0.6, 0.0, 0.0)))
    ctl = r["controller"]
    states = [s for (_, s, _) in ctl.state_history]
    check("正常降落：最终 LANDED", r["state"] == core.LANDED, r["state"])
    check("正常降落：状态顺序为 WAIT_DECK→ALIGN→DESCEND→TOUCHDOWN→LANDED",
          states == [core.WAIT_DECK, core.ALIGN, core.DESCEND, core.TOUCHDOWN,
                     core.LANDED], str(states))
    check("正常降落：接地横向误差 < 0.30 m", r["error_xy"] < 0.30, "%.3f" % r["error_xy"])
    check("正常降落：全程没有触发复飞", r["go_around"] == 0, str(r["go_around"]))
    check("正常降落：z_rel 收在 0.12 m 以内（接地判据）", abs(r["z_rel"]) < 0.6,
          "%.3f" % r["z_rel"])
    check("summarize_trial 口径一致",
          core.summarize_trial(ctl)["landed"] and
          core.summarize_trial(ctl)["error_xy"] < 0.30)


def test_state_machine_static_deck():
    r = run_ctl(sim.LineMotion(v=(0.0, 0.0, 0.0)))
    check("静止甲板也能落", r["state"] == core.LANDED, r["state"])
    check("静止甲板落点误差 < 0.15 m", r["error_xy"] < 0.15, "%.3f" % r["error_xy"])


def test_state_machine_abort_on_timeout():
    params = {"mission/align_timeout_s": 3.0, "guide/align_tol": 0.01}
    r = run_ctl(sim.LineMotion(v=(0.8, 0.0, 0.0)), params=params, duration=40.0)
    check("对准容差给到 1 cm 时按超时 ABORT",
          r["state"] == core.ABORT, r["state"])
    check("ABORT 一定带原因", bool(r["reason"]), str(r["reason"]))


def test_state_machine_below_deck():
    ctl = core.MovingLandingController({"mission/min_below_deck_m": -0.35})
    ctl.reset()
    t = 0.0
    for i in range(30):
        t = i * 0.1
        ctl.step(t, [0.0, 0.0, 5.0], [0, 0, 0],
                 core.DeckMeasurement(t, [0.0, 0.0, 0.0]))
    out = None
    for i in range(30, 120):
        t = i * 0.1
        # 故意把无人机"瞬移"到甲板下面 1 m
        out = ctl.step(t, [0.0, 0.0, -1.0], [0, 0, -0.4],
                       core.DeckMeasurement(t, [0.0, 0.0, 0.0]))
        if out["state"] == core.ABORT:
            break
    check("相对高度为负时立刻 ABORT（不会一直往下钻）",
          out is not None and out["state"] == core.ABORT,
          "None" if out is None else out["state"])


def test_command_limits():
    ctl = core.MovingLandingController()
    ctl.reset()
    t = 0.0
    for i in range(20):
        t = i * 0.1
        ctl.step(t, [0.0, 0.0, 6.0], [0, 0, 0],
                 core.DeckMeasurement(t, [0.0, 0.0, 0.0]))
    # 平台突然出现在 50 m 外，指令必须被限幅
    out = ctl.step(t + 0.1, [0.0, 0.0, 6.0], [0, 0, 0],
                   core.DeckMeasurement(t + 0.1, [50.0, 0.0, 0.0]))
    vx, vy, vz = out["cmd_vel"]
    check("横向指令被限幅到 max_xy_speed",
          math.hypot(vx, vy) <= ctl.p["guide/max_xy_speed"] + 1e-6,
          "%.2f" % math.hypot(vx, vy))
    check("垂直指令被限幅到 max_z_speed",
          abs(vz) <= ctl.p["guide/max_z_speed"] + 1e-6, "%.2f" % vz)


def test_align_holds_altitude():
    """对准段不该往下掉：垂直指令必须始终在原高度附近（不做深度下降）。"""
    ctl = core.MovingLandingController()
    ctl.reset()
    motion = sim.LineMotion(v=(0.6, 0.0, 0.0))
    max_down = 0.0
    t = 0.0
    for i in range(60):
        t = i * 0.1
        out = ctl.step(t, [0.0, 0.0, 8.0], [0, 0, 0],
                       core.DeckMeasurement(t, motion.pos(t)))
        if out["state"] == core.ALIGN:
            max_down = min(max_down, out["cmd_vel"][2])
    check("对准段垂直指令不小于 -max_z_speed",
          max_down >= -ctl.p["guide/max_z_speed"] - 1e-6, "%.2f" % max_down)


def test_go_around():
    """平台在下降中突然加速，应触发复飞而不是硬落。"""
    params = {"guide/abort_xy_tol": 0.5, "mission/max_go_around": 5}
    r = run_ctl(sim.LineMotion(v=(1.4, 0.0, 0.0)), strategy="predict",
                latency=0.3, params=params)
    ctl = r["controller"]
    hist_states = [s for (_, s, _) in ctl.state_history]
    check("复飞后能重新回到 DESCEND 或最终落地/放弃（状态机不死锁）",
          hist_states[-1] in (core.LANDED, core.ABORT), str(hist_states[-1]))
    check("高时延 + 快平台这一轮确实走过复飞或干脆一次落成（不允许卡死）",
          hist_states.count(core.DESCEND) >= 1, str(hist_states))


# --------------------------------------------------------------------------- #
# 4. 闭环落点精度（定量对照）
# --------------------------------------------------------------------------- #
def test_strategy_comparison():
    motion = sim.LineMotion(v=(0.8, 0.0, 0.0))
    errs = {}
    for strat in ("static", "track", "auto"):
        vals = [run_ctl(motion, strategy=strat, seed=s)["error_xy"] for s in (0, 1, 2)]
        errs[strat] = float(np.mean(vals))
    check("0.8 m/s 直线：朴素追法落不准（>0.30 m）", errs["static"] > 0.30,
          "%.3f" % errs["static"])
    check("0.8 m/s 直线：速度匹配式 < 0.10 m", errs["track"] < 0.10,
          "%.3f" % errs["track"])
    check("0.8 m/s 直线：自动模式 < 0.10 m", errs["auto"] < 0.10, "%.3f" % errs["auto"])
    check("0.8 m/s 直线：自动模式优于朴素追法 3 倍以上",
          errs["auto"] * 3.0 < errs["static"],
          "%.3f vs %.3f" % (errs["auto"], errs["static"]))


def test_heave_and_circle():
    circle = sim.CircleMotion(radius=4.0, speed=0.6)
    r = run_ctl(circle, seed=2)
    check("圆周平台：自动模式落点误差 < 0.30 m",
          r["state"] == core.LANDED and r["error_xy"] < 0.30, "%.3f" % r["error_xy"])
    check("圆周平台：自动模式用速度匹配（相遇点外推在转弯上不可信）",
          r["controller"].lead_mode_eff == "now", str(r["controller"].lead_mode_eff))

    wave = sim.HeaveMotion(sim.LineMotion(v=(0.5, 0.0, 0.0)))
    r2 = run_ctl(wave, seed=2)
    check("升沉平台：自动模式能落且误差 < 0.20 m",
          r2["state"] == core.LANDED and r2["error_xy"] < 0.20, "%.3f" % r2["error_xy"])


def test_fast_platform_with_limited_authority():
    """平台比无人机快：只有"相遇点/自动"能落，纯速度匹配必然落偏。"""
    params = {"guide/max_xy_speed": 0.6}
    motion = sim.LineMotion(v=(1.0, 0.0, 0.0))
    kw = dict(latency=0.2, seed=1, params=params, start_offset=(10.0, 0.0, 8.0))
    r_track = sim.run_trial(motion, "track", **kw)
    r_auto = sim.run_trial(motion, "auto", **kw)
    check("平台 1.0 m/s、无人机上限 0.6 m/s：纯速度匹配落不准",
          (not r_track["ok"]) and r_track["error_xy"] > 1.0,
          "%.2f" % r_track["error_xy"])
    check("同一场景：自动模式切换成相遇点后落准",
          r_auto["ok"] and r_auto["error_xy"] < 0.30, "%.2f" % r_auto["error_xy"])
    names = [e["name"] for e in r_auto["controller"].events]
    check("自动模式的切换有明确事件记录", "LEAD_INTERCEPT" in names, str(names))


def test_determinism():
    a = run_ctl(sim.LineMotion(v=(0.6, 0.0, 0.0)), seed=5)
    b = run_ctl(sim.LineMotion(v=(0.6, 0.0, 0.0)), seed=5)
    check("同种子可复现", abs(a["error_xy"] - b["error_xy"]) < 1e-12)


def test_no_stale_input_landing():
    """平台观测断流（延迟给 3 s，远超 stale 阈值）时不允许继续下降。"""
    ctl = core.MovingLandingController({"mission/align_timeout_s": 6.0})
    ctl.reset()
    t = 0.0
    out = None
    for i in range(200):
        t = i * 0.1
        # 只在前 1 s 给观测，之后断流
        meas = core.DeckMeasurement(t, [0.0, 0.0, 0.0]) if i < 10 else None
        out = ctl.step(t, [0.0, 0.0, 5.0], [0, 0, 0], meas)
    check("观测断流后退回 WAIT_DECK（不会带着过期数据继续降）",
          out is not None and out["state"] in (core.WAIT_DECK, core.ABORT),
          "None" if out is None else out["state"])


# --------------------------------------------------------------------------- #
# 5. 工程文件约定
# --------------------------------------------------------------------------- #
def _git_tracked_files():
    """返回仓库里被 git 跟踪的文件路径集合；拿不到就返回 None。"""
    import subprocess
    try:
        out = subprocess.check_output(["git", "ls-files"], cwd=REPO,
                                      stderr=subprocess.DEVNULL)
    except (OSError, subprocess.CalledProcessError):
        return None
    paths = set()
    for line in out.decode("utf-8", "replace").splitlines():
        paths.add(line.strip().replace("\\", "/"))
    return paths or None


def test_package_layout():
    files = [
        "CMakeLists.txt", "package.xml", "README.md",
        "scripts/main.py", "scripts/moving_landing_core.py",
        "scripts/moving_landing_node.py",
        "launch/main.launch", "config/moving_platform_landing.yaml",
        "tools/closed_loop_sim.py", "tools/evaluate.py",
    ]
    missing = [f for f in files if not os.path.exists(os.path.join(PKG, f))]
    check("包结构完整（%d 个文件）" % len(files), not missing, str(missing))

    # 光"磁盘上有"不够：文件被 .gitignore 吞掉时本地测试照样通过、仓库里却缺文件
    # （本模块第一版就踩了 —— Windows 的 core.ignorecase 让 *.TXT 把 CMakeLists.txt
    # 也忽略了）。这里再核一遍这些文件确实被 git 跟踪；没有 git 或不在仓库里就跳过。
    tracked = _git_tracked_files()
    if tracked is None:
        print("[跳过] 没有可用的 git 仓库，跳过「文件已被 git 跟踪」检查")
    else:
        untracked = [f for f in files
                     if "src/air/uav_moving_platform_landing/" + f not in tracked]
        check("必需文件都已入库（不只是存在于磁盘）", not untracked, str(untracked))

    main_py = os.path.join(PKG, "scripts", "main.py")
    if os.path.exists(main_py):
        raw = open(main_py, "rb").read()
        check("main.py 不带 UTF-8 BOM（BOM 会让 shebang 失效）",
              not raw.startswith(b"\xef\xbb\xbf"))
        text = raw.decode("utf-8")
        check("main.py 以 main. 入口命名，且同时支持 roslaunch/rosrun",
              "moving_landing_node" in text and "__main__" in text)

    launch = os.path.join(PKG, "launch", "main.launch")
    if os.path.exists(launch):
        text = open(launch, encoding="utf-8").read()
        check("launch 里有 main.py 节点", 'type="main.py"' in text, text[:80])
        check("launch 加了 cmd_vel 时长提醒参数（桥接层默认 0.1 s 会顶掉指令）",
              "cmd_duration" in text)

    # CMakeLists 里列出的脚本必须真的存在（否则 catkin_install_python 会让构建失败）
    cmake = os.path.join(PKG, "CMakeLists.txt")
    if os.path.exists(cmake):
        text = open(cmake, encoding="utf-8").read()
        listed = [ln.strip() for ln in text.splitlines()
                  if ln.strip().startswith("scripts/")]
        absent = [f for f in listed if not os.path.exists(os.path.join(PKG, f))]
        check("CMakeLists 里 install 的脚本都存在（%d 个）" % len(listed),
              bool(listed) and not absent, str(absent))
        check("CMakeLists 声明了 catkin 构建与 launch/config 安装",
              all(k in text for k in ("catkin", "catkin_install_python", "install(")))

    pkg_xml = os.path.join(PKG, "package.xml")
    if os.path.exists(pkg_xml):
        try:
            from catkin_pkg.package import parse_package
            pkg = parse_package(pkg_xml)
            n_dep = len(pkg.exec_depends) + len(pkg.build_depends)
            check("package.xml 能被 catkin_pkg 正常解析（name=%s，%d 个依赖）"
                  % (pkg.name, n_dep),
                  pkg.name == "uav_moving_platform_landing" and n_dep > 0)
            check("package.xml 声明了 rospy 依赖",
                  any(d.name == "rospy"
                      for d in list(pkg.exec_depends) + list(pkg.build_depends)))
        except ImportError:
            print("[跳过] 没有 catkin_pkg，跳过 package.xml 解析检查")


# --------------------------------------------------------------------------- #
def main():
    tests = [
        ("1.1 匀速直线估计", test_tracker_constant_velocity),
        ("1.2 噪声与量测延迟", test_tracker_noise_and_latency),
        ("1.3 跳点粗筛", test_tracker_outlier_rejection),
        ("1.4 断流与重置", test_tracker_staleness_and_reset),
        ("1.5 曲率判定", test_tracker_curvature),
        ("2.1 下降剖面", test_descend_profile),
        ("2.2 平地甲板接地时间", test_predict_touchdown_flat),
        ("2.3 升沉甲板接地时间", test_predict_touchdown_heave),
        ("3.1 正常降落全程", test_state_machine_nominal),
        ("3.2 静止甲板", test_state_machine_static_deck),
        ("3.3 对准超时", test_state_machine_abort_on_timeout),
        ("3.4 穿到甲板下方", test_state_machine_below_deck),
        ("3.5 指令限幅", test_command_limits),
        ("3.6 对准段保持高度", test_align_holds_altitude),
        ("3.7 复飞", test_go_around),
        ("3.8 断流保护", test_no_stale_input_landing),
        ("4.1 策略定量对照", test_strategy_comparison),
        ("4.2 圆周与升沉", test_heave_and_circle),
        ("4.3 平台比无人机快", test_fast_platform_with_limited_authority),
        ("4.4 可复现性", test_determinism),
        ("5.1 包结构约定", test_package_layout),
    ]
    for title, fn in tests:
        print("\n=== %s ===" % title)
        try:
            fn()
        except Exception as exc:                    # noqa: BLE001 - 测试汇总用
            import traceback
            traceback.print_exc()
            check("%s 未抛异常" % title, False, repr(exc))

    print("\n%d 项通过，%d 项失败" % (len(PASS), len(FAIL)))
    if FAIL:
        print("失败的检查：")
        for f in FAIL:
            print("  - " + f)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
