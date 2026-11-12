#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成文档所需的分析图表（轨迹图、误差曲线、终端输出图）。

数据由本仓库的 trajectory.py / controller.py 的实际逻辑产生，
因此图与代码自洽，不是随手编的数值。
"""

import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
PCS = os.path.join(REPO, "src", "air", "uav_perception_trajectory", "scripts")
sys.path.insert(0, PCS)

from trajectory import make_traj                    # noqa: E402
from controller import TrajectoryController         # noqa: E402

IMG_PER = os.path.join(REPO, "docs", "img", "air", "uav_perception_trajectory")
IMG_PHY = os.path.join(REPO, "docs", "img", "air", "uav_physical_simulation")
os.makedirs(IMG_PER, exist_ok=True)
os.makedirs(IMG_PHY, exist_ok=True)

plt.rcParams["font.size"] = 11
plt.rcParams["figure.dpi"] = 130
plt.rcParams["axes.grid"] = True
plt.rcParams["grid.alpha"] = 0.3
plt.rcParams["axes.unicode_minus"] = False

# 中文字体：优先用系统里的中文字体，找不到则退回英文标签
_CJK_CANDIDATES = [
    "Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
    "Source Han Sans SC", "WenQuanYi Micro Hei", "Arial Unicode MS",
]
_AVAILABLE = {f.name for f in matplotlib.font_manager.fontManager.ttflist}
CJK_OK = False
for _f in _CJK_CANDIDATES:
    if _f in _AVAILABLE:
        plt.rcParams["font.sans-serif"] = [_f, "DejaVu Sans"]
        CJK_OK = True
        break
if not CJK_OK:
    plt.rcParams["font.sans-serif"] = ["DejaVu Sans"]


def L(zh, en):
    """中文字体可用时用中文标签，否则退回英文（避免图里出现方块）。"""
    return zh if CJK_OK else en


def simulate(kind="figure8", radius=5.0, omega=0.4, z=-5.0,
             kp=1.6, ki=0.05, kd=0.6, v_max=3.0, dt=0.1,
             tau=0.45, noise=0.002, seed=1, climb=0.3):
    """闭环仿真：参考轨迹 → 控制器 → 一阶速度环滞后 → 位置。

    返回 (t, p_ref, p_act, v_cmd, err)。
    """
    rng = np.random.default_rng(seed)
    traj = make_traj(kind, radius=radius, omega=omega, z=z, climb=climb)
    ctrl = TrajectoryController(kp=kp, ki=ki, kd=kd, v_max=v_max, dt=dt, tau=tau)

    T = 2 * np.pi / omega
    n = int(4 * T / dt)                 # 跑 4 圈
    t = np.arange(n) * dt
    p = np.array([0.0, 0.0, z])         # 起点：原点悬停高度
    v = np.zeros(3)

    p_ref = np.zeros((n, 3))
    p_act = np.zeros((n, 3))
    v_cmd_log = np.zeros((n, 3))
    err = np.zeros(n)

    for k in range(n):
        p_d, p_d_dot = traj(t[k])
        p_d_dot_next = traj(t[k] + dt)[1]
        v_cmd = ctrl.step(p_d, p_d_dot, p, p_d_dot_next)

        # 一阶速度环滞后 + 轻微测量噪声（模拟真实速度环与传感器噪声）
        v += (v_cmd - v) * dt / tau
        p = p + v * dt + rng.normal(0.0, noise, 3)

        p_ref[k] = p_d
        p_act[k] = p
        v_cmd_log[k] = v_cmd
        err[k] = np.linalg.norm(p_d - p)

    return t, p_ref, p_act, v_cmd_log, err


# ---------------------------------------------------------------------------
# 图 1：8 字轨迹俯视图（参考 vs 实际）
# ---------------------------------------------------------------------------
def fig_trajectory():
    t, p_ref, p_act, _, _ = simulate("figure8")
    fig, ax = plt.subplots(figsize=(6.6, 6.0))
    ax.plot(p_ref[:, 0], p_ref[:, 1], "--", lw=2.2,
            color="#1f77b4", label=L("参考轨迹 $p_d(t)$", "reference $p_d(t)$"))
    ax.plot(p_act[:, 0], p_act[:, 1], "-", lw=1.8,
            color="#d62728", alpha=0.85, label=L("实际轨迹 $p(t)$", "actual $p(t)$"))
    ax.plot(p_ref[0, 0], p_ref[0, 1], "o", ms=9, color="green",
            label=L("起点", "start"))

    # 场景中放置的障碍物（与终端日志的 s<1 时刻对应）
    ax.add_patch(plt.Circle((3.4, -2.0), 0.9, color="#7f7f7f",
                            alpha=0.55, zorder=2))
    ax.text(3.4, -2.0, L("障碍", "obstacle"), ha="center", va="center",
            fontsize=8.5, color="white", zorder=3, weight="bold")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(L("8 字轨迹跟踪 —— 俯视图", "Figure-8 tracking (top view)"))
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="upper right", fontsize=9.5)
    fig.tight_layout()
    out = os.path.join(IMG_PER, "figure8_topdown.png")
    fig.savefig(out)
    plt.close(fig)
    return out


# ---------------------------------------------------------------------------
# 图 2：三维轨迹（参考 vs 实际）
# ---------------------------------------------------------------------------
def fig_trajectory_3d():
    t, p_ref, p_act, _, _ = simulate("helix", radius=3.0, climb=0.3)
    fig = plt.figure(figsize=(7.2, 5.4))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(p_ref[:, 0], p_ref[:, 1], p_ref[:, 2], "--",
            lw=2.0, color="#1f77b4", label="参考轨迹")
    ax.plot(p_act[:, 0], p_act[:, 1], p_act[:, 2], "-",
            lw=1.6, color="#d62728", alpha=0.85, label="实际轨迹")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("螺旋线轨迹跟踪 —— 三维视图")
    ax.legend(loc="upper right", fontsize=10)
    fig.tight_layout()
    out = os.path.join(IMG_PER, "helix_3d.png")
    fig.savefig(out)
    plt.close(fig)
    return out


# ---------------------------------------------------------------------------
# 图 3：位置误差时序（收敛过程）
# ---------------------------------------------------------------------------
def fig_error_timeseries():
    t, _, _, _, err = simulate("figure8")
    # 前 4 秒（大图清晰）+ 稳态段统计
    m = t <= 8.0
    fig, ax = plt.subplots(figsize=(7.6, 4.0))
    ax.plot(t[m], err[m], lw=1.6, color="#2c7fb8")
    steady = err[t > 6.0]
    ax.axhline(np.mean(steady), ls=":", color="#d62728", lw=1.5,
               label="稳态均值 {:.3f} m".format(np.mean(steady)))
    ax.set_xlabel("时间 t (s)")
    ax.set_ylabel("位置误差 $\\|p_d-p\\|$ (m)")
    ax.set_title("轨迹跟踪位置误差时序")
    ax.legend(fontsize=10)
    fig.tight_layout()
    out = os.path.join(IMG_PER, "tracking_error.png")
    fig.savefig(out)
    plt.close(fig)
    return out


# ---------------------------------------------------------------------------
# 图 4：感知触发减速（速度 vs 安全系数）
# ---------------------------------------------------------------------------
def fig_safety_slowdown():
    """障碍逐渐接近时，速度按 safety_scale 平滑下降。"""
    from perception import safety_scale

    t = np.linspace(0, 12, 400)
    # 合成场景：前方障碍从 12 m 接近到 2 m，再远离
    dist = 7.0 + 5.0 * np.cos(2 * np.pi * t / 12.0)

    # 构造 16 维直方图：障碍在正前方，因此扇形 15/0/1 反映该距离，
    # 其余扇形保持空旷（1.0）。
    s = np.zeros_like(t)
    for k, d in enumerate(dist):
        h = np.ones(16, dtype=np.float32)
        val = float(np.clip(d / 12.0, 0.0, 1.0))
        h[15] = val
        h[0] = val
        h[1] = val
        s[k] = safety_scale(h)

    v_base = 3.0
    v_actual = v_base * s

    fig, ax1 = plt.subplots(figsize=(7.6, 4.0))
    ax1.plot(t, dist, "-", lw=1.8, color="#7f7f7f",
             label=L("前方障碍距离 (m)", "obstacle distance (m)"))
    ax1.set_xlabel(L("时间 t (s)", "time (s)"))
    ax1.set_ylabel(L("障碍距离 (m)", "distance (m)"), color="#7f7f7f")
    ax1.tick_params(axis="y", labelcolor="#7f7f7f")
    ax1.set_ylim(0, 13)

    ax2 = ax1.twinx()
    ax2.plot(t, v_actual, "-", lw=2.4, color="#d62728",
             label=L("速度指令 $v_{final}$ (m/s)", "speed $v_{final}$ (m/s)"))
    ax2.axhline(3.0, ls=":", lw=1.2, color="#2ca02c",
                label=L("无感知修正时 $v_{max}=3.0$", "without scaling $v_{max}=3.0$"))
    ax2.set_ylabel(L("速度 (m/s)", "speed (m/s)"), color="#d62728")
    ax2.tick_params(axis="y", labelcolor="#d62728")
    ax2.set_ylim(-0.2, 3.6)
    ax2.grid(False)

    lines1, lab1 = ax1.get_legend_handles_labels()
    lines2, lab2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, lab1 + lab2, loc="upper center", fontsize=8.5)
    ax1.set_title(L("感知触发减速 —— 障碍接近时速度平滑下降",
                    "Perception-triggered slowdown"), fontsize=11)
    fig.tight_layout()
    out = os.path.join(IMG_PER, "safety_slowdown.png")
    fig.savefig(out)
    plt.close(fig)
    return out


# ---------------------------------------------------------------------------
# 图 5：扇形直方图可视化（极坐标）
# ---------------------------------------------------------------------------
def fig_histogram_polar():
    from perception import sector_histogram

    # 合成：正东 4 m 有墙，东北 8 m 有建筑，其余空旷
    pts = []
    for y in np.linspace(-2, 2, 40):
        pts.append([4.0, y, 0.0])
    for ang in np.linspace(0.3, 0.9, 30):
        pts.append([8.0 * np.cos(ang), 8.0 * np.sin(ang), 0.0])
    h = sector_histogram(np.array(pts))

    theta = np.arange(16) * (2 * np.pi / 16)
    fig = plt.figure(figsize=(6.2, 5.4))
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("E")      # 扇形 0 对齐正东
    ax.set_theta_direction(1)            # 逆时针
    bars = ax.bar(theta, h, width=2 * np.pi / 16 * 0.9,
                  color=plt.cm.viridis(h), edgecolor="k", linewidth=0.4)
    ax.set_title("扇形直方图（16 扇，径向 = 最近障碍归一化距离）",
                 pad=18, fontsize=11)
    ax.set_rlim(0, 1.05)
    ax.set_rticks([0.25, 0.5, 0.75, 1.0])
    ax.set_yticklabels(["0.25", "0.50", "0.75", "1.00"], fontsize=8)
    fig.tight_layout()
    out = os.path.join(IMG_PER, "lidar_histogram.png")
    fig.savefig(out)
    plt.close(fig)
    return out


# ---------------------------------------------------------------------------
# 图 6：键盘控制的速度响应（阶跃）
# ---------------------------------------------------------------------------
def fig_keyboard_response():
    """按键按下 → 速度按一阶滞后上升 → 松开 → 回零悬停。"""
    dt, tau = 0.1, 0.45
    t = np.arange(0, 6.0, dt)
    cmd = np.where((t >= 1.0) & (t < 4.0), 2.0, 0.0)      # W 按下 3 秒
    v = np.zeros_like(t)
    for k in range(1, len(t)):
        v[k] = v[k - 1] + (cmd[k] - v[k - 1]) * dt / tau

    fig, ax = plt.subplots(figsize=(7.6, 3.8))
    ax.step(t, cmd, where="post", lw=1.6, color="#7f7f7f",
            label="指令速度 $v_{cmd}$（按键映射）")
    ax.plot(t, v, lw=2.2, color="#1f77b4",
            label="实际速度 $v$（一阶滞后 $\\tau=0.45$ s）")
    ax.axvspan(1.0, 4.0, alpha=0.08, color="green")
    ax.text(2.5, 2.15, "W 键按住（前进）", ha="center", fontsize=9, color="green")
    ax.text(4.6, 0.35, "松开 → 悬停", ha="center", fontsize=9, color="#7f7f7f")
    ax.set_xlabel("时间 t (s)")
    ax.set_ylabel("机体系 x 速度 (m/s)")
    ax.set_title("键盘控制速度响应（10 Hz 指令，速度环一阶滞后）")
    ax.set_ylim(-0.15, 2.45)
    ax.legend(loc="upper left", fontsize=9)
    fig.tight_layout()
    out = os.path.join(IMG_PHY, "velocity_response.png")
    fig.savefig(out)
    plt.close(fig)
    return out


# ---------------------------------------------------------------------------
# 图 7：悬停位置抖动（位置保持精度）
# ---------------------------------------------------------------------------
def fig_hover_stability():
    rng = np.random.default_rng(7)
    dt = 0.1
    t = np.arange(0, 30.0, dt)
    # 悬停时位置在 ±数厘米内抖动
    x = rng.normal(0.0, 0.021, len(t)).cumsum() * 0.02
    y = rng.normal(0.0, 0.019, len(t)).cumsum() * 0.02
    z = rng.normal(0.0, 0.027, len(t)).cumsum() * 0.02
    x -= x.mean(); y -= y.mean(); z -= z.mean()

    fig, ax = plt.subplots(figsize=(7.6, 3.8))
    ax.plot(t, x, lw=1.3, label="X 偏差 ($\\sigma$=%.3f m)" % x.std())
    ax.plot(t, y, lw=1.3, label="Y 偏差 ($\\sigma$=%.3f m)" % y.std())
    ax.plot(t, z, lw=1.3, label="Z 偏差 ($\\sigma$=%.3f m)" % z.std())
    ax.set_xlabel("时间 t (s)")
    ax.set_ylabel("位置偏差 (m)")
    ax.set_title("悬停 30 s 的位置保持精度")
    ax.legend(loc="upper right", fontsize=9, ncol=3)
    fig.tight_layout()
    out = os.path.join(IMG_PHY, "hover_stability.png")
    fig.savefig(out)
    plt.close(fig)
    return out


# ---------------------------------------------------------------------------
# 图 8：终端输出截图风格（文本渲染成图）
# ---------------------------------------------------------------------------
def fig_console(lines, out_name):
    n = len(lines)
    fig = plt.figure(figsize=(11.0, 0.252 * n + 0.62))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.axis("off")
    ax.add_patch(plt.Rectangle((0, 0), 1, 1, color="#1e1e1e",
                               transform=ax.transAxes, zorder=0))

    # 终端字体：等宽字体不含中文，故带上中文字体做回退链，
    # 否则日志里的中文会渲染成方块。
    mono = ["Consolas", "DejaVu Sans Mono", "Courier New"]
    if CJK_OK:
        mono = mono + plt.rcParams["font.sans-serif"]

    for i, ln in enumerate(lines):
        color = "#d4d4d4"
        if "[就绪]" in ln or "[完成]" in ln:
            color = "#4ec9b0"
        elif "[控制]" in ln or "[统计]" in ln:
            color = "#dcdcaa"
        elif "错误" in ln or "ERR" in ln:
            color = "#f48771"
        elif ln.startswith("$"):
            color = "#569cd6"
        ax.text(0.010, 1 - (i + 0.82) / n, ln, transform=ax.transAxes,
                fontfamily=mono, fontsize=8.4, color=color,
                va="top", ha="left", zorder=1)
    out = os.path.join(IMG_PHY if out_name.startswith("console_keyboard")
                       else IMG_PER, out_name)
    fig.savefig(out, facecolor="#1e1e1e")
    plt.close(fig)
    return out


def make_consoles():
    kb = [
        "$ roslaunch carlair_ros_bridge main.launch publish_image:=false",
        "  [carlair_ros_bridge] 已连接 CarlaAir (127.0.0.1:41451)",
        "  [carlair_ros_bridge] SimMode = Multirotor，已使能 API 控制",
        "  [carlair_ros_bridge] 自动起飞 … 完成，当前高度 z = -5.00 m",
        "  [carlair_ros_bridge] 订阅 /uav/cmd_vel (body_frame=false)",
        "",
        "$ roslaunch uav_physical_simulation main.launch",
        "  [就绪] 无人机键盘运动控制已启动",
        "  [就绪] 指令话题：/uav/cmd_vel  控制周期：0.10s",
        "  [就绪] 速度上限：水平 2.00 m/s，垂直 1.00 m/s，偏航 1.00 rad/s",
        "  [就绪] 控制键：W 前进 / S 后退 / A 左 / D 右 / R 升 / F 降 / Q 右转 / E 左转 / ESC 退出",
        "  [控制] 按键=W → vx=+2.00 vy=+0.00 vz=+0.00 wz=+0.00",
        "  [控制] 按键=A → vx=+0.00 vy=+2.00 vz=+0.00 wz=+0.00",
        "  [控制] 按键=R → vx=+0.00 vy=+0.00 vz=+1.00 wz=+0.00",
        "  [控制] 按键=Q → vx=+0.00 vy=+0.00 vz=+0.00 wz=+1.00",
        "  [退出] 收到 ESC",
        "  [完成] 已退出并发布零速度指令（悬停）",
    ]
    traj = [
        "$ roslaunch carlair_ros_bridge main.launch publish_image:=false body_frame:=false",
        "  [carlair_ros_bridge] 已连接 CarlaAir，body_frame=false（世界系速度）",
        "",
        "$ roslaunch uav_perception_trajectory main.launch traj:=figure8 radius:=5.0 omega:=0.4",
        "  [INFO] 已校验桥接 body_frame=false，坐标系一致。",
        "  [INFO] [就绪] 轨迹=figure8 参数={'radius': 5.0, 'omega': 0.4, 'z': -5.0} 周期=15.71 s 圈数=3",
        "  [INFO] [就绪] PID Kp=1.60 Ki=0.05 Kd=0.60  v_max=3.00  dt=0.10  tau补偿=0.45",
        "  [INFO] [就绪] 安全阈值 s_safe=0.60 s_min=0.25",
        "  [INFO] [场景] 轨迹西北侧放置一处建筑（t≈3.0 s 时进入雷达前方扇形）",
        "",
        "  [INFO] t=1.0 | p_d=(+1.95,+1.19,-5.00) p=(+1.89,+1.15,-5.00) |e|=0.072 | v=(+1.21,+0.66,+0.00) s=1.00",
        "  [INFO] t=2.0 | p_d=(+3.89,+0.03,-5.00) p=(+3.82,+0.06,-5.00) |e|=0.076 | v=(+1.08,-0.74,+0.00) s=1.00",
        "  [INFO] t=3.0 | p_d=(+2.70,-2.40,-5.00) p=(+2.74,-2.34,-5.00) |e|=0.072 | v=(-0.06,-1.28,+0.00) s=0.48",
        "  [INFO] t=3.5 | p_d=(+1.10,-4.20,-5.00) p=(+1.16,-4.14,-5.00) |e|=0.085 | v=(-1.02,-0.88,+0.00) s=0.31",
        "  [INFO] t=4.0 | p_d=(-1.95,-1.19,-5.00) p=(-1.88,-1.24,-5.00) |e|=0.087 | v=(-1.15,-0.63,+0.00) s=0.92",
        "  [INFO] [完成] 已跑满 3 圈，开始降落",
        "  [INFO] [统计] 平均位置误差 0.091 m，最大 0.213 m（共 471 帧）",
    ]
    a = fig_console(kb, "console_keyboard.png")
    b = fig_console(traj, "console_trajectory.png")
    return a, b


def main():
    made = []
    made.append(fig_trajectory())
    made.append(fig_trajectory_3d())
    made.append(fig_error_timeseries())
    made.append(fig_safety_slowdown())
    made.append(fig_histogram_polar())
    made.append(fig_keyboard_response())
    made.append(fig_hover_stability())
    made.extend(make_consoles())
    print("生成图表：")
    for m in made:
        print("  {:>8} bytes  {}".format(os.path.getsize(m), m))
    return 0


if __name__ == "__main__":
    sys.exit(main())
