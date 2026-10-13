#!/usr/bin/env python3
"""领航-跟随编队控制的无仿真器回放（控制器回归测试）。

背景：HoloOcean 自 2.x 起需要 Epic Games 账号授权才能获取发行包，部分开发环境无法安装。
本脚本用**简化运动学模型**替代 HoloOcean，复现 `formation_controller.py` 的**外环控制律**，
以便在没有仿真器的机器上验证编队逻辑、产出可复现的误差数据与曲线。

它验证什么：外环控制律、机体系偏移的期望位置生成、收敛性与稳态精度。
它不验证什么：HoloOcean 的 6-DOF 水动力、推进器动态、传感器噪声（这些需要真实场景跑）。

用法（无需 ROS，无需仿真器）：
    python3 formation_sim.py --duration 80 --outdir .
输出：
    formation_sim_metrics.csv   逐帧采样
    formation_sim_error.png     编队误差曲线（可直接放进章节）
    （终端）                    四项指标 + 可直接粘贴的 Markdown 表格
"""

import argparse
import csv
import math
import os
import sys

# ------------------------------------------------------------------ 模型参数
DT = 0.02                 # 仿真步长（50 Hz，与插件 updateRate 一致）

# ---- 外环参数：与 formation_controller.py 的默认值保持一致 ----
K_POS = 0.25              # 位置比例增益 (1/s)
K_DAMP = 0.8              # 相对速度阻尼增益
LEADER_SPEED = 1.2        # 领航者前进速度 (m/s)
TURN_START = 20.0         # 领航者开始转向的时刻 (s)
TURN_RATE = 1.0           # 领航者转向角速率 (deg/s)，受被控对象转向能力上限约束（见章节 6.4）
TURN_TOTAL = 90.0         # 领航者总转向角 (deg)

K_PSI = 1.20              # 航向一阶响应 (1/s)
K_V = 1.50                # 速度一阶响应 (1/s)
K_Z = 1.00                # 深度一阶响应 (1/s)

Z_MIN, Z_MAX = 0.5, 20.0
DEPTH_SIGN = -1.0         # z 坐标 -> 深度指令的符号（HoloOcean 中 z 向下为负）


def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Vehicle:
    """简化运动学：位置 + 航向 + 前进速度，一阶响应。"""

    def __init__(self, x, y, z, psi=0.0, v=0.0):
        self.x, self.y, self.z, self.psi, self.v = x, y, z, psi, v

    def step(self, psi_cmd, v_cmd, z_cmd):
        self.psi += K_PSI * wrap_pi(psi_cmd - self.psi) * DT
        self.v += K_V * (v_cmd - self.v) * DT
        self.z += K_Z * (z_cmd - self.z) * DT
        self.x += self.v * math.cos(self.psi) * DT
        self.y += self.v * math.sin(self.psi) * DT

    @property
    def speed(self):
        return self.v


def leader_command(t):
    """领航者航迹脚本：直行 -> 限角速率转向 90° -> 再直行。

    角速率限制（TURN_RATE）来自被控对象的物理约束：鱼雷模型的 Nomoto 增益给出
    约 3.75 deg/s 的最大偏航角速度，直接下发 90° 阶跃会让跟随者无法跟踪（见章节 6.4）。
    """
    if t < TURN_START:
        psi = 0.0
    else:
        psi = math.radians(min(TURN_TOTAL, (t - TURN_START) * TURN_RATE))
    return psi, LEADER_SPEED


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=80.0)
    ap.add_argument('--offset', type=float, nargs=3, default=[-3.0, 0.0, 0.0],
                    help='跟随者相对领航者的机体系偏移 [dx dy dz]')
    ap.add_argument('--thresh', type=float, default=0.3, help='收敛判据 (m)')
    ap.add_argument('--hold', type=float, default=0.0, help='判据需连续满足的时长 (s)')
    ap.add_argument('--outdir', default='.')
    args = ap.parse_args()

    dx, dy, dz = args.offset

    # 领航者从原点出发；跟随者从期望编队位置出发（初始编队误差 ≈ 0），与 6.3 节的实测条件一致
    leader = Vehicle(0.0, 0.0, -3.0)
    follower = Vehicle(dx, dy, -3.0 + dz)

    leader_yaw_rate = 0.0
    prev_psi = leader.psi

    rows = []
    t = 0.0
    while t <= args.duration:
        l_psi_cmd, l_v_cmd = leader_command(t)
        leader.step(l_psi_cmd, l_v_cmd, -3.0)

        # 领航者偏航角速率（低通估计），供目标点速度前馈使用
        rate = wrap_pi(leader.psi - prev_psi) / DT
        leader_yaw_rate = 0.8 * leader_yaw_rate + 0.2 * rate
        prev_psi = leader.psi

        # ---- 与 formation_controller.py 相同的外环控制律：速度矢量控制 ----
        c, s = math.cos(leader.psi), math.sin(leader.psi)
        tx = leader.x + (c * dx - s * dy)
        ty = leader.y + (s * dx + c * dy)
        tz = leader.z + dz
        ex, ey, ez = tx - follower.x, ty - follower.y, tz - follower.z
        dist = math.sqrt(ex * ex + ey * ey)

        # 目标点速度 = 领航者平动速度 + 偏移点随领航者偏航旋转的切向速度
        vtx = leader.speed * math.cos(leader.psi) - leader_yaw_rate * (ty - leader.y)
        vty = leader.speed * math.sin(leader.psi) + leader_yaw_rate * (tx - leader.x)

        # 期望速度矢量 = 目标点速度(前馈) + 位置误差反馈 − 相对速度阻尼
        fvx = follower.speed * math.cos(follower.psi)
        fvy = follower.speed * math.sin(follower.psi)
        vdx = vtx + K_POS * ex - K_DAMP * (fvx - vtx)
        vdy = vty + K_POS * ey - K_DAMP * (fvy - vty)

        psi_cmd = math.atan2(vdy, vdx)
        v_cmd = math.hypot(vdx, vdy)

        # 深度指令（正值向下的“深度”），再换算回模型里的 z 坐标
        dep_cmd = max(Z_MIN, min(Z_MAX, DEPTH_SIGN * tz))
        z_cmd = DEPTH_SIGN * dep_cmd

        follower.step(psi_cmd, v_cmd, z_cmd)

        err = math.sqrt(ex * ex + ey * ey + ez * ez)
        rows.append((t, leader.x, leader.y, leader.z, follower.x, follower.y, follower.z, err, v_cmd))
        t += DT

    errs = [r[7] for r in rows]
    rms = math.sqrt(sum(e * e for e in errs) / len(errs))
    emax = max(errs)

    # 收敛时间：误差峰值之后首次回到阈值以内。
    # 跟随者从期望队形出发、起点即在阈值内，直接取"首个低于阈值"会恒为 0，没有意义。
    i_pk = max(range(len(errs)), key=lambda i: errs[i])
    conv = None
    need = int(args.hold / DT)
    for i in range(i_pk, len(errs) - need):
        if all(e < args.thresh for e in errs[i:i + need]):
            conv = rows[i][0]
            break

    tail = errs[int(len(errs) * 0.7):]
    steady = sum(tail) / len(tail)
    vstd = None
    if conv is not None:
        vs = [r[8] for r in rows if r[0] >= conv]
        if vs:
            m = sum(vs) / len(vs)
            vstd = math.sqrt(sum((v - m) ** 2 for v in vs) / len(vs))

    csv_path = os.path.join(args.outdir, 'formation_sim_metrics.csv')
    with open(csv_path, 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['t_sec', 'leader_x', 'leader_y', 'leader_z',
                    'follower_x', 'follower_y', 'follower_z', 'formation_error_m', 'speed_cmd_mps'])
        w.writerows(rows)

    print('简化运动学模型回放（无仿真器）')
    print('  偏移 [%.1f %.1f %.1f] m，时长 %.0f s，步长 %.3f s，采样 %d 帧'
          % (dx, dy, dz, args.duration, DT, len(rows)))
    print('  原始数据：%s' % csv_path)
    print()
    print('| 指标 | 数值 |')
    print('| --- | --- |')
    print('| 编队误差 RMS | %.3f m |' % rms)
    print('| 最大瞬时误差 | %.3f m |' % emax)
    if conv is None:
        print('| 收敛时间（到 ±%.1f m 并保持 %.0f s） | 未在窗口内收敛 |' % (args.thresh, args.hold))
    else:
        print('| 峰值后收敛时间（回到 ±%.1f m 并保持 %.0f s） | %.2f s |' % (args.thresh, args.hold, conv))
    print('| 稳态误差（后 30%% 采样均值） | %.3f m |' % steady)
    if vstd is not None:
        print('| 收敛后速度指令波动（标准差） | %.3f m/s |' % vstd)

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        plt.figure(figsize=(8, 3.6))
        plt.plot([r[0] for r in rows], errs, linewidth=1.4, label='编队误差（简化运动学模型）')
        plt.axhline(args.thresh, color='gray', linestyle='--', linewidth=1,
                    label='收敛阈值 %.1f m' % args.thresh)
        if conv is not None:
            plt.axvline(conv, color='tab:green', linestyle=':', linewidth=1.2,
                        label='收敛时刻 %.1f s' % conv)
        plt.xlabel('时间 (s)')
        plt.ylabel('编队误差 (m)')
        plt.title('编队误差收敛曲线（无仿真器回放）')
        plt.grid(alpha=0.3)
        plt.legend()
        plt.tight_layout()
        png = os.path.join(args.outdir, 'formation_sim_error.png')
        plt.savefig(png, dpi=160)
        print()
        print('曲线图：%s' % png)
    except ImportError:
        print()
        print('（未安装 matplotlib，跳过绘图：pip install matplotlib 后重跑即可出图）')

    return 0


if __name__ == '__main__':
    sys.exit(main())
