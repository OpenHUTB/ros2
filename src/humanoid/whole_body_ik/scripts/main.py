#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""人形机器人全身逆运动学与可操作性分析 —— 演示入口。

运行::

    python3 main.py                # 完整演示
    python3 main.py --quick        # 跳过耗时部分
    python3 main.py --gif          # 额外生成 GIF（需要 Pillow）
    python3 main.py --outdir DIR

这个文件就是 roslaunch 里 ``type="main.py"`` 对应的可执行入口。

输出：

* 终端打印正解/逆解校验、可操作性、奇异点对比
* demo_out/manipulability_vs_elbow.png
* demo_out/singularity_damping.png
* demo_out/ik_convergence.png
"""

from __future__ import annotations

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from ik_solver import (  # noqa: E402
    adaptive_damping,
    damped_step,
    null_space_project,
    pseudoinverse_step,
    solve_position_ik,
    smallest_singular_value,
)
from kinematics import (  # noqa: E402
    make_humanoid_arm,
    make_planar_2r,
    planar_2r_fk,
    planar_2r_ik,
)
from linalg import mat_vec, norm  # noqa: E402
from manipulability import (  # noqa: E402
    condition_number,
    is_singular,
    joint_limit_cost,
    manipulability,
    manipulability_ellipsoid,
    singular_values,
)
from simple_plot import plot_series  # noqa: E402

BLUE = (40, 110, 220)
RED = (215, 60, 55)
GREEN = (35, 165, 90)
ORANGE = (235, 150, 30)
PURPLE = (140, 80, 190)
GRAY = (150, 150, 150)


def hr(title=""):
    line = "=" * 74
    if title:
        print(line)
        print(title)
        print(line)
    else:
        print(line)


#: config/ik_params.yaml 里没写或解析失败时用的缺省值
SOLVER_DEFAULTS = {"tol": 1e-6, "max_iter": 200, "damping": "adaptive"}


def load_params(path):
    """读取 config/ik_params.yaml 里的求解器参数。

    只解析 ``key: value`` 这种最简形式，避免引入 PyYAML。命令行不传
    ``--params`` 时 main() 会自动去找模块内的 config/ik_params.yaml。

    能被识别的键：tol / max_iter / damping。其中 damping 是字符串
    （``adaptive`` 或 ``fixed``），其余转成浮点或整数。
    """
    if not path or not os.path.isfile(path):
        return dict(SOLVER_DEFAULTS), False

    out = dict(SOLVER_DEFAULTS)
    with open(path, "r", encoding="utf-8") as fh:
        for raw in fh:
            line = raw.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, val = line.split(":", 1)
            key, val = key.strip(), val.strip()
            if key == "tol":
                out["tol"] = float(val)
            elif key == "max_iter":
                out["max_iter"] = int(float(val))
            elif key == "damping" and val in ("adaptive", "fixed"):
                out["damping"] = val
    return out, True


# ---------------------------------------------------------------------------
def demo_fk_ik():
    """正解：通用链 vs 解析式；逆解：数值 vs 解析。"""
    print()
    hr("一、正运动学与逆运动学校验")
    arm = make_planar_2r(1.0, 1.0)
    print("  2R 平面臂：连杆 %.2f m + %.2f m，工作空间半径 %.2f m"
          % (1.0, 1.0, arm.reach()))

    print()
    print("  正解对照（通用链 vs 解析式）：")
    print("  %-18s %-26s %10s" % ("关节角", "通用链末端位置", "偏差"))
    print("  " + "-" * 60)
    max_diff = 0.0
    for q in ([0.0, 0.0], [0.3, -0.5], [1.2, 0.8], [-2.0, 1.5]):
        p_generic, _ = arm.fk(q)
        p_analytic = planar_2r_fk(q, 1.0, 1.0)
        d = max(abs(a - b) for a, b in zip(p_generic, p_analytic))
        max_diff = max(max_diff, d)
        print("  %-18s [%7.4f, %7.4f, %7.4f]  %10.2e" % (
            "(%.2f, %.2f)" % (q[0], q[1]), p_generic[0], p_generic[1],
            p_generic[2], d))
    print("  最大偏差 = %.2e" % max_diff)

    print()
    print("  逆解对照（数值逆解 vs 解析逆解）：")
    print("  %-20s %-28s %12s %6s" % ("目标位置", "数值解 (q0, q1)", "末端误差", "迭代"))
    print("  " + "-" * 70)
    for tgt in ([1.2, 0.6], [0.0, 1.5], [1.5, -0.4]):
        res = solve_position_ik(arm, [tgt[0], tgt[1], 0.0], q0=[0.1, 0.1])
        print("  %-20s [%8.5f, %8.5f]  %12.2e %6d" % (
            "(%.2f, %.2f)" % (tgt[0], tgt[1]), res.q[0], res.q[1],
            res.error, res.iterations))

    # 解析解对比
    tgt = [1.2, 0.6]
    q_analytic = planar_2r_ik(tgt[0], tgt[1], 1.0, 1.0)
    res = solve_position_ik(arm, [tgt[0], tgt[1], 0.0], q0=[0.1, 0.1])
    print()
    print("  以目标 (1.2, 0.6) 为例：")
    print("    解析解 q = [%.6f, %.6f]" % (q_analytic[0], q_analytic[1]))
    print("    数值解 q = [%.6f, %.6f]" % (res.q[0], res.q[1]))
    print("    注：2R 有肘上/肘下两组解，数值解收敛到哪组取决于初值；")
    print("        这里两者末端位置一致就说明解是对的。")
    return arm


# ---------------------------------------------------------------------------
def demo_manipulability(outdir):
    """可操作性随肘角变化。"""
    print()
    hr("二、可操作性分析（Yoshikawa 指标）")
    arm = make_planar_2r(1.0, 1.0)

    print("  2R 臂的可操作性有闭式解：w = |l1·l2·sin(q1)|")
    print()
    print("  %8s %14s %14s %12s %10s" % ("肘角 q1", "数值 w", "解析 |sin q1|", "条件数", "是否奇异"))
    print("  " + "-" * 66)
    for q1 in (0.0, 0.2, 0.5, 1.0, math.pi / 2, 2.2, math.pi - 0.1):
        j = arm.position_jacobian([0.4, q1])[:2]
        w = manipulability(j)
        exact = abs(math.sin(q1))
        cond = condition_number(j)
        print("  %8.3f %14.9f %14.9f %12.2f %10s" % (
            q1, w, exact, cond, "是" if is_singular(j) else "否"))

    print()
    print("  w 在 q1 = π/2 时最大（肘部 90°），在 q1 = 0 或 π 时为 0（完全伸直）。")

    # 曲线
    qs, ws, conds = [], [], []
    for i in range(181):
        q1 = i * math.pi / 180.0
        j = arm.position_jacobian([0.4, q1])[:2]
        qs.append(q1)
        ws.append(manipulability(j))
        c = condition_number(j)
        conds.append(c if c < 100 else 100.0)

    plot_series(
        os.path.join(outdir, "manipulability_vs_elbow.png"),
        [("w = |sin q1|", qs, ws, BLUE)],
        title="MANIPULABILITY VS ELBOW ANGLE",
        xlabel="elbow angle q1 (rad)", ylabel="manipulability w",
    )
    plot_series(
        os.path.join(outdir, "condition_vs_elbow.png"),
        [("condition (capped 100)", qs, conds, ORANGE)],
        title="CONDITION NUMBER VS ELBOW ANGLE",
        xlabel="elbow angle q1 (rad)", ylabel="condition number",
    )
    print()
    print("  曲线已保存: manipulability_vs_elbow.png, condition_vs_elbow.png")
    return arm


# ---------------------------------------------------------------------------
def demo_ellipsoid(outdir):
    """可操作性椭球：把单位圆经雅可比映射成椭圆。

    2R 平面臂的末端速度 v = J·u，u 是关节速度单位圆。所以椭圆的形状
    直接反映了各个方向能产生多快的末端速度：
    肘部越接近伸直，椭圆越扁（某个方向几乎动不了）。
    """
    print()
    hr("二·补、可操作性椭球（末端速度椭球）")
    arm = make_planar_2r(1.0, 1.0)

    print("  v = J·u，其中 |u| = 1。椭圆的半轴长度就是 J 的奇异值。")
    print()
    print("  %8s %14s %14s" % ("肘角 q1", "椭圆长半轴", "椭圆短半轴"))

    series = []
    colors = [BLUE, ORANGE, PURPLE]
    for q1, color in zip((0.35, 1.0, math.pi / 2), colors):
        j = arm.position_jacobian([0.4, q1])[:2]
        sv = singular_values(j)

        # 采样单位圆，映射到速度空间
        xs, ys = [], []
        n = 120
        for k in range(n + 1):
            ang = 2.0 * math.pi * k / n
            u = [math.cos(ang), math.sin(ang)]
            v = [sum(j[r][c] * u[c] for c in range(2)) for r in range(2)]
            xs.append(v[0])
            ys.append(v[1])

        series.append(("q1 = %.2f" % q1, xs, ys, color))
        print("  %8.2f %14.4f %14.4f" % (q1, sv[0], sv[1]))

    plot_series(
        os.path.join(outdir, "manipulability_ellipsoid.png"),
        series,
        title="END-EFFECTOR VELOCITY ELLIPSOID",
        xlabel="vx  (m/s per rad/s)", ylabel="vy  (m/s per rad/s)",
    )
    print()
    print("  肘部越伸直（q1 越小），椭圆越扁，对应可操作性 w 越小。")
    print("  曲线已保存: manipulability_ellipsoid.png")


def demo_singularity(outdir):
    """奇异点：伪逆 vs 阻尼最小二乘。"""
    print()
    hr("三、奇异点处理：伪逆会爆炸，DLS 不会")
    arm = make_planar_2r(1.0, 1.0)

    print("  让肘角 q1 从 0.2 逐渐趋近 0（完全伸直 = 奇异位形），")
    print("  给一个沿径向的末端误差 0.01 m，比较两种解法的关节步长：")
    print()
    print("  %8s %13s %15s %15s %12s" % (
        "q1", "sigma_min", "|dq| 伪逆", "|dq| DLS", "自适应 lambda"))
    print("  " + "-" * 70)

    q1s, pinv_norms, dls_norms, sigmas, lams = [], [], [], [], []
    for q1 in (0.20, 0.10, 0.05, 0.02, 0.01, 0.005):
        j = arm.position_jacobian([0.0, q1])[:2]
        err = [0.01, 0.0]
        s = smallest_singular_value(j)
        lam = adaptive_damping(j)
        try:
            p_norm = norm(pseudoinverse_step(j, err))
            p_text = "%.4e" % p_norm
        except ValueError:
            p_norm = float("nan")
            p_text = "奇异失败"
        d_norm = norm(damped_step(j, err, lam))

        q1s.append(q1)
        pinv_norms.append(p_norm)
        dls_norms.append(d_norm)
        sigmas.append(s)
        lams.append(lam)

        print("  %8.3f %13.3e %15s %15.4e %12.4f" % (
            q1, s, p_text, d_norm, lam))

    print()
    print("  伪逆的步长按 1/sigma_min 增长（换成对数坐标就是一条斜线），")
    print("  这正是真实机器人上会看到关节速度飙到失控的原因。")
    print("  DLS 用自适应阻尼把它压住，代价是奇异方向上收敛变慢。")

    plot_series(
        os.path.join(outdir, "singularity_damping.png"),
        [("pseudo-inverse |dq|", q1s, pinv_norms, RED),
         ("DLS |dq|", q1s, dls_norms, BLUE)],
        title="STEP SIZE NEAR SINGULARITY",
        xlabel="elbow angle q1 (rad)  -- smaller q1 = closer to singular",
        ylabel="|dq| (rad)",
    )
    plot_series(
        os.path.join(outdir, "adaptive_lambda.png"),
        [("lambda", q1s, lams, PURPLE),
         ("sigma_min", q1s, sigmas, GRAY)],
        title="ADAPTIVE DAMPING",
        xlabel="elbow angle q1 (rad)", ylabel="lambda / sigma_min",
    )
    print("  曲线已保存: singularity_damping.png, adaptive_lambda.png")


# ---------------------------------------------------------------------------
def demo_redundancy(outdir):
    """7 自由度手臂：冗余度与零空间。"""
    print()
    hr("四、冗余自由度与零空间")
    arm = make_humanoid_arm()
    print("  %r" % (arm,))
    print("  自由度 7 > 任务维度 6，所以有 1 维零空间可以自由调整。")

    q = [0.2, -0.3, 0.1, 0.8, 0.0, 0.3, 0.0]
    p, _r = arm.fk(q)
    j6 = arm.jacobian(q)
    jp = arm.position_jacobian(q)

    print()
    print("  %-22s %-14s" % ("指标", "数值"))
    print("  " + "-" * 40)
    sv = singular_values(j6)
    print("  %-22s %s" % ("奇异值 (6 个)", ", ".join("%.4f" % x for x in sv)))
    print("  %-22s %.6f" % ("可操作性 w (6x7)", manipulability(j6)))
    print("  %-22s %.6f" % ("可操作性 w (3x7 位置)", manipulability(jp)))
    print("  %-22s %.4f" % ("条件数", condition_number(j6)))
    print("  %-22s %.6f" % ("最小奇异值", sv[-1]))

    ell = manipulability_ellipsoid(jp)
    print("  %-22s [%s]" % ("位置椭球半轴",
                            ", ".join("%.4f" % x for x in ell["axes"])))
    print("  %-22s %.6f" % ("椭球体积", ell["volume"]))

    # 零空间验证
    print()
    print("  零空间投影验证：")
    z = [1.0, -0.5, 0.3, -0.2, 0.8, -0.6, 0.4]
    proj = null_space_project(jp, z, lam=1e-8)
    print("    任取一个方向 z，|z| = %.6f" % norm(z))
    print("    投影到零空间后 |(I - J+J)z| = %.6f" % norm(proj))
    print("    |J · 投影| = %.2e   ← 接近 0 说明末端不动"
          % norm(mat_vec(jp, proj)))

    # 关节限位代价
    print()
    print("  关节限位次级目标（用于零空间避限位）：")
    cost, grad = joint_limit_cost(q, arm.joints)
    print("    当前位形代价 = %.6f" % cost)
    print("    梯度 = [%s]" % ", ".join("%+.3e" % g for g in grad))
    q_bad = [0.0, -0.3, 0.1, 0.0, 0.0, 0.0, 0.0]
    cost_bad, _ = joint_limit_cost(q_bad, arm.joints)
    print("    关节贴限位时代价 = %.6f  （明显更大）" % cost_bad)
    return arm


# ---------------------------------------------------------------------------
def demo_tracking(arm, outdir, solver_kw=None):
    """让 7 自由度手臂末端画圆，记录收敛过程。"""
    print()
    hr("五、末端轨迹跟踪（画一个圆）")
    radius = 0.12
    center = [0.0, 0.0, -0.45]
    n = 24
    print("  目标：末端在 z = %.2f 平面上沿半径 %.2f m 的圆走一圈" % (
        center[2], radius))

    q = None
    errs = []
    its = []
    for i in range(n + 1):
        ang = 2.0 * math.pi * i / n
        target = [center[0] + radius * math.cos(ang),
                  center[1],
                  center[2] + radius * math.sin(ang)]
        res = solve_position_ik(arm, target, q0=q, **(solver_kw or SOLVER_DEFAULTS))
        q = res.q
        errs.append(res.error)
        its.append(res.iterations)

    print("  跟踪完成：%d 个目标点" % (n + 1))
    print("  最大末端误差 = %.2e m" % max(errs))
    print("  平均迭代次数 = %.1f" % (sum(its) / len(its)))
    print("  最大迭代次数 = %d" % max(its))

    idx = list(range(len(errs)))
    # 误差在 1e-10 ~ 1e-6 之间，直接用米作图刻度全是 0.00，
    # 换成纳米（1e-9 m）刻度才看得清。
    errs_nm = [e * 1e9 for e in errs]
    plot_series(
        os.path.join(outdir, "ik_convergence.png"),
        [("end-effector error", idx, errs_nm, BLUE)],
        title="IK TRACKING ERROR",
        xlabel="waypoint index on the circle", ylabel="error (nm)",
    )
    plot_series(
        os.path.join(outdir, "ik_iterations.png"),
        [("iterations", idx, its, GREEN)],
        title="ITERATIONS PER TARGET",
        xlabel="waypoint index on the circle", ylabel="iterations",
    )
    print("  曲线已保存: ik_convergence.png, ik_iterations.png")


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="人形机器人全身逆运动学与可操作性分析")
    ap.add_argument("--outdir", default=os.path.join(HERE, "demo_out"))
    ap.add_argument("--quick", action="store_true", help="跳过轨迹跟踪")
    ap.add_argument("--gif", action="store_true", help="额外生成 GIF（需要 Pillow）")
    ap.add_argument("--params", default=None,
                    help="求解器参数文件，默认用模块内 config/ik_params.yaml")
    # roslaunch 启动 <node> 时会在命令行末尾追加 __name:=... __log:=...
    # 这两个是 ROS 保留参数。parse_args() 遇到不认识的参数会直接报错退出，
    # 节点连 main() 都进不去；用 parse_known_args() 忽略掉它们。
    args, _unknown = ap.parse_known_args()

    if not os.path.isdir(args.outdir):
        os.makedirs(args.outdir)

    print()
    hr("人形机器人全身逆运动学与可操作性分析")
    print()
    print("  内容：正运动学 → 雅可比 → 逆解（伪逆 / 阻尼最小二乘 / 零空间）")
    print("        → 可操作性椭球 → 奇异点处理 → 冗余自由度利用")
    print("  依赖：只用 Python 标准库")

    # 求解器参数：命令行优先，其次模块内 config/ik_params.yaml，最后代码缺省
    params_path = args.params
    if params_path is None:
        cand = os.path.join(os.path.dirname(HERE), "config", "ik_params.yaml")
        if os.path.isfile(cand):
            params_path = cand
    solver_kw, found = load_params(params_path)
    print()
    print("  求解器参数: tol=%g, max_iter=%d, damping=%s%s" % (
        solver_kw["tol"], solver_kw["max_iter"], solver_kw["damping"],
        "" if found else "（未找到参数文件，用代码缺省）"))

    demo_fk_ik()
    demo_manipulability(args.outdir)
    demo_ellipsoid(args.outdir)
    demo_singularity(args.outdir)
    arm7 = demo_redundancy(args.outdir)
    if not args.quick:
        demo_tracking(arm7, args.outdir, solver_kw=solver_kw)

    if args.gif:
        print()
        hr("六、生成动图")
        try:
            from animate import animate_arm_circle
            animate_arm_circle(os.path.join(args.outdir, "ik_circle.gif"))
        except ImportError:
            print("  需要 Pillow：pip3 install pillow")

    print()
    hr("完成")
    print("  曲线目录: %s" % os.path.abspath(args.outdir))
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
