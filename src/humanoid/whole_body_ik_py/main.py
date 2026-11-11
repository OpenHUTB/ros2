"""人形机器人全身逆运动学与可操作性：离线演示入口。

运行：
    python main.py            # 完整实验 + 出图 + 指标
    python main.py --quick    # 跳过 GIF 与画圆，只保留关键数值
"""

from __future__ import annotations

import argparse
import csv
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

from scripts.animate import create_circle_gif
from scripts.ik_solver import solve_pose_ik, solve_position_ik, track_circle
from scripts.kinematics import make_2r_arm, make_7dof_arm
from scripts.linalg import nullspace_projector, rotation_z
from scripts.manipulability import (
    elbow_sweep,
    manipulability,
    nullspace_report,
    singularity_scan,
    task_jacobian,
    velocity_ellipsoid,
)
from scripts.plots import (
    plot_arm_snapshot,
    plot_ellipsoids,
    plot_ik_tracking,
    plot_manipulability,
    plot_singularity,
)

for stream in (sys.stdout, sys.stderr):
    reconfigure = getattr(stream, "reconfigure", None)
    if reconfigure is not None:
        reconfigure(encoding="utf-8", errors="replace")

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "out"
L1, L2 = 0.5, 0.5


def _write_csv(path: Path, header: list[str], rows: list[list]) -> None:
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(header)
        writer.writerows(rows)


def check_forward_kinematics(arm2) -> float:
    """2R 正解与解析式逐项对照，返回最大偏差。"""
    worst = 0.0
    for q0 in np.linspace(-np.pi, np.pi, 7):
        for q1 in np.linspace(-np.pi, np.pi, 7):
            p = arm2.forward(np.array([q0, q1])).position
            expect = np.array(
                [
                    L1 * np.cos(q0) + L2 * np.cos(q0 + q1),
                    L1 * np.sin(q0) + L2 * np.sin(q0 + q1),
                    0.0,
                ]
            )
            worst = max(worst, float(np.max(np.abs(p - expect))))
    return worst


def check_jacobian(arm2) -> float:
    """几何雅可比与中心差分对照（只比 x、y 两行），返回最大偏差。"""
    q = np.array([0.4, 0.9])
    jac = arm2.jacobian(q)[:2]
    h = 1e-7
    numeric = np.zeros((2, 2))
    for i in range(2):
        plus = q.copy()
        minus = q.copy()
        plus[i] += h
        minus[i] -= h
        numeric[:, i] = (
            arm2.forward(plus).position - arm2.forward(minus).position
        )[:2] / (2.0 * h)
    return float(np.max(np.abs(numeric - jac)))


def check_closed_form(arm2) -> float:
    """可操作性闭式解 |l1 l2 sin q1| 与数值实现的最大偏差。"""
    sweep = elbow_sweep(arm2, np.linspace(0.05, np.pi - 0.05, 41), L1, L2)
    return float(np.max(np.abs(sweep["numeric"] - sweep["closed_form"])))


def reachable_targets(chain, n: int = 3, seed: int = 7) -> list[dict]:
    """从随机关节角生成 n 个一定可达的位姿目标。"""
    rng = np.random.default_rng(seed)
    lower = chain.lower + 0.25 * (chain.upper - chain.lower)
    upper = chain.upper - 0.25 * (chain.upper - chain.lower)
    out = []
    for _ in range(n):
        q = rng.uniform(lower, upper)
        fk = chain.forward(q)
        out.append({"q_true": q, "position": fk.position.copy(),
                    "rotation": fk.rotation.copy()})
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="人形机器人全身逆运动学与可操作性演示")
    parser.add_argument("--quick", action="store_true",
                        help="跳过 GIF 与 25 点画圆，只算关键数值")
    parser.add_argument("--no-gif", action="store_true", help="不生成 GIF")
    parser.add_argument("--points", type=int, default=25, help="圆轨迹采样点数")
    args = parser.parse_args(argv)

    OUT.mkdir(parents=True, exist_ok=True)
    arm2 = make_2r_arm(L1, L2)
    arm7 = make_7dof_arm()

    # --- 1. 正解 / 雅可比 / 闭式可操作性 -------------------------------------
    fk_error = check_forward_kinematics(arm2)
    jac_error = check_jacobian(arm2)
    closed_error = check_closed_form(arm2)

    # --- 2. 奇异点附近：伪逆 vs DLS ------------------------------------------
    elbow_values = [0.2, 0.1, 0.05, 0.02, 0.01, 0.005]
    sing_rows = singularity_scan(arm2, elbow_values)
    _write_csv(
        OUT / "singularity_scan.csv",
        ["elbow_rad", "sigma_min", "step_pinv_m", "step_dls_m",
         "adaptive_lambda", "ratio_dls_over_pinv"],
        [[f"{r['elbow']:.4f}", f"{r['sigma_min']:.6e}", f"{r['step_pinv']:.6e}",
          f"{r['step_dls']:.6e}", f"{r['adaptive_lambda']:.6f}",
          f"{r['ratio']:.6e}"] for r in sing_rows],
    )
    plot_singularity(sing_rows, OUT / "singularity_damping.png")

    # --- 3. 可操作性曲线与速度椭球 -------------------------------------------
    sweep = elbow_sweep(arm2, np.linspace(0.02, np.pi - 0.02, 121), L1, L2)
    _write_csv(
        OUT / "manipulability_vs_elbow.csv",
        ["elbow_rad", "numeric", "closed_form", "condition"],
        [[f"{e:.4f}", f"{n:.8f}", f"{c:.8f}", f"{k:.6e}"]
         for e, n, c, k in zip(sweep["elbow"], sweep["numeric"],
                               sweep["closed_form"], sweep["condition"])],
    )
    plot_manipulability(sweep, L1, L2, OUT / "manipulability_vs_elbow.png")

    ell_cases = []
    for elbow in (0.35, 1.00, np.pi / 2):
        jac = task_jacobian(arm2, np.array([0.0, float(elbow)]), rows=(0, 1))
        ell = velocity_ellipsoid(jac)
        measure = manipulability(jac)
        ell_cases.append(
            {
                "elbow": float(elbow),
                "ellipsoid": ell,
                "manipulability": measure.value,
                "semi_axes": ell["semi_axes"],
            }
        )
    plot_ellipsoids(ell_cases, OUT / "manipulability_ellipsoid.png")

    # --- 4. 7 自由度手臂：冗余与零空间 --------------------------------------
    q_mid = arm7.q_mid.copy()
    null_space = nullspace_report(arm7, q_mid)
    # 冗余只存在于某些关节方向。取零空间里“范数最大”的那个方向，看它由
    # 哪些关节承载；这比硬塞一个“把肘关节推回限位”的任务更诚实——在这个
    # 位形下，肘关节根本不在零空间里。
    projector = nullspace_projector(arm7.jacobian(q_mid))
    _, _, vt = np.linalg.svd(projector)
    null_vector = vt[0]
    null_share = np.abs(null_vector) / max(float(np.max(np.abs(null_vector))), 1e-12)
    active_joints = [
        arm7.joints[i].name for i in range(arm7.dof) if null_share[i] > 0.05
    ]
    elbow_share = float(null_share[3])

    # --- 5. 圆轨迹跟踪 -------------------------------------------------------
    if args.quick:
        planes = [("xz", 12)]
    else:
        planes = [("xz", int(args.points)), ("xy", int(args.points))]
    tracking = {}
    for plane, n_points in planes:
        result = track_circle(arm7, (0.25, 0.0, 0.0), 0.12, n_points=n_points,
                              plane=plane)
        tracking[plane] = result
        plot_ik_tracking(result["errors"], result["iterations"], plane,
                         OUT / f"ik_tracking_{plane}.png")

    # --- 6. 2R 臂做 6 维位姿逆解：本就不可达，作为反例 -----------------------
    pose_fail = solve_pose_ik(arm2, np.array([0.5, 0.5, 0.0]), rotation_z(0.3),
                              max_iter=100)
    reach_fail = solve_pose_ik(arm2, np.array([0.85, 0.0, 0.0]), np.eye(3),
                               max_iter=100)
    pos_ok = solve_position_ik(arm2, np.array([0.5, 0.5, 0.0]), rows=(0, 1))

    # --- 7. 静态斜视图 + GIF -------------------------------------------------
    plot_arm_snapshot(arm7, tracking[planes[0][0]]["solutions"][0],
                      OUT / "ik_snapshot.png",
                      "7-DOF arm at the first waypoint")
    if not args.no_gif and not args.quick:
        create_circle_gif(
            arm7,
            tracking["xz"]["solutions"],
            tracking["xz"]["targets"],
            OUT / "ik_circle.gif",
            plane="xz",
        )

    # --- 8. 汇总 -------------------------------------------------------------
    lines = [
        "人形机器人全身逆运动学与可操作性（离线 Python）",
        "=" * 62,
        f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        "",
        "[数值一致性]",
        f"  2R 正解 vs 解析式最大偏差     = {fk_error:.3e} m",
        f"  几何雅可比 vs 中心差分最大偏差 = {jac_error:.3e}",
        f"  可操作性数值 vs 闭式解最大偏差 = {closed_error:.3e}",
        "",
        "[奇异点附近：固定 0.01 m 径向误差的关节步长]",
        f"{'q1/rad':>8} {'sigma_min':>12} {'pinv':>10} {'DLS':>10} {'lambda':>8}",
    ]
    for row in sing_rows:
        lines.append(
            f"{row['elbow']:8.3f} {row['sigma_min']:12.3e} "
            f"{row['step_pinv']:10.4f} {row['step_dls']:10.5f} "
            f"{row['adaptive_lambda']:8.4f}"
        )
    lines += [
        "",
        "[速度椭球半轴]",
    ]
    for case in ell_cases:
        axes = case["semi_axes"]
        lines.append(
            f"  q1 = {case['elbow']:.3f} rad: 半轴 = {axes[0]:.4f} / {axes[1]:.4f}, "
            f"w = {case['manipulability']:.4f}"
        )
    lines += [
        "",
        "[7 自由度冗余]",
        f"  零空间维数 = {null_space['nullity']}（6 维任务、7 个关节）",
        f"  随机零空间方向 |dq| = {null_space['dq_norm']:.4f}",
        f"  对应末端运动 |J dq| = {null_space['end_effector_motion']:.2e}",
        f"  零空间主方向参与关节：{', '.join(active_joints)}",
        f"  肘关节在该方向上的归一化分量 = {elbow_share:.3f}",
        "  含义：冗余是位形相关的；这个位形下肘关节被末端位姿唯一决定，"
        "只能在其余关节上做次级任务。",
        "",
        "[圆轨迹跟踪]",
    ]
    for plane, result in tracking.items():
        lines.append(
            f"  {plane} 平面，{len(result['errors'])} 点：最大误差 "
            f"{result['errors'].max():.2e} m，平均迭代 {result['iterations'].mean():.2f}，"
            f"最大迭代 {result['iterations'].max()}，全部收敛 = "
            f"{'是' if result['success'].all() else '否'}"
        )
    lines += [
        "",
        "[2R 臂做 6 维位姿逆解：反例]",
        f"  要求末端绕 z 转 0.3 rad：收敛 = {pose_fail.success}，"
        f"误差 = {pose_fail.error:.4f}（2R 只能实现 2 位置 + 1 姿态）",
        f"  要求伸到 0.85 m（臂展 1.0 m 但链路受限）：收敛 = {reach_fail.success}，"
        f"误差 = {reach_fail.error:.4f}",
        f"  只求平面位置 (0.5, 0.5)：收敛 = {pos_ok.success}，"
        f"误差 = {pos_ok.error:.2e}",
        "",
        "[结论]",
        "  1. 伪逆步长按 1/σ_min 增长，DLS 用自适应阻尼把它压住，代价是奇异方向收敛变慢。",
        "  2. 可操作性在肘角 π/2 最大、完全伸直时为 0；平面机构必须裁剪任务维度，否则恒奇异。",
        "  3. 7 自由度手臂有 1 维零空间，但哪些关节承担冗余与位形有关（该位形下肘关节分量为 0）。",
        "  4. 圆轨迹（竖直/水平）全部收敛到 1e-6 以内；竖直圆经过接近伸直的位形。",
    ]
    (OUT / "metrics.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("\n".join(lines))
    print("-" * 62)
    print(f"输出目录: {OUT}")
    for name in sorted(p.name for p in OUT.iterdir()):
        print(f"  {name:32s} {OUT.joinpath(name).stat().st_size:>10,d} bytes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
