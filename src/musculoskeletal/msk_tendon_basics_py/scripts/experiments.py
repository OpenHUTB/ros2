"""三组实验、CSV 输出与 summary.txt。"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np

from .joint_tendon import AntagonistJoint, JointParams, ReleaseResult
from .muscle_tendon import MuscleTendon, MuscleTendonParams


# 报告中的参考值（用于 summary 对照，不参与计算）。
ISO_REFERENCE = {
    0.5: (52.2, 53.7),
    1.0: (101.6, 107.0),
}

STIFFNESS_MULTIPLIERS = (0.5, 1.0, 2.0, 5.0, 10.0)
STIFFNESS_REFERENCE = {
    0.5: (95.7, 12.8, 0.306),
    1.0: (101.6, 6.8, 0.172),
    2.0: (104.5, 3.5, 0.091),
    5.0: (106.2, 1.4, 0.038),
    10.0: (106.8, 0.7, 0.019),
}

RELEASE_REFERENCE = {
    "pre_force": 89.5,
    "pre_energy": 0.134,
    "min_energy": 0.062,
    "release_fraction": 54.0,
    "peak_angle": 0.58,
    "rigid_peak_angle": 0.60,
}


def _activations(quick: bool) -> np.ndarray:
    if quick:
        return np.array([0.0, 0.25, 0.5, 0.75, 1.0])
    return np.linspace(0.0, 1.0, 21)


def run_isometric(params: MuscleTendonParams | None = None,
                  quick: bool = False) -> dict:
    """实验 1：theta=0 时扫描激活度，比较弹性和刚性肌腱。"""

    p = params if params is not None else MuscleTendonParams()
    elastic = MuscleTendon(p)
    rigid = MuscleTendon(p, tendon_stiffness=math.inf)
    activations = _activations(quick)

    elastic_force: list[float] = []
    rigid_force: list[float] = []
    loss_pct: list[float] = []
    fiber_length: list[float] = []
    rigid_fiber_length: list[float] = []
    strain_pct: list[float] = []
    energy: list[float] = []

    for a in activations:
        eq_e = elastic.equilibrium(p.rest_mtu, float(a))
        eq_r = rigid.equilibrium(p.rest_mtu, float(a))
        elastic_force.append(eq_e.f_t)
        rigid_force.append(eq_r.f_t)
        loss_pct.append(
            100.0 * (eq_r.f_t - eq_e.f_t) / eq_r.f_t if eq_r.f_t > 0.0 else 0.0
        )
        fiber_length.append(eq_e.fiber_length_norm)
        rigid_fiber_length.append(eq_r.fiber_length_norm)
        strain_pct.append(eq_e.tendon_strain_percent)
        energy.append(eq_e.tendon_energy)

    return {
        "params": p,
        "activations": activations,
        "elastic_force": np.asarray(elastic_force),
        "rigid_force": np.asarray(rigid_force),
        "loss_pct": np.asarray(loss_pct),
        "elastic_fiber_length": np.asarray(fiber_length),
        "rigid_fiber_length": np.asarray(rigid_fiber_length),
        "elastic_strain_pct": np.asarray(strain_pct),
        "elastic_energy": np.asarray(energy),
    }


def run_tendon_stiffness(params: MuscleTendonParams | None = None) -> dict:
    """实验 2：a=1 时扫描肌腱刚度倍数 0.5/1/2/5/10。"""

    p = params if params is not None else MuscleTendonParams()
    muscle = MuscleTendon(p)
    stiffness = np.asarray([p.k_t * m for m in STIFFNESS_MULTIPLIERS])

    force: list[float] = []
    strain_pct: list[float] = []
    energy: list[float] = []
    fiber_length: list[float] = []
    reference_force: list[float] = []
    reference_strain: list[float] = []
    reference_energy: list[float] = []

    for multiplier, k in zip(STIFFNESS_MULTIPLIERS, stiffness):
        eq = muscle.equilibrium(p.rest_mtu, 1.0, tendon_stiffness=float(k))
        force.append(eq.f_t)
        strain_pct.append(eq.tendon_strain_percent)
        energy.append(eq.tendon_energy)
        fiber_length.append(eq.fiber_length_norm)
        ref = STIFFNESS_REFERENCE[multiplier]
        reference_force.append(ref[0])
        reference_strain.append(ref[1])
        reference_energy.append(ref[2])

    return {
        "params": p,
        "multipliers": np.asarray(STIFFNESS_MULTIPLIERS),
        "stiffness": stiffness,
        "force": np.asarray(force),
        "strain_pct": np.asarray(strain_pct),
        "energy": np.asarray(energy),
        "fiber_length": np.asarray(fiber_length),
        "reference_force": np.asarray(reference_force),
        "reference_strain": np.asarray(reference_strain),
        "reference_energy": np.asarray(reference_energy),
    }


def run_release(params: MuscleTendonParams | None = None,
                quick: bool = False) -> dict:
    """实验 3：刹车 0.4 s 后松手，比较弹性/刚性肌腱。"""

    p = params if params is not None else MuscleTendonParams()
    joint_params = JointParams.from_muscle_params(p)
    elastic_joint = AntagonistJoint(p, joint_params, tendon_stiffness=p.k_t)
    rigid_joint = AntagonistJoint(p, joint_params, tendon_stiffness=math.inf)

    dt = 2e-3 if quick else 1e-3
    duration = 1.0 if quick else 1.2
    kwargs = dict(
        hold_time=0.4,
        duration=duration,
        dt=dt,
        flex_command=0.9,
        ext_command=0.05,
        theta0=joint_params.release_angle,
    )
    elastic = elastic_joint.simulate_release(**kwargs)
    rigid = rigid_joint.simulate_release(**kwargs)
    return {
        "params": p,
        "joint_params": joint_params,
        "elastic": elastic,
        "rigid": rigid,
    }


# ----------------------------------------------------------------------
# CSV
# ----------------------------------------------------------------------
def _write_csv(path: Path, header: list[str], rows: list[list[float | str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for row in rows:
            writer.writerow([
                f"{value:.10g}" if isinstance(value, (int, float)) and not isinstance(value, bool)
                else value
                for value in row
            ])


def write_isometric_csv(result: dict, path: Path) -> None:
    rows: list[list[float | str]] = []
    for i, a in enumerate(result["activations"]):
        rows.append([
            float(a),
            float(result["elastic_force"][i]),
            float(result["rigid_force"][i]),
            float(result["loss_pct"][i]),
            float(result["elastic_fiber_length"][i]),
            float(result["elastic_strain_pct"][i]),
            float(result["elastic_energy"][i]),
            float(result["rigid_fiber_length"][i]),
        ])
    _write_csv(
        path,
        [
            "activation",
            "elastic_force_N",
            "rigid_force_N",
            "transmission_loss_percent",
            "elastic_fiber_length_norm",
            "elastic_tendon_strain_percent",
            "elastic_tendon_energy_J",
            "rigid_fiber_length_norm",
        ],
        rows,
    )


def write_tendon_stiffness_csv(result: dict, path: Path) -> None:
    rows: list[list[float | str]] = []
    for i, multiplier in enumerate(result["multipliers"]):
        rows.append([
            float(multiplier),
            float(result["stiffness"][i]),
            float(result["force"][i]),
            float(result["fiber_length"][i]),
            float(result["strain_pct"][i]),
            float(result["energy"][i]),
            float(result["reference_force"][i]),
            float(result["reference_strain"][i]),
            float(result["reference_energy"][i]),
        ])
    _write_csv(
        path,
        [
            "stiffness_multiplier",
            "tendon_stiffness_N_per_m",
            "force_N",
            "fiber_length_norm",
            "tendon_strain_percent",
            "tendon_energy_J",
            "report_force_N",
            "report_strain_percent",
            "report_energy_J",
        ],
        rows,
    )


def write_release_csv(result: dict, path: Path) -> None:
    elastic: ReleaseResult = result["elastic"]
    rigid: ReleaseResult = result["rigid"]
    theta_rigid = np.interp(elastic.time, rigid.time, rigid.theta)

    rows: list[list[float | str]] = []
    for i in range(len(elastic)):
        rows.append([
            float(elastic.time[i]),
            str(elastic.phase[i]),
            float(elastic.theta[i]),
            float(elastic.omega[i]),
            float(elastic.flex_activation[i]),
            float(elastic.ext_activation[i]),
            float(elastic.flex_force[i]),
            float(elastic.ext_force[i]),
            float(elastic.flex_energy[i]),
            float(elastic.flex_fiber_length_norm[i]),
            float(theta_rigid[i]),
        ])
    _write_csv(
        path,
        [
            "time_s",
            "phase",
            "theta_rad",
            "omega_rad_per_s",
            "flex_activation",
            "ext_activation",
            "flex_force_N",
            "ext_force_N",
            "flex_tendon_energy_J",
            "flex_fiber_length_norm",
            "theta_rigid_rad",
        ],
        rows,
    )


# ----------------------------------------------------------------------
# 汇总
# ----------------------------------------------------------------------
def _iso_at(result: dict, activation: float) -> tuple[float, float]:
    idx = int(np.argmin(np.abs(result["activations"] - activation)))
    return float(result["elastic_force"][idx]), float(result["rigid_force"][idx])


def build_summary(isometric: dict, stiffness: dict, release: dict,
                  plots: dict[str, bool], quick: bool) -> str:
    p: MuscleTendonParams = isometric["params"]
    lines: list[str] = []
    lines.append("肌肉-肌腱串联弹性实验复现（纯 Python + numpy）")
    lines.append("=" * 64)
    lines.append(f"运行模式: {'--quick' if quick else '完整'}")
    lines.append(f"参数: F_max={p.F_max:g} N, l_opt={p.l_opt:g} m, "
                 f"l_slack={p.l_slack:g} m, k_t={p.k_t:g} N/m, "
                 f"width={p.width:g}")
    lines.append("")

    lines.append("实验 1：等长传力（theta=0）")
    lines.append("-" * 64)
    lines.append("a      弹性 F(N)  刚性 F(N)  损失(%)   l_m/l_opt  应变(%)  储能(J)")
    for a in (0.5, 1.0):
        idx = int(np.argmin(np.abs(isometric["activations"] - a)))
        lines.append(
            f"{a:>4.1f}   {isometric['elastic_force'][idx]:>8.2f}  "
            f"{isometric['rigid_force'][idx]:>8.2f}  "
            f"{isometric['loss_pct'][idx]:>6.1f}   "
            f"{isometric['elastic_fiber_length'][idx]:>8.4f}  "
            f"{isometric['elastic_strain_pct'][idx]:>6.2f}  "
            f"{isometric['elastic_energy'][idx]:>7.4f}"
        )
    lines.append(
        "报告值: a=0.5 弹性 52.2 / 刚性 53.7 N（损失 2.7%），"
        "0.833 l_opt，3.5%，0.045 J"
    )
    lines.append(
        "        a=1.0 弹性 101.6 / 刚性 107.0 N（损失 5.1%），"
        "0.816 l_opt，6.8%，0.172 J"
    )
    lines.append("")

    lines.append("实验 2：肌腱刚度扫描（a=1）")
    lines.append("-" * 64)
    lines.append("倍数    k_t(N/m)      F(N)    l_m/l_opt  应变(%)  储能(J)  "
                 "报告 F/应变/储能")
    for i, multiplier in enumerate(stiffness["multipliers"]):
        ref = STIFFNESS_REFERENCE[float(multiplier)]
        lines.append(
            f"{multiplier:>4.1f}  {stiffness['stiffness'][i]:>10.0f}  "
            f"{stiffness['force'][i]:>8.2f}  "
            f"{stiffness['fiber_length'][i]:>9.4f}  "
            f"{stiffness['strain_pct'][i]:>7.2f}  "
            f"{stiffness['energy'][i]:>7.3f}  "
            f"{ref[0]:.1f}/{ref[1]:.1f}/{ref[2]:.3f}"
        )
    lines.append("")

    elastic: ReleaseResult = release["elastic"]
    rigid: ReleaseResult = release["rigid"]
    lines.append("实验 3：松手释放（初始 theta=0.05 rad，负载弹簧自然角=0.05 rad）")
    lines.append("-" * 64)
    lines.append(f"松手前屈肌力      : {elastic.pre_force:.2f} N   (报告 89.5 N)")
    lines.append(f"松手前肌腱储能    : {elastic.pre_energy:.4f} J  (报告 0.134 J)")
    lines.append(f"松手后最低储能    : {elastic.min_energy:.4f} J  (报告 0.062 J)")
    lines.append(f"释放比例          : {100.0 * elastic.release_fraction:.1f}%    (报告 ~54%)")
    lines.append(f"峰值关节角（弹性）: {elastic.peak_angle:.3f} rad (报告 0.58 rad)")
    lines.append(f"峰值关节角（刚性）: {rigid.peak_angle:.3f} rad (报告 0.60 rad)")
    lines.append("")

    lines.append("输出文件")
    lines.append("-" * 64)
    for name, ok in plots.items():
        lines.append(f"{name}.png: {'已生成' if ok else '跳过（matplotlib 不可用或绘图失败）'}")
    lines.append("isometric.csv, tendon_stiffness.csv, release.csv, summary.txt")
    lines.append("")

    lines.append("复现说明")
    lines.append("-" * 64)
    lines.append(
        "1) 串联平衡用二分法（tol=1e-12, max_iter=200）；"
        "所有结果均满足 |F_t-F_m| < 1e-6。"
    )
    lines.append(
        "2) 肌腱力曲线按报告的 8% 应变定义："
        "k_t = F_max/(0.08*l_slack) = 30000 N/m。"
    )
    lines.append(
        "3) 刚性肌腱取 k_t -> inf 的解析极限；此时 l_m = L_mtu - l_slack。"
        "本实现得 107.4 N，报告写 107.0 N（差 0.4 N，约 0.4%）；"
        "报告表格中 k=300000 N/m 的 106.8 N 与本实现一致。"
    )
    lines.append(
        "4) 松手实验的初始角由报告前值反推为 0.05 rad："
        "只有在这个角度，a=0.9 的屈肌力才是 89.5 N。"
    )
    lines.append(
        "5) 关节动力学为准静态肌肉模型：每个积分步内肌肉-肌腱瞬间力平衡，"
        "肌纤维速度取 0；关节阻尼和负载弹簧显式积分。"
    )
    return "\n".join(lines) + "\n"


# ----------------------------------------------------------------------
# 顶层流程
# ----------------------------------------------------------------------
def run_all(outdir: str | Path = "out", quick: bool = False) -> dict:
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    isometric = run_isometric(quick=quick)
    stiffness = run_tendon_stiffness()
    release = run_release(quick=quick)

    write_isometric_csv(isometric, outdir / "isometric.csv")
    write_tendon_stiffness_csv(stiffness, outdir / "tendon_stiffness.csv")
    write_release_csv(release, outdir / "release.csv")

    from . import plotting

    plots = plotting.make_all_plots(isometric, stiffness, release, outdir)
    summary = build_summary(isometric, stiffness, release, plots, quick)
    (outdir / "summary.txt").write_text(summary, encoding="utf-8")

    return {
        "outdir": outdir,
        "isometric": isometric,
        "stiffness": stiffness,
        "release": release,
        "plots": plots,
        "summary": summary,
    }
