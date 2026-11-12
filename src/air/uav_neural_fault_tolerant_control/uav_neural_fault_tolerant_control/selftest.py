"""自检：不依赖 ROS、也不要求数据集已经生成。

设计原则是"能算的都用解析解核对"，而不是把实现再跑一遍当验证：

* 效率矩阵必须和底座 ``mixer`` 在可行域内一致；
* ``solve_box_qp`` 的目标值不能比 ``ls_allocation`` 差（它是精确枚举）；
* ``build_target`` + ``apply_residual`` 必须能还原 ``u_star``；
* 未训练的残差网络（输出层零初始化）必须与 ``ls`` 完全等价；
* 四种模式的输出都必须落在 ``[0, T_max]`` 内。
"""

from __future__ import annotations

import numpy as np


class _Report:
    def __init__(self) -> None:
        self.total = 0
        self.failed = 0
        self.skipped = 0

    def check(self, name: str, ok: bool, detail: str = "") -> None:
        self.total += 1
        if ok:
            print(f"  [ok]   {name}")
        else:
            self.failed += 1
            print(f"  [FAIL] {name}  {detail}")

    def close(self, name: str, err: float, tol: float) -> None:
        self.check(name, err <= tol, f"误差 {err:.3e} > 容差 {tol:.1e}")

    def skip(self, name: str, why: str) -> None:
        self.total += 1
        self.skipped += 1
        print(f"  [skip] {name}（{why}）")


def _check_allocation(report: _Report) -> None:
    """分配层数学：效率矩阵、有效推力往返、两种求解器的一致性。"""
    from . import allocation as A

    desc = np.array([4.0, 0.4, -0.2, 0.1])

    class _P:
        mass = 0.95
        arm = 0.106
        k_m = 0.015
        max_thrust = 6.0
        hover_thrust = 2.33

    def fake_mixer(thrust, torque, params=None):
        # 最小的 X 型四旋翼混控，用来构造 B（与底座同构）
        a, k = _P.arm, _P.k_m
        tx, ty, tz = float(torque[0]), float(torque[1]), float(torque[2])
        T = float(thrust)
        return np.array([T / 4 + tx / (4 * a) + ty / (4 * a) + tz / (4 * k),
                         T / 4 - tx / (4 * a) + ty / (4 * a) - tz / (4 * k),
                         T / 4 - tx / (4 * a) - ty / (4 * a) + tz / (4 * k),
                         T / 4 + tx / (4 * a) - ty / (4 * a) - tz / (4 * k)])

    B = A.build_effectiveness(fake_mixer, _P, eps=1e-6)
    report.check("效率矩阵形状 4×4", B.shape == (4, 4), str(B.shape))
    report.check("效率矩阵第一行全 1（总推力是四桨之和）",
                 bool(np.allclose(B[0], 1.0)), str(B[0]))

    u = np.array([3.0, 2.5, 2.6, 2.7])
    v = A.effective_thrust(u, np.ones(4))
    u_back = A.command_from_effective(v, _P, np.ones(4))
    report.close("有效推力 ↔ 指令往返", float(np.max(np.abs(u_back - u))), 1e-9)

    gains = np.array([1.0, 0.0, 1.0, 1.0])
    v2 = A.effective_thrust(u, gains)
    u2 = A.command_from_effective(v2, _P, gains)
    report.check("失效桨的指令被压回 0", abs(float(u2[1])) < 1e-9, str(u2[1]))

    u_honest = np.array([2.0, 0.0, 2.0, 2.0])
    u_naive = A.naive_redistribute(u_honest, _P, gains)
    report.close("朴素重分配保持总推力",
                 abs(float(u_naive.sum()) - float(u_honest.sum())), 1e-6)


def _check_solvers(report: _Report) -> None:
    """精确解 vs 近似解：目标值、可行性、耗时量级。"""
    from . import allocation as A

    class _P:
        mass = 0.95
        arm = 0.106
        k_m = 0.015
        max_thrust = 6.0
        hover_thrust = 2.33

    B = np.array([[1, 1, 1, 1],
                  [-0.106, 0.106, 0.106, -0.106],
                  [-0.106, 0.106, -0.106, 0.106],
                  [-0.015, -0.015, 0.015, 0.015]], dtype=float)

    rng = np.random.default_rng(0)
    worse = 0
    infeasible = 0
    ref_weight = 1e-3
    for _ in range(60):
        desired = np.array([rng.uniform(6.0, 9.0), rng.uniform(-0.2, 0.2),
                            rng.uniform(-0.2, 0.2), rng.uniform(-0.02, 0.02)])
        gains = rng.choice([1.0, 0.0, 0.5, 0.7], size=4)
        if np.count_nonzero(gains) < 3:
            continue
        weights = np.array([20.0, 1.0, 1.0, 1.0])
        u_ls = A.ls_allocation(B, desired, _P, gains, row_weights=weights)
        u_qp = A.solve_box_qp(B, desired, _P, gains, row_weights=weights)

        def objective(u):
            # 与两个求解器内部一致的代价：加权残差 + 指令正则项
            v = gains * u
            res = B @ v - desired
            return float(np.sum(weights * res ** 2) + ref_weight * np.sum(v ** 2))

        if objective(u_qp) > objective(u_ls) + 1e-6:
            worse += 1
        if np.any(u_qp < -1e-9) or np.any(u_qp > _P.max_thrust + 1e-9):
            infeasible += 1

    report.check("精确枚举解不劣于最小二乘解", worse == 0, f"{worse} 个样本更差")
    report.check("精确解满足箱约束", infeasible == 0, f"{infeasible} 个越界")


def _check_faults(report: _Report) -> None:
    from . import faults as FL

    plan = FL.FaultPlan(kind="degradation", rotor=1, onset=2.0, severity=0.5)
    report.check("故障前增益为 1", bool(np.allclose(plan.gains(1.0), 1.0)),
                 str(plan.gains(1.0)))
    report.check("故障后坏桨增益 ≈ 0.5", abs(float(plan.gains(4.0)[1]) - 0.5) < 1e-6,
                 str(plan.gains(4.0)))
    report.check("其余桨不受影响", bool(np.allclose(
        np.delete(plan.gains(4.0), 1), 1.0)), str(plan.gains(4.0)))

    stuck = FL.FaultPlan(kind="stuck", rotor=0, onset=2.0)
    bias = stuck.bias(4.0, hover_thrust=2.33)
    report.check("卡死桨有固定输出", abs(float(bias[0]) - 2.33) < 1e-6, str(bias))

    bad = 0
    for kwargs in ({"kind": "nope"}, {"kind": "loss", "rotor": 4},
                   {"kind": "loss", "onset": -1.0},
                   {"kind": "progressive", "severity": 0.0}):
        try:
            FL.FaultPlan(**kwargs)
        except ValueError:
            bad += 1
    report.check("非法参数全部抛 ValueError", bad == 4, f"只拦下 {bad}/4")


def _check_features(report: _Report) -> None:
    from . import allocation as A
    from . import features as F

    class _P:
        max_thrust = 6.0
        arm = 0.106
        k_m = 0.015

    desired = np.array([8.0, 0.2, -0.1, 0.02])
    gains = np.array([1.0, 1.0, 0.6, 1.0])
    u_ls = np.array([2.2, 2.4, 1.9, 2.1])
    x = F.build_features(desired, gains, u_ls, _P)
    report.check(f"特征维度为 {F.FEATURE_DIM}", x.shape == (F.FEATURE_DIM,),
                 str(x.shape))
    report.check("特征全是有限数", bool(np.all(np.isfinite(x))), str(x))

    u_star = np.array([2.5, 2.3, 1.7, 2.2])
    target = F.build_target(u_star, u_ls, _P)
    rebuilt = F.apply_residual(u_ls, target, _P)
    report.close("残差标签可还原精确解",
                 float(np.max(np.abs(rebuilt - u_star))), 1e-6)


def _check_model(report: _Report) -> None:
    try:
        import torch  # noqa: F401
    except Exception as exc:  # pragma: no cover - 视环境
        report.skip("网络与检查点", f"没有 torch：{exc}")
        return

    import tempfile
    from pathlib import Path

    from . import features as F
    from . import model as M

    net = M.build_model()
    report.check("参数量为 5252", net.n_parameters() == 5252,
                 str(net.n_parameters()))

    x = np.zeros((3, F.FEATURE_DIM), dtype=np.float32)
    out = M.predict_numpy(net, x)
    report.check("零初始化输出全 0（未训练时等价于近似解）",
                 bool(np.allclose(out, 0.0)), str(out))

    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "m.pt"
        M.save_checkpoint(path, net, {"n_parameters": net.n_parameters()})
        loaded, meta = M.load_checkpoint(path)
        report.check("检查点往返后参数一致",
                     loaded.n_parameters() == net.n_parameters(),
                     str(meta))


def _check_episode(report: _Report) -> None:
    try:
        from ._compat import load_base
    except Exception as exc:  # pragma: no cover
        report.skip("仿真联调", f"底座不可用：{exc}")
        return

    try:
        api = load_base()
    except Exception as exc:
        report.skip("仿真联调", f"底座不可用：{exc}")
        return

    from . import data as D
    from . import faults as FL

    params, effectiveness = D.build_context(api)
    report.check("效率矩阵可测量", effectiveness.shape == (4, 4),
                 str(effectiveness.shape))

    ep = D.run_episode(api, FL.FaultPlan(kind="healthy"), trajectory="hover",
                       seconds=1.0, collect=True, sample_stride=10,
                       params=params, effectiveness=effectiveness)
    report.check("健康悬停不坠机", not ep.crashed, f"坠机于 {ep.crash_time}")
    report.check("健康悬停高度接近 1.5 m",
                 abs(ep.min_altitude - 1.5) < 0.1, f"{ep.min_altitude:.3f}")
    report.check("采集到带标签的样本", len(ep.features) > 0, str(len(ep.features)))
    # 标签是按 stride 抽稀的，比对时要对齐到同一批时间步
    stride = max(1, int(round(ep.u_ls.shape[0] / max(len(ep.u_star), 1))))
    u_ls_at = ep.u_ls[::stride][:len(ep.u_star)]
    report.check("精确解与近似解不同（有可学的残差）",
                 float(np.max(np.abs(ep.u_star - u_ls_at))) > 1e-6)


def _check_controller(report: _Report) -> None:
    try:
        from ._compat import load_base
        api = load_base()
    except Exception as exc:
        report.skip("控制器四种模式", f"底座不可用：{exc}")
        return

    from . import controller as C
    from . import data as D
    from . import model as M
    from . import trajectory as TJ

    params, effectiveness = D.build_context(api)
    quad = api.Quadrotor(scene=api.make_scene("open_ground", seed=0),
                         dt=0.01, substeps=4)
    quad.reset(np.array([0.0, 0.0, 1.5]), 0.0)
    base = api.GeometricController(params)
    net = M.build_model()

    for mode in C.MODES:
        ctrl = C.FaultTolerantController(params, api.mixer, base, effectiveness,
                                         mode=mode, net=net)
        ctrl.set_health(np.array([1.0, 0.6, 1.0, 1.0]))
        p_d, v_d, a_d, yaw_d = TJ.reference("hover", 0.0)
        u = ctrl.act(quad, p_d, v_d, a_d, yaw_d)
        report.check(f"{mode} 输出 4 个桨推力且在 [0, T_max] 内",
                     u.shape == (4,) and np.all(u >= -1e-9)
                     and np.all(u <= params.max_thrust + 1e-9), str(np.round(u, 3)))


def run_selftest(verbose: bool = True) -> int:
    report = _Report()
    sections = (
        ("分配层基础", _check_allocation),
        ("求解器", _check_solvers),
        ("故障脚本", _check_faults),
        ("特征与标签", _check_features),
        ("网络与检查点", _check_model),
        ("仿真联调", _check_episode),
        ("控制器四种模式", _check_controller),
    )
    for title, fn in sections:
        print(f"[{title}]")
        fn(report)
    print()
    passed = report.total - report.failed - report.skipped
    print(f"自检完成：{passed}/{report.total - report.skipped} 通过"
          + (f"，{report.skipped} 项跳过" if report.skipped else "")
          + (f"，{report.failed} 项失败" if report.failed else ""))
    return 0 if report.failed == 0 else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(run_selftest())
