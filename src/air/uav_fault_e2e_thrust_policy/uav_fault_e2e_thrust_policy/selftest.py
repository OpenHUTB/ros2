"""无权重自检：纯数学/形状/接口层面的检查，不依赖训练好的模型。

torch 或 mujoco 缺失时对应分组记 SKIP 而不是失败——这样在只装了 numpy 的机器上
也能验证观测与指标的实现。
"""

from __future__ import annotations

import numpy as np

from . import obs as O


class _Report:
    def __init__(self) -> None:
        self.items: list[tuple[str, str, str]] = []

    def check(self, name: str, ok: bool, detail: str = "") -> bool:
        self.items.append((name, "OK" if ok else "FAIL", detail))
        return bool(ok)

    def skip(self, name: str, reason: str) -> None:
        self.items.append((name, "SKIP", reason))

    def failed(self) -> int:
        return sum(1 for _, status, _ in self.items if status == "FAIL")

    def show(self, verbose: bool) -> None:
        if verbose:
            for name, status, detail in self.items:
                extra = f"  ({detail})" if detail else ""
                print(f"[{status}] {name}{extra}")
        else:
            print(f"自检 {len(self.items) - self.failed()}/{len(self.items)} 通过")


def _check_obs(r: _Report) -> None:
    depth = np.full((64, 64), 2.0, dtype=np.float32)
    depth[0:32, 0:32] = 1.0
    small = O.crop_depth(depth)
    r.check("深度图池化到 32×32", small.shape == (32, 32), str(small.shape))
    r.check("深度归一化范围", float(small.min()) >= 0.0 and float(small.max()) <= 1.0,
            f"{small.min():.3f}~{small.max():.3f}")
    r.check("远/近区分", float(small[0, 0]) < float(small[-1, -1]),
            f"{small[0, 0]:.3f} < {small[-1, -1]:.3f}")

    class _S:
        pos = np.zeros(3)
        vel = np.array([1.0, 0.0, 0.0])
        quat = np.array([1.0, 0.0, 0.0, 0.0])
        omega = np.array([0.0, 0.0, 0.5])
        roll = 0.0
        pitch = 0.0

    proprio = O.proprio_features(_S(), np.ones(4), None, 6.0)
    r.check("本体量维度", proprio.shape == (O.PROPRIO_DIM,), str(proprio.shape))
    r.check("本体量数值有限", bool(np.all(np.isfinite(proprio))))
    ob = O.build_observation(depth, proprio)
    r.check("观测打包", ob["depth"].shape == (1, 32, 32) and ob["proprio"].shape == (O.PROPRIO_DIM,),
            f"{ob['depth'].shape} {ob['proprio'].shape}")
    flat = O.flatten_observation(ob)
    r.check("展平观测", flat.shape == (O.OBS_DIM,) or flat.size == O.DEPTH_DIM + O.PROPRIO_DIM,
            str(flat.shape))


def _check_faults(r: _Report) -> None:
    from .faults import DEFAULT_PLANS, FAULT_KINDS, FaultPlan

    for spec in DEFAULT_PLANS:
        plan = spec if isinstance(spec, FaultPlan) else FaultPlan(**spec)
        before = plan.gains(0.0)
        after = plan.gains(max(plan.onset_time() or 0.0, 0.0) + 1.0)
        r.check(f"故障 {plan.kind} 前后不同",
                not np.allclose(before, after) or plan.kind == "healthy",
                f"{before.round(2)} → {after.round(2)}")
    r.check("故障类型齐全", set(FAULT_KINDS) >= {"healthy", "loss", "degradation", "stuck"},
            str(FAULT_KINDS))
    try:
        FaultPlan(kind="loss", rotor=9)
        r.check("非法桨序号被拒", False)
    except ValueError:
        r.check("非法桨序号被拒", True)


def _check_expert_and_env(r: _Report) -> None:
    try:
        from . import _compat

        api = _compat.load_base()
    except Exception as exc:  # noqa: BLE001
        r.skip("环境与专家", f"缺少底座：{exc}")
        return
    from .env import ThrustEnv
    from .faults import FaultPlan

    env = ThrustEnv(api, plan=FaultPlan(kind="healthy"), seed=0)
    try:
        ob = env.reset()
        r.check("环境观测形状",
                ob["depth"].shape == (1, 32, 32) and ob["proprio"].shape == (O.PROPRIO_DIM,))
        cmd = env.expert_command()
        r.check("专家指令有界", cmd.shape == (4,) and float(cmd.min()) >= 0.0
                and float(cmd.max()) <= 1.0, f"{np.round(cmd, 3)}")
        r.check("专家悬停指令接近悬停推力",
                abs(float(cmd.mean()) - env.params.hover_thrust / env.params.max_thrust) < 0.25,
                f"mean={cmd.mean():.3f} hover={env.params.hover_thrust / env.params.max_thrust:.3f}")
        _, info = env.step(cmd)
        r.check("一步之后时间推进", info.t > 0.0, f"t={info.t:.2f}")
        # 专家不应该在健康情况下掉高度
        for _ in range(50):
            ob, info = env.step(env.expert_command())
        r.check("专家悬停稳定", float(info.position[2]) > 1.2,
                f"z={info.position[2]:.2f}")
    finally:
        env.close()


def _check_policy(r: _Report) -> None:
    try:
        from . import policy as P
    except Exception as exc:  # noqa: BLE001
        r.skip("网络结构", f"缺少 torch：{exc}")
        return
    try:
        P._torch()
    except Exception as exc:  # noqa: BLE001
        r.skip("网络结构", f"缺少 torch：{exc}")
        return
    import numpy as np

    expected = {"mlp": None, "cnn": None, "cnn_stack": None}
    depth_out = {}
    for arch in P.ARCHS:
        module = P.build_module(arch)
        stack = P.DEPTH_STACK if arch == "cnn_stack" else 1
        depth = np.zeros((2, stack, O.DEPTH_SIZE, O.DEPTH_SIZE), dtype=np.float32)
        proprio = np.zeros((2, O.PROPRIO_DIM), dtype=np.float32)
        out = P.predict(module, {"depth": depth, "proprio": proprio})
        expected[arch] = P.n_parameters(module)
        depth_out[arch] = out.shape
        r.check(f"{arch} 输出形状", out.shape == (2, 4), str(out.shape))
        r.check(f"{arch} 参数量为正", expected[arch] > 1000, str(expected[arch]))
    r.check("三种结构参数量不同", len({round(v, -3) for v in expected.values()}) >= 2,
            str(expected))
    r.check("cnn 比 mlp 小", expected["cnn"] < expected["mlp"],
            f"cnn {expected['cnn']} vs mlp {expected['mlp']}")


def _check_data(r: _Report) -> None:
    from . import data as D

    fake = {
        "depth": np.random.rand(6, 1, O.DEPTH_SIZE, O.DEPTH_SIZE).astype(np.float32),
        "proprio": np.zeros((6, O.PROPRIO_DIM), dtype=np.float32),
        "label": np.zeros((6, 4), dtype=np.float32),
        "episode": np.array([0, 0, 0, 1, 1, 1]),
        "step": np.array([0, 1, 2, 0, 1, 2]),
    }
    stacked = D.stack_frames(fake, 3)
    r.check("三帧堆叠形状", stacked.shape == (6, 3, 32, 32), str(stacked.shape))
    r.check("首帧复制不跨 episode",
            np.allclose(stacked[0, 0], fake["depth"][0, 0]) and
            np.allclose(stacked[3, 0], fake["depth"][3, 0]))
    r.check("末通道是当前帧", np.allclose(stacked[2, -1], fake["depth"][2, 0]))
    masks = D.split_by_episode(fake, (0.6, 0.2, 0.2), seed=0)
    r.check("按 episode 切分不重叠",
            not (masks["train"] & masks["test"]).any(),
            f"train={masks['train'].sum()} test={masks['test'].sum()}")


def _check_metrics(r: _Report) -> None:
    from .evaluate import FlightRow, aggregate

    rows = [
        FlightRow("expert", "healthy", 0, True, False, 8.0, 10.0, 1.5, 1.49, 20.0, 0.0, 800),
        FlightRow("cnn", "healthy", 0, False, True, 1.2, 2.0, 0.3, 0.3, 120.0, 0.4, 120),
    ]
    summary = aggregate(rows)
    r.check("成功率统计", summary["expert"]["success_rate"] == 1.0
            and summary["cnn"]["success_rate"] == 0.0)
    r.check("坠机率统计", summary["cnn"]["crash_rate"] == 1.0)


def run_selftest(verbose: bool = True) -> int:
    r = _Report()
    _check_obs(r)
    _check_faults(r)
    _check_expert_and_env(r)
    _check_policy(r)
    _check_data(r)
    _check_metrics(r)
    r.show(verbose)
    return 0 if r.failed() == 0 else 1
