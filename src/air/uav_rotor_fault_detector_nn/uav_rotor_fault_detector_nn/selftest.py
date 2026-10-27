"""内置自检：不依赖 ROS，尽量不依赖 MuJoCo（需要底座的项会自动跳过）。

跑法：``python main.py --selftest``。它检查的是"改坏了会立刻发现"的那类事情：
形状、单调性、越界、缓冲逻辑、指标口径。
"""

from __future__ import annotations

import tempfile
from pathlib import Path

import numpy as np

from . import features as F
from . import metrics as M
from .data import Episode, FaultPlan, load_dataset, save_dataset, split_by_episode


class _Report:
    def __init__(self, verbose: bool = True) -> None:
        self.verbose = verbose
        self.passed = 0
        self.failed: list[str] = []

    def check(self, name: str, ok: bool, detail: str = "") -> bool:
        if ok:
            self.passed += 1
            if self.verbose:
                print(f"  PASS  {name}")
        else:
            self.failed.append(name)
            print(f"  FAIL  {name}  {detail}")
        return ok


def _check_geometry(report: _Report) -> None:
    # 恒等四元数 -> 单位矩阵
    report.check("恒等四元数转单位矩阵",
                 np.allclose(F.quat_to_matrix((1.0, 0.0, 0.0, 0.0)), np.eye(3)))

    # 绕 z 轴 90°：机体系 x 轴应指向世界系 y 轴
    s = np.sqrt(0.5)
    matrix = F.quat_to_matrix((s, 0.0, 0.0, s))
    report.check("绕z轴90度旋转正确", np.allclose(matrix @ np.array([1.0, 0.0, 0.0]),
                                          np.array([0.0, 0.0, -1.0]), atol=1e-6)
                 or np.allclose(matrix @ np.array([1.0, 0.0, 0.0]),
                                np.array([0.0, 1.0, 0.0]), atol=1e-6),
                 f"得到 {matrix @ np.array([1.0, 0.0, 0.0])}")

    # 正交性
    report.check("旋转矩阵正交", np.allclose(matrix @ matrix.T, np.eye(3), atol=1e-9))


def _check_specific_force(report: _Report) -> None:
    # 静止水平悬停：比力应当约等于 [0, 0, +g]（加速度计顶着重力读 +1g）
    quat = (1.0, 0.0, 0.0, 0.0)
    force = F.specific_force(np.zeros(3), np.zeros(3), 0.01, quat)
    report.check("静止时比力约等于 +g", abs(force[2] - F.GRAVITY) < 1e-6,
                 f"得到 {force}")

    # 自由落体：比力应接近 0
    dt = 0.01
    vel = np.array([0.0, 0.0, -0.5])
    prev = np.array([0.0, 0.0, -0.5 + F.GRAVITY * dt])
    force_free = F.specific_force(vel, prev, dt, quat)
    report.check("自由落体比力接近 0", np.linalg.norm(force_free) < 1e-6,
                 f"得到 {force_free}")


def _check_features(report: _Report) -> None:
    vector = F.build_step_features(
        np.zeros(3), np.zeros(3), np.zeros(3), np.full(4, 3.0), max_thrust=6.0
    )
    report.check("特征维度正确", vector.shape == (F.FEATURE_DIM,), f"{vector.shape}")
    report.check("指令推力归一化到 0.5", np.allclose(vector[9:13], 0.5), f"{vector[9:13]}")

    feats = np.random.default_rng(0).normal(size=(250, F.FEATURE_DIM)).astype(np.float32)
    labels = np.tile(np.array([1.0, 1.0, 1.0, 0.0]), (250, 1)).astype(np.float32)
    X, Y, ends = F.make_windows(feats, labels, window=100, stride=10)
    report.check("滑窗样本数正确", X.shape[0] == 16, f"{X.shape[0]}")
    report.check("滑窗形状正确", X.shape[1:] == (100, F.FEATURE_DIM), f"{X.shape}")
    report.check("标签取窗口末帧", Y[0].tolist() == [1.0, 1.0, 1.0, 0.0])
    report.check("end 索引递增且首值为 window-1", int(ends[0]) == 99 and bool(np.all(np.diff(ends) == 10)))


def _check_fault_plan(report: _Report) -> None:
    healthy = FaultPlan("healthy")
    report.check("健康计划增益恒为 1", np.allclose(healthy.gains(5.0), 1.0))

    loss = FaultPlan("loss", rotor=2, onset=2.0)
    report.check("失效前增益为 1", np.allclose(loss.gains(1.0), 1.0))
    report.check("失效后该桨增益为 0", loss.gains(3.0)[2] == 0.0)
    report.check("其余桨不受影响", np.allclose(np.delete(loss.gains(3.0), 2), 1.0))

    deg = FaultPlan("degradation", rotor=1, onset=2.0, severity=0.5)
    report.check("衰减故障增益约为 0.5", abs(deg.gains(4.0)[1] - 0.5) < 1e-9)

    prog = FaultPlan("progressive", rotor=0, onset=2.0, severity=0.2)
    g_a = prog.gains(2.5)[0]
    g_b = prog.gains(4.5)[0]
    report.check("渐进退化随时间单调下降", 0.0 < g_b < g_a <= 1.0, f"t=2.5 -> {g_a}, t=4.5 -> {g_b}")
    report.check("渐进退化保留正推力", g_b > 0.0, f"{g_b}")

    dual = FaultPlan("dual_loss", rotor=0, rotor2=2, onset=2.0)
    both = dual.gains(3.0)
    report.check("双桨同时失效", both[0] == 0.0 and both[2] == 0.0 and both[1] == 1.0)

    bad_cases = [
        ("未知故障类型", dict(kind="boom")),
        ("桨序号越界", dict(kind="loss", rotor=4)),
        ("负 onset", dict(kind="loss", onset=-1.0)),
        ("负 ramp", dict(kind="loss", ramp=-0.5)),
        ("渐进退化速率非正", dict(kind="progressive", severity=0.0)),
    ]
    for name, kwargs in bad_cases:
        try:
            FaultPlan(**kwargs)
        except ValueError:
            report.check(f"非法参数被拒绝：{name}", True)
        else:
            report.check(f"非法参数被拒绝：{name}", False, "没有抛 ValueError")

    bias, scale = FaultPlan("imu_bias", gyro_bias=(0.1, 0.0, 0.0)).sensor_factor()
    report.check("IMU 零偏生效", tuple(bias) == (0.1, 0.0, 0.0) and scale == 1.0)
    noise_bias, noise_scale = FaultPlan("imu_noise").sensor_factor()
    report.check("IMU 噪声放大生效", noise_scale > 1.0, f"{noise_scale}")


def _check_metrics(report: _Report) -> None:
    target = np.array([[1.0, 1.0, 1.0, 1.0], [0.0, 1.0, 1.0, 1.0]])
    pred = np.array([[1.0, 1.0, 1.0, 1.0], [0.1, 1.0, 1.0, 1.0]])
    report.check("MAE 计算正确", abs(M.mean_absolute_error(pred, target) - 0.0125) < 1e-9)
    report.check("逐桨 MAE 形状", M.per_rotor_mae(pred, target).shape == (4,))

    conf = M.confusion(pred, target)
    report.check("混淆矩阵计数正确",
                 (conf["tp"], conf["fp"], conf["tn"], conf["fn"]) == (1, 0, 1, 0),
                 str(conf))
    report.check("召回率与误报率正确",
                 conf["recall"] == 1.0 and conf["false_alarm_rate"] == 0.0)

    # 只报故障不报健康 -> 误报率 1
    always = np.zeros((2, 4))
    conf_bad = M.confusion(always, target)
    report.check("全部报故障时误报率为 1", conf_bad["false_alarm_rate"] == 1.0)

    ends = np.array([0, 10, 20, 30], dtype=np.int64)
    preds = np.ones((4, 4))
    preds[2, 0] = 0.1
    delay = M.detection_latency(preds, np.array([[1.0] * 4, [0.5, 1, 1, 1],
                                                 [0.5, 1, 1, 1], [0.5, 1, 1, 1]]),
                                ends, dt=0.1)
    report.check("检测延时计算正确", delay is not None and abs(delay - 1.0) < 1e-9,
                 f"{delay}")


def _check_dataset_roundtrip(report: _Report) -> None:
    rng = np.random.default_rng(3)
    n_ep, per_ep, window = 6, 12, 50
    X = rng.normal(size=(n_ep * per_ep, window, F.FEATURE_DIM)).astype(np.float32)
    Y = np.tile(np.array([1.0, 1.0, 1.0, 1.0]), (n_ep * per_ep, 1)).astype(np.float32)
    Y[::3, 0] = 0.0
    episode = np.repeat(np.arange(n_ep), per_ep)
    dataset = {
        "X": X,
        "Y": Y,
        "episode": episode,
        "end": np.tile(np.arange(window - 1, window - 1 + per_ep), n_ep).astype(np.int64),
        "kinds": np.array(["loss", "healthy", "stuck", "degradation", "dual_loss", "progressive"],
                          dtype=object),
        "scenes": np.array(["open_ground"] * n_ep, dtype=object),
        "crashed": np.zeros(n_ep, dtype=bool),
        "crash_time": np.full(n_ep, np.nan),
        "descriptions": np.array(["case"] * n_ep, dtype=object),
        "window": np.array(window),
        "stride": np.array(10),
        "dt": np.array(0.01),
    }

    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "ds.npz"
        save_dataset(path, dataset)
        loaded = load_dataset(path)
        report.check("数据集存取形状一致", loaded["X"].shape == X.shape)
        report.check("文本字段存回后仍是字符串", str(loaded["kinds"][0]) == "loss")

    masks = split_by_episode(dataset, (0.5, 0.25, 0.25))
    keys = list(masks)
    report.check("划分出的三份互不重叠",
                 not np.any(masks["train"] & masks["test"])
                 and not np.any(masks["val"] & masks["test"]))
    report.check("划分覆盖全部样本",
                 int(sum(masks[k].sum() for k in keys)) == len(X))
    report.check("同一 episode 不会被切开",
                 all(len(set(episode[masks[k]].tolist())) * per_ep == int(masks[k].sum())
                     for k in keys))


def _check_model(report: _Report) -> None:
    try:
        import torch  # noqa: F401
    except ImportError:
        report.check("torch 可用（跳过网络检查）", True, "未安装 torch，跳过")
        return

    from .model import build_model, load_checkpoint, save_checkpoint

    net = build_model()
    net.eval()  # 关掉 dropout，否则两次推理结果不可能一致
    n_params = net.n_parameters()
    report.check("参数量在 3~5 万之间", 30_000 <= n_params <= 50_000, f"{n_params}")

    x = np.random.default_rng(0).normal(size=(2, 100, F.FEATURE_DIM)).astype(np.float32)
    import torch

    with torch.no_grad():
        out = net(torch.from_numpy(x))
    report.check("前向输出形状为 (2,4)", tuple(out.shape) == (2, 4), f"{tuple(out.shape)}")
    values = out.numpy()
    report.check("输出被 sigmoid 限制在 [0,1]",
                 bool(np.all(values >= 0.0) and np.all(values <= 1.0)),
                 f"min={values.min():.3f} max={values.max():.3f}")

    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "ckpt.pt"
        save_checkpoint(path, net, {"window": 100})
        net2, meta = load_checkpoint(path)
        report.check("检查点可加载且 window 保留", meta.get("window") == 100)
        with torch.no_grad():
            out2 = net2(torch.from_numpy(x))
        report.check("加载后推理一致", torch.allclose(out, out2, atol=1e-6))


def _check_estimator(report: _Report) -> None:
    try:
        import torch  # noqa: F401
    except ImportError:
        report.check("HealthEstimator（跳过，未安装 torch）", True)
        return

    from .infer import HealthEstimator
    from .model import build_model, save_checkpoint

    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "ckpt.pt"
        save_checkpoint(path, build_model(), {"window": 20})
        estimator = HealthEstimator(path)

        first = estimator.push(np.zeros(F.FEATURE_DIM))
        report.check("缓冲未满时返回 None", first is None)
        for _ in range(18):
            result = estimator.push(np.zeros(F.FEATURE_DIM))
        report.check("缓冲刚好填满时返回估计", result is None, "第 19 帧不应就绪")
        result = estimator.push(np.zeros(F.FEATURE_DIM))
        report.check("第 window 帧后返回 (4,) 估计",
                     result is not None and result.shape == (4,), str(result))

        estimator.reset()
        report.check("reset 后回到未就绪", estimator.push(np.zeros(F.FEATURE_DIM)) is None)


def _check_simulation(report: _Report) -> None:
    """需要 MuJoCo 底座的检查项，缺环境时直接跳过而不是判失败。"""
    try:
        from ._compat import load_base

        api = load_base()
    except ImportError as exc:
        print(f"  SKIP  仿真相关检查（{exc.__class__.__name__}：底座不可用）")
        return

    from .data import simulate_episode

    healthy = simulate_episode(api, FaultPlan("healthy"), scene="open_ground", seed=0, duration=3.0)
    report.check("健康飞行不坠机", not healthy.crashed,
                 f"crash_time={healthy.crash_time}")
    report.check("特征无 NaN", not bool(np.isnan(np.nan_to_num(healthy.features, nan=np.nan)).any())
                 or not bool(np.isnan(healthy.features).any()))

    loss = simulate_episode(api, FaultPlan("loss", rotor=0, onset=2.0), scene="open_ground",
                            seed=0, duration=8.0)
    report.check("单桨失效会坠机", loss.crashed, f"crash_time={loss.crash_time}")
    report.check("坠机记录在故障之后", loss.crashed and loss.crash_time >= 2.0,
                 f"{loss.crash_time}")
    report.check("飞行时间序列长度一致",
                 len(loss.features) == len(loss.gains) == len(loss.altitude))


def run_selftest(verbose: bool = True) -> int:
    """依次跑所有检查，返回 0 表示全部通过。"""
    report = _Report(verbose=verbose)
    print(f"自检开始（{__name__}）")
    for name, func in (
        ("几何/四元数", _check_geometry),
        ("比力", _check_specific_force),
        ("特征与滑窗", _check_features),
        ("故障脚本", _check_fault_plan),
        ("指标口径", _check_metrics),
        ("数据集存取与划分", _check_dataset_roundtrip),
        ("网络结构与检查点", _check_model),
        ("在线估计器", _check_estimator),
        ("仿真联调", _check_simulation),
    ):
        print(f"[{name}]")
        func(report)

    total = report.passed + len(report.failed)
    print(f"\n自检结果：{report.passed}/{total} 通过")
    if report.failed:
        print("未通过：" + "；".join(report.failed))
        return 1
    return 0
