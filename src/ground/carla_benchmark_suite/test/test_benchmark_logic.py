#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 作业综合整合与性能评价 —— 单元测试。

不依赖 CARLA、不依赖 ROS，验证：
  1. 指标计算正确（RMSE / MAE / AoS / 相关系数 / 方向一致率）
  2. 指标计算对空输入与单元素输入不崩
  3. AoS 衡量转向平滑度的语义正确
  4. **各功能包内 nn_models.py 副本一致性**（回归测试）

     `nn_models.py` 在每个功能包内各有一份副本（为了让每个包都能独立编译运行）。
     多份副本带来一个真实风险：修了 A 包的 bug 却忘了修 B 包，
     导致同一个算法在不同包里行为不一致。本测试把这一风险制度化卡住——
     若副本出现分歧就失败，提示同步。

     实例：作业四修复了 maxpool 反向传播与推理 tanh 两处缺陷后，
     性能评价包仍用旧副本，基准测试中端到端 MAE 一度高达 36.12（正常应 <0.3）。

  5. 基准评测套件能被导入且评测项齐全
  6. 调度器能正确定位兄弟功能包
  7. 指标对比图能正常导出

运行：  python3 test/test_benchmark_logic.py
"""

import glob
import hashlib
import os
import sys
import tempfile

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

import importlib.util  # noqa: E402

_spec = importlib.util.spec_from_file_location("bench_main", os.path.join(_PKG, "main.py"))
bench = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(bench)


# ---------------------------------------------------------------- 指标
def test_compute_metrics_lateral_rmse():
    """横向误差 RMSE 应为平方均值开根。"""
    e = np.array([0.0, 3.0, 4.0])          # 平方和=25，均值=25/3
    m = bench.compute_metrics(lateral_err=e)
    expect = float(np.sqrt(25.0 / 3.0))
    assert abs(m["lateral_rmse"] - expect) < 1e-9, \
        f"RMSE 计算错误: {m['lateral_rmse']} != {expect}"
    assert abs(m["lateral_max"] - 4.0) < 1e-9


def test_compute_metrics_aos():
    """AoS = 相邻转向变化量的平均绝对值，衡量转向平滑度。"""
    smooth = np.array([0.1, 0.1, 0.1, 0.1])       # 无变化
    jerky = np.array([0.1, -0.1, 0.1, -0.1])      # 抖动
    ms = bench.compute_metrics(steer=smooth)
    mj = bench.compute_metrics(steer=jerky)
    assert abs(ms["aos"]) < 1e-12, f"平滑序列 AoS 应为 0，实际 {ms['aos']}"
    assert abs(mj["aos"] - 0.2) < 1e-9, f"抖动序列 AoS 应为 0.2，实际 {mj['aos']}"
    assert mj["aos"] > ms["aos"], "AoS 应能区分平滑与抖动"


def test_compute_metrics_regression():
    """回归指标：MAE / RMSE / 相关系数 / 方向一致率。"""
    y = np.array([-1.0, -0.5, 0.5, 1.0])
    p = np.array([-0.9, -0.4, 0.6, 0.9])
    m = bench.compute_metrics(pred=p, target=y)
    assert abs(m["reg_mae"] - 0.1) < 1e-9, f"MAE 错误: {m['reg_mae']}"
    assert abs(m["reg_rmse"] - 0.1) < 1e-9, f"RMSE 错误: {m['reg_rmse']}"
    assert m["reg_corr"] > 0.99, f"完全同向的序列相关系数应接近 1，实际 {m['reg_corr']}"
    assert abs(m["reg_sign_acc"] - 1.0) < 1e-9, f"方向应全对，实际 {m['reg_sign_acc']}"


def test_compute_metrics_handles_empty_and_single():
    """空输入与单元素输入不应抛异常（基准套件里可能出现零长度轨迹）。"""
    assert bench.compute_metrics() == {}
    m = bench.compute_metrics(steer=[0.5], speed=[3.0], lateral_err=[0.0], latency_ms=[1.0])
    assert m["aos"] == 0.0, "单元素序列的 AoS 应为 0"
    assert m["steer_mean"] == 0.5


def test_steer_saturation_ratio():
    """转向饱和比例：被打到 ±1 边界的比例。"""
    s = np.array([1.0, -1.0, 0.5, 0.0])
    m = bench.compute_metrics(steer=s)
    assert abs(m["steer_saturation_ratio"] - 0.5) < 1e-9, \
        f"饱和比例应为 0.5，实际 {m['steer_saturation_ratio']}"


# ---------------------------------------------------------------- 副本一致性
def test_sibling_nn_models_are_consistent():
    """各功能包内的 `nn_models.py` 副本必须保持一致。

    每个包自带一份副本是为了"独立可编译/可运行"，但多副本会导致
    "修了一个包、忘了另一个包"的缺陷。此测试在副本分歧时失败并给出差异提示。
    """
    ground = os.path.dirname(_PKG)
    pattern = os.path.join(ground, "carla_*", "carla_*", "nn_models.py")
    files = sorted(glob.glob(pattern))
    if len(files) < 2:
        # 只拿到本包（其他功能包未随本分支检出）时无法比较，跳过
        print("    （仅发现 %d 份副本，跳过一致性比较）" % len(files))
        return

    digests = {}
    for f in files:
        with open(f, "rb") as fh:
            digests[f] = hashlib.md5(fh.read()).hexdigest()

    uniq = set(digests.values())
    assert len(uniq) == 1, (
        "nn_models.py 副本出现分歧，请同步以下文件：\n  "
        + "\n  ".join(f"{os.path.relpath(f, ground)}  {d[:12]}" for f, d in digests.items())
        + "\n（历史教训：性能评价包用了旧副本，端到端 MAE 一度高达 36.12）")


def test_nn_models_has_tanh_and_argmax_pool():
    """本包的 nn_models.py 必须含两处关键修复（回归测试）。"""
    src_path = os.path.join(_PKG, "carla_benchmark_suite", "nn_models.py")
    with open(src_path, encoding="utf-8") as f:
        src = f.read()
    assert "np.tanh(fc_out)" in src, \
        "nn_models.py 缺少推理 tanh（会导致输出无界、MAE 高达数十）"
    assert "(xc == am)" in src or "argmax" in src, \
        "nn_models.py 的 maxpool 反向未按 argmax 回传（会导致网络学不动）"


# ---------------------------------------------------------------- 套件
def test_benchmark_registry_complete():
    """基准评测项注册表应包含三个可离线评测的模块。"""
    for key in ("perception", "navigation", "end_to_end"):
        assert key in bench.BENCHMARKS, f"缺少评测项 {key}"
    for key in ("control", "perception", "navigation", "end_to_end"):
        assert key in bench.MODULES, f"缺少模块登记 {key}"
    # 每个模块都要有功能包名、任务描述与说明
    for key, info in bench.MODULES.items():
        for field in ("package", "task", "desc"):
            assert info.get(field), f"模块 {key} 缺少字段 {field}"


def test_module_path_resolution():
    """调度器应能定位到磁盘上存在的兄弟功能包，缺失时返回 None 而非抛异常。"""
    p = bench._module_main_path("end_to_end")
    if p is not None:
        assert os.path.isfile(p), f"返回的路径不存在: {p}"
        assert p.endswith("main.py")
    assert bench._module_main_path("不存在的模块") is None


def test_render_metric_bars_exports_png():
    """指标对比图应能导出为有效 PNG。"""
    fake = {
        "perception": {"percept_acc": 0.995, "control_mse": 0.003, "lateral_rmse": 0.231},
        "navigation": {"plan_mse": 0.006, "known_ratio": 0.29, "occupied_cells": 1380,
                       "nav_min_dist": 1.458},
        "end_to_end": {"reg_mae": 0.094, "reg_sign_acc": 0.98},
    }
    with tempfile.TemporaryDirectory() as d:
        p = bench.render_metric_bars(fake, os.path.join(d, "m.png"))
        assert os.path.isfile(p), "对比图未生成"
        with open(p, "rb") as f:
            head = f.read(8)
        assert head == b"\x89PNG\r\n\x1a\n", "导出的不是有效 PNG"


def test_render_metric_bars_handles_empty():
    """全部评测项失败时，对比图应导出空白图而不是崩溃。"""
    with tempfile.TemporaryDirectory() as d:
        p = bench.render_metric_bars({}, os.path.join(d, "empty.png"))
        assert os.path.isfile(p), "空数据也应产出文件"


def _run_all():
    fns = sorted(k for k in list(globals()) if k.startswith("test_"))
    passed, failed = 0, []
    for fn in fns:
        try:
            globals()[fn]()
            passed += 1
            print(f"[PASS] {fn}")
        except Exception as exc:  # noqa: BLE001
            failed.append(fn)
            print(f"[FAIL] {fn}: {exc}")
    print(f"\n==== 测试结果：PASS={passed}  FAIL={len(failed)} ====")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(_run_all())
