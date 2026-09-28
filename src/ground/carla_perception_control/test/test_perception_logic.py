#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 感知 + 轨迹跟踪（神经网络版）—— 单元测试。

不依赖 CARLA 服务端、不依赖 ROS，只验证神经网络与特征提取逻辑：
  1. 感知分类器能收敛（精度）
  2. 控制策略网络能收敛（MSE）
  3. 特征归一化把各维度压到相近尺度
  4. 合成数据集形状正确
  5. 模型保存/加载往返一致

运行：  python3 test/test_perception_logic.py
       或 pytest test/
"""

import os
import sys
import tempfile

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

import importlib.util  # noqa: E402

_spec = importlib.util.spec_from_file_location("pc_main", os.path.join(_PKG, "main.py"))
main_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(main_mod)


def test_perception_classifier_converges():
    """感知 NN：传感器特征 → 障碍类别，训练精度应 > 0.9。"""
    feat_X, feat_Y, _cX, _cY = main_mod.synth_dataset(400, seed=0)
    net = main_mod.MLPClassifier([4, 12, 3], seed=0)
    net.train(feat_X, feat_Y, epochs=300, lr=0.2, verbose=0)
    acc = float(np.mean(net.predict(feat_X) == feat_Y))
    assert acc > 0.9, f"感知 NN 精度过低: {acc}"


def test_control_policy_converges():
    """控制 NN：状态 → 转向角，回归 MSE 应足够小。

    注意：控制网络输入做了标准化（量纲差异大），因此误差必须在**标准化后的
    输入**上评估——这与在线推理 `main_mod.nn_control` 的做法一致。
    """
    _fX, _fY, ctrl_X, ctrl_Y = main_mod.synth_dataset(400, seed=0)
    sens, ctrl, _h = main_mod.train(_fX, _fY.astype(int), ctrl_X, ctrl_Y, epochs=300)
    xin = (ctrl_X - ctrl.input_mu) / ctrl.input_sd
    mse = float(np.mean(np.square(ctrl.predict(xin).ravel() - ctrl_Y)))
    assert mse < 0.02, f"控制 NN MSE 过大: {mse}"


def test_pure_pursuit_law_zero_on_straight():
    """纯跟踪几何律在航向差为 0（正对目标）时应给出零转向。"""
    assert abs(main_mod.pure_pursuit_law(0.0, 6.0)) < 1e-9, "直线行驶不应产生转向"
    # 左偏/右偏应给出相反符号的转向
    assert main_mod.pure_pursuit_law(0.5, 6.0) > 0
    assert main_mod.pure_pursuit_law(-0.5, 6.0) < 0


def test_waypoint_interpolation():
    """路点加密应保持首尾端点并显著增加点数。"""
    raw = [(0.0, 0.0), (10.0, 0.0)]
    dense = main_mod.interpolate_waypoints(raw, step=2.0)
    assert dense[0] == (0.0, 0.0), "加密后起点应保持"
    assert dense[-1] == (10.0, 0.0), "加密后终点应保持"
    assert len(dense) > len(raw), "加密后点数应增加"


def test_lateral_error_is_perpendicular_distance():
    """横向误差应为点到折线的垂距：点在折线上时为 0，偏离 3 m 时为 3。"""
    line = [(0.0, 0.0), (10.0, 0.0)]
    assert main_mod._dist_to_polyline((5.0, 0.0), line) < 1e-9
    assert abs(main_mod._dist_to_polyline((5.0, 3.0), line) - 3.0) < 1e-6


def test_feature_normalization_range():
    """特征归一化后各维应落在相近尺度（约 [0,1]）。"""
    feat = main_mod._feat_feature(0.5, 200.0, 30, 10.0)
    assert feat.shape == (4,), f"特征维度应为 4，实际 {feat.shape}"
    assert np.all(feat >= -1.0) and np.all(feat <= 1.0), f"归一化越界: {feat}"


def test_synth_dataset_shapes():
    feat_X, feat_Y, ctrl_X, ctrl_Y = main_mod.synth_dataset(120, seed=3)
    assert feat_X.shape == (120, 4)
    assert feat_Y.shape == (120,)
    assert ctrl_X.shape == (120, 2)
    assert ctrl_Y.shape == (120,)


def test_model_save_load_roundtrip():
    feat_X, feat_Y, ctrl_X, ctrl_Y = main_mod.synth_dataset(200, seed=5)
    sens, ctrl, _h = main_mod.train(feat_X, feat_Y.astype(int),
                                    ctrl_X, ctrl_Y, epochs=50)
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "nn.json")
        main_mod.save_model(p, sens, ctrl)
        loaded = main_mod.load_model(p)
    assert np.array_equal(loaded["sens"].predict(feat_X), sens.predict(feat_X)), \
        "感知 NN 存取后预测不一致"
    # 控制网络：用同一组标准化参数比较预测，且标准化参数本身也要能往返
    xin = (ctrl_X - ctrl.input_mu) / ctrl.input_sd
    xin_loaded = (ctrl_X - loaded["ctrl"].input_mu) / loaded["ctrl"].input_sd
    assert np.allclose(loaded["ctrl"].predict(xin_loaded), ctrl.predict(xin), atol=1e-4), \
        "控制 NN 存取后预测不一致"
    assert np.allclose(loaded["ctrl"].input_mu, ctrl.input_mu, atol=1e-6), \
        "标准化均值未正确保存"
    assert np.allclose(loaded["ctrl"].input_sd, ctrl.input_sd, atol=1e-6), \
        "标准化方差未正确保存"


def test_offline_demo_runs():
    """离线取证模式应能在无 CARLA 环境下跑通并导出曲线图。"""
    with tempfile.TemporaryDirectory() as d:
        rc = main_mod.run_offline_demo(d, epochs=30, sim_time=3.0)
        assert rc == 0
        produced = sorted(os.listdir(d))
        assert "percept_loss.png" in produced, f"缺少感知损失曲线: {produced}"
        assert "lateral_error.png" in produced, f"缺少横向误差曲线: {produced}"


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
