"""AEB 单元测试：传感器优先的制动决策、本模块 actor 归属、激光雷达测距边界。

对应 PR #955 评审：清理只删本模块对象；制动判据必须用传感器观测；补充外部 actor 与传感器遮挡/缺失的测试。
"""
import os
import sys

import numpy as np
import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from carla_ttc_emergency_braking.app import (  # noqa: E402
    decide_brake,
    front_distance,
    owned_by_module,
    should_brake,
)


def _cfg():
    with open(os.path.join(ROOT, "config", "params.yaml"), encoding="utf-8") as fh:
        return yaml.safe_load(fh)


# ---------------------------------------------------------------- 决策（纯传感器输入）
def test_brake_when_target_within_ttc():
    brake, ttc, src = decide_brake(20.0, 13.9, 22.0, 1.6)
    assert brake is True and src == "lidar"
    assert ttc == pytest.approx(20.0 / 13.9)


def test_no_brake_when_no_target_and_sensor_ok():
    """没有观测到目标、传感器正常 -> 不制动（不能因为真值里有车就刹车）。"""
    brake, ttc, src = decide_brake(None, 13.9, 22.0, 1.6, sensor_lost=False)
    assert brake is False and ttc == -1.0 and src == "no_target"


def test_brake_when_sensor_lost():
    """遮挡/掉线（连续无有效观测）-> 失效安全，直接制动。"""
    brake, _ttc, src = decide_brake(None, 13.9, 22.0, 1.6, sensor_lost=True)
    assert brake is True and src == "sensor_fault"


def test_no_brake_when_target_far():
    brake, ttc, src = decide_brake(55.0, 13.9, 22.0, 1.6)
    assert brake is False and src == "lidar"


def test_ttc_threshold_from_config():
    cfg = _cfg()["control"]
    distance = 20.0
    speed = distance / (float(cfg["ttc_threshold"]) - 0.1)   # TTC 略小于阈值 -> 该制动
    brake, ttc, _ = decide_brake(distance, speed, 1.0, float(cfg["ttc_threshold"]))
    assert ttc <= float(cfg["ttc_threshold"])
    assert brake is True


# ---------------------------------------------------------------- 归属判断（清理范围）
def test_owned_by_module_only_own_prefix():
    assert owned_by_module("aeb_ego") is True
    assert owned_by_module("aeb_lidar") is True
    assert owned_by_module("hero") is False          # 别人的车
    assert owned_by_module("") is False
    assert owned_by_module(None) is False
    assert owned_by_module("autopilot_vehicle") is False


# ---------------------------------------------------------------- 激光雷达测距
def _points(rows):
    return np.array(rows, dtype=np.float32)


def test_front_distance_empty():
    assert front_distance(np.empty((0, 4), dtype=np.float32)) is None


def test_front_distance_ignores_ground_and_side_points():
    # 有效簇：10 m 处 6 个点（达标）；干扰：地面点、旁车道点、更远的 30 m 簇
    pts = _points([[10.0 + 0.05 * i, 0.0, 0.2, 0.5] for i in range(6)]
                  + [[10.0, 0.0, -1.9, 0.5], [10.0, 3.0, 0.0, 0.5]]
                  + [[30.0 + 0.05 * i, 0.0, 0.2, 0.5] for i in range(6)])
    assert front_distance(pts, max_range=60.0) == pytest.approx(10.1, abs=0.3)


def test_front_distance_rejects_single_point_noise():
    """单个噪声点不能当成目标（旧实现取 2% 分位数就会误刹）。"""
    pts = _points([[5.0, 0.0, 0.2, 0.5]] + [[40.0 + 0.05 * i, 0.0, 0.2, 0.5] for i in range(8)])
    assert front_distance(pts, max_range=60.0) == pytest.approx(40.2, abs=0.4)


def test_front_distance_clusters_spanning_gaps():
    """一辆车在 50 m 处的点会跨越若干 0.5 m 区间，聚类后仍应识别为同一目标。"""
    pts = _points([[50.0 + 0.3 * i, 0.05 * (i % 3), 0.2, 0.5] for i in range(8)])
    assert front_distance(pts, max_range=60.0) == pytest.approx(51.0, abs=1.0)


def test_front_distance_occluded_returns_none():
    """全部点都在走廊外（例如被遮挡后只剩路边建筑）-> 没有前方目标。"""
    pts = _points([[5.0, 8.0, 0.5, 0.5], [6.0, -9.0, 0.5, 0.5]])
    assert front_distance(pts, max_range=60.0) is None


def test_front_distance_out_of_range():
    pts = _points([[200.0, 0.0, 0.2, 0.5]])
    assert front_distance(pts, max_range=60.0) is None


# ---------------------------------------------------------------- 判据与配置一致性
def test_should_brake_basic():
    assert should_brake(10.0, 10.0, 22.0, 1.6)[0] is True     # 距离过近
    assert should_brake(55.0, 10.0, 22.0, 1.6)[0] is False    # 距离足够
    assert should_brake(None, 10.0, 22.0, 1.6)[0] is False    # 无目标


def test_config_has_sensor_timeout():
    cfg = _cfg()["control"]
    assert float(cfg["sensor_timeout_s"]) > 0
