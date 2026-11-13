"""AEB 单元测试：传感器优先的制动决策、本模块 actor 归属、语义激光雷达前车选取。

对应 PR #955 评审：
- 清理只删本模块创建的 actor（role_name 前缀）
- 制动判据必须用传感器观测（语义激光雷达实例），真值仅评估
- 补充外部 actor、传感器遮挡/缺失的用例
"""
import os
import sys
import types

import numpy as np
import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from carla_ttc_emergency_braking import app as aeb_app  # noqa: E402
from carla_ttc_emergency_braking.app import (  # noqa: E402
    VEHICLE_TAGS,
    client_flavor,
    decide_brake,
    front_vehicle_distance,
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
    """真值里有车但传感器没观测到 -> 不制动（证明决策不走真值通路）。"""
    brake, ttc, src = decide_brake(None, 13.9, 22.0, 1.6, sensor_lost=False)
    assert brake is False and ttc == -1.0 and src == "no_target"


def test_brake_when_sensor_lost():
    """点云流中断（遮挡/掉线）-> 失效安全，直接制动。"""
    brake, _ttc, src = decide_brake(None, 13.9, 22.0, 1.6, sensor_lost=True)
    assert brake is True and src == "sensor_fault"


def test_no_brake_when_target_far():
    brake, _ttc, src = decide_brake(55.0, 13.9, 22.0, 1.6)
    assert brake is False and src == "lidar"


def test_ttc_threshold_from_config():
    cfg = _cfg()["control"]
    distance = 20.0
    speed = distance / (float(cfg["ttc_threshold"]) - 0.1)
    brake, ttc, _ = decide_brake(distance, speed, 1.0, float(cfg["ttc_threshold"]))
    assert ttc <= float(cfg["ttc_threshold"])
    assert brake is True


# ---------------------------------------------------------------- 归属判断（清理范围）
def test_owned_by_module_only_own_prefix():
    assert owned_by_module("aeb_ego") is True
    assert owned_by_module("aeb_lidar") is True
    assert owned_by_module("hero") is False            # 别人的车
    assert owned_by_module("autopilot_vehicle") is False
    assert owned_by_module("") is False
    assert owned_by_module(None) is False


# ---------------------------------------------------------------- 语义激光雷达前车选取
def _cloud(rows):
    """rows: [(x, y, z, instance, tag), ...] -> (xyz, idx, tag)"""
    arr = np.array(rows, dtype=np.float64)
    return (arr[:, :3].astype(np.float32),
            arr[:, 3].astype(np.uint32),
            arr[:, 4].astype(np.uint32))


def test_front_vehicle_none_on_empty():
    assert front_vehicle_distance(None, None, None) == (None, None)
    empty = (np.empty((0, 3), dtype=np.float32), np.empty(0, np.uint32), np.empty(0, np.uint32))
    assert front_vehicle_distance(*empty) == (None, None)


def test_front_vehicle_picks_nearest_vehicle_instance():
    car = VEHICLE_TAGS[0]
    rows = [(30.0 + i * 0.1, 0.2, 0.2, 7, car) for i in range(6)]
    rows += [(10.0 + i * 0.1, -0.2, 0.2, 9, car) for i in range(6)]      # 更近的车
    d, inst = front_vehicle_distance(*_cloud(rows))
    assert inst == 9 and d == pytest.approx(10.0, abs=0.2)


def test_front_vehicle_prefers_own_lane_over_closer_neighbour():
    """评审反例：本车道前车 30 m，相邻车道 10 m 有更近的车 -> 仍选本车道那辆。"""
    car = VEHICLE_TAGS[0]
    rows = [(30.0 + i * 0.1, 0.0, 0.2, 7, car) for i in range(6)]
    rows += [(10.0 + i * 0.1, 3.5, 0.2, 9, car) for i in range(6)]
    d, inst = front_vehicle_distance(*_cloud(rows))
    assert inst == 7
    assert d == pytest.approx(30.0, abs=0.2)


def test_front_vehicle_rejects_instance_centred_outside_lane():
    """邻道车只有边缘点压到 2.2 m 以内，也不能按单点就把整辆车当本车道目标。"""
    car = VEHICLE_TAGS[0]
    rows = [(10.0 + i * 0.1, 2.0 if i < 2 else 3.3, 0.2, 3, car) for i in range(8)]
    assert front_vehicle_distance(*_cloud(rows)) == (None, None)


def test_vehicle_tags_default_only_car():
    """本环境实测车辆标签只有 14；15/16/18（桥/轨道/红绿灯）不能被当前车。"""
    assert VEHICLE_TAGS == (14,)
    for tag in (15, 16, 18):
        rows = [(6.0 + i * 0.1, 0.0, 0.2, 5, tag) for i in range(8)]
        assert front_vehicle_distance(*_cloud(rows)) == (None, None)


def test_vehicle_tags_configurable():
    """换平台时标签编号可能不同，通过参数传入即可，不在代码里写死。"""
    rows = [(8.0 + i * 0.1, 0.0, 0.2, 2, 10) for i in range(8)]
    d, inst = front_vehicle_distance(*_cloud(rows), vehicle_tags=(10,))
    assert inst == 2 and d == pytest.approx(8.0, abs=0.3)


def test_front_vehicle_ignores_non_vehicle_semantics():
    """路沿/建筑（非车辆标签）不能被当成前车。"""
    rows = [(5.0 + i * 0.1, 0.0, 0.2, 3, 7) for i in range(8)]          # tag=7 = 路面
    assert front_vehicle_distance(*_cloud(rows)) == (None, None)


def test_front_vehicle_ignores_sparse_noise():
    """单个车辆实例点不足以成目标（抗单点噪声）。"""
    rows = [(8.0, 0.0, 0.2, 5, VEHICLE_TAGS[0])]
    assert front_vehicle_distance(*_cloud(rows)) == (None, None)


def test_front_vehicle_out_of_range():
    rows = [(200.0 + i * 0.1, 0.0, 0.2, 4, VEHICLE_TAGS[0]) for i in range(8)]
    assert front_vehicle_distance(*_cloud(rows)) == (None, None)


def test_front_vehicle_ignores_ground_points():
    """车底/地面回波（z 太低）不应参与测距。"""
    rows = [(6.0 + i * 0.1, 0.0, -1.9, 6, VEHICLE_TAGS[0]) for i in range(8)]
    assert front_vehicle_distance(*_cloud(rows)) == (None, None)


# ---------------------------------------------------------------- 判据与配置一致性
def test_should_brake_basic():
    assert should_brake(10.0, 10.0, 22.0, 1.6)[0] is True
    assert should_brake(55.0, 10.0, 22.0, 1.6)[0] is False
    assert should_brake(None, 10.0, 22.0, 1.6)[0] is False


def test_config_has_sensor_timeout():
    cfg = _cfg()["control"]
    assert float(cfg["sensor_timeout_s"]) > 0


def test_ttc_exactly_at_threshold_brakes():
    """TTC 恰好等于阈值（<=）应触发制动。"""
    brake, ttc, _ = decide_brake(16.0, 10.0, 1.0, 1.6)
    assert ttc == pytest.approx(1.6)
    assert brake is True


def test_distance_exactly_min_brake_distance_brakes():
    """距离恰好等于 min_brake_distance（<=）应触发制动。"""
    brake, _ttc, _ = decide_brake(22.0, 30.0, 22.0, 1.6)
    assert brake is True


def test_low_speed_has_no_ttc():
    """speed <= 0.3 m/s 时不计算 TTC（返回 -1），且距离够远则不动。"""
    brake, ttc = should_brake(50.0, 0.3, 22.0, 1.6)
    assert ttc == -1.0
    assert brake is False


# ---------------------------------------------------------------- 客户端识别
def test_client_flavor_uses_actually_imported_client(monkeypatch):
    """两个客户端都装时，按实际 import 到的 Client 能力判断，而不是 distribution 顺序。"""

    class HutbClient:
        def set_files_base_folder(self, *_):
            pass

    class UpstreamClient:      # 上游 carla 没有地图按需下载接口
        pass

    monkeypatch.setattr(aeb_app, "carla", types.SimpleNamespace(
        Client=HutbClient, __name__="carla", __file__=".../carla/__init__.py"))
    assert client_flavor() == "hutb"

    monkeypatch.setattr(aeb_app, "carla", types.SimpleNamespace(
        Client=UpstreamClient, __name__="carla", __file__=".../carla/__init__.py"))
    assert client_flavor() == "carla"


def test_client_flavor_unknown_without_client(monkeypatch):
    monkeypatch.setattr(aeb_app, "carla", None)
    assert client_flavor() == "unknown"


def test_config_declares_lane_gate_and_tags():
    cfg = _cfg()["sensor"]
    assert float(cfg["target_half_width"]) == pytest.approx(2.2)
    assert [int(t) for t in cfg["vehicle_tags"]] == [14]
