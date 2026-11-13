# -*- coding: utf-8 -*-
"""节点状态机与打包完整性。状态机不依赖 rclpy，可以离线测；rclpy 的部分跳过。"""
import json
import math
import os
import re
import sys

import numpy as np
import pytest

from uav_target_geolocation.core.geometry import (los_to,
                                                 rotate_by_angle_err)
from uav_target_geolocation.ros.geo_node import (GeoState, los_body_to_world,
                                                 node_params, quat_to_matrix)


def _feed_station(state: GeoState, pos, n: int = 100, bias=(0.0, 0.0)):
    state.on_odom(pos, (1.0, 0.0, 0.0, 0.0))
    truth = (0.0, 0.0, 0.0)
    base = los_to(truth, pos)
    out = None
    for _ in range(n):
        out = state.on_los(base)
    return out


def test_quat_to_matrix_is_identity_for_unit_w():
    R = quat_to_matrix((1.0, 0.0, 0.0, 0.0))
    assert R == pytest.approx(np.eye(3))


def test_quat_to_matrix_rotates_x_to_y_for_90deg_yaw():
    c, s = np.cos(np.pi / 4), np.sin(np.pi / 4)
    R = quat_to_matrix((c, 0.0, 0.0, s))
    assert R @ np.array([1.0, 0.0, 0.0]) == pytest.approx([0.0, 1.0, 0.0],
                                                          abs=1e-12)


def test_los_body_to_world_normalizes_and_rejects_garbage():
    v = los_body_to_world((0.0, 0.0, -5.0), (1.0, 0.0, 0.0, 0.0))
    assert float(np.linalg.norm(v)) == pytest.approx(1.0)
    with pytest.raises(ValueError):
        los_body_to_world((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        los_body_to_world((float("nan"), 0.0, -1.0), (1.0, 0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        los_body_to_world((0.0, 0.0, -1.0), (0.0, 0.0, 0.0, 0.0))


def test_state_machine_single_then_fused():
    st = GeoState()
    out1 = _feed_station(st, (60.0, 0.0, 80.0))
    assert out1["ok"] and out1["mode"] == "single"
    assert abs(out1["x"]) < 1e-6 and abs(out1["y"]) < 1e-6

    _ = _feed_station(st, (0.0, 60.0, 80.0))          # 攒满自动收尾并交会
    assert len(st.stations_pos) == 2
    diag = json.loads(st.diag())
    assert diag["n_stations"] == 2
    assert diag["intersection_deg"] == pytest.approx(90.0, abs=0.1)


def test_state_machine_rejects_los_without_odom():
    st = GeoState()
    out = st.on_los((0.0, 0.0, -1.0))
    assert not out["ok"] and "里程计" in out["reason"]


def test_state_machine_manual_boundary_without_samples_is_reported():
    st = GeoState()
    out = st.on_new_station()
    assert not out["ok"]


def test_node_params_merges_and_validates():
    p = node_params({"samples_per_station": 50})
    assert p["samples_per_station"] == 50
    assert p["odom_topic"] == "/uav/odom"
    with pytest.raises(KeyError):
        node_params({"no_such_key": 1})
    with pytest.raises(ValueError):
        node_params({"samples_per_station": 0})
    with pytest.raises(ValueError):
        node_params({"los_bias_deg": [10.0, 0.0]})
    with pytest.raises(ValueError):
        node_params({"ground_z": 5.0})


def test_node_import_does_not_require_rclpy():
    """import geo_node 不应该拉起 rclpy（rclpy 只在 main() 里 import）。"""
    import uav_target_geolocation.ros.geo_node as mod
    assert hasattr(mod, "GeoState") and hasattr(mod, "main")


def test_rclpy_paths_are_skipped_gracefully():
    rclpy = pytest.importorskip("rclpy")
    del rclpy                                    # 有 ROS 2 的机器才会走到这
    from uav_target_geolocation.ros.geo_node import main
    assert callable(main)


# --------------------------------------------------------------------------- #
# 打包完整性：评审机器上最常翻车的是 data_files 与包名不一致
# --------------------------------------------------------------------------- #
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_package_xml_name_matches():
    with open(os.path.join(_ROOT, "package.xml"), encoding="utf-8") as f:
        text = f.read()
    m = re.search(r"<name>([^<]+)</name>", text)
    assert m and m.group(1) == "uav_target_geolocation"
    assert "<name>uav_mujoco_sim</name>" not in text


def test_config_yaml_has_geo_node_section():
    import yaml
    with open(os.path.join(_ROOT, "config", "uav_target_geolocation.yaml"),
              encoding="utf-8") as f:
        data = yaml.safe_load(f)
    params = data["geo_node"]["ros__parameters"]
    for key in ("samples_per_station", "min_stations_for_fusion",
                "odom_topic", "ground_topic", "ground_z"):
        assert key in params
    assert params["ground_z"] <= 0.0


def test_resource_marker_exists():
    assert os.path.isfile(os.path.join(_ROOT, "resource",
                                       "uav_target_geolocation"))


def test_setup_data_files_reference_existing_paths(monkeypatch):
    """执行 setup.py 并检查 data_files：这里曾经是评审最爱翻车的地方。

    本机没有 setuptools 也能测：注入一个只记录参数的桩模块。
    """
    import importlib.util
    import types

    captured = {}
    fake = types.ModuleType("setuptools")
    fake.setup = lambda **kw: captured.update(kw)
    fake.find_packages = lambda **kw: []
    monkeypatch.setitem(sys.modules, "setuptools", fake)

    spec = importlib.util.spec_from_file_location(
        "tg_setup", os.path.join(_ROOT, "setup.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    flat = [p for _, files in captured["data_files"] for p in files]
    assert "README.md" in flat and "main.py" in flat
    for rel in flat:
        assert os.path.isfile(os.path.join(_ROOT, rel)), rel
    entry = captured["entry_points"]["console_scripts"]
    assert "main = uav_target_geolocation.cli:main" in entry
    assert "geo_node = uav_target_geolocation.ros.geo_node:main" in entry


def test_readme_documents_the_three_cases():
    with open(os.path.join(_ROOT, "README.md"), encoding="utf-8") as f:
        text = f.read()
    for name in ("single_station", "two_station_orthogonal", "two_station_shallow"):
        assert name in text


# --------------------------------------------------------------------------- #
# 零位偏置补偿（评审指出 los_bias_deg 只存不用后补的行为）
# --------------------------------------------------------------------------- #
def _geo_error_with(params, los_body):
    st = GeoState(params)
    st.on_odom((60.0, 0.0, 80.0), (1.0, 0.0, 0.0, 0.0))
    truth = los_to((0.0, 0.0, 0.0), (60.0, 0.0, 80.0))
    for _ in range(3):
        out = st.on_los(los_body)
    return math.hypot(out["x"], out["y"])


def test_los_bias_parameter_is_actually_applied():
    """los_bias_deg=(1,0) 时应在机体系里把 +1° 标定偏置扣掉。"""
    truth = los_to((0.0, 0.0, 0.0), (60.0, 0.0, 80.0))
    biased = rotate_by_angle_err(truth, 1.0, axis="x")   # 测量 = 真值偏 1°
    good = _geo_error_with({"samples_per_station": 3,
                           "los_bias_deg": [1.0, 0.0]}, biased)
    raw = _geo_error_with({"samples_per_station": 3}, biased)
    assert good < 0.3
    # 横向（绕 x 转）1° 在 h=80 m 上 ≈ h·tan1° ≈ 1.40 m（俯仰向才是 2.18 m/°）
    assert raw > 1.0


def test_zero_bias_leaves_los_untouched():
    st = GeoState({"los_bias_deg": [0.0, 0.0]})
    v = [0.1, -0.2, -1.0]
    assert st._debias(v) == pytest.approx(np.asarray(v, dtype=float))


# --------------------------------------------------------------------------- #
# 参数只写一处：config yaml ↔ 节点默认值 ↔ launch 不再抄第二份
# --------------------------------------------------------------------------- #
def test_config_yaml_keys_match_node_defaults():
    yaml = pytest.importorskip("yaml")
    from uav_target_geolocation.ros.geo_node import DEFAULTS
    path = os.path.join(_ROOT, "config", "uav_target_geolocation.yaml")
    data = yaml.safe_load(open(path, encoding="utf-8"))
    params = data["geo_node"]["ros__parameters"]
    assert set(params) == set(DEFAULTS), \
        f"config 与节点默认值键不一致：{set(params) ^ set(DEFAULTS)}"
    assert float(params["ground_z"]) == float(DEFAULTS["ground_z"])


def test_launch_reads_params_file_instead_of_a_second_copy():
    text = open(os.path.join(_ROOT, "launch", "main.launch.py"),
                encoding="utf-8").read()
    assert "params_file" in text and "config" in text
    assert "los_sigma_deg" not in text     # 不允许把参数再抄一遍


def test_single_station_estimate_off_origin_target():
    """回归：单站打点必须用 Ray(origin, direction)。

    之前把单位方向向量喂给了 make_ray(origin, target) 的 target 槽——
    本模块视线恰好都指向世界原点，错误才没露馅；目标一挪开就错得离谱。
    """
    st = GeoState({"samples_per_station": 5})
    pos = (60.0, 30.0, 80.0)
    tgt = np.array([10.0, 5.0, 0.0])
    st.on_odom(pos, (1.0, 0.0, 0.0, 0.0))
    los = los_to(tgt, np.asarray(pos))
    out = None
    for _ in range(5):
        out = st.on_los(los)
    assert out["ok"] and out["mode"] == "single"
    assert math.hypot(out["x"] - tgt[0], out["y"] - tgt[1]) < 0.05


# --------------------------------------------------------------------------- #
# 时间同步与坐标系检查（评审指出：视线直接用最近一次里程计姿态，无 stale 保护）
# --------------------------------------------------------------------------- #
def test_stale_odom_timestamp_is_rejected():
    st = GeoState({"max_stamp_skew_s": 0.2})
    st.on_odom((60.0, 0.0, 80.0), (1.0, 0.0, 0.0, 0.0), stamp_s=10.0)
    out = st.on_los((0.0, 0.0, -1.0), stamp_s=10.5)
    assert not out["ok"] and "时间戳" in out["reason"]


def test_fresh_timestamps_are_accepted():
    st = GeoState({"max_stamp_skew_s": 0.2})
    st.on_odom((60.0, 0.0, 80.0), (1.0, 0.0, 0.0, 0.0), stamp_s=10.0)
    out = st.on_los((0.0, 0.0, -1.0), stamp_s=10.1)
    assert out["ok"] and out["mode"] == "accumulating"


def test_odom_frame_id_mismatch_is_rejected():
    st = GeoState({"odom_frame_id": "map", "los_frame_id": "body"})
    st.on_odom((60.0, 0.0, 80.0), (1.0, 0.0, 0.0, 0.0), frame_id="odom")
    out = st.on_los((0.0, 0.0, -1.0), frame_id="body")
    assert not out["ok"] and "坐标系" in out["reason"]


def test_los_frame_id_mismatch_is_rejected():
    st = GeoState({"odom_frame_id": "map", "los_frame_id": "body"})
    st.on_odom((60.0, 0.0, 80.0), (1.0, 0.0, 0.0, 0.0), frame_id="map")
    out = st.on_los((0.0, 0.0, -1.0), frame_id="odom")
    assert not out["ok"] and "坐标系" in out["reason"]


def test_matching_frames_and_timestamps_are_accepted():
    st = GeoState({"odom_frame_id": "map", "los_frame_id": "body",
                   "max_stamp_skew_s": 0.2})
    st.on_odom((60.0, 0.0, 80.0), (1.0, 0.0, 0.0, 0.0),
               stamp_s=10.0, frame_id="map")
    out = st.on_los((0.0, 0.0, -1.0), stamp_s=10.05, frame_id="body")
    assert out["ok"]


def test_offline_usage_without_timestamps_still_works():
    """离线（不传时间戳/坐标系）仍能跑，向后兼容。"""
    st = GeoState()
    st.on_odom((60.0, 0.0, 80.0), (1.0, 0.0, 0.0, 0.0))
    out = st.on_los((0.0, 0.0, -1.0))
    assert out["ok"] and out["mode"] == "accumulating"
