"""语义激光雷达解析的单元测试（不需要 CARLA 服务端）。

重点覆盖评审指出的隐蔽 bug：每个点是 6 个 32 位字，其中 ObjIdx / ObjTag 是**整数**，
必须按 uint32 读；若当成 float32 会把实例号读成 1e-44 量级的极小浮点数。
"""
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
APP = os.path.join(ROOT, 'carla_lidar_obstacle_detection', 'app.py')


@pytest.fixture(scope='module')
def helpers():
    src = open(APP, encoding='utf-8').read()
    start = src.index('def parse_semantic_lidar(')
    end = src.index('def detect_objects(')
    ns = {'np': np}
    exec(compile(src[start:end], APP, 'exec'), ns)  # noqa: S102
    return ns


def build_raw(points):
    """points: [(x, y, z, cos, obj_idx, obj_tag), ...] -> 原始字节。"""
    words = []
    for x, y, z, c, idx, tag in points:
        words.extend(np.array([x, y, z, c], dtype=np.float32).view(np.uint32).tolist())
        words.extend([idx, tag])
    return np.array(words, dtype=np.uint32).tobytes()


def test_xyz_and_indices_are_parsed_correctly(helpers):
    raw = build_raw([(12.5, -1.5, 0.8, 0.9, 194, 14),
                     (30.0, 2.0, -0.2, 0.5, 0, 1)])
    xyz, idx, tag = helpers['parse_semantic_lidar'](raw)
    assert xyz.shape == (2, 3)
    assert xyz[0, 0] == pytest.approx(12.5, abs=1e-4)
    assert idx.tolist() == [194, 0]
    assert tag.tolist() == [14, 1]


def test_index_is_not_read_as_tiny_float(helpers):
    """如果按 float32 读实例号，194 会变成 2.7e-43 之类；这里必须严格等于 194。"""
    raw = build_raw([(5.0, 0.0, 0.0, 1.0, 194, 14)])
    _xyz, idx, _tag = helpers['parse_semantic_lidar'](raw)
    assert int(idx[0]) == 194


def test_class_from_tag(helpers):
    cls = helpers['class_from_tag']
    assert cls(14) == 'vehicle'
    assert cls(15) == 'vehicle'
    assert cls(12) == 'walker'
    assert cls(1) == 'other'
def test_replay_runs_without_carla():
    """离线回放回归：world=None 时 detect_objects 不能崩（评审复核时踩到的阻塞 bug）。"""
    import numpy as np
    from carla_lidar_obstacle_detection.app import detect_objects, load_params

    npz = os.path.join(ROOT, "docs", "replay_sample.npz")
    if not os.path.exists(npz):
        import pytest as _pytest
        _pytest.skip("replay sample not shipped")
    data = np.load(npz)
    cfg = load_params(os.path.join(ROOT, "config", "params.yaml"))
    dets = detect_objects(None, (data["xyz"], data["obj_idx"], data["obj_tag"]), cfg)
    assert isinstance(dets, list)
    assert all("cls" in d and "dist" in d for d in dets)


def test_detect_objects_none_world_empty_cloud():
    from carla_lidar_obstacle_detection.app import detect_objects, load_params

    cfg = load_params(os.path.join(ROOT, "config", "params.yaml"))
    assert detect_objects(None, None, cfg) == []
