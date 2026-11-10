# -*- coding: utf-8 -*-
"""膝关节模块测试。

分两层：
  1. 控制器纯逻辑（不依赖 MuJoCo，任何环境都能跑）
  2. 模型集成（需要 mujoco；缺失时自动 skip）

`colcon test` 会跑全部用例，因此模型加载、屈肌/伸肌方向、双向限位
这些原本只在 validate_model.py 里验证的行为，现在也在 CI 覆盖范围内。
"""
import math
import os

import pytest

from mujoco_musculoskeletal_knee_tracking.controller import (
    clamp,
    pd_to_activation,
)

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(os.path.dirname(HERE), "mujoco_musculoskeletal_knee_tracking",
                     "model", "muscle_knee.xml")

MT = 17.5          # 模型实际最大关节力矩 = 500 N × 0.035 m


# ---------------- 控制器纯逻辑 ----------------

def test_clamp_bounds():
    assert clamp(0.0) == 0.0
    assert clamp(1.0) == 1.0
    assert clamp(-1.0) == 0.0
    assert clamp(2.0) == 1.0
    assert clamp(0.5) == 0.5


def test_positive_error_activates_flexor_only():
    flex, ext = pd_to_activation(0.5, 0.0, kp=30.0, kd=3.0, max_torque=MT)
    assert flex > 0.0 and ext == 0.0


def test_negative_error_activates_extensor_only():
    flex, ext = pd_to_activation(-0.5, 0.0, kp=30.0, kd=3.0, max_torque=MT)
    assert ext > 0.0 and flex == 0.0


def test_activation_clamped_to_unit_interval():
    flex, _ = pd_to_activation(3.0, 0.0, kp=30.0, kd=3.0, max_torque=MT)
    assert 0.0 <= flex <= 1.0
    _, ext = pd_to_activation(-3.0, 0.0, kp=30.0, kd=3.0, max_torque=MT)
    assert 0.0 <= ext <= 1.0


def test_velocity_damping_reduces_activation():
    f_static, _ = pd_to_activation(0.5, 0.0, kp=30.0, kd=3.0, max_torque=MT)
    f_damped, _ = pd_to_activation(0.5, 1.0, kp=30.0, kd=3.0, max_torque=MT)
    assert f_damped <= f_static


def test_upper_limit_blocks_flexion():
    flex, _ = pd_to_activation(1.0, 0.0, kp=30.0, kd=3.0, max_torque=MT,
                               limit_reached=True)
    assert flex == 0.0


def test_lower_limit_blocks_extension():
    _, ext = pd_to_activation(-1.0, 0.0, kp=30.0, kd=3.0, max_torque=MT,
                              limit_low_reached=True)
    assert ext == 0.0


def test_non_finite_leads_to_safe_stop():
    for bad in (float("nan"), float("inf"), float("-inf")):
        flex, ext = pd_to_activation(bad, 0.0, kp=30.0, kd=3.0, max_torque=MT)
        assert flex == 0.0 and ext == 0.0


# ---------------- 模型集成 ----------------

mujoco = pytest.importorskip("mujoco")


def _load():
    with open(MODEL, "r", encoding="utf-8") as f:
        return mujoco.MjModel.from_xml_string(f.read())


def test_model_loads():
    model = _load()
    assert model.nq == 1
    assert model.nu == 2
    names = {model.actuator(i).name for i in range(model.nu)}
    assert names == {"flexor", "extensor"}


def test_flexor_increases_angle():
    model = _load()
    data = mujoco.MjData(model)
    data.ctrl[model.actuator("flexor").id] = 1.0
    mujoco.mj_forward(model, data)
    for _ in range(400):
        mujoco.mj_step(model, data)
    assert math.degrees(data.qpos[0]) > 1.0


def test_extensor_decreases_angle():
    model = _load()
    data = mujoco.MjData(model)
    data.qpos[0] = math.radians(30.0)
    data.ctrl[model.actuator("extensor").id] = 1.0
    mujoco.mj_forward(model, data)
    for _ in range(400):
        mujoco.mj_step(model, data)
    assert math.degrees(data.qpos[0]) < 28.0


def test_joint_range_matches_params():
    model = _load()
    lo, hi = model.jnt_range[model.joint("knee").id]
    assert round(math.degrees(lo)) == -15
    assert round(math.degrees(hi)) == 90


def test_max_torque_consistent_with_model():
    """参数里的 max_torque 应等于 单肌力 × wrap 半径（力从模型里读，不写死）。"""
    from mujoco_musculoskeletal_knee_tracking import config_loader

    cfg = config_loader.load(os.path.dirname(HERE))
    model = _load()
    # MuJoCo 的 muscle 执行器把 force 存在 gainprm[2]
    force = float(model.actuator_gainprm[model.actuator("flexor").id][2])
    wrap_r = float(model.geom("knee_wrap").size[0])
    assert force > 0.0
    assert abs(force * wrap_r - cfg["ctrl.max_torque"]) < 1e-6
