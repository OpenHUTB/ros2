# -*- coding: utf-8 -*-
"""控制器纯逻辑单元测试 (不依赖 ROS2 / MuJoCo)."""
import math

import pytest

from mujoco_musculoskeletal_knee_tracking.controller import (
    clamp,
    pd_to_activation,
)


def test_clamp_bounds():
    assert clamp(0.0) == 0.0
    assert clamp(1.0) == 1.0
    assert clamp(-1.0) == 0.0
    assert clamp(2.0) == 1.0
    assert clamp(0.5) == 0.5


def test_positive_error_activates_flexor_only():
    # 需要屈曲: 目标大于当前 -> 仅屈肌激活
    flex, ext = pd_to_activation(0.5, 0.0, kp=30.0, kd=3.0, max_torque=25.0)
    assert flex > 0.0 and ext == 0.0


def test_negative_error_activates_extensor_only():
    # 需要伸展: 目标小于当前 -> 仅伸肌激活
    flex, ext = pd_to_activation(-0.5, 0.0, kp=30.0, kd=3.0, max_torque=25.0)
    assert ext > 0.0 and flex == 0.0


def test_activation_clamped_to_unit_interval():
    # 大误差不导致激活越界
    flex, ext = pd_to_activation(3.0, 0.0, kp=30.0, kd=3.0, max_torque=25.0)
    assert 0.0 <= flex <= 1.0 and ext == 0.0
    flex, ext = pd_to_activation(-3.0, 0.0, kp=30.0, kd=3.0, max_torque=25.0)
    assert 0.0 <= ext <= 1.0 and flex == 0.0


def test_velocity_damping_reduces_activation():
    # 相同误差下, 正角速度(同向屈曲)会降低所需屈肌激活
    f_static, _ = pd_to_activation(0.5, 0.0, kp=30.0, kd=3.0, max_torque=25.0)
    f_damped, _ = pd_to_activation(0.5, 1.0, kp=30.0, kd=3.0, max_torque=25.0)
    assert f_damped <= f_static


def test_limit_reached_blocks_flexion():
    flex, ext = pd_to_activation(1.0, 0.0, kp=30.0, kd=3.0, max_torque=25.0,
                                 limit_reached=True)
    assert flex == 0.0


def test_non_finite_leads_to_safe_stop():
    flex, ext = pd_to_activation(float("nan"), 0.0, kp=30.0, kd=3.0, max_torque=25.0)
    assert flex == 0.0 and ext == 0.0
    flex, ext = pd_to_activation(float("inf"), 0.0, kp=30.0, kd=3.0, max_torque=25.0)
    assert flex == 0.0 and ext == 0.0
