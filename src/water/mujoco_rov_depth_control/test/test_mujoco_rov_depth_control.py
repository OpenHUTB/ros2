"""水下定深控制器的单元测试（不需要 MuJoCo 运行时）。

- `cascade_thrust`：串级控制律的纯函数，验证符号、限幅与前馈
- `schedule`：任务时间轴展开
- 有 MuJoCo 时再补一个 2 秒短仿真回归（无 MuJoCo 环境自动跳过）
"""
import os

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
APP = os.path.join(ROOT, 'mujoco_rov_depth_control', 'app.py')


@pytest.fixture(scope='module')
def ns():
    src = open(APP, encoding='utf-8').read()
    start = src.index('def cascade_thrust(')
    end = src.index('def run(')
    out = {'GRAVITY': 9.81}
    exec(compile(src[start:end], APP, 'exec'), out)  # noqa: S102
    return out


@pytest.fixture(scope='module')
def cfg():
    with open(os.path.join(ROOT, 'config', 'params.yaml'), encoding='utf-8') as fh:
        return yaml.safe_load(fh)


def test_needs_downward_thrust_when_deep_below_target(ns, cfg):
    """目标在下方 5 m 且静止 -> 推力应为负（向下）。"""
    u = ns['cascade_thrust'](-5.0, 0.0, cfg, mass=16.6, net_buoyancy=16.6 * 9.81 * 0.02)
    assert u < 0


def test_needs_ascent_when_deeper_than_target(ns, cfg):
    """err = 目标 - 当前 > 0 表示当前比目标更深 -> 需要上浮（推力为正）。"""
    u = ns['cascade_thrust'](5.0, 0.0, cfg, mass=16.6, net_buoyancy=16.6 * 9.81 * 0.02)
    assert u > 0


def test_thrust_is_limited(ns, cfg):
    limit = float(cfg['model']['max_thrust'])
    # 用大质量放大需求推力，确保打到限幅
    u = ns['cascade_thrust'](-1000.0, 0.0, cfg, mass=100.0, net_buoyancy=0.0)
    assert u == pytest.approx(-limit)


def test_at_target_with_zero_velocity_only_compensates_buoyancy(ns, cfg):
    net = 16.6 * 9.81 * 0.02
    u = ns['cascade_thrust'](0.0, 0.0, cfg, mass=16.6, net_buoyancy=net)
    assert u == pytest.approx(-net, abs=1e-6)


def test_velocity_gain_opposes_motion(ns, cfg):
    """向下运动（vz<0）时应产生向上的修正量。"""
    net = 0.0
    u_still = ns['cascade_thrust'](0.0, 0.0, cfg, mass=16.6, net_buoyancy=net)
    u_moving = ns['cascade_thrust'](0.0, -0.5, cfg, mass=16.6, net_buoyancy=net)
    assert u_moving > u_still


def test_schedule_expands_segments(ns, cfg):
    segs = ns['schedule'](cfg)
    assert len(segs) == len(cfg['mission']['segments'])
    assert segs[0][0] == 0.0
    # 时间连续
    for (t0, t1, _d), (n0, _n1, _nd) in zip(segs, segs[1:]):
        assert n0 == pytest.approx(t1)
    assert segs[-1][1] == pytest.approx(sum(float(s[1]) for s in cfg['mission']['segments']))


def test_control_frequency_matches_docs(cfg):
    dt = float(cfg['control']['dt'])
    every = int(cfg['control']['control_every'])
    assert 1.0 / (dt * every) == pytest.approx(20.0)   # 文档与 summary 统一写 20 Hz


def test_short_simulation_regression(ns, cfg):
    """有 MuJoCo 时跑 2 秒，验证深度确实朝目标方向变化（无 MuJoCo 则跳过）。"""
    try:
        import mujoco
    except ImportError as exc:                       # 未安装，或被系统应用控制策略拦截
        pytest.skip("MuJoCo 不可用: %s" % exc, allow_module_level=False)
    import numpy as np

    src = open(APP, encoding='utf-8').read()
    ns2 = {'np': np}
    start = src.index('def rov_xml(')
    end = src.index('def cascade_thrust(')
    exec(compile(src[start:end], APP, 'exec'), ns2)  # noqa: S102

    model = mujoco.MjModel.from_xml_string(ns2['rov_xml'](cfg))
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'rov')
    mass = float(model.body_mass[body_id])
    buoyancy = mass * 9.81 * float(cfg['model']['buoyancy_ratio'])
    net = buoyancy - mass * 9.81        # 与 run() 一致：net_buoyancy = B - m*g
    dt = float(cfg['control']['dt'])
    z0 = float(data.qpos[2])
    for step in range(int(2.0 / dt)):
        err = -1.0 - float(data.qpos[2])
        u = ns['cascade_thrust'](err, float(data.qvel[2]), cfg, mass, net)
        data.xfrc_applied[body_id, 2] = u + buoyancy   # 与 run() 一致：补浮力，不再减重量
        mujoco.mj_step(model, data)
    assert float(data.qpos[2]) < z0      # 应该已经开始下潜
