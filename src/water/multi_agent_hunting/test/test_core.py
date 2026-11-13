"""multi_agent_hunting 核心逻辑的单元测试。"""
import numpy as np
import pytest

from multi_agent_hunting.core import (
    AcousticModem,
    AuctionAllocator,
    AUVAgent,
    HuntingScenario,
    PIDController,
    Task,
    captured,
    closing_ratio_at,
    intercept_point,
)


def _config(**overrides):
    config = {
        'seed': 1,
        'dt': 0.1,
        'max_steps': 200,
        'auction_interval_steps': 50,
        'agents': [{'name': 'AUV_1'}, {'name': 'AUV_2'}],
        'tasks': [
            {'name': 'Left', 'position': [-5.0, 0.0, 0.0], 'priority': 2},
            {'name': 'Right', 'position': [5.0, 0.0, 0.0], 'priority': 1},
        ],
        'target_initial_pos': [0.0, 0.0, 0.0],
        'target_velocity': [0.0, 0.0, 0.0],
        'comm_delay_ms': 0,
        'packet_loss': 0.0,
        'energy_weight': 5.0,
        'closing_ratio': 0.35,
        'closing_time': 20.0,
        'capture_radius': 0.5,
        'idle_drain_rate': 0.0,
        'move_drain_rate': 0.1,
        'start_radius': 3.0,
        'world_bounds': 50.0,
        'verbose': False,
    }
    config.update(overrides)
    return config


# ----------------------------- 水声信道 -----------------------------
def test_message_waits_for_delay():
    modem = AcousticModem(delay_ms=200, packet_loss=0.0)

    assert modem.broadcast('AUV_1', {'task': 'Left'}, now=0.0) is True
    assert modem.poll(0.1) == []
    assert modem.pending() == 1

    ready = modem.poll(0.2)
    assert len(ready) == 1
    assert ready[0].sender == 'AUV_1'
    assert modem.pending() == 0


def test_zero_delay_delivers_immediately():
    modem = AcousticModem(delay_ms=0, packet_loss=0.0)
    modem.broadcast('AUV_1', {'task': 'Left'}, now=1.0)

    assert len(modem.poll(1.0)) == 1


def test_full_packet_loss_drops_everything():
    modem = AcousticModem(delay_ms=0, packet_loss=1.0)

    assert modem.broadcast('AUV_1', {'task': 'Left'}, now=0.0) is False
    assert modem.sent == 1
    assert modem.dropped == 1
    assert modem.poll(10.0) == []


@pytest.mark.parametrize('loss', [-0.1, 1.5])
def test_invalid_packet_loss_rejected(loss):
    with pytest.raises(ValueError):
        AcousticModem(packet_loss=loss)


def test_invalid_delay_rejected():
    with pytest.raises(ValueError):
        AcousticModem(delay_ms=-1.0)


# ----------------------------- 拍卖 -----------------------------
def test_auction_prefers_nearer_agent():
    allocator = AuctionAllocator(energy_weight=0.0)
    agents = {
        'near': {'position': np.array([0.0, 0.0, 0.0]), 'energy': 50.0},
        'far': {'position': np.array([9.0, 0.0, 0.0]), 'energy': 50.0},
    }
    tasks = [Task('A', np.array([1.0, 0.0, 0.0]))]

    assert allocator.allocate(agents, tasks) == {'near': tasks[0]}


def test_energy_weight_changes_assignment():
    agents = {
        'near_empty': {'position': np.array([0.0, 0.0, 0.0]),
                       'energy': 0.0},
        'far_full': {'position': np.array([10.0, 0.0, 0.0]),
                     'energy': 100.0},
    }
    tasks = [Task('A', np.array([1.0, 0.0, 0.0]))]

    greedy = AuctionAllocator(energy_weight=0.0).allocate(agents, tasks)
    energy_aware = AuctionAllocator(energy_weight=50.0).allocate(
        agents, tasks)

    assert list(greedy) == ['near_empty']
    assert list(energy_aware) == ['far_full']


def test_each_task_assigned_to_one_agent():
    allocator = AuctionAllocator(energy_weight=0.0)
    agents = {
        'a': {'position': np.array([0.0, 0.0, 0.0]), 'energy': 100.0},
        'b': {'position': np.array([1.0, 0.0, 0.0]), 'energy': 100.0},
    }
    tasks = [
        Task('high', np.array([0.0, 0.0, 0.0]), priority=2),
        Task('low', np.array([1.0, 0.0, 0.0]), priority=1),
        Task('extra', np.array([2.0, 0.0, 0.0]), priority=1),
    ]

    assignments = allocator.allocate(agents, tasks)

    assert len(assignments) == 2
    assert len({task.name for task in assignments.values()}) == 2
    assert assignments['a'].name == 'high'


# ----------------------------- 包抄与捕获 -----------------------------
@pytest.mark.parametrize('ratio, expected', [
    (0.0, [-5.0, 0.0, 0.0]),
    (0.5, [-2.5, 0.0, 0.0]),
    (1.0, [0.0, 0.0, 0.0]),
])
def test_intercept_point_shrinks_toward_target(ratio, expected):
    point = intercept_point(np.array([-5.0, 0.0, 0.0]),
                            np.array([0.0, 0.0, 0.0]), ratio)
    assert np.allclose(point, expected)


@pytest.mark.parametrize('ratio, expected', [
    (2.0, [0.0, 0.0, 0.0]),
    (-1.0, [-5.0, 0.0, 0.0]),
])
def test_intercept_point_clamps_ratio(ratio, expected):
    point = intercept_point(np.array([-5.0, 0.0, 0.0]),
                            np.array([0.0, 0.0, 0.0]), ratio)
    assert np.allclose(point, expected)


def test_captured_when_all_inside():
    positions = [np.array([0.1, 0.0, 0.0]), np.array([-0.2, 0.0, 0.0])]
    assert captured(positions, np.zeros(3), 0.5) is True


def test_not_captured_when_one_outside():
    positions = [np.array([0.1, 0.0, 0.0]), np.array([0.9, 0.0, 0.0])]
    assert captured(positions, np.zeros(3), 0.5) is False


def test_empty_positions_not_captured():
    assert captured([], np.zeros(3), 0.5) is False


@pytest.mark.parametrize('elapsed, ratio', [
    (0.0, 0.35),
    (10.0, 0.85),
    (100.0, 1.0),
])
def test_closing_ratio_grows_over_time(elapsed, ratio):
    assert closing_ratio_at(elapsed, 0.35, 20.0) == pytest.approx(ratio)


def test_closing_ratio_time_zero_closes_immediately():
    assert closing_ratio_at(0.0, 0.35, 0.0) == 1.0


# ----------------------------- 电量 -----------------------------
def test_energy_drain_scales_with_command():
    weak = AUVAgent('weak', controller=PIDController())
    strong = AUVAgent('strong', controller=PIDController())

    weak.update_energy(1.0, np.array([0.1, 0.0, 0.0]), 0.0, 1.0)
    strong.update_energy(1.0, np.array([1.0, 0.0, 0.0]), 0.0, 1.0)

    assert strong.energy < weak.energy


def test_energy_never_negative():
    auv = AUVAgent('a', controller=PIDController())
    auv.update_energy(1e6, np.array([1.0, 1.0, 1.0]), 0.0, 1.0)

    assert auv.energy == 0.0


def test_position_reads_dry_run_key():
    auv = AUVAgent('a', controller=PIDController())
    auv.state = {'Location': [0.5, 1.0, -2.0]}

    assert np.allclose(auv.position(), [0.5, 1.0, -2.0])


def test_position_reads_holoocean_sensor_key():
    """HoloOcean 的观测按传感器类型做键，位置在 LocationSensor 里。"""
    auv = AUVAgent('a', controller=PIDController())
    auv.state = {'LocationSensor': [1.0, 2.0, 3.0]}

    assert np.allclose(auv.position(), [1.0, 2.0, 3.0])


# ----------------------------- 复现性 -----------------------------
def test_same_seed_same_start_positions():
    first = HuntingScenario(_config(seed=7))
    second = HuntingScenario(_config(seed=7))
    first.reset_start_positions()
    second.reset_start_positions()

    for name in first.auvs:
        assert np.allclose(first.auvs[name].position(),
                           second.auvs[name].position())


def test_different_seed_different_positions():
    first = HuntingScenario(_config(seed=7))
    second = HuntingScenario(_config(seed=8))
    first.reset_start_positions()
    second.reset_start_positions()

    assert not np.allclose(first.auvs['AUV_1'].position(),
                           second.auvs['AUV_1'].position())


# ----------------------------- 端到端 -----------------------------
def test_end_to_end_without_packet_loss():
    scenario = HuntingScenario(_config(packet_loss=0.0, comm_delay_ms=0))

    result = scenario.run(max_steps=20)

    assert set(result['assignments']) == {'AUV_1', 'AUV_2'}
    assert result['dropped'] == 0


def test_full_packet_loss_freezes_agents():
    """全丢包时收不到任务，AUV 不应移动——证明丢包真正生效。"""
    scenario = HuntingScenario(_config(packet_loss=1.0))
    scenario.reset_start_positions()
    before = {name: auv.position() for name, auv in scenario.auvs.items()}

    result = scenario.run(max_steps=20)

    assert result['assignments'] == {}
    for name, auv in scenario.auvs.items():
        assert np.allclose(auv.position(), before[name])


def test_delay_postpones_assignment():
    """延迟大于仿真时长时，AUV 在时限内收不到任务。"""
    scenario = HuntingScenario(_config(comm_delay_ms=5000))

    result = scenario.run(max_steps=10)

    assert result['assignments'] == {}


def test_capture_reached_after_closing():
    """收缩完成后三艘 AUV 都能进入捕获半径。"""
    scenario = HuntingScenario(_config(closing_time=2.0,
                                       capture_radius=1.5))

    result = scenario.run(max_steps=400)

    assert result['captured'] is True
    assert result['steps'] < 400
