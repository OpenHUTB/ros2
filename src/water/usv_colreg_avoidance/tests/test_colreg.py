"""会遇判定与避让决策的单元测试。

这些测试不依赖 ROS，直接 python -m pytest tests 就能跑。
"""
import math

import pytest

from usv_colreg_avoidance import scenarios, sim
from usv_colreg_avoidance.colreg import (ColregConfig, Role, Situation, Vessel,
                                         classify, cpa, decide, relative_bearing,
                                         required_turn, threat_cleared, wrap180)


def test_wrap180_归一化():
    assert wrap180(370.0) == pytest.approx(10.0)
    assert wrap180(-190.0) == pytest.approx(170.0)
    assert wrap180(180.0) == pytest.approx(-180.0)
    assert wrap180(-180.0) == pytest.approx(-180.0)


def test_相对方位右舷为正():
    own = Vessel('A', 0, 0, heading=0.0, speed=5.0)
    starboard = Vessel('B', 100, 100, heading=270.0, speed=5.0)   # 东北方向 → 右舷
    port = Vessel('C', -100, 100, heading=90.0, speed=5.0)        # 西北方向 → 左舷
    assert relative_bearing(own, starboard) == pytest.approx(45.0)
    assert relative_bearing(own, port) == pytest.approx(-45.0)


def test_cpa_已知几何():
    own = Vessel('A', 0, 0, heading=90.0, speed=10.0)      # 向东
    other = Vessel('B', 1000, 500, heading=270.0, speed=10.0)  # 向西
    dcpa, tcpa = cpa(own, other)
    assert dcpa == pytest.approx(500.0, abs=1e-6)
    assert tcpa == pytest.approx(50.0, abs=1e-6)


def test_对遇判定():
    own = Vessel('A', 0, 0, heading=0.0, speed=9.0)
    other = Vessel('B', 0, 3000, heading=180.0, speed=9.0)
    assert classify(own, other) is Situation.HEAD_ON
    assert classify(other, own) is Situation.HEAD_ON


def test_右舷交叉判定与角色():
    own = Vessel('A', 0, 0, heading=0.0, speed=8.0)
    other = Vessel('B', 600, 600, heading=270.0, speed=8.0)
    assert classify(own, other) is Situation.CROSSING_GIVE_WAY

    decision = decide(own, [other])
    assert decision.role is Role.GIVE_WAY
    assert decision.turn >= 30.0          # Rule 16 要求大幅度


def test_左舷交叉判直航():
    own = Vessel('A', 0, 0, heading=0.0, speed=8.0)
    other = Vessel('B', -600, 600, heading=90.0, speed=8.0)
    assert classify(own, other) is Situation.CROSSING_STAND_ON

    decision = decide(own, [other])
    assert decision.role is Role.STAND_ON
    assert decision.turn == 0.0           # Rule 17(a) 保向保速


def test_追越双方角色互补():
    fast = Vessel('A', 0, 0, heading=0.0, speed=12.0)
    slow = Vessel('B', 0, 800, heading=0.0, speed=5.0)
    assert classify(fast, slow) is Situation.OVERTAKING
    assert classify(slow, fast) is Situation.OVERTAKEN
    assert decide(fast, [slow]).role is Role.GIVE_WAY
    assert decide(slow, [fast]).role is Role.STAND_ON


def test_追越责任不因方位变化解除():
    """Rule 13(d)：追越船擦过去后，被追越船看到的方位变成侧前方，仍应按被追越处理。"""
    slow = Vessel('B', 0, 0, heading=0.0, speed=5.0)
    # 快船已经跑到慢船右侧稍后，方位约 99°，仍比慢船快
    fast = Vessel('A', 260, -100, heading=0.0, speed=12.0)
    assert abs(relative_bearing(slow, fast)) < 112.5
    assert classify(slow, fast) is Situation.OVERTAKEN
    assert decide(slow, [fast]).role is Role.STAND_ON


def test_required_turn_把_DCPA_拉到安全值():
    config = ColregConfig()
    own = Vessel('A', 0, 0, heading=0.0, speed=9.0)
    other = Vessel('B', 0, 3000, heading=180.0, speed=9.0)
    turn = required_turn(own, other, config)
    assert turn > 0.0
    after, _ = cpa(own.moved(heading=own.heading + turn), other)
    assert after >= config.d_safe * (1.0 + config.turn_margin)


def test_威胁解除看的是过正横():
    config = ColregConfig()
    own = Vessel('A', 0, 0, heading=0.0, speed=10.0)
    ahead = Vessel('B', 0, 300, heading=180.0, speed=10.0)
    abeam = Vessel('C', 257, -120, heading=180.0, speed=10.0)   # 已过正横且拉开
    assert not threat_cleared(own, ahead, config)
    assert threat_cleared(own, abeam, config)


@pytest.mark.parametrize('name', ['head_on', 'crossing', 'overtaking', 'multi'])
def test_场景避让后不碰撞(name):
    config = ColregConfig()
    result = sim.run(scenarios.by_name(name), config=config)
    assert not result.metrics['collision']
    assert result.metrics['min_separation'] >= config.d_safe * 0.95


@pytest.mark.parametrize('name', ['head_on', 'crossing', 'overtaking', 'multi'])
def test_不避让的对照基线会碰撞(name):
    config = ColregConfig()
    result = sim.run(scenarios.by_name(name), config=config, avoid=False)
    assert result.metrics['collision']


def test_对遇双方都向右转():
    config = ColregConfig()
    result = sim.run(scenarios.head_on(), config=config)
    for i, initial in enumerate(result.initial):
        # 避让结束后会拐回原航线，所以看过程中的最大右偏角而不是最终航向
        max_right = max(wrap180(frame.ships[i][2] - initial.heading)
                        for frame in result.frames)
        assert max_right > 15.0, '对遇时两船都应当向右转'
        assert max_right < 90.0, '右转幅度不应大到掉头'


def test_直航船全程保向():
    config = ColregConfig()
    result = sim.run(scenarios.crossing(), config=config)
    assert result.metrics['peak_turn'][1] == pytest.approx(0.0, abs=1.0), \
        '交叉局面里直航船不应主动转向'
