"""启动期与里程计中断时的兜底行为（评审意见的回归测试）。

核心约束只有一条：**里程计不可用时绝不能把任务层的速度透传到底盘**。

之前 ``node.py`` 用 ``self._last_odom_t = 0.0`` 表示"还没收到"，再用
``now - last if last > 0 else 0.0`` 算年龄，于是"从未收到"被算成 0 秒——
也就是"数据刚刚更新过"，stale 保护形同虚设。这些用例锁住修正后的行为。
"""
import math

import pytest

from ros2_stuck_recovery.detect import DetectorConfig, StuckDetector
from ros2_stuck_recovery.guard import OdomWatch, evaluate, odom_available
from ros2_stuck_recovery.recovery import STATE_CRUISE, STATE_STALE, RecoverySupervisor
from ros2_stuck_recovery.robot import Twist

DT = 0.05
TASK = Twist(0.25, 0.0)          # 任务层"一直往前走"


def make(stale: float = 0.5):
    return OdomWatch(), StuckDetector(), RecoverySupervisor()


# ---------------------------------------------------------------- 数据年龄语义
def test_watch_reports_inf_before_first_message():
    """从未收到里程计时年龄必须是 inf（不可用），不能是 0（刚更新）。"""
    watch = OdomWatch()
    assert watch.seen is False
    assert math.isinf(watch.age(0.0))
    assert math.isinf(watch.age(1e6))
    assert odom_available(watch.age(1.0), stale_odom=0.5) is False


def test_watch_age_after_message():
    watch = OdomWatch()
    watch.update(3.0)
    assert watch.seen is True
    assert watch.age(3.2) == pytest.approx(0.2)
    assert odom_available(watch.age(3.2), stale_odom=0.5) is True
    # 时钟回退时不应出现负年龄
    assert watch.age(2.0) == pytest.approx(0.0)


# ---------------------------------------------------------------- 启动期 fail closed
def test_no_odom_never_passes_command_through():
    """启动期没有 /odom、任务层却在发 0.25 m/s：必须全程零速。"""
    watch, det, sup = make()
    zone = (0.0, 0.0, 0.0)
    for k in range(40):                       # 2 s，足够跑满确认窗口
        now = k * DT
        out, verdict, age = evaluate(
            now, TASK, watch, det, sup,
            v_odom=0.0, w_odom=0.0, w_imu=0.0, range_front=1.5, pose=zone,
        )
        assert out.v == 0.0 and out.w == 0.0
        assert out.state == STATE_STALE
        assert verdict.stuck is False
        assert math.isinf(age)
    assert sup.triggers == 0                  # 没有凭"没有里程计"就判卡死


def test_no_odom_does_not_poison_detector():
    """里程计恢复的瞬间不能拿黑屏期的旧样本判卡死，否则会无端倒车。"""
    watch, det, sup = make()
    zone = (0.0, 0.0, 0.0)
    t = 0.0
    for _ in range(40):                       # 2 s 无里程计
        evaluate(t, TASK, watch, det, sup, v_odom=0.0, w_odom=0.0,
                 w_imu=0.0, range_front=1.5, pose=zone)
        t += DT

    # 里程计上线，正常行驶：指令与位移一致
    for _ in range(40):
        watch.update(t)
        out, verdict, age = evaluate(
            t, TASK, watch, det, sup,
            v_odom=TASK.v, w_odom=0.0, w_imu=0.0, range_front=2.0, pose=zone,
        )
        assert out.state == STATE_CRUISE
        assert out.v == pytest.approx(TASK.v)
        t += DT
    assert sup.triggers == 0


def test_odom_arrival_resumes_passthrough():
    """里程计到位后应立刻恢复透传，而不是卡在 STALE。"""
    watch, det, sup = make()
    zone = (0.0, 0.0, 0.0)
    evaluate(0.0, TASK, watch, det, sup, v_odom=0.0, w_odom=0.0,
             w_imu=0.0, range_front=1.5, pose=zone)
    assert sup.state == STATE_STALE

    watch.update(DT)
    out, _, _ = evaluate(DT, TASK, watch, det, sup, v_odom=TASK.v, w_odom=0.0,
                         w_imu=0.0, range_front=1.5, pose=zone)
    assert out.state == STATE_CRUISE
    assert out.v == pytest.approx(TASK.v)


# ---------------------------------------------------------------- 运行中中断
def test_odom_timeout_stops_after_running():
    """跑起来之后里程计中断，超过 stale_odom 就停车。"""
    watch, det, sup = make()
    zone = (0.0, 0.0, 0.0)
    watch.update(0.0)
    t = 0.0
    for _ in range(4):                        # 0.2 s 正常
        out, _, _ = evaluate(t, TASK, watch, det, sup, v_odom=TASK.v,
                             w_odom=0.0, w_imu=0.0, range_front=2.0, pose=zone)
        assert out.state == STATE_CRUISE
        t += DT

    for _ in range(20):                       # 之后 1 s 不再收到里程计
        out, _, age = evaluate(t, TASK, watch, det, sup, v_odom=TASK.v,
                               w_odom=0.0, w_imu=0.0, range_front=2.0, pose=zone)
        t += DT
    assert age > 0.5
    assert out.state == STATE_STALE
    assert out.v == 0.0 and out.w == 0.0


def test_stale_does_not_accumulate_starvation_evidence():
    """中断期间"有指令零位移"不算卡死证据：恢复后不能立刻触发脱困。"""
    watch, det, sup = make()
    zone = (0.0, 0.0, 0.0)
    watch.update(0.0)
    t = 0.0
    for _ in range(60):                       # 3 s 中断
        evaluate(t, TASK, watch, det, sup, v_odom=0.0, w_odom=0.0,
                 w_imu=0.0, range_front=1.5, pose=zone)
        t += DT
    assert sup.triggers == 0

    watch.update(t)                           # 恢复，且这次是真的顶住了墙
    for _ in range(40):
        out, verdict, _ = evaluate(t, TASK, watch, det, sup, v_odom=0.0,
                                   w_odom=0.0, w_imu=0.0, range_front=1.5, pose=zone)
        t += DT
        watch.update(t)
    assert sup.triggers >= 1                  # 该报的时候仍然要报
    assert verdict.stuck is True or sup.state != STATE_CRUISE


def test_detector_config_still_honoured():
    """确认接线没把检测器参数丢掉（防止迁移时漏参数）。"""
    det = StuckDetector(DetectorConfig(confirm=0.30))
    assert det.cfg.confirm == pytest.approx(0.30)
