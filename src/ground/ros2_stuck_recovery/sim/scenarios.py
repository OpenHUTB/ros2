"""可复现场景库。每个场景都对应一种真实会遇到的"静默卡死"。"""
from __future__ import annotations

from .stuck_world import Rect, Scenario, Segment
from ros2_stuck_recovery.robot import Twist

FORWARD = Twist(0.25, 0.0)

SCENARIOS: dict[str, Scenario] = {
    # 正面顶住一面短墙：里程计不动，最典型的一类
    "blocked_wall": Scenario(
        name="blocked_wall",
        title="正面顶住短墙",
        start=(0.0, 0.0, 0.0),
        barriers=[Segment(2.00, -1.20, 2.00, 1.20)],
        task=FORWARD,
        duration=24.0,
    ),
    # 斜插墙角：车头 45° 顶进两面墙的夹角，必须退一步再换方向才能出来
    "corner": Scenario(
        name="corner",
        title="斜插墙角",
        start=(0.55, 0.55, 0.785),
        barriers=[
            Segment(1.00, 0.55, 1.00, 1.00),
            Segment(0.75, 1.00, 1.00, 1.00),
        ],
        task=FORWARD,
        duration=24.0,
    ),
    # 湿滑地面：编码器说在走、车几乎没动，只有"前向距离没变"能抓到
    "slip": Scenario(
        name="slip",
        title="湿滑地面轮胎空转",
        start=(0.15, 0.0, 0.0),
        barriers=[Segment(1.00, -3.00, 1.00, 3.00)],
        fault="slip",
        slip_zone=Rect(0.25, -1.00, 0.45, 1.00),
        fault_until=60.0,
        mu_kinetic=0.12,
        mu_static=0.90,
        task=FORWARD,
        duration=9.0,
    ),
    # 单轮失去抓地：编码器照转、车绕另一侧轮打转，里程计与 IMU 打架
    "one_wheel": Scenario(
        name="one_wheel",
        title="单轮失去抓地（异物卡住 2.5 s）",
        start=(0.0, 0.0, 0.0),
        barriers=[],
        fault="one_wheel",
        fault_until=2.5,
        task=FORWARD,
        duration=16.0,
    ),
    # 永久单轮失效：模块应当有限次尝试后停车上报，而不是无休止挣扎
    "one_wheel_permanent": Scenario(
        name="one_wheel_permanent",
        title="单轮永久失效（应停车上报）",
        start=(0.0, 0.0, 0.0),
        barriers=[],
        fault="one_wheel",
        fault_until=1e9,
        task=FORWARD,
        duration=26.0,
        expect="abort",
    ),
    # 贴着墙平行行驶：测距恒定、里程计正常，最容易把"测距不变"误判成打滑
    "wall_hug": Scenario(
        name="wall_hug",
        title="贴墙平行行驶（误报检查）",
        start=(0.0, 0.55, 0.0),
        barriers=[Segment(0.00, 1.00, 9.00, 1.00)],
        task=FORWARD,
        duration=14.0,
        expect="no_trigger",
    ),
    # 无障碍正常行驶：检查会不会误报
    "free": Scenario(
        name="free",
        title="空旷场地正常行驶（误报检查）",
        start=(0.0, 0.0, 0.0),
        barriers=[],
        task=FORWARD,
        duration=14.0,
        expect="no_trigger",
    ),
}

ESCAPE_SCENARIOS = ["blocked_wall", "corner", "slip", "one_wheel"]
ALL_SCENARIOS = list(SCENARIOS.keys())


def get(name: str) -> Scenario:
    if name not in SCENARIOS:
        raise KeyError(f"未知场景 {name}，可选：{', '.join(SCENARIOS)}")
    return SCENARIOS[name]
