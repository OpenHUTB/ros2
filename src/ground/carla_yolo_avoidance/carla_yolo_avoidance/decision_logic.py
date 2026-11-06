# -*- coding: utf-8 -*-
"""避障决策逻辑（纯函数，不依赖 ROS，方便单元测试）。

决策规则（阈值全部来自 config.py，写死在代码里）：

    | 条件                          | 油门    | 刹车        | 状态        |
    |-------------------------------|---------|-------------|-------------|
    | 检测结果超时（YOLO 挂了）      | 0.0     | 1.0(失效安全)| STALE_BRAKE |
    | 视野内没有车辆                 | 0.45    | 0.0         | CRUISE      |
    | 距离 < 8 m                     | 0.0     | 1.0         | FULL_BRAKE  |
    | 8 m <= 距离 < 20 m（安全阈值） | 0.0     | 0.35        | SLOW_DOWN   |
    | 距离 >= 20 m                   | 0.45    | 0.0         | CRUISE      |

全程不做转向控制，steer 恒为 0.0。
"""

from dataclasses import dataclass

from . import config


@dataclass
class Decision:
    """一条控制决策。"""

    throttle: float
    brake: float
    steer: float
    state: str
    reason: str

    def as_list(self):
        """转成 [throttle, brake, steer]，用于发布 Float32MultiArray。"""
        return [self.throttle, self.brake, self.steer]


def is_detection_stale(detection_age_s):
    """检测结果是否已失效。

    参数:
        detection_age_s: 距上一次收到有效检测结果的秒数；None 表示从未收到过。
    """
    if detection_age_s is None:
        return True
    return detection_age_s > config.DETECTION_STALE_TIMEOUT_S


def select_nearest_in_lane(detections, lateral_limit_m=None):
    """从检测列表里挑出"本车道内最近的车辆"。

    参数:
        detections: list[dict]，每项形如
            {'class': 'car', 'distance_m': 12.3, 'lateral_m': -0.4, ...}
            其中 distance_m / lateral_m 可能为 None（距离不可信）
        lateral_limit_m: 横向限制（米），None 时取 config.ROI_LATERAL_LIMIT_M
    返回:
        (min_distance_m, nearest_detection) —— 没有可用目标时返回 (None, None)
    """
    if lateral_limit_m is None:
        lateral_limit_m = config.ROI_LATERAL_LIMIT_M

    nearest = None
    min_distance = None

    for det in detections:
        distance = det.get('distance_m')
        if distance is None:            # 框太小 / 距离不可信
            continue

        lateral = det.get('lateral_m')
        if lateral is not None and abs(lateral) > lateral_limit_m:
            continue                    # 旁边车道的目标，不参与本车避障

        if min_distance is None or distance < min_distance:
            min_distance = distance
            nearest = det

    return min_distance, nearest


def decide(min_distance_m, detection_age_s, has_detection):
    """核心决策函数。

    参数:
        min_distance_m:  本车道内最近车辆的距离（米），无目标传 None
        detection_age_s: 距上一次收到检测结果的秒数（None = 从未收到）
        has_detection:   当前是否有可用的检测数据
    返回:
        Decision
    """
    # 1) 数据过期 -> 失效安全：宁可停住，也不要盲目前进
    if is_detection_stale(detection_age_s):
        if config.BRAKE_WHEN_DETECTION_STALE:
            return Decision(
                throttle=0.0,
                brake=config.FULL_BRAKE,
                steer=0.0,
                state='STALE_BRAKE',
                reason='检测数据超时(>{:.1f}s)，失效安全刹停'.format(
                    config.DETECTION_STALE_TIMEOUT_S
                ),
            )
        return Decision(0.0, 0.0, 0.0, 'STALE_COAST', '检测数据超时，滑行')

    # 2) 没有目标 -> 正常前进
    if not has_detection or min_distance_m is None:
        return Decision(
            throttle=config.CRUISE_THROTTLE,
            brake=0.0,
            steer=0.0,
            state='CRUISE',
            reason='前方无车辆，正常前进',
        )

    distance = float(min_distance_m)

    # 3) 进入急刹区间
    if distance < config.FULL_BRAKE_DISTANCE_M:
        return Decision(
            throttle=0.0,
            brake=config.FULL_BRAKE,
            steer=0.0,
            state='FULL_BRAKE',
            reason='距离 {:.2f} m < {:.2f} m，全力刹车'.format(
                distance, config.FULL_BRAKE_DISTANCE_M
            ),
        )

    # 4) 小于安全阈值 -> 减速刹车
    if distance < config.SAFE_DISTANCE_M:
        return Decision(
            throttle=config.SLOW_DOWN_THROTTLE,
            brake=config.SLOW_DOWN_BRAKE,
            steer=0.0,
            state='SLOW_DOWN',
            reason='距离 {:.2f} m < 安全阈值 {:.2f} m，减速刹车'.format(
                distance, config.SAFE_DISTANCE_M
            ),
        )

    # 5) 安全距离 -> 正常前进
    return Decision(
        throttle=config.CRUISE_THROTTLE,
        brake=0.0,
        steer=0.0,
        state='CRUISE',
        reason='距离 {:.2f} m >= 安全阈值 {:.2f} m，正常前进'.format(
            distance, config.SAFE_DISTANCE_M
        ),
    )
