# -*- coding: utf-8 -*-
"""
奖励函数模块：reward（任务二/四：端到端强化学习的奖励塑形）

把"沿车道平稳向前开"这件事拆成可微、可加权的几项物理量。智能体（策略网络）
只看到单目前视图像，奖励由仿真侧可观测量（车速、航向偏差、横向偏离、碰撞、
是否越界）计算，定义在本模块。

计算原理（奖励塑形公式）：

    r_t  =  w_v * v_forward                       # 鼓励前进
          - w_c * |c_err|                         # 惩罚横向偏离车道中心
          - w_h * heading_err                     # 惩罚车头指向与车道切线的夹角
          - w_s * |steer_t|^2                     # 惩罚频繁打方向（平滑性）
          - w_a * |acceleration|                  # 惩罚急加速/急刹
          - r_collision * 1[collision]            # 发生碰撞的大负惩罚
          - r_offtrack  * 1[off_lane]             # 驶出车道的大负惩罚

其中：
  * v_forward   车体纵向速度 (m/s)，只取前进分量，倒车不计奖励；
  * c_err       车体相对车道中心线的横向偏差 (m)；
  * heading_err 车头航向与车道切线方向的夹角 (rad)，取绝对值；
  * steer_t     本步施加的前轮转角 (-1 ~ 1 归一化)；
  * 1[·]       示性函数，条件成立取 1，否则 0。

权重为经验先验，默认值见 RewardWeights；可在 config/rl_driving.yaml 中覆盖。
"""

from collections import namedtuple


# 权重容器：命名元组，默认值对应"温和、安全、稳开"的驾驶偏好
RewardWeights = namedtuple(
    'RewardWeights',
    ['w_forward', 'w_centre', 'w_heading', 'w_steer', 'w_accel',
     'collision', 'offtrack', 'min_speed_bonus'],
)

DEFAULT_WEIGHTS = RewardWeights(
    w_forward=1.0,      # 前进速度项权重
    w_centre=0.15,      # 横向偏离惩罚
    w_heading=0.20,     # 航向偏差惩罚
    w_steer=0.05,       # 转角平滑惩罚
    w_accel=0.05,       # 加加速度/急动惩罚
    collision=10.0,     # 碰撞大惩罚（回合通常立刻终止）
    offtrack=5.0,       # 越界惩罚
    min_speed_bonus=0.5,  # 维持在目标速度附近的小正奖励
)


def compute_reward(velocity_forward, c_err, heading_err,
                   steer_cmd, accel_cmd, collision, offtrack,
                   target_speed=5.0, weights=DEFAULT_WEIGHTS):
    """按上面的奖励公式计算单步标量奖励。

    参数均为仿真侧（CARLA ros-bridge）可获取的量：
      velocity_forward : float  车体纵向速度 m/s（向后为负）
      c_err            : float   车道中心横向偏差 m
      heading_err      : float   航向夹角 rad
      steer_cmd        : float   本步归一化转角 (-1~1)
      accel_cmd        : float   本步归一化加减速 (-1~1)
      collision        : bool    是否发生碰撞
      offtrack         : bool    是否驶出车道
      target_speed     : float   期望巡航速度 m/s
    返回：(reward: float, info: dict)
    """
    w = weights

    r_forward = w.w_forward * max(velocity_forward, 0.0)
    r_centre = -w.w_centre * abs(c_err)
    r_heading = -w.w_heading * abs(heading_err)
    r_steer = -w.w_steer * (steer_cmd ** 2)
    r_accel = -w.w_accel * abs(accel_cmd)

    # 速度维持项：离目标速度越近奖励越高，用高斯形核
    speed_err = velocity_forward - target_speed
    r_speed = w.min_speed_bonus * max(0.0, 1.0 - (speed_err / target_speed) ** 2)

    r = r_forward + r_centre + r_heading + r_steer + r_accel + r_speed

    done = False
    if collision:
        r -= w.collision
        done = True
    if offtrack:
        r -= w.offtrack
        # 越界不直接终止，给智能体一次"打回来"的机会

    info = {
        'r_forward': r_forward, 'r_centre': r_centre, 'r_heading': r_heading,
        'r_steer': r_steer, 'r_accel': r_accel, 'r_speed': r_speed,
        'collision': collision, 'offtrack': offtrack,
    }
    return float(r), done, info
