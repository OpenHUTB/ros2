"""车道偏离预警（Lane Departure Warning，LDW）决策核心。

纯离线逻辑：不依赖 ROS 与 CARLA，可独立单元测试。

几何模型：

  1) 带符号横向偏移：自车位置向车道中心线投影——沿道路方向的
     单位向量 dir=(cos y, sin y)，左侧法向 left=(-sin y, cos y)，
     offset = (ego − wp)·left，左偏为正、右偏为负；
  2) 航向偏差：ego_yaw − road_yaw 归一化到 [−π, π]，
     正值表示车头朝左偏，用于趋势参考。

决策（三级滞回状态机，阈值为绝对横向距离）：

  normal ──|o| ≥ warn_enter──▶ deviation ──|o| ≥ depart_enter──▶ departure
  deviation ──|o| < warn_exit──▶ normal
  departure ──|o| < depart_exit──▶ deviation

事件仅在真实跃迁时发出一次：
  deviation_start（带 left/right 方向）、departure_start（带方向）、
  deviation_clear（回 normal）、departure_end（回 deviation）。
"""

from dataclasses import dataclass

import numpy as np

NORMAL = "normal"
DEVIATION = "deviation"
DEPARTURE = "departure"


def normalize_angle(a):
    """角度归一化到 [−π, π]。"""
    return float(math_atan2(np.sin(a), np.cos(a)))


def math_atan2(y, x):
    return float(np.arctan2(y, x))


def lateral_offset(ego_xy, wp_xy, road_yaw):
    """自车相对车道中心线的带符号横向偏移（m），左偏为正。

    ego_xy/wp_xy: (x, y)；road_yaw: 车道中心线在 wp 处的朝向（rad，逆时针）。
    """
    dx = ego_xy[0] - wp_xy[0]
    dy = ego_xy[1] - wp_xy[1]
    return float(-dx * np.sin(road_yaw) + dy * np.cos(road_yaw))


def heading_error(ego_yaw, road_yaw):
    """航向偏差（rad）= ego_yaw − road_yaw，归一化到 [−π, π]，左偏为正。"""
    return normalize_angle(ego_yaw - road_yaw)


def side_of(offset):
    """偏移方向：左偏 'left'，右偏 'right'，居中 ''。"""
    if offset > 0:
        return "left"
    if offset < 0:
        return "right"
    return ""


def reject_jump(prev, cur, max_step=1.5):
    """横向偏移跳变剔除（纯函数）。

    相邻两拍（0.05 s）横向偏移变化超过 max_step 视为地图投影跳变
    （车速 6 m/s 时物理上不可能横移 1.5 m），将其限幅为单步最大变化。
    返回 (accepted, rejected)；首拍直通。
    """
    if prev is None:
        return cur, False
    d = cur - prev
    if abs(d) > max_step:
        return float(prev + math_copysign(max_step, d)), True
    return float(cur), False


def math_copysign(x, y):
    return float(np.copysign(x, y))


@dataclass
class LDWParams:
    """LDW 决策参数（与 config/ldw_params.yaml 一一对应）。"""
    lane_width: float = 3.5   # 车道宽（m），bridge 会按路点动态覆盖
    warn_enter: float = 0.5   # 进入 deviation 的横向偏移（m）
    warn_exit: float = 0.3    # 回 normal 的横向偏移（m，滞回下界）
    depart_enter: float = 0.9 # 进入 departure 的横向偏移（m，接近压线）
    depart_exit: float = 0.65 # 回 deviation 的横向偏移（m，滞回）


def ldw_decide(state, offset, p=None):
    """单步三级滞回决策（纯函数）→ (new_state, event)。

    event 仅在真实跃迁时非空，带方向：
      deviation_start_left / deviation_start_right / departure_start_left /
      departure_start_right / deviation_clear / departure_end
    """
    p = p or LDWParams()
    old = state
    a = abs(offset)
    if state == NORMAL:
        if a >= p.depart_enter:
            state = DEPARTURE
        elif a >= p.warn_enter:
            state = DEVIATION
    elif state == DEVIATION:
        if a >= p.depart_enter:
            state = DEPARTURE
        elif a < p.warn_exit:
            state = NORMAL
    elif state == DEPARTURE:
        if a < p.depart_exit:
            state = DEVIATION
    else:
        raise ValueError(f"非法状态 {state!r}，应为 normal/deviation/departure")

    event = ""
    if state != old:
        side = side_of(offset)
        if state == DEPARTURE:
            event = f"departure_start_{side}"
        elif state == DEVIATION:
            event = ("departure_end" if old == DEPARTURE
                     else f"deviation_start_{side}")
        else:
            event = "deviation_clear"
    return state, event


class LDWController:
    """LDW 有状态封装：三级滞回状态机 + 最近一次事件记忆。

    tick() 每周期喂入带符号横向偏移，返回
    {state, offset, side, event}；连续保持时 event 为空。
    """

    def __init__(self, params=None):
        self.p = params or LDWParams()
        self._state = NORMAL

    def tick(self, offset, dt=0.05):
        state, event = ldw_decide(self._state, offset, self.p)
        self._state = state
        return {
            "state": state,
            "offset": float(offset),
            "side": side_of(offset),
            "event": event,
        }

    def reset(self):
        """复位到 normal。"""
        self._state = NORMAL
