"""闯红灯预警（Red Light Warning，RLW）决策核心。

纯离线逻辑：不依赖 ROS 与 CARLA，可独立单元测试。

判定模型：

  1) 判停距离：舒适减速刹停所需距离 d_stop = v² / (2·a_comfort)；
  2) 三级滞回状态机（normal / approaching / violation_risk）：

       normal ──黄/红灯且 d<warn_enter──▶ approaching
       normal/approaching ──红灯且 d<d_stop──▶ violation_risk
       approaching ──绿灯或 d>warn_exit──▶ normal
       violation_risk ──绿灯──▶ normal
       violation_risk ──红灯且 d>brake_exit──▶ approaching

     语义：黄灯只提示接近（approaching），不判"将闯线"——现实中
     黄灯越线不违规；violation_risk 仅在红灯且舒适刹停不可行时触发。
  3) 事件仅在真实跃迁时发出一次：approach_start / violation_start /
     violation_clear / approach_clear。
"""

from dataclasses import dataclass

NORMAL = "normal"
APPROACHING = "approaching"
VIOLATION_RISK = "violation_risk"

VALID_LIGHTS = ("red", "yellow", "green", "off")


@dataclass
class RLWParams:
    """RLW 决策参数（与 config/rlw_params.yaml 一一对应）。"""
    a_comfort: float = 2.5    # 舒适减速上限（m/s²）
    warn_enter: float = 30.0  # 黄/红灯进入 approaching 的停止线距离（m）
    warn_exit: float = 35.0   # 回 normal 的停止线距离（m，滞回上界）
    brake_margin: float = 1.2 # violation 退出滞回系数（d > d_stop×margin 才回退）


def stopping_distance(speed, a_comfort=2.5):
    """舒适减速刹停距离（m）= v²/(2a)，速度非负。"""
    if speed < 0:
        raise ValueError(f"速度须非负，收到 {speed}")
    return float(speed) ** 2 / (2.0 * float(a_comfort))


def rlw_decide(state, light_state, dist, speed, p=None):
    """单步三级滞回决策（纯函数）→ (new_state, event)。

    light_state: red / yellow / green / off（off=前方无受控灯）
    dist: 自车到停止线距离（m，无灯时可设较大值）
    event 仅在真实跃迁时非空。
    """
    if light_state not in VALID_LIGHTS:
        raise ValueError(f"非法灯态 {light_state!r}，应为 {VALID_LIGHTS}")
    p = p or RLWParams()
    old = state
    d_stop = stopping_distance(speed, p.a_comfort)
    brake_exit = d_stop * p.brake_margin
    controlled = light_state in ("red", "yellow")

    if state == NORMAL:
        if light_state == "red" and dist < d_stop:
            state = VIOLATION_RISK
        elif controlled and dist < p.warn_enter:
            state = APPROACHING
    elif state == APPROACHING:
        if light_state == "red" and dist < d_stop:
            state = VIOLATION_RISK
        elif not controlled or dist > p.warn_exit:
            state = NORMAL
    elif state == VIOLATION_RISK:
        if light_state == "green" or light_state == "off":
            state = NORMAL
        elif light_state == "red" and dist > brake_exit:
            state = APPROACHING
    else:
        raise ValueError(f"非法状态 {state!r}，应为 normal/approaching/violation_risk")

    event = ""
    if state != old:
        table = {
            (NORMAL, APPROACHING): "approach_start",
            (NORMAL, VIOLATION_RISK): "violation_start",
            (APPROACHING, VIOLATION_RISK): "violation_start",
            (APPROACHING, NORMAL): "approach_clear",
            (VIOLATION_RISK, NORMAL): "violation_clear",
            (VIOLATION_RISK, APPROACHING): "violation_clear",
        }
        event = table[(old, state)]
    return state, event


class RLWController:
    """RLW 有状态封装：三级滞回状态机 + 热身期 + 最近事件记忆。

    tick() 每周期喂入（灯态、停止线距离、车速），返回
    {state, light, dist, speed, event}；连续保持时 event 为空。

    warmup_ticks：热身拍数（默认 2）。CARLA 生成后前几拍交通灯
    感知未稳定（可能瞬时取到 None→off 或错误灯态），热身期内不输出
    决策（保持 normal、无事件），与 LDW 模块的冷启动抑制一致。
    """

    def __init__(self, params=None, warmup_ticks=2):
        self.p = params or RLWParams()
        self._state = NORMAL
        self._ticks = 0
        self.warmup_ticks = int(warmup_ticks)

    def tick(self, light_state, dist, speed, dt=0.05):
        if light_state not in VALID_LIGHTS:
            raise ValueError(f"非法灯态 {light_state!r}，应为 {VALID_LIGHTS}")
        self._ticks += 1
        state, event = (NORMAL, "")
        if self._ticks > self.warmup_ticks:
            state, event = rlw_decide(self._state, light_state, dist, speed, self.p)
        self._state = state
        return {
            "state": state,
            "light": light_state,
            "dist": float(dist),
            "speed": float(speed),
            "event": event,
        }

    def reset(self):
        """复位到 normal 并重新热身。"""
        self._state = NORMAL
        self._ticks = 0
