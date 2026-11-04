# -*- coding: utf-8 -*-
"""车辆灯光决策逻辑（纯函数 + 一个状态机，不依赖 CARLA）。

拆出来的原因：**灯光逻辑本身是可以脱离仿真器验证的**——给定太阳高度角、降水强度、
方向盘与刹车量，灯光该是什么状态完全是确定的。这里把决策写纯，CARLA 那边只负责
"读状态 → 调这里 → 调 ``set_light_state``"，于是核心逻辑能用单元测试锁住，
不必每改一次都连仿真器。

涉及的三类灯：

* **自动大灯**：太阳高度角低于阈值（黄昏/夜间）或在下雨/起雾时打开近光灯；
* **雾灯**：降水强度高时与近光灯一起打开；
* **转向灯**：方向盘超过阈值时打开，**回正后自动熄灭**（真实汽车的方向盘回正自熄灭，
  由 :class:`BlinkerStateMachine` 实现）；
* **刹车灯**：刹车量超过阈值时点亮。
"""

# ---------------- 阈值（放在模块顶部，便于文档引用与集中调参） ----------------
SUN_HEADLIGHT_ON = 5.0      # 太阳高度角低于 5° 判为黄昏，开近光灯
SUN_NIGHT = -5.0            # ≤ -5° 判为夜间（仅用于日志分级，决策与上一条一致）
PRECIPITATION_FOG = 30.0    # 降水强度 ≥ 30 时同时开雾灯
STEER_BLINKER_ON = 0.15     # 方向盘绝对值 ≥ 0.15 打转向灯
STEER_BLINKER_OFF = 0.05    # 方向盘回到 0.05 以内（含）转向灯自动熄灭
BRAKE_LIGHT_ON = 0.05       # 刹车量 ≥ 0.05 点亮刹车灯

LIGHT_NAMES = ('low_beam', 'fog', 'left_blinker', 'right_blinker', 'brake_light')


def _as_float(value, name):
    """把输入统一成 ``float``；非数值（含 ``None``、非数字字符串）一律抛 ``ValueError``。

    统一异常类型，避免调用方要同时处理 ``TypeError`` 与 ``ValueError``。
    """
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError('%s 必须是数值，收到 %r' % (name, value)) from exc


def sun_phase(sun_altitude):
    """太阳高度角 → 中文时段描述（只用于日志与文档，不参与决策）。"""
    altitude = _as_float(sun_altitude, 'sun_altitude')
    if altitude > SUN_HEADLIGHT_ON:
        return '白天'
    if altitude > SUN_NIGHT:
        return '黄昏/黎明'
    return '夜间'


def decide_lights(sun_altitude, precipitation, steer, brake):
    """根据环境与驾驶输入给出**应点亮**的灯光（不含转向灯自锁）。

    ``steer`` 与 ``brake`` 都是 CARLA 的归一化控制量（刹车 0~1，方向盘 -1~1）。
    ``steer`` 传**驾驶员指令**，不要传车道保持算出的微调量：后者只是保持车道的修正，
    不代表转向意图，拿它判断转向灯会在弯道里乱闪。

    返回 ``dict``，键为 :data:`LIGHT_NAMES` 中的项，另附 ``reason`` 便于日志排查。
    """
    sun = _as_float(sun_altitude, 'sun_altitude')
    precip = _as_float(precipitation, 'precipitation')
    steer = _as_float(steer, 'steer')
    brake = _as_float(brake, 'brake')

    dark = sun <= SUN_HEADLIGHT_ON
    rainy = precip >= PRECIPITATION_FOG

    reasons = []
    if dark:
        reasons.append('太阳高度角 %.1f° ≤ %.1f°' % (sun, SUN_HEADLIGHT_ON))
    if rainy:
        reasons.append('降水 %.0f ≥ %.0f' % (precip, PRECIPITATION_FOG))

    return {
        'low_beam': dark or rainy,
        'fog': rainy,
        'left_blinker': steer <= -STEER_BLINKER_ON,
        'right_blinker': steer >= STEER_BLINKER_ON,
        'brake_light': brake >= BRAKE_LIGHT_ON,
        'reason': '；'.join(reasons) if reasons else '白天、无降水：仅按驾驶输入点灯',
    }


class BlinkerStateMachine:
    """转向灯状态机：打方向点亮，方向盘回正后自动熄灭。

    真实汽车的方向盘下方有回位机构：打了转向灯后必须把方向盘**打够角度再回正**才会
    自动熄灭。这里用两个阈值复现这一点：

    * 方向盘绝对值 ≥ :data:`STEER_BLINKER_ON` → 点亮对应方向（并"锁存"）；
    * 锁存后方向盘**回到** :data:`STEER_BLINKER_OFF` 以内（含） → 自动熄灭；
    * 两侧都达到阈值（理论上不会发生）时保持上一状态，避免来回闪。

    左右灯互斥：确定一侧时另一侧必然熄灭。
    """

    def __init__(self):
        self.active = None          # 'left' / 'right' / None
        self.latched = False        # 是否已经"打过方向"（用于回正熄灭）

    def update(self, steer):
        """输入方向盘量，返回 ``('left'|'right'|None, 是否发生了变化)``。"""
        steer = _as_float(steer, 'steer')
        previous = self.active

        left_cmd = steer <= -STEER_BLINKER_ON
        right_cmd = steer >= STEER_BLINKER_ON

        if left_cmd and not right_cmd:
            if self.active != 'left':
                self.active = 'left'
                self.latched = True
        elif right_cmd and not left_cmd:
            if self.active != 'right':
                self.active = 'right'
                self.latched = True
        elif self.latched and abs(steer) <= STEER_BLINKER_OFF:
            # 已经打过方向，方向盘回到中位 → 自动熄灭
            self.active = None
            self.latched = False

        return self.active, self.active != previous

    def reset(self):
        self.active = None
        self.latched = False


def compose_state(decision, blinker_state):
    """把"环境/驾驶决策"与"转向灯状态机输出"合成最终灯光状态。

    ``blinker_state`` 为 ``'left'`` / ``'right'`` / ``None``，优先级高于
    :func:`decide_lights` 里按方向盘瞬时值给出的结果（两者通常一致，
    差异出现在"方向盘刚回正、转向灯还没熄"的那几帧）。
    """
    state = {name: bool(decision.get(name, False)) for name in LIGHT_NAMES}
    if blinker_state == 'left':
        state['left_blinker'], state['right_blinker'] = True, False
    elif blinker_state == 'right':
        state['left_blinker'], state['right_blinker'] = False, True
    return state


def state_to_names(state):
    """把状态字典转成中文名列表，用于日志打印（空列表表示全部熄灭）。"""
    labels = {
        'low_beam': '近光灯',
        'fog': '雾灯',
        'left_blinker': '左转向灯',
        'right_blinker': '右转向灯',
        'brake_light': '刹车灯',
    }
    return [labels[key] for key in LIGHT_NAMES if state.get(key)]
