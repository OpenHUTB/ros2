"""定点保持制导：欠驱动船在流场里"悬停"的几何解。

先看一个容易被忽略的事实：**在有流的水面上，"停在原地"不等于"不推进"**。
若流速为 ``c``，要让船的世界速度为零，必须满足

    u·dir(psi) + c = 0    =>    u = |c|, psi = 流向的反方向

也就是船必须一直以 |c| 的速度逆流顶着，位置才不动。任何"到了点就关机"的逻辑，
在流里都会立刻被冲走。

把这一条写成通用形式：给定期望世界速度 ``v_des``（由位置误差算出，指向目标），
需要求解

    u·dir(psi) + c = v_des

解就是

    psi_des = atan2( (v_des - c)_y , (v_des - c)_x )
    u_des   = |v_des - c|

- 无流时 ``c = 0``，退化成"船头指向目标、速度正比于距离"；
- 完美保持时 ``v_des = 0``，得到 ``psi_des = 流向的反方向``、``u_des = |c|``，正是顶流悬停；
- 顺流漂向目标时，``v_des - c`` 会自然变小，制导自动减速，不会冲过头。

航向与目标方位之间的夹角就是 crab angle（蟹行角），它在低速、大流时最大，因此
需要一个上限来防止船横向"打横"失速。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .geometry import body_to_world, clamp, wrap_angle


@dataclass(frozen=True)
class GuidanceConfig:
    """制导参数。"""

    position_gain: float = 0.55       # 位置误差 -> 期望速度的增益 1/s
    approach_speed: float = 0.35      # 位置修正的最大前进分量 m/s
    slow_radius: float = 1.20         # 进入该半径后按比例减速 m
    arrive_radius: float = 0.12       # 认为"已在点上"的半径 m
    deadband_radius: float = 0.15     # 死区：之内不再做位置修正，但仍需顶流
    speed_floor: float = 0.035        # 期望船体速度低于该值时不再推进：弱流下的"静止"，避免追着噪声打转
    crab_limit_deg: float = 88.0      # crab 角上限：定点保持时船头接近逆流，偏角天然可达 90°
    command_speed_cap: float = 0.90   # 期望船体速度上限 m/s
    heading_rate_limit_deg: float = 25.0  # 航向指令最大变化率 deg/s（扫描出来的折中值）
    align_floor: float = 0.15           # 航向偏差极大时保留的最小前进比例
    current_deadband: float = 0.15      # 估计流低于该值时视为无流：否则 0.05 的估计残差
                                        # 会让船在静水里长期斜着开（实测航向偏 30°）


@dataclass(frozen=True)
class GuidanceOutput:
    """一个制导周期的输出。"""

    u_cmd: float            # 期望前进速度 m/s（含顶流分量）
    psi_cmd: float          # 期望航向 rad
    distance: float         # 到目标点的距离 m
    bearing: float          # 目标方位角 rad
    crab_deg: float         # 航向相对目标方位的偏置（度，带符号）
    in_deadband: bool       # 位置误差是否在死区内（此时仍在顶流）
    holding: bool           # 是否需要推进器出力（无流且已到位时为 False）
    desired_world_velocity: tuple[float, float] = (0.0, 0.0)


class StationKeepingGuidance:
    """把位置误差与流估计合成为（速度, 航向）指令。"""

    def __init__(self, config: GuidanceConfig | None = None) -> None:
        self.config = config or GuidanceConfig()
        self._psi_cmd = 0.0

    def reset(self, psi: float = 0.0) -> None:
        self._psi_cmd = psi

    def update(
        self,
        x: float,
        y: float,
        psi: float,
        target: tuple[float, float],
        current_body: tuple[float, float] = (0.0, 0.0),
        dt: float = 0.05,
    ) -> GuidanceOutput:
        """计算本周期制导指令。

        ``current_body`` 是船体系下的流速估计（由 ``monitor.FlowEstimator`` 给出），
        控制器不需要事先知道真实流场。
        """
        cfg = self.config
        dx = target[0] - x
        dy = target[1] - y
        distance = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) if distance > 1e-9 else psi

        # 1. 位置误差 -> 期望世界速度（指向目标，靠近时线性减速）
        if distance <= cfg.deadband_radius:
            v_des = (0.0, 0.0)
            in_deadband = True
        else:
            in_deadband = False
            speed = clamp(cfg.position_gain * distance, 0.0, cfg.approach_speed)
            if distance < cfg.slow_radius:
                speed = cfg.approach_speed * (distance / cfg.slow_radius)
            v_des = (speed * math.cos(bearing), speed * math.sin(bearing))

        # 2. 流估计转到世界系；低于死区时按"无流"处理，避免顺着估计残差斜着走
        if math.hypot(current_body[0], current_body[1]) < cfg.current_deadband:
            current_body = (0.0, 0.0)
        c_world = body_to_world(psi, current_body[0], current_body[1])

        # 3. 求解 u·dir(psi) + c = v_des
        need_x = v_des[0] - c_world[0]
        need_y = v_des[1] - c_world[1]
        need_speed = math.hypot(need_x, need_y)

        if need_speed < cfg.speed_floor:
            # 无流、无误差：真正可以停机
            self._psi_cmd = psi
            return GuidanceOutput(
                u_cmd=0.0,
                psi_cmd=psi,
                distance=distance,
                bearing=bearing,
                crab_deg=0.0,
                in_deadband=in_deadband,
                holding=False,
                desired_world_velocity=v_des,
            )

        psi_cmd = math.atan2(need_y, need_x)
        u_cmd = min(need_speed, cfg.command_speed_cap)

        # 4. crab 角限幅：流太强、需要的偏置超过阈值时，牺牲一点位置精度换取可控
        crab = wrap_angle(psi_cmd - bearing)
        limit = math.radians(cfg.crab_limit_deg)
        if abs(crab) > limit:
            crab = math.copysign(limit, crab)
            psi_cmd = wrap_angle(bearing + crab)

        # 5. 航向指令限速。距离很小时，1~2 cm 的定位噪声会让 atan2 算出的方位角
        #    整周乱跳；不限制变化率，船就会在原地打转而不前进。
        max_delta = math.radians(cfg.heading_rate_limit_deg) * max(dt, 1e-3)
        delta = wrap_angle(psi_cmd - self._psi_cmd)
        psi_cmd = wrap_angle(self._psi_cmd + clamp(delta, -max_delta, max_delta))

        # 6. 航向没对准就别全速冲。欠驱动船的转向能力有限，一边大角度转向一边全速
        #    前进，轨迹就是一圈一圈的螺旋——实测正是这个原因导致"绕圈不收敛"。
        #    按航向误差收缩前进分量：差 90° 时几乎停推，先把头转过来。
        align = math.cos(wrap_angle(psi_cmd - psi))
        if align < cfg.align_floor:
            align = cfg.align_floor
        u_cmd *= align

        self._psi_cmd = psi_cmd
        return GuidanceOutput(
            u_cmd=u_cmd,
            psi_cmd=psi_cmd,
            distance=distance,
            bearing=bearing,
            crab_deg=math.degrees(crab),
            in_deadband=in_deadband,
            holding=True,
            desired_world_velocity=v_des,
        )

    @property
    def last_heading_command(self) -> float:
        return self._psi_cmd
