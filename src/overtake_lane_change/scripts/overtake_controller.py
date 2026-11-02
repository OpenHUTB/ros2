"""超车变道控制器。

状态机：
    CRUISE(巡航) -> SIGNAL(打左转灯) -> CHANGING(向左变道)
        -> OVERTAKING(加速超越) -> CHANGING_BACK(向右变回) -> CRUISE(恢复巡航)

纵向：速度比例控制，限幅加减速；打灯/变道期间跟住前车保持间距。
横向：比例控制横向速度，限幅，平滑完成车道变换。

车距统一按保险杠间距（中心距 - 车长）计算。
"""

import math


class OvertakeController:
    CRUISE = 'CRUISE'              # 巡航
    SIGNAL = 'SIGNAL'              # 打左转灯（等待变道）
    CHANGING = 'CHANGING'          # 向左变道（进入超车道）
    OVERTAKING = 'OVERTAKING'      # 加速超越前车
    CHANGING_BACK = 'CHANGING_BACK'  # 向右变回原车道

    def __init__(self, lane_width=3.5, v_cruise=15.0, v_overtake=17.0,
                 trigger_dist=40.0, signal_time=1.0, overtake_margin=15.0,
                 a_max=2.5, d_max=4.0, v_lat_max=2.0, k_lat=1.5, k_v=1.0,
                 car_len=4.5, safety_gap=1.0):
        self.lane_width = lane_width      # 车道宽度（m）
        self.v_cruise = v_cruise          # 巡航速度（m/s）
        self.v_overtake = v_overtake      # 超车速度（m/s）
        self.trigger_dist = trigger_dist  # 触发超车的保险杠间距（m）
        self.signal_time = signal_time    # 打灯时长（s）
        self.overtake_margin = overtake_margin  # 超越前车后回道的间距（m）
        self.a_max = a_max                # 最大加速度（m/s^2）
        self.d_max = d_max                # 最大减速度（m/s^2）
        self.v_lat_max = v_lat_max        # 最大横向速度（m/s）
        self.k_lat = k_lat                # 横向比例系数
        self.k_v = k_v                    # 纵向速度比例系数
        self.car_len = car_len            # 车长（m）
        self.safety_gap = safety_gap      # 最小安全保险杠间距（m）

        self.state = self.CRUISE
        self.target_lane = 0              # 0=原车道，1=超车道
        self.signal_timer = 0.0
        self.vy = 0.0
        self.events = []                  # 状态切换事件（供验证/统计）

    def reset(self):
        self.state = self.CRUISE
        self.target_lane = 0
        self.signal_timer = 0.0
        self.vy = 0.0
        self.events = []

    # ---- 纵向控制 ----
    def _speed_ctrl(self, v_host, v_target):
        """速度比例控制，输出限幅加速度。"""
        a = self.k_v * (v_target - v_host)
        return max(-self.d_max, min(self.a_max, a))

    def _follow(self, v_host, v_lead):
        """跟车：目标速度=前车速度（保持当前间距不缩小）。"""
        return self._speed_ctrl(v_host, v_lead)

    # ---- 横向控制 ----
    def _lat_ctrl(self, y_host, y_target):
        """横向比例控制，输出限幅横向速度。"""
        vy = self.k_lat * (y_target - y_host)
        return max(-self.v_lat_max, min(self.v_lat_max, vy))

    # ---- 主入口 ----
    def compute(self, x_host, y_host, v_host, x_lead, v_lead, dt):
        """每步输入本车与前车状态，输出 (加速度, 横向速度, 状态, 信息)。

        前车在前方：x_lead > x_host。
        """
        gap = x_lead - x_host              # 中心距（m）
        b_gap = gap - self.car_len         # 保险杠间距（m）
        a = 0.0
        vy = 0.0
        y_target = 0.0

        if self.state == self.CRUISE:
            y_target = 0.0
            # 仅当前车仍在本车前方（gap>0）且间距过近时才触发超车；
            # 超车完成后本车已在前车前方（gap<0），不再重复触发。
            if gap > 0.0 and b_gap < self.trigger_dist:
                self.state = self.SIGNAL
                self.signal_timer = 0.0
                self.target_lane = 1
                self.events.append(('signal', x_host))
                a = self._follow(v_host, v_lead)
            else:
                a = self._speed_ctrl(v_host, self.v_cruise)

        elif self.state == self.SIGNAL:
            y_target = 0.0
            self.signal_timer += dt
            a = self._follow(v_host, v_lead)   # 打灯期间跟住前车
            if self.signal_timer >= self.signal_time:
                self.state = self.CHANGING
                self.events.append(('changing', x_host))

        elif self.state == self.CHANGING:
            y_target = self.lane_width
            a = self._follow(v_host, v_lead)   # 变道期间仍跟住前车
            vy = self._lat_ctrl(y_host, y_target)
            if abs(y_host - y_target) < 0.08:
                y_host = y_target
                vy = 0.0
                self.state = self.OVERTAKING
                self.events.append(('overtaking', x_host))

        elif self.state == self.OVERTAKING:
            y_target = self.lane_width
            a = self._speed_ctrl(v_host, self.v_overtake)
            if gap < -self.overtake_margin:    # 已超过前车 margin 米
                self.state = self.CHANGING_BACK
                self.target_lane = 0
                self.events.append(('changing_back', x_host))

        elif self.state == self.CHANGING_BACK:
            y_target = 0.0
            a = self._speed_ctrl(v_host, self.v_overtake)
            vy = self._lat_ctrl(y_host, y_target)
            if abs(y_host - y_target) < 0.08:
                y_host = y_target
                vy = 0.0
                self.state = self.CRUISE
                self.events.append(('done', x_host))

        # 安全兜底：仅在超车前的跟车阶段（本车在前车正后方）生效。
        # 超越阶段两车会并排/交错，超车完成后本车已在前车前方（gap<0），均不触发。
        if self.state in (self.CRUISE, self.SIGNAL, self.CHANGING) and gap > 0.0:
            if b_gap < self.safety_gap + 1.0 and a > -self.d_max:
                a = -self.d_max

        a = max(-self.d_max, min(self.a_max, a))
        vy = max(-self.v_lat_max, min(self.v_lat_max, vy))
        self.vy = vy

        info = {
            'state': self.state,
            'y_target': y_target,
            'b_gap': b_gap,
            'signal_on': self.state in (self.SIGNAL, self.CHANGING),
            'signal_right': self.state == self.CHANGING_BACK,
            'lane': 1 if y_host > self.lane_width * 0.5 else 0,
        }
        return a, vy, self.state, info
