import numpy as np


class AEBController:
    def __init__(self, ttc_threshold=5.0, gain=1.4, v_stop=0.1):
        # 参数全部外部可配置，和main脚本统一
        self.ttc_threshold = ttc_threshold
        self.gain = gain
        self.v_stop = v_stop

    def get_brake(self, ttc: float, v_ego: float) -> float:
        """
        :param ttc: TTC碰撞时间(s)，无风险一般传入99
        :param v_ego: 自车速度 m/s
        :return: brake 0~1
        """
        # 低速保护，速度过小不制动
        if abs(v_ego) < self.v_stop:
            return 0.0

        # TTC异常/无碰撞风险
        if ttc >= self.ttc_threshold or ttc <= 0:
            return 0.0

        # 比例制动计算，带gain增益，限幅0‑1
        raw_brake = self.gain * (1.0 - ttc / self.ttc_threshold)
        brake = np.clip(raw_brake, 0.0, 1.0)
        return float(brake)
