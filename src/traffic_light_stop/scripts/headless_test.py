"""红绿灯启停无界面验证脚本。

跑完一个完整灯周期（红→黄→绿），检查：
1. 红灯期间车辆在停止线前停稳，不越线；
2. 变绿后车辆自动起步并通过停止线；
3. 全程速度不超过巡航速度。

用法：
    python headless_test.py
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vehicle_model import VehicleModel
from traffic_light import TrafficLight
from stop_controller import StopController

V_CRUISE = 10.0
STOP_X = 50.0
SAFETY = 1.0
DT = 0.05
PHASES = [(4.0, 'green'), (1.0, 'yellow'), (9.0, 'red'), (12.0, 'green')]
DURATION = 25.0


def run():
    model = VehicleModel(x=0.0, v=0.0)
    light = TrafficLight(PHASES)
    ctrl = StopController(v_cruise=V_CRUISE, stop_x=STOP_X,
                          safety=SAFETY, a_max=3.0, a_acc=2.0)

    t = 0.0
    min_gap_to_stop = 1e9        # 红灯期间车头距停止线的最小距离（>0 即未越线）
    stop_x_at_red = None         # 红灯期间停稳时的车头位置
    passed_time = None           # 通过停止线的时间
    max_speed = 0.0
    red_start, red_end = 5.0, 14.0

    n = int(DURATION / DT)
    for _ in range(n):
        color = light.update(DT)
        a = ctrl.compute(model.head(), model.v, color)
        model.step(a)
        t += DT
        max_speed = max(max_speed, model.v)

        if red_start <= t <= red_end:
            gap = STOP_X - model.head()
            min_gap_to_stop = min(min_gap_to_stop, gap)
            if model.v < 0.05 and stop_x_at_red is None:
                stop_x_at_red = model.head()
        if passed_time is None and model.head() > STOP_X:
            passed_time = t

    print("=== 红绿灯启停 headless 验证 ===")
    print("红灯期间车头距停止线最小距离: %.2f m" % min_gap_to_stop)
    print("红灯停稳位置(车头): %.2f m (停止线 %g m)" %
          (stop_x_at_red if stop_x_at_red is not None else float('nan'), STOP_X))
    print("绿灯后通过停止线时间: %s" %
          ("%.1f s" % passed_time if passed_time else "未通过"))
    print("全程最大速度: %.2f m/s (巡航 %g m/s)" % (max_speed, V_CRUISE))

    ok = True
    if min_gap_to_stop <= 0:
        print("[FAIL] 红灯期间越过了停止线")
        ok = False
    if stop_x_at_red is None:
        print("[FAIL] 红灯期间没有停稳")
        ok = False
    elif not (0.3 <= (STOP_X - stop_x_at_red) <= 2.5):
        print("[WARN] 停稳位置距停止线 %.2f m，不在 0.3~2.5 m 的理想区间"
              % (STOP_X - stop_x_at_red))
    if passed_time is None or passed_time < red_end:
        print("[FAIL] 绿灯后未通过停止线")
        ok = False
    if max_speed > V_CRUISE + 0.5:
        print("[FAIL] 超速")
        ok = False
    print("结果: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(run())
