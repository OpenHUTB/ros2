"""无界面自动验证：超车变道控制律。

场景：前车 x=80m 以 8m/s 匀速慢行，本车 x=0m 以 15m/s 巡航。
验证：超车成功、回原车道、不超速、变道平滑、不碰撞、恢复巡航、状态机完整。
"""

import sys

from vehicle_model import VehicleModel
from overtake_controller import OvertakeController


def run_sim(dt=0.01, duration=26.0):
    host = VehicleModel(x=0.0, y=0.0, v=15.0)
    lead = VehicleModel(x=80.0, y=0.0, v=8.0)
    ctrl = OvertakeController()

    max_v = 0.0
    min_bgap = 1e9          # 超越前（跟车阶段）的最小保险杠间距
    max_lat_step = 0.0
    t = 0.0
    while t < duration:
        a, vy, state, info = ctrl.compute(
            host.x, host.y, host.v, lead.x, lead.v, dt)
        host.update(a, vy, dt)
        lead.update(0.0, 0.0, dt)          # 前车匀速

        b_gap = (lead.x - host.x) - ctrl.car_len
        max_v = max(max_v, host.v)
        if (lead.x - host.x) > 0.0 and state in (ctrl.CRUISE, ctrl.SIGNAL, ctrl.CHANGING):
            min_bgap = min(min_bgap, b_gap)
        max_lat_step = max(max_lat_step, abs(vy) * dt)
        t += dt

    stats = dict(max_v=max_v, min_bgap=min_bgap,
                 max_lat_step=max_lat_step, events=ctrl.events)
    return host, lead, ctrl, stats


def main():
    host, lead, ctrl, s = run_sim()
    evs = s['events']
    ev_names = [e[0] for e in evs]

    checks = [
        ('超车成功：本车超过前车 20m 以上',
         host.x - lead.x > 20.0, host.x - lead.x),
        ('回到原车道：|y| < 0.1',
         abs(host.y) < 0.1, host.y),
        ('全程未超速：v <= 18 m/s',
         s['max_v'] <= ctrl.v_overtake + 1.0, s['max_v']),
        ('变道平滑：无瞬移（相邻帧横向位移受限）',
         s['max_lat_step'] <= ctrl.v_lat_max * 0.01 * 1.2, s['max_lat_step']),
        ('全程不碰撞：超越前保险杠间距 > 0.5m',
         s['min_bgap'] > 0.5, s['min_bgap']),
        ('恢复巡航：|v - 15| < 0.5',
         abs(host.v - ctrl.v_cruise) < 0.5, host.v),
        ('状态机完整走完：signal->changing->overtaking->changing_back->done',
         ev_names == ['signal', 'changing', 'overtaking', 'changing_back', 'done'],
         ev_names),
    ]

    passed = 0
    for name, ok, val in checks:
        text = f"{val:.3f}" if isinstance(val, float) else str(val)
        print(('PASS' if ok else 'FAIL'), '|', name, '->', text)
        passed += int(ok)

    print(f'--- {passed}/{len(checks)} passed ---')
    print(f'状态切换事件: {ev_names}')
    print(f'仿真时长: {26.0}s | 本车终点: x={host.x:.1f}m y={host.y:.3f}m v={host.v:.2f}m/s')
    return 0 if passed == len(checks) else 1


if __name__ == '__main__':
    sys.exit(main())
