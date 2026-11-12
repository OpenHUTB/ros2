"""离线演示：用脚本模拟两路速度源，打印仲裁结果表。

不依赖 ROS，`ros2 run system_integration main_demo` 或
`python3 -m system_integration.demo` 都能跑。
"""

from __future__ import annotations

from .arbiter import VelocityArbiter

SCENARIO = [
    # (t, 动作, 说明)
    (0.0,  ('mode', 'manual'), '切到手动'),
    (1.0,  ('manual', 0.6, 0.0), '手动给前进 0.6'),
    (2.0,  ('manual', 0.6, 0.0), '手动继续'),
    (3.5,  ('step',), '手动停发 1.5 s，应触发看门狗'),
    (4.0,  ('mode', 'auto'), '切到自动'),
    (4.2,  ('auto', 0.8, 0.2), '自动给出 0.8 / 0.2'),
    (5.0,  ('auto', 3.0, 5.0), '自动给出超限速度，应被限幅'),
    (6.0,  ('estop', True), '触发急停'),
    (7.0,  ('auto', 0.8, 0.0), '急停期间自动仍在发'),
    (8.0,  ('estop', False), '解除急停'),
    (8.1,  ('auto', 0.8, 0.0), '自动补发一帧'),
    (8.2,  ('step',), '恢复后跟随（数据新鲜）'),
    (9.5,  ('step',), '自动再次超时，应回零'),
]


def run(arbiter=None, scenario=None, verbose=True):
    arb = arbiter or VelocityArbiter(timeout=1.0)
    scenario = SCENARIO if scenario is None else scenario
    rows = []

    for t, action, note in scenario:
        kind = action[0]
        if kind == 'mode':
            arb.set_mode(action[1])
        elif kind == 'estop':
            arb.set_estop(action[1], now=t)
        elif kind == 'manual':
            arb.feed_manual(action[1], action[2], now=t)
        elif kind == 'auto':
            arb.feed_auto(action[1], action[2], now=t)

        v, w, st = arb.step(now=t)
        rows.append({'t': t, 'action': note, 'v': v, 'w': w,
                     'source': st['source'], 'reason': st['reason']})
        if verbose:
            print('%5.1f  %-28s  v=%5.2f  w=%5.2f  %-8s %s'
                  % (t, note, v, w, st['source'], st['reason']))

    return rows


def main(argv=None):
    print('=== 速度仲裁演示（看门狗 1.0 s，限幅 ±1.0 m/s / ±1.5 rad/s）===')
    rows = run()
    stale = sum(1 for r in rows if r['reason'] == 'stale')
    estop = sum(1 for r in rows if r['reason'] == 'estop')
    print('\n共 %d 步：看门狗触发 %d 次，急停生效 %d 步' % (len(rows), stale, estop))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
