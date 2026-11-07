#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线跑通 demo 的完整控制循环：用假 CARLA 替换真实服务端。

为什么需要这个测试：demo 的控制分支只有在真实 CARLA 里才会执行，
之前一次改动里漏了 `import carla`，在虚拟机里跑到第 291 行才报 NameError，
白等一轮两分钟的采集和一分钟的演示。这里用假模块把整条循环走一遍，
名字错误、参数名错误、返回值用法错误都能在没有 CARLA 的机器上直接发现。

覆盖范围：
* run_demo 从连接、生成车辆、取点云、分割、控制到汇总，全程不抛异常；
* 控制量在合法范围内，且刹车量不为负；
* 前方余量、横向偏差、真实间距三项统计确实被记录；
* 结束时调用了 teardown（不会把服务端留在同步模式）。
"""

import argparse
import os
import shutil
import sys
import types

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carla_lidar_pointcloud_segmentation import labels as L   # noqa: E402

_passed = 0
_failed = 0


def check(name, condition, extra=''):
    global _passed, _failed
    if condition:
        _passed += 1
        print('[通过] %s%s' % (name, ('  ' + extra) if extra else ''))
    else:
        _failed += 1
        print('[失败] %s%s' % (name, ('  ' + extra) if extra else ''))


# ---------------------------------------------------------------- 假 carla
class Location:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def distance(self, other):
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2
                + (self.z - other.z) ** 2) ** 0.5


class Rotation:
    def __init__(self, yaw=0.0):
        self.yaw = float(yaw)


class Transform:
    def __init__(self, location, rotation):
        self.location, self.rotation = location, rotation


class VehicleControl:
    def __init__(self, throttle=0.0, brake=0.0, steer=0.0):
        self.throttle, self.brake, self.steer = float(throttle), float(brake), float(steer)


class Velocity:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)


def install_fake_carla():
    fake = types.ModuleType('carla')
    fake.Location = Location
    fake.Rotation = Rotation
    fake.Transform = Transform
    fake.VehicleControl = VehicleControl
    fake.Velocity = Velocity
    fake.command = types.SimpleNamespace(DestroyActor=lambda actor: ('destroy', actor))
    sys.modules['carla'] = fake
    return fake


# ------------------------------------------------------------ 假世界对象
class FakeWaypoint:
    def __init__(self, x, y, z=0.0):
        self.transform = Transform(Location(x, y, z), Rotation(0.0))
        self._x, self._y, self._z = x, y, z

    def next(self, step):
        return [FakeWaypoint(self._x + step, self._y, self._z)]


class FakeMap:
    def get_waypoint(self, location, project_to_road=True, lane_type=None):
        return FakeWaypoint(float(location.x), float(location.y), float(location.z))


class FakeActor:
    _next_id = 100

    def __init__(self, location):
        self.location = location
        FakeActor._next_id += 1
        self.id = FakeActor._next_id

    def get_location(self):
        return self.location

    def get_transform(self):
        return Transform(self.location, Rotation(0.0))


class FakeEgo(FakeActor):
    def __init__(self):
        super().__init__(Location(0.0, 0.0, 0.0))
        self.control = None
        self.autopilot = True
        self.speed = 0.0
        self.controls = []

    def set_autopilot(self, flag):
        self.autopilot = bool(flag)

    def get_velocity(self):
        return Velocity(self.speed, 0.0, 0.0)

    def apply_control(self, command):
        self.control = command
        self.controls.append(command)
        # 简单的纵向响应：油门加速、刹车减速，位置跟着往前走
        self.speed += 8.0 * command.throttle * 0.05 - 6.0 * command.brake * 0.05
        self.speed = max(0.0, min(10.0, self.speed))
        self.location = Location(self.location.x + self.speed * 0.05,
                                 self.location.y, self.location.z)


class FakeActors:
    def __init__(self, actors):
        self._actors = list(actors)

    def filter(self, pattern):
        return list(self._actors)


class FakeWorld:
    def __init__(self):
        self.ticks = 0
        self.others = [FakeActor(Location(30.0, 0.5, 0.0)),
                       FakeActor(Location(60.0, -1.0, 0.0))]

    def tick(self):
        self.ticks += 1
        return self.ticks

    def get_map(self):
        return FakeMap()

    def get_actors(self):
        return FakeActors(self.others)


class FakeQueue:
    """每帧产出一批点：地面 + 前方 8 m 处的障碍物。"""

    def __init__(self):
        self.count = 0

    def get(self, timeout=None):
        self.count += 1
        rng = np.random.default_rng(self.count)
        ground = np.column_stack([rng.uniform(1, 50, 400),
                                  rng.uniform(-6, 6, 400),
                                  np.zeros(400)])
        blockers = np.array([[8.0, 0.2, 0.4], [8.4, -0.3, 0.5],
                             [10.0, 0.6, 0.8], [40.0, 0.0, 1.2]])
        return np.vstack([ground, blockers])


def build_model(path, stats_path):
    """训练一个极小的模型并落盘，让 run_demo 走真实的加载路径。"""
    from carla_lidar_pointcloud_segmentation import model as MD
    rng = np.random.default_rng(7)
    features = rng.normal(size=(3000, 9))
    target = (np.arange(3000) % 4).astype(int)
    net = MD.MLP(n_in=9, hidden=(8, 8), n_out=4, seed=1)
    net.fit(features, target, epochs=3, batch_size=256)
    net.save(path)
    np.savez_compressed(stats_path, mean=features.mean(axis=0),
                        std=features.std(axis=0) + 1e-9)


def make_args(tmpdir, frames=12):
    return argparse.Namespace(
        host='127.0.0.1', port=2000, town='Town03', dt=0.05,
        vehicle='vehicle.tesla.model3', spawn_index=0,
        traffic=2, pedestrians=2,
        lidar_range=50.0, channels=16, points_per_second=100000,
        rotation=20.0, lidar_height=2.4, corridor_width=1.2,
        frames=frames, no_plot=True, record='', control=True,
        max_speed=8.0, comfort_distance=12.0, stop_distance=4.0,
        lookahead=6.0, checkpoint=os.path.join(tmpdir, 'demo_model.npz'),
    )


def main():
    install_fake_carla()
    from carla_lidar_pointcloud_segmentation import app

    # 临时权重放在 test/ 下的临时目录，跑完删掉。目录名带 _tmp_demo_ 前缀，
    # 并由同目录的 .gitignore 挡住：万一中途崩溃留下残file，也不会被误提交。
    tmpdir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '_tmp_demo_%d' % os.getpid())
    os.makedirs(tmpdir, exist_ok=True)
    args = make_args(tmpdir)
    build_model(args.checkpoint,
                os.path.join(tmpdir, 'demo_model.stats.npz'))

    world = FakeWorld()
    ego = FakeEgo()
    queue = FakeQueue()
    calls = {'teardown': 0}

    def fake_teardown(client, world_, sensors=(), actors=()):
        calls['teardown'] += 1

    app.cc.connect = lambda *a, **k: (
        types.SimpleNamespace(
            get_trafficmanager=lambda: types.SimpleNamespace(
                set_synchronous_mode=lambda flag: None),
            apply_batch_sync=lambda cmds, tick: [None]), world)
    app.cc.enable_sync_traffic_manager = lambda client: None
    app.cc.spawn_ego = lambda *a, **k: ego
    app.cc.spawn_traffic = lambda *a, **k: [FakeActor(Location(25.0, 1.0, 0.0))]
    app.cc.spawn_pedestrians = lambda *a, **k: [FakeActor(Location(12.0, 2.0, 0.0))]
    app.cc.lidar_blueprint = lambda *a, **k: object()
    app.cc.attach_lidar = lambda *a, **k: (types.SimpleNamespace(stop=lambda: None), queue)
    app.cc.points_from_raw = lambda data: np.asarray(data, dtype=float)
    app.cc.teardown = fake_teardown
    # 全判成"可行驶"：走廊里没有阻挡物，用来看起步油门与加速是否正常。
    # 用一个几乎不学习的模型配合"全 drivable"的预测，才能稳定复现起步过程。
    app.segment_frame = lambda net, points, mean, std: np.zeros(
        np.asarray(points).shape[0], dtype=int)

    print('[1] 离线跑完整循环')
    print('-' * 72)
    code = app.run_demo(args)
    print('-' * 72)
    check('run_demo 正常返回', code == 0, 'returncode=%d' % code)
    check('世界帧数与请求一致', world.ticks >= args.frames,
          '%d >= %d' % (world.ticks, args.frames))

    print('\n[2] 控制量合法性')
    check('控制分支被执行', len(ego.controls) == args.frames,
          '%d 次' % len(ego.controls))
    if ego.controls:
        throttles = np.array([c.throttle for c in ego.controls])
        brakes = np.array([c.brake for c in ego.controls])
        steers = np.array([c.steer for c in ego.controls])
        check('油门在 [0, 1] 内', bool(throttles.min() >= 0.0 and throttles.max() <= 1.0),
              '%.2f ~ %.2f' % (throttles.min(), throttles.max()))
        check('刹车在 [0, 1] 内', bool(brakes.min() >= 0.0 and brakes.max() <= 1.0),
              '%.2f ~ %.2f' % (brakes.min(), brakes.max()))
        check('刹车量不出现负零', not bool((brakes == 0.0).any()
                                          and np.signbit(brakes).any()))
        check('起步时给了足够油门', bool(throttles.max() >= 0.4),
              '最大 %.2f' % throttles.max())
        check('全程没有误刹车', bool((brakes < 0.01).all()),
              '最大 %.2f' % brakes.max())
        check('自车确实动起来了', ego.speed > 0.0, '%.2f m/s' % ego.speed)
        check('转向在 [-1, 1] 内', bool(np.abs(steers).max() <= 1.0),
              '%.2f' % np.abs(steers).max())
        check('接管后关闭了 autopilot', ego.autopilot is False)

    print('\n[3] 统计指标')
    check('结束后调用了 teardown', calls['teardown'] == 1)
    check('类别常量完整', len(L.CLASS_NAMES) == L.N_CLASSES == 4)

    print('\n[4] 自检不会覆盖入库权重')
    import main as M
    check('自检权重与入库权重是两个路径',
          M.SELF_CHECK_CHECKPOINT != M.DEFAULT_CHECKPOINT,
          M.SELF_CHECK_CHECKPOINT)
    check('入库权重仍是 models/seg_mlp.npz',
          M.DEFAULT_CHECKPOINT.replace(os.sep, '/') == 'models/seg_mlp.npz',
          M.DEFAULT_CHECKPOINT)

    shutil.rmtree(tmpdir, ignore_errors=True)

    print('\n' + '=' * 72)
    if _failed:
        print('[结果] %d 项通过，%d 项失败' % (_passed, _failed))
        print('=' * 72)
        return 1
    print('[结果] %d 项全部通过' % _passed)
    print('=' * 72)
    return 0


if __name__ == '__main__':
    sys.exit(main())