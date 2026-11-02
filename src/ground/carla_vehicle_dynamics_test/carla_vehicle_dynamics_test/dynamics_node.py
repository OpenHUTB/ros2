# -*- coding: utf-8 -*-
"""ROS 2 节点：运行动力性测试并把结果发布成话题。

节点启动后**一次性**执行所选测试（同步模式下的测量循环会占用主线程，因此不做
多线程调度），完成后：

* 发布 ``std_msgs/String`` 到 ``/dynamics/result``（JSON 文本，便于下游记录）；
* 用 ROS 日志打印对比表；
* 退出，由 ``ros2 launch`` 回收进程。

参数全部以**字符串**声明（与仓库其它模块一致，便于 ``ros2 launch`` 传递）。
"""

import json
import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carla_vehicle_dynamics_test.app import (                     # noqa: E402
    DEFAULT_VEHICLES, build_parser, print_table, run_one_vehicle, save_csv)
from carla_vehicle_dynamics_test.carla_utils import CarlaSession, require_carla  # noqa: E402

NODE_NAME = 'dynamics_test_node'

# 默认值全部从 app.build_parser() 取，避免节点参数与命令行参数"各写一份"后走样
# （曾经就出现过节点里 steer 写 0.3、app.py 里写 0.05 的不一致）。
_DEFAULTS = build_parser().parse_args([])

PARAMETERS = [
    ('host', _DEFAULTS.host),
    ('port', str(_DEFAULTS.port)),
    ('town', _DEFAULTS.town),
    ('mode', _DEFAULTS.mode),
    ('vehicle', _DEFAULTS.vehicle),
    ('vehicles', '' if _DEFAULTS.vehicles is None else str(_DEFAULTS.vehicles)),
    ('v_target_kmh', str(_DEFAULTS.v_target_kmh)),
    ('steer', str(_DEFAULTS.steer)),
    ('turn_speed', str(_DEFAULTS.turn_speed)),
    ('turn_time', str(_DEFAULTS.turn_time)),
    ('max_time', str(_DEFAULTS.max_time)),
    ('timeout', str(_DEFAULTS.timeout)),
    ('retries', str(_DEFAULTS.retries)),
    ('log_every', str(_DEFAULTS.log_every)),
    ('out_dir', _DEFAULTS.out_dir),
]


class DynamicsTestNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        for name, default in PARAMETERS:
            self.declare_parameter(name, default)
        self.publisher = self.create_publisher(String, '/dynamics/result', 10)

    def _string(self, name):
        return str(self.get_parameter(name).value)

    def run_tests(self):
        """执行测试并把结果发布出去，返回结果列表。"""
        require_carla()
        args = build_parser().parse_args([])          # 取与 main.py 完全一致的默认值
        args.host = self._string('host')
        args.port = int(float(self._string('port')))
        args.town = self._string('town')
        args.mode = self._string('mode')
        args.vehicle = self._string('vehicle')
        vehicles_arg = self._string('vehicles').strip()
        args.vehicles = vehicles_arg or None
        args.v_target_kmh = float(self._string('v_target_kmh'))
        args.steer = float(self._string('steer'))
        args.turn_speed = float(self._string('turn_speed'))
        args.turn_time = float(self._string('turn_time'))
        args.max_time = float(self._string('max_time'))
        args.timeout = float(self._string('timeout'))
        args.retries = int(float(self._string('retries')))
        args.log_every = float(self._string('log_every'))
        args.out_dir = self._string('out_dir')

        vehicles = DEFAULT_VEHICLES if args.vehicles == 'all' else [args.vehicle]
        self.get_logger().info('动力性测试开始：模式 %s，车型 %s，目标车速 %.0f km/h'
                               % (args.mode, ', '.join(vehicles), args.v_target_kmh))

        session = CarlaSession(args.host, args.port, timeout=args.timeout)
        results = []
        try:
            session.ensure_town(args.town)
            session.enable_sync(args.dt)
            for blueprint in vehicles:
                results.append(run_one_vehicle(session, blueprint, args))
        finally:
            session.close()

        if results:
            print_table(results, args)
            os.makedirs(args.out_dir, exist_ok=True)
            save_csv(results, os.path.join(args.out_dir, 'dynamics_results.csv'))
            summary = [{k: v for k, v in r.items()
                        if not k.endswith('_samples') and not k.endswith('_points')}
                       for r in results]
            self.publisher.publish(String(data=json.dumps(summary, ensure_ascii=False)))
            self.get_logger().info('结果已发布到 /dynamics/result（%d 个车型）' % len(summary))
        return results


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DynamicsTestNode()
        node.run_tests()
    except KeyboardInterrupt:
        pass
    except Exception as exc:                                     # noqa: BLE001
        print('[错误] 测试失败：%s' % exc)
        print('       请确认：宿主机 CARLA 已启动、host 参数正确、防火墙已放行 2000 端口。')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
