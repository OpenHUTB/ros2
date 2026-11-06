# -*- coding: utf-8 -*-
"""ROS 2 节点：跑演示的同时把灯光状态发布成话题。

* 发布 ``/vehicle/lights``（``std_msgs/String``）：当前点亮的灯光，例如 ``近光灯、雾灯``，
  全灭时为 ``无``；
* 参数全部以**字符串**声明，便于 ``ros2 launch`` 传递（与仓库其它模块一致）。
"""

import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carla_auto_light.app import (build_parser, finalize_args, parse_positive_float,
                                  run_demo)                      # noqa: E402
from carla_auto_light.carla_utils import LightSession, require_carla   # noqa: E402
from carla_auto_light.light_logic import state_to_names          # noqa: E402

NODE_NAME = 'auto_light_node'

PARAMETERS = [
    ('host', '127.0.0.1'),
    ('port', '2000'),
    ('town', 'Town03'),
    ('dt', '0.05'),
    ('vehicle', 'vehicle.tesla.model3'),
    ('spawn_index', '-1'),
    ('phase_seconds', '6.0'),
]


class AutoLightNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        for name, default in PARAMETERS:
            self.declare_parameter(name, default)
        self.publisher = self.create_publisher(String, '/vehicle/lights', 10)

    def _string(self, name):
        return str(self.get_parameter(name).value)

    def _publish_state(self, phase, action, state):
        names = state_to_names(state)
        text = '、'.join(names) if names else '无'
        self.publisher.publish(String(data=text))

    def run(self):
        require_carla()
        args = build_parser().parse_args([])          # 与 main.py 一致的默认值
        args = finalize_args(args)                    # 折算 realtime / follow 开关
        args.host = self._string('host')
        args.port = int(float(self._string('port')))
        args.town = self._string('town')
        args.dt = float(self._string('dt'))
        args.vehicle = self._string('vehicle')
        args.spawn_index = int(float(self._string('spawn_index')))
        # 与 main.py 共用同一个正数校验：否则 launch 传 0/负数时会静默跳过整段演示
        args.phase_seconds = parse_positive_float(self._string('phase_seconds'))

        self.get_logger().info('灯光演示开始：%s，每个阶段 %.1f s'
                               % (args.vehicle, args.phase_seconds))
        session = LightSession(args.host, args.port)
        try:
            session.ensure_town(args.town)
            session.enable_sync(args.dt)
            records = run_demo(session, args, on_state=self._publish_state)
        finally:
            session.close()
        self.get_logger().info('演示结束，共 %d 个阶段；灯光状态已发布到 /vehicle/lights'
                               % len(records))
        return records


def main(args=None):
    # stdout 默认是块缓冲，print() 的输出会堆到进程结束才刷出，
    # 于是"演示结束"那行 ROS 日志会跑到所有阶段输出前面。改成行缓冲即可保持真实顺序。
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:                                            # noqa: BLE001
        pass

    rclpy.init(args=args)
    node = None
    try:
        node = AutoLightNode()
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as exc:                                     # noqa: BLE001
        print('[错误] 演示失败：%s' % exc)
        print('       请确认：宿主机 CARLA 已启动、host 参数正确、2000 端口放行。')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
