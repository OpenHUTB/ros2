"""ROS 2 节点：连 CARLA，逐帧出视觉偏移与地图真值，发预警状态。

节点本身不控车，只发布偏移和状态；需要图像显示时另行用 ros2 topic echo。

所有参数都按字符串声明，由 launch 用 value_type=str 传入——这一点和仓库里
其他 CARLA 模块保持一致，避免 launch_ros 猜类型导致节点直接退出。
"""
import json
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from . import carla_common as cc
from . import dataset
from . import lanes
from . import metrics
from . import model as net


class VisualLaneDepartureNode(Node):
    """把 main.py 的演示循环搬进 ROS，边跑边发布。"""

    PARAMETERS = (
        ('host', '127.0.0.1'),
        ('port', '2000'),
        ('town', 'Town03'),
        ('vehicle', 'vehicle.tesla.model3'),
        ('spawn_index', '0'),
        ('checkpoint', 'models/lane_regressor.npz'),
        ('dt', '0.05'),
        ('width', '800'),
        ('height', '600'),
        ('fov', '90.0'),
        ('frames', '0'),
        ('print_every', '20'),
        ('settle', '40'),
        ('min_speed', '1.0'),
        ('max_offset', '1.6'),
    )

    def __init__(self):
        super().__init__('visual_lane_departure_warning')
        for name, default in self.PARAMETERS:
            self.declare_parameter(name, default)

        self.offset_publisher = self.create_publisher(Float32, '~/offset', 10)
        self.truth_publisher = self.create_publisher(Float32, '~/offset_map', 10)
        self.state_publisher = self.create_publisher(String, '~/state', 10)
        self.event_publisher = self.create_publisher(String, '~/event', 10)

        self._stop = threading.Event()
        self._actor = None
        self._camera = None
        self._summary = {}
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.get_logger().info('视觉车道偏离预警节点已启动')

    # ------------------------------------------------------------------

    def _param(self, name):
        return self.get_parameter(name).value

    def _run(self):
        """后台线程：连 CARLA 逐帧推理，退出时清场。"""
        try:
            self._loop()
        except Exception as error:              # 连接失败要报出来，不能静默
            self.get_logger().error('运行中断：%s' % error)
        finally:
            self._stop.set()
            # 循环在后台线程里结束，主线程还停在 rclpy.spin()，
            # 不主动关一次的话，--frames 跑完节点也不会退出
            if rclpy.ok():
                rclpy.shutdown()

    @staticmethod
    def _resolve_checkpoint(path):
        """权重先按给定路径找，找不到再去包的 share 目录找。

        ros2 launch 的工作目录不一定是包源码目录，默认值 models/lane_regressor.npz
        在别处启动时会直接落空，这里补一条兜底路径。
        """
        if os.path.isfile(path):
            return path
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('carla_visual_lane_departure_warning')
            candidate = os.path.join(share, os.path.basename(path)) \
                if os.path.isabs(path) else os.path.join(share, path)
            if os.path.isfile(candidate):
                return candidate
        except Exception:                       # 没装 ament 索引时保持原样
            pass
        return path

    def _loop(self):
        checkpoint = self._resolve_checkpoint(self._param('checkpoint'))
        if not os.path.isfile(checkpoint):
            self.get_logger().error('找不到权重 %s，先跑 main.py --collect 与 --train'
                                    % checkpoint)
            return

        regressor = net.LaneRegressor.load(checkpoint)
        client, world = cc.connect(self._param('host'), int(self._param('port')),
                                   self._param('town') or None)
        dt = float(self._param('dt'))
        cc.enable_sync(world, dt)
        self._actor = cc.spawn_ego(world, client, self._param('vehicle'),
                                   int(self._param('spawn_index')))
        self._camera, frames = cc.attach_camera(
            world, self._actor, int(self._param('width')),
            int(self._param('height')), float(self._param('fov')))

        machine = lanes.LaneDepartureStateMachine()
        speeds = []
        visual_series, map_series = [], []
        total = int(self._param('frames'))
        every = max(1, int(self._param('print_every')))
        settle = int(self._param('settle'))
        min_speed = float(self._param('min_speed'))
        max_offset = float(self._param('max_offset'))
        skipped = 0
        index = 0

        try:
            while rclpy.ok() and not self._stop.is_set():
                if total and index >= total:
                    break
                world.tick()
                image = frames.get(timeout=5.0)
                rgb = cc.image_to_array(image)

                predicted = regressor.predict(
                    net.flatten_images(dataset.resize_gray(rgb)[None]))[0]
                visual_offset = -(predicted[0] + predicted[1]) / 2.0

                transform = self._actor.get_transform()
                waypoint = world.get_map().get_waypoint(transform.location,
                                                        project_to_road=True)
                # 真值取自车到中心线的垂距，不用 10 m 前瞻
                map_offset = dataset.lane_offset(waypoint, transform)

                # 话题每帧都发，统计只算地图给了有效车道的帧：路口那种
                # 真值在两条车道之间跳的帧算进去会把误差拉高，口径也会
                # 与 main.py --demo 对不上
                speed = float(self._actor.get_velocity().length())
                usable = (2.8 < waypoint.lane_width < 4.2
                          and abs(map_offset) <= max_offset
                          and speed >= min_speed
                          and index >= settle)
                if usable:
                    visual_series.append(visual_offset)
                    map_series.append(map_offset)
                    speeds.append(speed)
                else:
                    skipped += 1

                state, event = machine.update(visual_offset)
                self.offset_publisher.publish(Float32(data=float(visual_offset)))
                self.truth_publisher.publish(Float32(data=float(map_offset)))
                self.state_publisher.publish(String(data=state))
                if event:
                    self.event_publisher.publish(String(data=event))
                    self.get_logger().info('事件 %s（视觉偏移 %+.2f m，地图 %+.2f m）'
                                           % (event, visual_offset, map_offset))

                if index % every == 0 and visual_series:
                    self.get_logger().info(
                        '帧 %4d 视觉 %+.2f m 地图 %+.2f m 误差 %+.2f m'
                        % (index, visual_offset, map_offset,
                           visual_offset - map_offset))
                index += 1
        finally:
            # 汇总先发，清理后做：清理阶段偶尔会 C++ abort，日志不能丢
            self._publish_summary(visual_series, map_series, speeds)
            cc.destroy_all(self._camera, self._actor)
            cc.restore_async(world)
            if skipped:
                self.get_logger().info(
                    '统计跳过 %d 帧（预热、车速过低或地图无有效车道）' % skipped)

    def _publish_summary(self, visual_series, map_series, speeds):
        """跑完把误差发到 /statistics，方便录屏时直接看到结论。"""
        if not visual_series:
            return
        result = metrics.evaluate_offsets(np.array(visual_series),
                                          np.array(map_series), speeds)
        self._summary = result
        self.get_logger().info(metrics.format_report(result))


def main(args=None):
    rclpy.init(args=args)
    node = VisualLaneDepartureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # launch 收到 Ctrl-C 时已经关过一次，这里再关会抛
        # RCLError: rcl_shutdown already called
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()