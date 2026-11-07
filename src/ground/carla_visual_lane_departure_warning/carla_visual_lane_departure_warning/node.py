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
        predictions, truths, speeds = [], [], []
        total = int(self._param('frames'))
        every = max(1, int(self._param('print_every')))
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
                truth = dataset.boundary_targets(waypoint, transform)
                map_offset = -(truth[0] + truth[1]) / 2.0

                predictions.append(predicted)
                truths.append(truth)
                speeds.append(float(self._actor.get_velocity().length()))

                state, event = machine.update(visual_offset)
                self.offset_publisher.publish(Float32(data=float(visual_offset)))
                self.truth_publisher.publish(Float32(data=float(map_offset)))
                self.state_publisher.publish(String(data=state))
                if event:
                    self.event_publisher.publish(String(data=event))
                    self.get_logger().info('事件 %s（视觉偏移 %+.2f m，地图 %+.2f m）'
                                           % (event, visual_offset, map_offset))

                if index % every == 0 and predictions:
                    self.get_logger().info(
                        '帧 %4d 视觉 %+.2f m 地图 %+.2f m 误差 %+.2f m'
                        % (index, visual_offset, map_offset,
                           visual_offset - map_offset))
                index += 1
        finally:
            cc.destroy_all(self._camera, self._actor)
            cc.restore_async(world)
            self._publish_summary(predictions, truths, speeds)

    def _publish_summary(self, predictions, truths, speeds):
        """跑完把误差发到 /statistics，方便录屏时直接看到结论。"""
        if not predictions:
            return
        result = metrics.evaluate(np.array(predictions), np.array(truths), speeds)
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()