"""ROS 2 节点：点云语义分割 + 沿参考轨迹的避障控制。

节点自己连 CARLA（不依赖 ros-bridge），每帧做三件事：

1. 从车顶激光雷达取一帧普通点云；
2. 用训练好的网络逐点分类，得到"可行驶面 / 车辆 / 行人 / 障碍物"；
3. 在车身走廊里找最近的阻挡物，换算成速度系数，配合纯跟踪算出的转角，
   直接给自车下发控制指令。

发布的话题：

==============================  ==========================  ==================================
话题                            类型                        内容
==============================  ==========================  ==================================
``/lidar/segmented``            sensor_msgs/PointCloud2     带类别强度的点云（强度=类别编号）
``/lidar/free_distance``        std_msgs/Float32            走廊内最近阻挡物距离，米
``/lidar/drivable_ratio``       std_msgs/Float32            前方可行驶点比例，0~1
``/vehicle/control``            std_msgs/Float32MultiArray  实际下发的 [油门, 刹车, 转角]
==============================  ==========================  ==================================

所有参数都以**字符串**声明，和仓库里其它模块一致，launch 里配套用
``ParameterValue(..., value_type=str)`` 传参。
"""

import math
import sys

import numpy as np

from . import carla_common as cc
from . import control
from . import features as F
from . import labels as L
from . import model as MD


def _declare(node, rclpy_mod, name, default):
    """统一按字符串声明参数，避免与 launch 的类型推断打架。"""
    from rclpy.parameter import Parameter
    if not node.has_parameter(name):
        node.declare_parameter(name, Parameter.Type.STRING)
    node.set_parameters([Parameter(name, Parameter.Type.STRING, str(default))])
    return default


def build_path_ahead(carla, world, ego, count=20, spacing=2.0):
    """沿当前车道向前取一串世界坐标点，作为给定参考轨迹。"""
    waypoint = world.get_map().get_waypoint(ego.get_location())
    path = []
    for _ in range(count):
        forward = waypoint.transform.get_forward_vector()
        x = waypoint.transform.location.x + forward.x * spacing
        y = waypoint.transform.location.y + forward.y * spacing
        path.append((x, y))
        nexts = waypoint.next(spacing)
        if not nexts:
            break
        waypoint = nexts[0]
    return path


class LidarSegmentationNode:
    """把 CARLA 连接、推理、控制串起来的节点主体。

    继承 ``rclpy.node.Node`` 的写法在模块导入时才需要 rclpy，
    这里用组合而不是继承，保证没装 ROS 的机器也能 import 本文件做静态检查。
    """

    def __init__(self, rclpy_mod, node_cls, checkpoint, host, port, town,
                 vehicle, spawn_index, traffic, corridor_width, max_speed=8.0):
        self.rclpy = rclpy_mod
        self.node = node_cls('lidar_segmentation_node')
        log = self.node.get_logger()

        self.net = MD.MLP.load(checkpoint)
        stats_path = checkpoint.replace('.npz', '.stats.npz')
        try:
            with np.load(stats_path) as data:
                self.mean, self.std = data['mean'], data['std']
        except Exception:      # noqa: BLE001
            log.warn('缺少标准化统计量 %s，使用零均值单位方差' % stats_path)
            self.mean = np.zeros(self.net.n_in)
            self.std = np.ones(self.net.n_in)

        from std_msgs.msg import Float32, Float32MultiArray
        from sensor_msgs.msg import PointCloud2

        self.pub_cloud = self.node.create_publisher(PointCloud2, '/lidar/segmented', 10)
        self.pub_free = self.node.create_publisher(Float32, '/lidar/free_distance', 10)
        self.pub_ratio = self.node.create_publisher(Float32, '/lidar/drivable_ratio', 10)
        self.pub_ctrl = self.node.create_publisher(Float32MultiArray, '/vehicle/control', 10)

        self.client, self.world = cc.connect(host, port, town, sync=True)
        self.ego = cc.spawn_ego(self.world, vehicle, spawn_index, autopilot=False)
        self.traffic = []
        if traffic > 0:
            self.traffic = cc.spawn_traffic(self.world, self.client, count=traffic)

        common = dict(lidar_range=60.0, channels=32, points_per_second=600000,
                      rotation_frequency=20.0, sensor_tick=0.0)
        self.sensor, self.queue = cc.attach_lidar(
            self.world, self.ego, cc.lidar_blueprint(self.world, semantic=False, **common))
        self.corridor_width = corridor_width
        self.max_speed = max_speed
        for _ in range(5):
            self.world.tick()

        self.timer = self.node.create_timer(0.05, self.step)
        log.info('节点已启动：地图 %s，走廊半宽 %.2f m' % (town, corridor_width))

    # ------------------------------------------------------------------ 主循环
    def step(self):
        """定时器回调：推进一帧、推理、发布、下发控制。"""
        import carla
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import Float32, Float32MultiArray

        self.world.tick()
        try:
            data = self.queue.get(timeout=2.0)
        except Exception:      # noqa: BLE001
            return

        points = cc.points_from_raw(data)
        feats = F.point_features(points, ground_z=F.estimate_ground_z(points))
        classes = self.net.predict(F.standardize_apply(feats, self.mean, self.std))

        free = control.free_distance(points, classes, half_width=self.corridor_width)
        ratio = control.drivable_ratio(classes, points=points)

        self.pub_cloud.publish(_to_cloud2(points, classes, PointCloud2, PointField))
        self.pub_free.publish(Float32(data=float(free)))
        self.pub_ratio.publish(Float32(data=float(ratio)))

        # 速度：距离 -> 系数 -> 目标车速 -> 简单 P 控制
        factor = control.speed_factor(free, comfort=14.0, stop=5.0)
        velocity = self.ego.get_velocity()
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        target = self.max_speed * factor
        error = target - speed
        throttle = float(np.clip(0.5 + 0.15 * error, 0.0, 1.0)) if error > 0 else 0.0
        brake = float(np.clip(-0.2 * error, 0.0, 1.0)) if error < 0 else 0.0

        # 转角：纯跟踪跟当前车道
        transform = self.ego.get_transform()
        yaw = math.radians(transform.rotation.yaw)
        pose = (transform.location.x, transform.location.y, yaw)
        path = build_path_ahead(carla, self.world, self.ego)
        delta = control.pure_pursuit(pose, path, wheelbase=2.9, lookahead=8.0)
        steer = control.normalized_steer(delta, max_steer=0.6)

        self.ego.apply_control(carla.VehicleControl(throttle=throttle, brake=brake,
                                                    steer=steer))
        self.pub_ctrl.publish(Float32MultiArray(data=[throttle, brake, steer]))

    def destroy(self):
        """退出前清理：传感器、车辆、异步模式。"""
        try:
            self.sensor.stop()
            self.sensor.destroy()
        except Exception:      # noqa: BLE001
            pass
        for actor in self.traffic + [self.ego]:
            try:
                actor.destroy()
            except Exception:      # noqa: BLE001
                pass
        cc.restore_async(self.world)
        self.node.destroy_node()


def _to_cloud2(points, classes, cloud_cls, field_cls):
    """把点与类别打包成 PointCloud2，强度字段放类别编号。"""
    cloud = cloud_cls()
    cloud.header.frame_id = 'lidar'
    cloud.height = 1
    cloud.width = int(points.shape[0])
    cloud.is_dense = False
    cloud.is_bigendian = False
    cloud.fields = [
        field_cls(name='x', offset=0, datatype=field_cls.FLOAT32, count=1),
        field_cls(name='y', offset=4, datatype=field_cls.FLOAT32, count=1),
        field_cls(name='z', offset=8, datatype=field_cls.FLOAT32, count=1),
        field_cls(name='intensity', offset=12, datatype=field_cls.FLOAT32, count=1),
    ]
    cloud.point_step = 16
    cloud.row_step = cloud.point_step * cloud.width
    packed = np.zeros((points.shape[0], 4), dtype=np.float32)
    packed[:, :3] = points
    packed[:, 3] = classes.astype(np.float32)
    cloud.data = packed.tobytes()
    return cloud


def main(argv=None):
    """ROS 2 入口：解析参数、起节点、退出时清理。"""
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=argv)
    node = Node('lidar_segmentation_params')
    defaults = {
        'checkpoint': 'models/seg_mlp.npz',
        'host': cc.DEFAULT_HOST,
        'port': str(cc.DEFAULT_PORT),
        'town': cc.DEFAULT_TOWN,
        'vehicle': cc.EGO_BLUEPRINT,
        'spawn_index': '0',
        'traffic': '12',
        'corridor_width': '1.2',
    }
    for name, value in defaults.items():
        _declare(node, rclpy, name, value)
    params = {name: node.get_parameter(name).value for name in defaults}
    node.destroy_node()

    runner = LidarSegmentationNode(
        rclpy, Node,
        checkpoint=params['checkpoint'], host=params['host'], port=int(params['port']),
        town=params['town'], vehicle=params['vehicle'],
        spawn_index=int(params['spawn_index']), traffic=int(params['traffic']),
        corridor_width=float(params['corridor_width']))
    try:
        rclpy.spin(runner.node)
    except KeyboardInterrupt:
        pass
    finally:
        runner.destroy()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())