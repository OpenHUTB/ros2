# -*- coding: utf-8 -*-
"""节点①：carla_camera_node

职责（对应作业要求 3-①）：
    1. 启动时校验 CARLA Python 客户端（import carla），失败给出友好提示；
    2. 连接 CARLA 服务端（IP 与端口由 ROS 参数提供，默认值在 config/params.yaml）；
    3. 生成自车（ego vehicle），可选在前方生成一辆静止障碍车，方便演示避障；
    4. 在自车车头挂载前视 RGB 相机，把图像发布到 ROS 图像话题；
    5. 订阅决策节点下发的控制指令，并通过 CARLA API 施加到自车（闭合控制回路）。

说明：决策节点只算出"油门/刹车"，真正写进 CARLA 仿真的是本节点，
这样三个节点职责清晰：1 负责仿真接口，2 负责感知，3 负责决策。
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from . import carla_env, config, image_utils


class CarlaCameraNode(Node):
    """连接 CARLA、发布相机图像、执行车辆控制。"""

    def __init__(self):
        super().__init__('carla_camera_node')

        # ---- 0) 环境检测：import carla 失败会打印友好提示并退出 ----
        self.get_logger().info('环境检查：{}'.format(carla_env.describe_environment()))

        # ---- 1) 读取 CARLA 地址参数（由 config/params.yaml 提供）----
        # 换 IP / 换端口只改 YAML，不需要改这里的代码
        self.carla_host, self.carla_port = self.read_carla_endpoint()
        self.carla = carla_env.import_carla_or_exit()

        self.get_logger().info(
            '正在连接 CARLA 服务端 {}:{} ...'.format(self.carla_host, self.carla_port)
        )
        self.client, self.world = carla_env.connect_to_carla(
            self.carla, self.carla_host, self.carla_port
        )
        self.get_logger().info('已连接 CARLA，当前地图 = {}'.format(self.world.get_map().name))

        # ---- 2) ROS 接口 ----
        # 原图：相机流用 sensor QoS（Best Effort，丢帧不影响功能）
        self.publisher_image = self.create_publisher(
            Image, config.TOPIC_CAMERA_IMAGE, qos_profile_sensor_data
        )
        # 控制指令：[油门, 刹车, 转向]
        self.subscription_control = self.create_subscription(
            Float32MultiArray, config.TOPIC_CONTROL_CMD, self.on_control_command, 10
        )

        self._camera = None
        self._ego_vehicle = None
        self._lead_vehicle = None
        self._shutdown_done = False

        # 控制量：默认刹停，收到决策节点的指令后才前进（安全兜底）
        self._throttle = 0.0
        self._brake = float(config.INITIAL_BRAKE)
        self._steer = 0.0

        self._last_publish_time = 0.0
        self._published_frames = 0
        self._last_speed_log_time = 0.0

        # ---- 3) 生成车辆与相机 ----
        self._spawn_ego_vehicle()
        self._spawn_lead_vehicle()
        self._attach_camera()

        # ---- 4) 定时把控制指令写进 CARLA ----
        self.create_timer(1.0 / config.CONTROL_APPLY_HZ, self.apply_control)
        self.get_logger().info(
            '节点就绪：图像话题={} 控制话题={}'.format(
                config.TOPIC_CAMERA_IMAGE, config.TOPIC_CONTROL_CMD
            )
        )

    # ------------------------------------------------------------------
    # ROS 参数：CARLA 服务地址
    # ------------------------------------------------------------------
    def read_carla_endpoint(self):
        """从 ROS 参数读取 CARLA 的 IP 与端口。

        ROS 参数名见 config.PARAM_CARLA_HOST / config.PARAM_CARLA_PORT，
        默认值写在 config/params.yaml 里，由 launch 文件加载：
        换机器、换端口只改 YAML，不需要改 Python 源码。

        参数缺失或类型不对时给出友好提示并退出，
        而不是抛一串看不懂的 rclpy 异常。
        """
        try:
            host = self.declare_parameter(
                config.PARAM_CARLA_HOST, ''
            ).get_parameter_value().string_value
            port = self.declare_parameter(
                config.PARAM_CARLA_PORT, 0
            ).get_parameter_value().integer_value
        except Exception as exc:
            # 例如 YAML 里把端口写成了带引号的字符串，类型对不上；
            # 这里按"缺参数"处理，给出统一的友好提示并退出
            self.get_logger().error(
                '读取 CARLA 地址参数失败：{}: {}'.format(type(exc).__name__, exc)
            )
            return carla_env.check_endpoint_or_exit('', 0)

        return carla_env.check_endpoint_or_exit(host, port)

    # ------------------------------------------------------------------
    # CARLA：生成车辆
    # ------------------------------------------------------------------
    def _pick_vehicle_blueprint(self, blueprint_id):
        """按车型名找蓝图；找不到就退化为第一个四轮车蓝图。"""
        bp_lib = self.world.get_blueprint_library()
        found = list(bp_lib.filter(blueprint_id))
        if found:
            return found[0]

        self.get_logger().warn('未找到车型 {}，自动改用其它四轮车'.format(blueprint_id))
        for bp in bp_lib.filter('vehicle.*'):
            if bp.has_attribute('number_of_wheels'):
                if bp.get_attribute('number_of_wheels').as_int() == 4:
                    return bp
        return list(bp_lib.filter('vehicle.*'))[0]

    def _spawn_ego_vehicle(self):
        """在第一个空闲出生点生成自车。"""
        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            self.get_logger().error('当前地图没有可用出生点，无法生成车辆')
            raise SystemExit(1)

        blueprint = self._pick_vehicle_blueprint(config.EGO_VEHICLE_BLUEPRINT)
        if blueprint.has_attribute('role_name'):
            blueprint.set_attribute('role_name', 'ego_vehicle')

        self._ego_spawn_transform = spawn_points[0]
        for index, transform in enumerate(spawn_points):
            vehicle = self.world.try_spawn_actor(blueprint, transform)
            if vehicle is not None:
                self._ego_vehicle = vehicle
                self._ego_spawn_transform = transform
                self.get_logger().info(
                    '已生成自车 {}（出生点 #{}）位置=({:.1f}, {:.1f})'.format(
                        vehicle.type_id, index, transform.location.x, transform.location.y
                    )
                )
                return

        self.get_logger().error('所有出生点都被占用，生成自车失败；请重启 CARLA 后重试')
        raise SystemExit(1)

    def _spawn_lead_vehicle(self):
        """在自车正前方生成一辆静止的障碍车，用于演示减速避障。"""
        if not config.SPAWN_LEAD_VEHICLE:
            self.get_logger().info('按配置跳过障碍车生成')
            return

        base = self._ego_spawn_transform
        yaw = math.radians(base.rotation.yaw)
        ahead = float(config.LEAD_VEHICLE_AHEAD_M)
        lateral = float(config.LEAD_VEHICLE_LATERAL_M)

        # CARLA(Unreal)坐标系：yaw=0 时前方为 +X，右方为 +Y
        location = self.carla.Location(
            x=base.location.x + ahead * math.cos(yaw) - lateral * math.sin(yaw),
            y=base.location.y + ahead * math.sin(yaw) + lateral * math.cos(yaw),
            z=base.location.z,
        )
        transform = self.carla.Transform(location, base.rotation)

        blueprint = self._pick_vehicle_blueprint(config.LEAD_VEHICLE_BLUEPRINT)
        if blueprint.has_attribute('role_name'):
            blueprint.set_attribute('role_name', 'lead_vehicle')

        vehicle = self.world.try_spawn_actor(blueprint, transform)
        if vehicle is None:
            self.get_logger().warn(
                '障碍车生成失败（该位置可能已被占用），本次演示将看不到前方目标'
            )
            return

        # 静止障碍：不启用自动驾驶，也不施加任何控制
        vehicle.set_autopilot(False)
        self._lead_vehicle = vehicle
        self.get_logger().info(
            '已生成障碍车 {}，位于自车前方 {:.1f} m'.format(vehicle.type_id, ahead)
        )

    def _attach_camera(self):
        """在自车车头挂载前视 RGB 相机。"""
        bp_lib = self.world.get_blueprint_library()
        camera_bp = bp_lib.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(config.CAMERA_IMAGE_WIDTH))
        camera_bp.set_attribute('image_size_y', str(config.CAMERA_IMAGE_HEIGHT))
        camera_bp.set_attribute('fov', str(config.CAMERA_FOV_DEG))
        camera_bp.set_attribute('role_name', 'front_camera')

        camera_transform = self.carla.Transform(
            self.carla.Location(*config.CAMERA_POSITION),
            self.carla.Rotation(*config.CAMERA_ROTATION),
        )
        self._camera = self.world.spawn_actor(
            camera_bp, camera_transform, attach_to=self._ego_vehicle
        )
        # listen 回调会在后台线程里被调用
        self._camera.listen(self.on_camera_image)
        self.get_logger().info(
            '前视相机已挂载：{}x{} fov={}°'.format(
                config.CAMERA_IMAGE_WIDTH, config.CAMERA_IMAGE_HEIGHT, config.CAMERA_FOV_DEG
            )
        )

    # ------------------------------------------------------------------
    # 相机回调：CARLA 图像 -> ROS 图像
    # ------------------------------------------------------------------
    def on_camera_image(self, carla_image):
        """收到 CARLA 相机帧后转成 ROS Image 并发布（按配置限频）。"""
        now = time.monotonic()
        if now - self._last_publish_time < 1.0 / config.CAMERA_PUBLISH_FPS:
            return
        self._last_publish_time = now

        try:
            # CARLA 的原始数据是 BGRA，这里取出 BGR 三通道
            raw = np.frombuffer(carla_image.raw_data, dtype=np.uint8)
            bgra = raw.reshape((carla_image.height, carla_image.width, 4))
            bgr = np.ascontiguousarray(bgra[:, :, :3])  # 必须拷贝，CARLA 会复用缓冲区
        except Exception as exc:  # 极少数情况下数据长度异常
            self.get_logger().warn('相机数据解析失败：{}'.format(exc))
            return

        msg = image_utils.bgr_to_ros_image(
            bgr, config.CAMERA_FRAME, self.get_clock().now().to_msg()
        )
        self.publisher_image.publish(msg)

        self._published_frames += 1
        if self._published_frames % config.LOG_EVERY_N == 0:
            self.get_logger().info(
                '已发布 {} 帧图像到 {}'.format(self._published_frames, config.TOPIC_CAMERA_IMAGE)
            )

    # ------------------------------------------------------------------
    # 控制
    # ------------------------------------------------------------------
    def on_control_command(self, msg):
        """接收决策节点的 [油门, 刹车, 转向]。"""
        if len(msg.data) < 2:
            self.get_logger().warn('收到的控制指令长度不足，已忽略：{}'.format(list(msg.data)))
            return
        self._throttle = self._clamp(msg.data[0])
        self._brake = self._clamp(msg.data[1])
        # 按要求不做转向控制：即使收到第三个元素也强制置 0
        self._steer = 0.0 if config.STEER_ALWAYS_ZERO else self._clamp(msg.data[2])

    def apply_control(self):
        """把当前控制量写入 CARLA 车辆。"""
        if self._ego_vehicle is None:
            return

        control = self.carla.VehicleControl()
        control.throttle = float(self._throttle)
        control.brake = float(self._brake)
        control.steer = float(self._steer)
        control.hand_brake = False
        control.manual_gear_shift = False
        try:
            self._ego_vehicle.apply_control(control)
        except Exception as exc:
            self.get_logger().warn('施加控制失败：{}'.format(exc))
            return

        now = time.monotonic()
        if now - self._last_speed_log_time >= 1.0:
            self._last_speed_log_time = now
            velocity = self._ego_vehicle.get_velocity()
            speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
            self.get_logger().info(
                '自车速度 {:.2f} m/s | 油门 {:.2f} 刹车 {:.2f} 转向 {:.2f}'.format(
                    speed, control.throttle, control.brake, control.steer
                )
            )

    @staticmethod
    def _clamp(value, low=0.0, high=1.0):
        """把控制量限制在 [0, 1]，避免非法值导致车辆行为异常。"""
        try:
            number = float(value)
        except (TypeError, ValueError):
            return low
        return max(low, min(high, number))

    # ------------------------------------------------------------------
    # 退出清理
    # ------------------------------------------------------------------
    def shutdown(self):
        """销毁相机与车辆，避免反复运行后 CARLA 里堆积 actor。幂等。"""
        if self._shutdown_done:
            return
        self._shutdown_done = True

        for actor, name in ((self._camera, '相机'),
                            (self._lead_vehicle, '障碍车'),
                            (self._ego_vehicle, '自车')):
            if actor is None:
                continue
            try:
                if name == '相机':
                    actor.stop()
                actor.destroy()
                self.get_logger().info('已销毁{}'.format(name))
            except Exception as exc:
                self.get_logger().warn('销毁{}失败（可忽略）：{}'.format(name, exc))

        self._camera = None
        self._lead_vehicle = None
        self._ego_vehicle = None


def main(args=None):
    """节点①的启动函数（供 main.py 调用）。"""
    rclpy.init(args=args)
    node = None
    try:
        node = CarlaCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
