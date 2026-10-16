# -*- coding: utf-8 -*-
"""ROS 2 Humble 节点：在 CARLA 中运行跟车场景，并把跟车状态发布到话题。

话题
----
``/acc/state`` : ``std_msgs/Float32MultiArray``

    ``data = [t, gap_true, v_rel_true, v_ego, throttle, brake, follow_state]``

    依次为：仿真时间(s)、真实车距(m)、相对速度(m/s，负值表示接近)、
    自车速度(m/s)、油门、刹车、跟车状态(0 远 / 1 近 / 2 危险)。

为什么不自定义消息类型：``Float32MultiArray`` 无需编译接口包，新手排错成本最低，
后续如果确实需要，再迁移到 ``carla_nn_car_following/msg/AccState.msg``。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from collections import deque

import carla

from .carla_client import CarlaSession, distance_between, speed_of
from .reference_acc import ReferenceACC
from .scenario import build as build_scenario, lane_steer, steer_to_point


def set_spectator_follow(world, vehicle, distance=9.0, height=4.5, pitch=-15.0):
    """让 CARLA 窗口的观察者跟随车辆（第三人称跟随镜头，录制动图用）。"""
    transform = vehicle.get_transform()
    forward = transform.get_forward_vector()
    location = carla.Location(x=transform.location.x - forward.x * distance,
                              y=transform.location.y - forward.y * distance,
                              z=transform.location.z + height)
    world.get_spectator().set_transform(
        carla.Transform(location, carla.Rotation(pitch=pitch, yaw=transform.rotation.yaw)))

NODE_NAME = 'acc_scene_node'

PARAMETERS = (
    ('host', '192.168.148.1'),
    ('port', '2000'),
    ('town', 'Town03'),
    ('weather', 'ClearNoon'),
    ('scenario', 'decelerate'),
    ('duration', '60.0'),
    ('dt', '0.05'),
    ('lead_distance', '25.0'),
    ('spawn_index', '-1'),
    ('controller', 'classic'),
    ('models_dir', 'models'),
    ('v_set_margin', '2.0'),
    ('d0', '5.0'),
    ('headway', '1.2'),
    ('kp', '0.35'),
    ('kv', '0.8'),
    ('k_throttle', '0.25'),
    ('k_brake', '0.35'),
    ('k_feedforward', '0.03'),
    ('radar_fov', '30.0'),
    ('radar_range', '100.0'),
    ('radar_vertical_fov', '4.0'),
    ('radar_velocity_tolerance', '8.0'),
    ('radar_azimuth_tolerance_deg', '8.0'),
)


class AccSceneNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        for name, default in PARAMETERS:
            self.declare_parameter(name, default)

        self.dt = self._float('dt')
        self.duration = self._float('duration')
        self.scenario = build_scenario(self._string('scenario'))
        controller_name = self._string('controller').strip().lower()
        self.acc = None
        self.nn_controller = None
        if controller_name == 'nn':
            from .nn_controller import NNController
            self.nn_controller = NNController.load(self._string('models_dir'))
        else:
            v_set = (float(getattr(self.scenario, 'v0', 0.0))
                     + self._float('v_set_margin')) or None
            self.acc = ReferenceACC(d0=self._float('d0'), headway=self._float('headway'),
                                    kp=self._float('kp'), kv=self._float('kv'),
                                    k_throttle=self._float('k_throttle'),
                                    k_brake=self._float('k_brake'),
                                    k_feedforward=self._float('k_feedforward'),
                                    v_set=v_set)

        self.publisher = self.create_publisher(Float32MultiArray, '/acc/state', 10)

        self.session = None
        self.ego = None
        self.lead = None
        self.extras = []
        self.previous_gap = {}
        self.target_id = None
        # 前车轨迹（面包屑）：自车沿前车 1.2 秒前走过的位置行驶，路口不会跟丢
        self.trail_len = max(1, int(round(1.2 / self.dt)))
        self.lead_trail = deque(maxlen=self.trail_len)
        self.t = 0.0
        self.steps = 0
        self.finished = False
        self.gap_errors = []
        self.min_gap = float('inf')
        self.min_ttc = float('inf')

        try:
            self._setup()   # 连接失败时抛出异常，让 ros2 launch 直接报错退出
        except Exception:
            # _setup() 内部可能已经生成车辆、挂载传感器并把世界切到同步模式，
            # 此时必须清理，否则 CARLA 会一直卡在同步模式（窗口冻结），
            # 且已生成的对象会残留在世界里。
            # 注意：__init__ 抛出异常时外部拿不到 node 对象，无法调用 destroy_node()，
            # 所以清理只能在这里做。
            self._cleanup_session()
            self.get_logger().warn(
                '初始化失败：已销毁已生成对象并恢复原始世界设置，避免 CARLA 停留在同步模式')
            raise
        self.timer = self.create_timer(self.dt, self.on_timer)
        self.get_logger().info(
            '已就绪：地图 %s，工况 %s，时长 %.0f s，步长 %.3f s'
            % (self.session.current_map_name(), self.scenario.label,
               self.duration, self.dt))

    # ---------------- 参数读取 ----------------
    def _string(self, name):
        return str(self.get_parameter(name).value)

    def _float(self, name):
        return float(self.get_parameter(name).value)

    # ---------------- 跟车指标（与所用控制器无关） ----------------
    def _desired_gap(self, v_ego):
        return self._float('d0') + self._float('headway') * v_ego

    def _follow_state(self, gap, v_ego):
        """0 = 远，1 = 近，2 = 危险。"""
        error = gap - self._desired_gap(v_ego)
        if error < -2.0:
            return 2
        if error < 3.0:
            return 1
        return 0

    # ---------------- 初始化 ----------------
    def _setup(self):
        self.session = CarlaSession(self._string('host'), int(self._string('port')))
        town = self._string('town')
        if town and town not in self.session.current_map_name():
            self.get_logger().info('正在加载地图 %s ...' % town)
            self.session.load_town(town)
        weather = self._string('weather')
        if weather:
            self.session.set_weather(weather)
        self.session.enable_sync(self.dt)
        self.carla_map = self.session.world.get_map()

        self.ego = self.session.spawn_ego(spawn_index=int(self._string('spawn_index')))
        self.lead = self.session.spawn_in_lane(self.ego, self._float('lead_distance'),
                                               role_name='lead_vehicle')
        if self.lead is None:
            raise RuntimeError('无法生成前车，请确认出生点未被占用')

        self.session.attach_radar(
            self.ego,
            fov=self._float('radar_fov'),
            radar_range=self._float('radar_range'),
            azimuth_tolerance_deg=self._float('radar_azimuth_tolerance_deg'),
            vertical_fov=self._float('radar_vertical_fov'),
            velocity_tolerance=self._float('radar_velocity_tolerance'),
            tick=self.dt)

        # 等雷达出第一帧有效数据再进入控制循环，避免首帧返回量程上限（100 m）
        for _ in range(40):
            if self.session.radar.min_depth is not None:
                break
            self.session.tick()
        if self.session.radar.min_depth is None:
            self.get_logger().warn('雷达在 40 帧内没有测到目标，请检查前车位置与雷达参数')

        # 两车以巡航速度开场，避免从静止起步造成的剧烈振荡（与 main.py 保持一致）
        cruise = float(getattr(self.scenario, 'v0', 0.0))
        if cruise > 1.0:
            for actor in (self.ego, self.lead):
                forward = actor.get_transform().get_forward_vector()
                actor.set_target_velocity(carla.Vector3D(x=forward.x * cruise,
                                                         y=forward.y * cruise,
                                                         z=0.0))
            self.session.settle(2)
            self.get_logger().info('初始速度设为 %.1f m/s（%.0f km/h）'
                                   % (cruise, cruise * 3.6))

    # ---------------- 定时器回调 ----------------
    def on_timer(self):
        if self.finished:
            return

        self.session.tick()
        t = self.t

        if self.scenario.wants_cut_in(t) and not self.extras:
            extra = self.session.spawn_in_lane(self.ego, self.scenario.cut_in_distance,
                                               blueprint_id='vehicle.audi.a2',
                                               role_name='cut_in_vehicle', free_radius=6.0)
            if extra is not None:
                self.extras.append(extra)
                self.scenario.mark_spawned()
                self.get_logger().info('t=%.1fs：前方出现加塞车辆' % t)
        self.scenario.update(t, self.dt, self.lead, self.extras, self.carla_map)

        features = self.session.radar.features(self.dt)
        v_ego = speed_of(self.ego)

        front = [self.lead] + self.extras
        nearest = min(front, key=lambda actor: distance_between(self.ego, actor))
        gap_true = distance_between(self.ego, nearest)

        # 目标切换时重新初始化雷达跟踪，面包屑改跟新目标
        if nearest.id != self.target_id:
            self.session.radar.reset_tracking()
            self.target_id = nearest.id
        self.lead_trail.append(nearest.get_location())
        previous = self.previous_gap.get(nearest.id)
        # 车距的导数就是相对速度 v_rel = v_lead - v_ego：车距在减小 -> 负值
        v_rel_true = 0.0 if previous is None else (gap_true - previous) / self.dt
        v_rel_true = max(-35.0, min(35.0, v_rel_true))     # 剔除单帧跳变造成的伪值
        self.previous_gap[nearest.id] = gap_true

        gap_hat = v_rel_hat = None
        if self.nn_controller is not None:
            # 神经网络控制器：由雷达特征估计车距/相对速度，再输出油门刹车
            throttle, brake, gap_hat, v_rel_hat = self.nn_controller.labels(features, v_ego)
        else:
            throttle, brake = self.acc.labels(gap_true, v_rel_true, v_ego)

        # 横向：优先沿前车轨迹（面包屑）行驶；纵向：经典 ACC 或神经网络
        if len(self.lead_trail) >= self.trail_len:
            steer = steer_to_point(self.ego, self.lead_trail[0], gain=1.0)
        else:
            steer = lane_steer(self.ego, self.carla_map)
        self.ego.apply_control(carla.VehicleControl(
            throttle=throttle, brake=brake, steer=steer))
        set_spectator_follow(self.session.world, self.ego)   # 第三人称跟随镜头

        message = Float32MultiArray()
        message.data = [float(t), float(gap_true), float(v_rel_true), float(v_ego),
                        float(throttle), float(brake),
                        float(self._follow_state(gap_true, v_ego))]
        if gap_hat is not None:
            message.data.extend([float(gap_hat), float(v_rel_hat)])
        self.publisher.publish(message)

        self.gap_errors.append(abs(gap_true - self._desired_gap(v_ego)))
        self.min_gap = min(self.min_gap, gap_true)
        closing = -v_rel_true
        if closing > 1e-6:
            self.min_ttc = min(self.min_ttc, gap_true / closing)

        self.steps += 1
        self.t += self.dt
        if self.steps % max(1, int(round(0.5 / self.dt))) == 0:
            estimate = ''
            if gap_hat is not None:
                estimate = ' 估计=%6.2fm(误差%+5.2f)' % (gap_hat, gap_hat - gap_true)
            self.get_logger().info(
                't=%5.1fs 雷达=%6.2fm 真值=%6.2fm v_rel=%+5.2f m/s 油门=%.2f 刹车=%.2f%s'
                % (t, features['d_radar'], gap_true, v_rel_true, throttle, brake, estimate))

        if t >= self.duration and not self.finished:
            self.finished = True
            self._stop_all_vehicles()
            mean_error = (sum(self.gap_errors) / len(self.gap_errors)) if self.gap_errors else 0.0
            self.get_logger().info(
                '场景结束：平均车距误差 %.2f m，最小车距 %.2f m，最小 TTC %s s'
                % (mean_error, self.min_gap,
                   ('%.2f' % self.min_ttc) if self.min_ttc != float('inf') else 'inf'))
            # 场景跑完后必须恢复异步模式：本节点不再 tick，若仍停留在同步模式，
            # CARLA 窗口会因为收不到 tick 而一直冻结不动。
            self.session.restore_async()
            self.get_logger().info(
                '已让自车与前车同时停稳并恢复异步模式（CARLA 窗口不会冻结）。'
                '需要退出请按一次 Ctrl+C；需要重新跟车请重新执行 ros2 launch。')

    def _stop_all_vehicles(self, frames=30):
        """让自车、前车与加塞车一起刹停，使演示结束时画面干净。

        只给自车刹车是不够的：前车会保留最后一条控制指令继续行驶，
        世界恢复异步模式后就会一直开到撞墙，看起来像是"程序出错了"。
        """
        actors = [self.ego, self.lead] + list(self.extras)
        for _ in range(int(frames)):
            for actor in actors:
                if actor is not None:
                    actor.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
            self.session.tick()

    # ---------------- 退出 ----------------
    def _cleanup_session(self):
        """销毁已生成的对象并恢复原始世界设置（避免 CARLA 停留在同步模式）。"""
        session = getattr(self, 'session', None)
        if session is not None:
            try:
                session.close()
            except Exception as exc:                      # 清理失败不能掩盖原始异常
                self.get_logger().warn('清理 CARLA 会话时出错：%s' % exc)
            self.session = None

    def destroy_node(self):
        self._cleanup_session()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AccSceneNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print('[错误] 节点启动失败：%s' % exc)
        print('       请确认：宿主机 CARLA 已启动、host 参数正确、防火墙已放行 2000 端口。')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
