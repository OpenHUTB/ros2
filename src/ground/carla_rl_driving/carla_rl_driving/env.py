# -*- coding: utf-8 -*-
"""
Gym 环境封装：env（任务二：传感器感知 + 运动控制的强化学习环境）

设计取舍（与 CARLA-GymDrive 等开源做法一致）：
  * **训练阶段**直接连 CARLA Python API 的同步模式：单步固定 tick、确定性重置，
    采样效率远高于走 ros-bridge 的异步话题流；
  * **部署/演示阶段**才通过 ros-bridge 的话题闭环（见 infer_node.py），
    这样既满足"ROS 集成 + launch 启动"，又不牺牲训练效率。

观测空间：单目前视图像 Box(3, 80, 160)，归一化 [0,1]（端到端 = 只吃图像）。
动作空间：Box(2) = [steer, accel]，均归一化到 [-1, 1]。
奖励：见 reward.py。

本文件把 CARLA 的 connect/spawn/tick/collision 等细节收敛成一个 gym.Env 接口，
上层训练脚本（train.py）只跟标准 `reset() / step()` 打交道。
"""

import gym
import numpy as np
from gym import spaces

from .reward import compute_reward, DEFAULT_WEIGHTS
from .utils import preprocess_image


class CarlaRLEnv(gym.Env):
    """CARLA 端到端驾驶 Gym 环境。

    Parameters
    ----------
    host, port : CARLA server 地址（默认 localhost:2000）
    town       : 测试场景（Town01 直道 / Town02 弯道 / Town04 路口）
    target_speed : 期望巡航速度 m/s
    """

    metadata = {'render.modes': ['rgb_array']}

    def __init__(self, host='127.0.0.1', port=2000, town='Town01',
                 target_speed=5.0, image_h=80, image_w=160, max_steps=600):
        super().__init__()
        self.host = host
        self.port = port
        self.town = town
        self.target_speed = target_speed
        self.image_h = image_h
        self.image_w = image_w
        self.max_steps = max_steps

        self.observation_space = spaces.Box(
            low=0.0, high=1.0, shape=(3, image_h, image_w), dtype=np.float32)
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        self._client = None
        self._world = None
        self._ego = None
        self._camera = None
        self._collision_sensor = None
        self._camera_image = None
        self._collision = False
        self._step = 0

    # ------------------------------------------------------------------ #
    # 内部：连接 CARLA、生成车辆与传感器（延迟到 reset 才真正连接）
    # ------------------------------------------------------------------ #
    def _connect(self):
        import carla  # 延迟导入：无 carla 环境仍可 import 本模块做语法检查
        self._client = carla.Client(self.host, self.port)
        self._client.set_timeout(10.0)
        self._world = self._client.load_world(self.town)
        settings = self._world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05   # 20 Hz 控制
        self._world.apply_settings(settings)

        bp_lib = self._world.get_blueprint_library()
        car_bp = bp_lib.filter('vehicle.tesla.model3')[0]
        spawn = self._world.get_map().get_spawn_points()[0]
        self._ego = self._world.spawn_actor(car_bp, spawn)

        cam_bp = bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(self.image_w * 5))
        cam_bp.set_attribute('image_size_y', str(self.image_h * 5))
        cam_tf = carla.Transform(carla.Location(x=1.5, z=2.0))
        self._camera = self._world.spawn_actor(
            cam_bp, cam_tf, attach_to=self._ego)
        self._camera.listen(self._on_camera)

        col_bp = bp_lib.find('sensor.other.collision')
        self._collision_sensor = self._world.spawn_actor(
            col_bp, carla.Transform(), attach_to=self._ego)
        self._collision_sensor.listen(lambda e: setattr(self, '_collision', True))

    def _on_camera(self, image):
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))[:, :, :3]  # BGRA->BGR
        self._camera_image = arr

    # ------------------------------------------------------------------ #
    # gym 接口
    # ------------------------------------------------------------------ #
    def reset(self):
        if self._client is None:
            self._connect()
        self._step = 0
        self._collision = False
        # 实际工程中此处把车辆传送到路线起点；这里给一个最小可读实现
        self._world.tick()
        obs = self._get_observation()
        return obs

    def step(self, action):
        import carla
        steer, accel = float(action[0]), float(action[1])
        control = carla.VehicleControl(
            throttle=max(accel, 0.0),
            brake=max(-accel, 0.0),
            steer=steer,
        )
        self._ego.apply_control(control)
        self._world.tick()
        self._step += 1

        obs = self._get_observation()
        v = self._forward_speed()
        # 横向/航向偏差在完整实现里由定位 + 地图车道线投影计算，
        # 这里给出占位量（完整工程见参考仓库 CARLA-GymDrive）。
        c_err, heading_err = self._lane_deviation()
        reward, done, info = compute_reward(
            velocity_forward=v, c_err=c_err, heading_err=heading_err,
            steer_cmd=steer, accel_cmd=accel,
            collision=self._collision, offtrack=False,
            target_speed=self.target_speed, weights=DEFAULT_WEIGHTS)
        if self._step >= self.max_steps:
            done = True
        info['speed'] = v
        return obs, reward, done, info

    def render(self, mode='rgb_array'):
        return self._camera_image

    def close(self):
        for a in (self._camera, self._collision_sensor, self._ego):
            if a is not None:
                try:
                    a.destroy()
                except Exception:
                    pass

    # ------------------------------------------------------------------ #
    # 观测/状态量
    # ------------------------------------------------------------------ #
    def _get_observation(self):
        if self._camera_image is None:
            return np.zeros((3, self.image_h, self.image_w), dtype=np.float32)
        return preprocess_image(self._camera_image,
                                target_h=self.image_h, target_w=self.image_w)

    def _forward_speed(self):
        if self._ego is None:
            return 0.0
        vel = self._ego.get_velocity()
        tf = self._ego.get_transform()
        # 速度投影到车辆前向，得到纵向速度
        forward = tf.get_forward_vector()
        return vel.x * forward.x + vel.y * forward.y

    def _lane_deviation(self):
        # 占位：完整实现用 self._world.get_map().get_waypoint() 投影计算
        return 0.0, 0.0
