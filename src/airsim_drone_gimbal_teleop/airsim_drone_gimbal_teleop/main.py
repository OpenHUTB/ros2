"""AirSim 无人机键盘遥控 + 机载云台相机预览（ROS 2 入口 ``drone_gimbal_teleop``）.

用法（ROS 2）::

    ros2 run airsim_drone_gimbal_teleop drone_gimbal_teleop --ip 192.168.1.10

用法（直接运行）::

    python3 src/airsim_drone_gimbal_teleop/airsim_drone_gimbal_teleop/main.py --takeoff

按键：W/S 前后、A/D 左右、Q/E 升降、Z/X 左右转、I/K 云台俯仰、J/L 云台偏航、ESC 退出并降落。
坐标约定：AirSim 使用 NED（x 向前、y 向右、z 向下），所以“上升”对应 z 为负。
"""
import argparse
import math

# ---------------------------------------------------------------- 参数常量
FORWARD_SPEED = 3.0      # m/s，W/S
LATERAL_SPEED = 2.0      # m/s，A/D
VERTICAL_SPEED = 1.5     # m/s，Q/E
YAW_RATE_DEG = 45.0      # deg/s，Z/X
GIMBAL_STEP_DEG = 2.0    # 每帧云台角度步长

#: 按键 -> 动作名（只做映射，方便离线测试；真正读键在 run() 里）
KEYMAP = {
    'w': 'forward', 's': 'backward',
    'a': 'left', 'd': 'right',
    'q': 'up', 'e': 'down',
    'z': 'yaw_left', 'x': 'yaw_right',
    'i': 'gimbal_up', 'k': 'gimbal_down',
    'j': 'gimbal_left', 'l': 'gimbal_right',
}

#: 方向键作为云台的别名（与 README 中“方向键也能调云台”一致）
ARROW_ALIASES = {
    'up': 'gimbal_up', 'down': 'gimbal_down',
    'left': 'gimbal_left', 'right': 'gimbal_right',
}


def clamp(value, low, high):
    """把数值夹到 [low, high]。"""
    return max(low, min(high, value))


def wrap_degrees(value):
    """把角度归一化到 ``(-180, 180]``：-180 与 +180 都返回 +180。

    例如 ``wrap_degrees(181) == -179``、``wrap_degrees(-181) == 179``。
    """
    wrapped = math.fmod(value + 180.0, 360.0)
    if wrapped <= 0.0:
        wrapped += 360.0
    return wrapped - 180.0


def command_from_actions(actions, forward_speed=FORWARD_SPEED, lateral_speed=LATERAL_SPEED,
                         vertical_speed=VERTICAL_SPEED, yaw_rate=YAW_RATE_DEG):
    """把动作集合转成 AirSim 速度指令 ``(vx, vy, vz, yaw_rate_deg_s)``。

    NED 坐标：x 向前、y 向右、z 向下；yaw_rate 正值表示顺时针（向右转）。
    “上升”输出 vz 为负。
    """
    actions = set(actions)
    vx = forward_speed * (('forward' in actions) - ('backward' in actions))
    vy = lateral_speed * (('right' in actions) - ('left' in actions))
    vz = vertical_speed * (('down' in actions) - ('up' in actions))
    yaw = yaw_rate * (('yaw_right' in actions) - ('yaw_left' in actions))
    return float(vx), float(vy), float(vz), float(yaw)


def gimbal_from_actions(actions, pitch, yaw, step=GIMBAL_STEP_DEG):
    """按动作更新云台 ``(pitch, yaw)``，pitch 夹在 ±90°，yaw 归一到 ``(-180, 180]``。"""
    actions = set(actions)
    if 'gimbal_up' in actions:
        pitch += step
    if 'gimbal_down' in actions:
        pitch -= step
    if 'gimbal_left' in actions:
        yaw -= step
    if 'gimbal_right' in actions:
        yaw += step
    return clamp(pitch, -90.0, 90.0), wrap_degrees(yaw)


def actions_from_pressed(pressed):
    """把“按下的按键名集合”转成动作集合（pressed 用小写键名，如 {'w', 'i'}）。"""
    actions = set()
    for key in pressed:
        action = KEYMAP.get(str(key).lower()) or ARROW_ALIASES.get(str(key).lower())
        if action:
            actions.add(action)
    return actions


def display_array(rgb):
    """把 AirSim 的 ``(H, W, 3)`` 图像转成 pygame.surfarray 需要的 ``(W, H, 3)``。

    只做 **x/y 轴转置**，不做旋转：``np.rot90`` 会把画面逆时针转 90°，
    导致预览画面方向错误（上下/左右颠倒）。pygame 的 surfarray 约定是
    ``surface[x][y]``，因此第 0 维必须是宽度。
    """
    import numpy as np

    array = np.asarray(rgb)
    if array.ndim != 3 or array.shape[2] != 3:
        raise ValueError('expected (H, W, 3) RGB array, got %r' % (array.shape,))
    return np.ascontiguousarray(array.transpose(1, 0, 2))


def _read_frame(client, camera):
    """抓一帧场景图，返回 ``(rgb, height, width)``；没有画面时返回 None。"""
    import airsim
    import numpy as np

    responses = client.simGetImages(
        [airsim.ImageRequest(camera, airsim.ImageType.Scene, False, False)])
    if not responses:
        return None
    response = responses[0]
    if response.height == 0 or response.width == 0 or not response.image_data_uint8:
        return None
    raw = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
    return raw.reshape(response.height, response.width, 3), response.height, response.width


def _shutdown(client, connected, pygame_ready):
    """统一的退出清理：降落 → 上锁 → 释放 API 控制权 → 关闭窗口。

    不管循环体里发生什么（SDL 初始化失败、起飞异常、Ctrl+C），
    只要连接成功过就一定会走这里，避免飞机保持解锁 + 占用 API 控制权。
    """
    if connected:
        try:
            print('[INFO] 正在降落 ...')
            client.landAsync().join()
            client.armDisarm(False)
            client.enableApiControl(False)
            print('降落完成，API 控制权已释放')
        except Exception as exc:  # noqa: BLE001 - 清理阶段不能再抛异常
            print('[WARN] 降落/释放控制权失败：{}'.format(exc))
            try:
                client.enableApiControl(False)
            except Exception:  # noqa: BLE001
                pass
    elif client is not None:
        try:
            client.enableApiControl(False)
        except Exception:  # noqa: BLE001
            pass
    if pygame_ready:
        try:
            import pygame
            pygame.quit()
        except Exception:  # noqa: BLE001
            pass


def run(args):
    """连接 AirSim、打开 pygame 窗口并进入遥控循环；返回进程退出码。"""
    try:
        import airsim
        import numpy as np
        import pygame
    except ImportError as exc:
        print('[ERROR] 缺少依赖：{}'.format(exc))
        print('[ERROR] 请先执行：pip3 install airsim pygame numpy')
        return 1

    client = None
    connected = False
    pygame_ready = False
    try:
        client = airsim.MultirotorClient(ip=args.ip, port=args.port)
        client.confirmConnection()
        client.enableApiControl(True)
        client.armDisarm(True)
        connected = True

        if args.takeoff:
            print('[INFO] 起飞到 {:.1f} m ...'.format(args.altitude))
            client.takeoffAsync().join()
            client.moveToZAsync(-abs(args.altitude), 2.0).join()

        # pygame 初始化放在 try 内：没有 DISPLAY / SDL 失败时也会走 finally 降落。
        pygame.init()
        pygame_ready = True
        screen = pygame.display.set_mode((args.width, args.height))
        pygame.display.set_caption('AirSim Drone Gimbal Teleop')
        clock = pygame.time.Clock()
        pitch = yaw = 0.0
        print('[INFO] 已连接 AirSim，按 ESC 退出并自动降落')

        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False

            frame = _read_frame(client, args.camera)
            if frame is not None:
                rgb, height, width = frame
                surface = pygame.surfarray.make_surface(display_array(rgb))
                screen.blit(surface, ((args.width - width) // 2, (args.height - height) // 2))

            pressed = pygame.key.get_pressed()
            keys = set()
            for name in ('w', 'a', 's', 'd', 'q', 'e', 'z', 'x', 'i', 'j', 'k', 'l'):
                if pressed[getattr(pygame, 'K_' + name)]:
                    keys.add(name)
            for key_name, arrow in (('up', pygame.K_UP), ('down', pygame.K_DOWN),
                                    ('left', pygame.K_LEFT), ('right', pygame.K_RIGHT)):
                if pressed[arrow]:
                    keys.add(key_name)
            actions = actions_from_pressed(keys)

            vx, vy, vz, yaw_rate = command_from_actions(actions)
            pitch, yaw = gimbal_from_actions(actions, pitch, yaw)

            client.simSetCameraPose(
                args.camera,
                airsim.Pose(airsim.Vector3r(0, 0, 0),
                            airsim.to_quaternion(math.radians(pitch), 0, math.radians(yaw))),
                external=False)
            if any((vx, vy, vz, yaw_rate)):
                client.moveByVelocityAsync(
                    vx, vy, vz, args.duration,
                    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate))

            pygame.display.flip()
            clock.tick(args.fps)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001 - 把连接/窗口/控制异常变成友好提示
        print('[ERROR] 运行失败：{}'.format(exc))
        return 1
    finally:
        _shutdown(client, connected, pygame_ready)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description='AirSim 无人机键盘遥控 + 机载云台相机预览')
    parser.add_argument('--ip', default='127.0.0.1', help='AirSim 主机 IP')
    parser.add_argument('--port', type=int, default=41451, help='AirSim RPC 端口')
    parser.add_argument('--camera', default='0', help='相机名称（默认前置 0）')
    parser.add_argument('--fps', type=int, default=30, help='画面刷新率')
    parser.add_argument('--width', type=int, default=640, help='窗口宽度')
    parser.add_argument('--height', type=int, default=480, help='窗口高度')
    parser.add_argument('--duration', type=float, default=0.1, help='单条速度指令持续时间（秒）')
    parser.add_argument('--takeoff', action='store_true', help='开始时先起飞到 --altitude')
    parser.add_argument('--altitude', type=float, default=3.0, help='起飞高度（米，正数）')
    args = parser.parse_args(argv)
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
