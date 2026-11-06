"""AirSim 无人机键盘遥控 + 机载云台相机预览.

用法（ROS 2）::

    ros2 run airsim_drone_gimbal_teleop drone_gimbal_teleop --ip 192.168.1.10

用法（直接运行）::

    python3 src/airsim_drone_gimbal_teleop/main.py --takeoff

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
    """把角度归一化到 (-180, 180]。"""
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
    """按动作更新云台 ``(pitch, yaw)``，pitch 夹在 ±90°，yaw 归一到 (-180, 180]。"""
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


def _read_frame(client, camera):
    """抓一帧场景图，返回 (rgb ndarray, height, width)；失败返回 None。"""
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


def run(args):
    """连接 AirSim、打开 pygame 窗口并进入遥控循环；返回进程退出码。"""
    import numpy as np
    import pygame
    import airsim

    client = airsim.MultirotorClient(ip=args.ip, port=args.port)
    try:
        client.confirmConnection()
        client.enableApiControl(True)
        client.armDisarm(True)
    except Exception as exc:  # noqa: BLE001 - 需要把连接失败变成友好提示
        print('[ERROR] 无法连接 AirSim（{}:{}）：{}'.format(args.ip, args.port, exc))
        print('[ERROR] 请先启动 AirSim 仿真器，并确认 RPC 端口已开放。')
        return 1

    if args.takeoff:
        print('[INFO] 起飞到 {:.1f} m ...'.format(args.altitude))
        client.takeoffAsync().join()
        client.moveToZAsync(-abs(args.altitude), 2.0).join()

    pygame.init()
    screen = pygame.display.set_mode((args.width, args.height))
    pygame.display.set_caption('AirSim Drone Gimbal Teleop')
    clock = pygame.time.Clock()
    pitch = yaw = 0.0
    print('[INFO] 已连接 AirSim，按 ESC 退出并自动降落')

    try:
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
                surface = pygame.surfarray.make_surface(np.ascontiguousarray(np.rot90(rgb)))
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
    finally:
        try:
            print('[INFO] 正在降落 ...')
            client.landAsync().join()
            client.armDisarm(False)
            client.enableApiControl(False)
        except Exception as exc:  # noqa: BLE001
            print('[WARN] 降落/释放控制权失败：{}'.format(exc))
        finally:
            pygame.quit()
    print('降落完成，API 控制权已释放')
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
