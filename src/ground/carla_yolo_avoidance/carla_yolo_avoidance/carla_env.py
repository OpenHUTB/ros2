# -*- coding: utf-8 -*-
"""环境检测模块。

作业要求：程序启动时自动校验 CARLA Python 客户端是否可用，
导入失败时给出清晰友好的控制台提示，并且**不写死固定 Python 版本**
（所有版本信息都在运行时通过 sys.version_info 动态获得）。

CARLA 的 IP 与端口同样不写死：节点从 ROS 参数读取（默认值在
config/params.yaml 里配置），非 ROS 的小工具则读同一份 YAML。

本模块不导入 carla / rclpy / torch，可以被单独运行：
    python3 -m carla_yolo_avoidance.carla_env
（等价的自检入口还有 python3 scripts/check_env.py）
"""

import os
import sys

from . import config

# 与 package.xml / 文档保持一致的 CARLA 版本号（用于提示文案，非解释器版本）
CARLA_VERSION = '0.9.16'

_LINE = '=' * 78
_DASH = '-' * 78


def python_version_string():
    """返回当前解释器的版本字符串，例如 '3.10.12'。"""
    return '{}.{}.{}'.format(*sys.version_info[:3])


def python_tag():
    """返回 CARLA egg/whl 文件名里使用的版本标记，例如 'py3.10'。

    这个值是运行时算出来的，不写死。
    """
    return 'py{major}.{minor}'.format(major=sys.version_info[0], minor=sys.version_info[1])


def _print_block(title, lines):
    """按统一格式打印提示块。"""
    print('')
    print(_LINE)
    print(title)
    print(_DASH)
    for line in lines:
        print(line)
    print(_LINE)
    print('')


def import_carla_or_exit():
    """尝试导入 carla，失败则打印友好提示并退出程序。

    返回:
        module: 成功时返回 carla 模块对象。

    退出码:
        1 —— 导入失败（由 print + SystemExit 共同完成，便于 launch 里看到明确原因）。
    """
    try:
        import carla  # noqa: F401  (延迟导入，属于本函数的核心逻辑)
        return carla
    except ImportError as exc:
        egg_example = (
            '$CARLA_ROOT/PythonAPI/carla/dist/carla-{ver}-{tag}-linux-x86_64.egg'.format(
                ver=CARLA_VERSION, tag=python_tag()
            )
        )
        _print_block(
            '[环境检测失败] 无法导入 CARLA Python 客户端：import carla',
            [
                '当前 Python 解释器 : {}'.format(python_version_string()),
                '解释器路径         : {}'.format(sys.executable),
                '原始报错           : {}: {}'.format(type(exc).__name__, exc),
                '',
                '这不是代码 bug，而是当前环境里没有安装 CARLA 客户端。',
                '请按下面任一方式处理（都在启动 ROS 2 节点的同一个终端里执行）：',
                '',
                '  【方式 1】pip 安装与本机 CARLA 版本一致的客户端',
                '      python -m pip install carla=={}'.format(CARLA_VERSION),
                '',
                '  【方式 2】直接使用 CARLA 发行包自带的客户端（版本最匹配，推荐）',
                '      export CARLA_ROOT=~/CARLA_{}'.format(CARLA_VERSION),
                '      export PYTHONPATH=$PYTHONPATH:{}'.format(egg_example),
                '      注意：egg 文件名里的 {} 必须与当前解释器一致，'.format(python_tag()),
                '            本机解释器为 Python {}，所以要用 {} 那个 egg。'.format(
                    python_version_string(), python_tag()
                ),
                '',
                '  【方式 3】确认 CARLA 服务端已启动（缺的是服务端同样会连不上）',
                '      CarlaUE4.exe -carla-port=<端口>          (Windows)',
                '      ./CarlaUE4.sh -carla-port=<端口>         (Linux)',
                '      <端口> 必须与 {} 里的 carla_port 一致。'.format(
                    'config/params.yaml'
                ),
                '',
                '自检命令： python -c "import carla; print(carla.__file__)"',
            ],
        )
        raise SystemExit(1)


def import_ultralytics_or_exit():
    """尝试导入 ultralytics 的 YOLO 类，失败则打印友好提示并退出。"""
    try:
        from ultralytics import YOLO  # noqa: F401
        return YOLO
    except ImportError as exc:
        _print_block(
            '[环境检测失败] 无法导入 YOLOv8 推理库：from ultralytics import YOLO',
            [
                '当前 Python 解释器 : {}'.format(python_version_string()),
                '解释器路径         : {}'.format(sys.executable),
                '原始报错           : {}: {}'.format(type(exc).__name__, exc),
                '',
                '请安装 ultralytics（会自动带上 PyTorch 等依赖）：',
                '      python -m pip install -U ultralytics',
                '',
                '如果只想用 CPU 跑（没有 NVIDIA 显卡时推荐，体积更小）：',
                '      python -m pip install -U ultralytics --extra-index-url '
                'https://download.pytorch.org/whl/cpu',
                '',
                '自检命令： python -c "from ultralytics import YOLO; print(YOLO)"',
                '首次运行会自动下载官方预训练权重 {}（约 6 MB）。'.format('yolov8n.pt'),
            ],
        )
        raise SystemExit(1)


def check_endpoint_or_exit(host, port):
    """校验 CARLA 地址参数；缺失或非法时打印友好提示并退出。

    地址不再写死在代码里，而是由 launch 文件加载 config/params.yaml 后
    通过 ROS 参数下发，因此这里要能给出"该去哪里改"的明确指引。

    返回:
        (host, port) —— 校验通过时原样返回，port 已转成 int
    """
    text = str(host or '').strip()
    try:
        number = int(port)
    except (TypeError, ValueError):
        number = 0

    if text and number > 0:
        return text, number

    params_file = 'src/carla_yolo_avoidance/config/params.yaml'
    _print_block(
        '[参数缺失] 没有配置 CARLA 服务地址（carla_host / carla_port）',
        [
            '当前读到的值       : carla_host="{}"  carla_port={}'.format(host, port),
            '',
            'CARLA 的 IP 与端口不在 Python 代码里写死，而是通过 ROS 参数配置。',
            '请按下面任一方式处理：',
            '',
            '  【方式 1】改配置文件（推荐，改完不需要重新编译）',
            '      打开 {}'.format(params_file),
            '      /carla_camera_node:',
            '        ros__parameters:',
            '          carla_host: "<CARLA 所在机器的 IP 或主机名>"',
            '          carla_port: <CARLA 服务端端口，整数，不要加引号>',
            '',
            '  【方式 2】单独运行节点时用命令行覆盖',
            '      ros2 run carla_yolo_avoidance carla_camera_node --ros-args \\',
            '          --params-file {}'.format(params_file),
            '      # 或者',
            '      ros2 run carla_yolo_avoidance carla_camera_node --ros-args \\',
            '          -p carla_host:=<IP> -p carla_port:=<端口>',
            '',
            '注意：carla_port 必须是整数（例如 YAML 里写 carla_port 后跟数字），',
            '      写成字符串会导致参数类型不匹配。',
        ],
    )
    raise SystemExit(1)


def connect_to_carla(carla, host, port):
    """连接 CARLA 服务端并返回 (client, world)。

    参数:
        carla: 已导入的 carla 模块
        host:  CARLA 服务端 IP / 主机名（来自 ROS 参数，config/params.yaml）
        port:  CARLA 服务端端口（来自 ROS 参数，config/params.yaml）
    连接失败时给出友好提示并退出。
    """
    try:
        client = carla.Client(str(host), int(port))
        client.set_timeout(config.CARLA_TIMEOUT_S)
        world = client.get_world()
        return client, world
    except Exception as exc:  # carla 在连接失败时抛 RuntimeError
        _print_block(
            '[连接失败] 无法连接 CARLA 服务端 {}:{}'.format(host, port),
            [
                '原始报错           : {}: {}'.format(type(exc).__name__, exc),
                '',
                '请依次检查：',
                '  1) CARLA 服务端是否已启动（本程序不负责启动 CARLA）：',
                '         CarlaUE4.exe -carla-port={port}          (Windows)'.format(port=port),
                '         ./CarlaUE4.sh -carla-port={port}         (Linux)'.format(port=port),
                '     端口必须与 config/params.yaml 里的 carla_port 一致。',
                '  2) 该端口是否被别的程序占用：',
                '         Windows: netstat -ano | findstr :{port}'.format(port=port),
                '         Linux  : ss -ltnp | grep {port}'.format(port=port),
                '  3) 地址填得对不对：如果 CARLA 跑在另一台机器上，',
                '     carla_host 要填那台机器的 IP，而不是回环地址。',
                '  4) 是否运行在 WSL / 容器里而 CARLA 跑在宿主机上：',
                '     此时回环地址指向容器自身，需要改填宿主机 IP，或开启',
                '     Windows 11 的 mirrored 网络模式。',
                '  5) CARLA 服务端刚启动时初始化较慢，可稍等 10 秒重试。',
            ],
        )
        raise SystemExit(1)


def describe_environment():
    """返回环境摘要字符串，节点启动日志里打印，方便复现问题。"""
    cuda = 'unknown'
    try:
        import torch  # ultralytics 的依赖，装了才有

        cuda = 'available ({})'.format(torch.cuda.get_device_name(0)) if torch.cuda.is_available() \
            else 'not available (CPU mode)'
    except Exception:
        cuda = 'torch not installed'
    return (
        'Python {} | executable={} | CUDA {} | CARLA {}'.format(
            python_version_string(), sys.executable, cuda, CARLA_VERSION
        )
    )


def main():
    """独立运行本模块时的自检入口：python -m carla_yolo_avoidance.carla_env

    地址用的是同一份 config/params.yaml，保证与 ROS 节点连的是同一个服务端。
    """
    print(_LINE)
    print('CARLA / YOLOv8 环境自检')
    print(_DASH)
    print('Python 版本      : {}'.format(python_version_string()))
    print('Python 版本标记  : {}'.format(python_tag()))
    print('解释器路径       : {}'.format(sys.executable))
    print('CARLA_ROOT       : {}'.format(os.environ.get('CARLA_ROOT', '(未设置，可忽略)')))
    print('PYTHONPATH       : {}'.format(os.environ.get('PYTHONPATH', '(未设置)')))
    print(_LINE)

    params_path = config.default_params_path()
    print('参数文件         : {}'.format(params_path))
    host, port = check_endpoint_or_exit(*config.load_carla_endpoint())
    print('CARLA 服务地址   : {}:{}（来自 config/params.yaml）'.format(host, port))

    carla = import_carla_or_exit()
    print('[OK] import carla 成功 -> {}'.format(getattr(carla, '__file__', '?')))

    YOLO = import_ultralytics_or_exit()
    print('[OK] import ultralytics.YOLO 成功 -> {}'.format(YOLO))

    client, world = connect_to_carla(carla, host, port)
    print('[OK] 已连接 CARLA {}:{}，当前地图 = {}'.format(host, port, world.get_map().name))
    print(_LINE)
    print('环境自检全部通过，可以 colcon build && ros2 launch。')
    print(_LINE)


if __name__ == '__main__':
    main()
