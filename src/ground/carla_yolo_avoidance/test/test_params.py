# -*- coding: utf-8 -*-
"""参数文件（config/params.yaml）的单元测试。

守护这条约定：**CARLA 的 IP / 端口不在 Python 代码里写死，
改 config/params.yaml 就能换地址**，其他人不需要改源码。

需要 PyYAML（ROS 2 Humble 自带 python3-yaml）；没装时本文件自动跳过。

运行方式（两种都行）：
    pytest src/carla_yolo_avoidance/test/test_params.py
    python src/carla_yolo_avoidance/test/test_params.py
"""

import os
import sys
import tempfile

_PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, _PACKAGE_ROOT)

from carla_yolo_avoidance import config  # noqa: E402

try:
    import yaml  # noqa: F401
    HAS_YAML = True
except ImportError:
    HAS_YAML = False


def test_address_not_hardcoded_in_code():
    """回归保护：地址常量不许再出现在代码里，只能来自 ROS 参数 / YAML。"""
    assert not hasattr(config, 'CARLA_HOST'), 'CARLA_HOST 不应再写死在 config.py 里'
    assert not hasattr(config, 'CARLA_PORT'), 'CARLA_PORT 不应再写死在 config.py 里'
    # 参数名以常量形式给出，节点用它调用 declare_parameter
    assert config.PARAM_CARLA_HOST == 'carla_host'
    assert config.PARAM_CARLA_PORT == 'carla_port'


def test_default_params_path():
    """参数文件路径由 __file__ 推导，指向 config/params.yaml 且真实存在。"""
    path = config.default_params_path()
    assert path.endswith(os.path.join('config', 'params.yaml'))
    assert os.path.isfile(path), '缺少文件：{}'.format(path)


def test_load_endpoint_from_shipped_yaml():
    """默认参数文件里能读出合法的 CARLA 地址。"""
    if not HAS_YAML:
        return          # 没装 PyYAML，跳过
    host, port = config.load_carla_endpoint()
    assert isinstance(host, str) and host.strip(), 'carla_host 不能为空'
    assert isinstance(port, int) and port > 0, 'carla_port 必须是正整数'


def test_edit_yaml_changes_endpoint():
    """只改 YAML 就能换地址（其他人不用碰 Python 源码）。"""
    if not HAS_YAML:
        return
    content = (
        '/carla_camera_node:\n'
        '  ros__parameters:\n'
        '    carla_host: "10.1.2.3"\n'
        '    carla_port: 3456\n'
    )
    directory = tempfile.mkdtemp(prefix='carla_params_')
    path = os.path.join(directory, 'params.yaml')
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(content)

    host, port = config.load_carla_endpoint(path)
    assert host == '10.1.2.3'
    assert port == 3456


def test_endpoint_tolerates_flat_yaml():
    """参数文件没按节点名分组时也能读出来（容错）。"""
    if not HAS_YAML:
        return
    content = 'carla_host: "sim-server"\ncarla_port: 2000\n'
    directory = tempfile.mkdtemp(prefix='carla_params_flat_')
    path = os.path.join(directory, 'params.yaml')
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(content)

    host, port = config.load_carla_endpoint(path)
    assert host == 'sim-server'
    assert port == 2000


def test_bad_port_is_reported_as_missing():
    """端口写成字符串等非法值时，按"缺参数"处理，由调用方给出友好提示。"""
    if not HAS_YAML:
        return
    content = (
        '/carla_camera_node:\n'
        '  ros__parameters:\n'
        '    carla_host: "127.0.0.1"\n'
        '    carla_port: "not-a-number"\n'
    )
    directory = tempfile.mkdtemp(prefix='carla_params_bad_')
    path = os.path.join(directory, 'params.yaml')
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(content)

    host, port = config.load_carla_endpoint(path)
    assert host == '127.0.0.1'
    assert port == 0


def test_missing_file_raises():
    """文件不存在时明确报错，而不是静默返回空地址。"""
    try:
        config.load_carla_endpoint(os.path.join(tempfile.gettempdir(), 'no_such_params.yaml'))
    except RuntimeError:
        pass
    else:
        raise AssertionError('参数文件不存在时应抛 RuntimeError')


if __name__ == '__main__':
    if not HAS_YAML:
        print('注意：没装 PyYAML，涉及 YAML 的用例会被跳过')
    failures = 0
    total = 0
    for _name in sorted(list(globals())):
        if _name.startswith('test_') and callable(globals()[_name]):
            total += 1
            try:
                globals()[_name]()
                print('PASS {}'.format(_name))
            except AssertionError as exc:
                failures += 1
                print('FAIL {} -> {}'.format(_name, exc))
    print('---- 通过 {} 个，失败 {} 个 ----'.format(total - failures, failures))
    sys.exit(1 if failures else 0)
