# -*- coding: utf-8 -*-
"""test_nodes_ros.py —— 用 ROS 接口替身把三个节点按真实话题跑一遍。

这不是「替代在 Ubuntu 上 ros2 launch」，而是在**没有 ROS 的机器**上把能在本地
查出来的问题全部查掉：消息字段名、参数声明、话题类型、install 后的路径、
launch 传参、闭环是否收敛。真正的 rclpy 行为差异（executor 调度、QoS、
message 序列化）仍然只能在装了 ROS 的机器上确认。

跑法：
    python3 -m pytest test/ -q
本机已装真实 rclpy 时本文件自动跳过。
"""
import ast
import csv
import glob
import importlib.util
import io
import os
import shutil
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

pyyaml = pytest.importorskip('yaml', reason='需要 pyyaml 解析 config/params.yaml')

try:
    import fake_ros
    fake_ros.install()
    _FAKE_OK = True
except Exception as exc:                       # 本机有真 rclpy / 替身文件缺失
    fake_ros = None
    _FAKE_OK = False
    _SKIP_REASON = str(exc)

if _FAKE_OK:
    _SKIP_REASON = ''

pytestmark = pytest.mark.skipif(
    not _FAKE_OK,
    reason='ROS 替身不可用（%s）' % (_SKIP_REASON if not _FAKE_OK else ''))


# ----------------------------------------------------------------------
# 工具
# ----------------------------------------------------------------------
def _boot(share_root=None):
    """每个用例一份干净的总线 + 已 init 的上下文。"""
    fake_ros.reset()
    fake_ros.set_share_root(share_root)
    import rclpy
    rclpy.init()


def _install_tree(tmp_path):
    """按 setup.py 的 data_files 复刻一份 colcon install 之后的 share 目录。"""
    share = tmp_path / 'install' / 'uav_camera_tracking' / 'share' / 'uav_camera_tracking'
    for sub in ('models', 'config', 'launch'):
        (share / sub).mkdir(parents=True, exist_ok=True)
    for name in ('detector.npz', 'control.npz', 'e2e.npz'):
        src = os.path.join(PKG_ROOT, 'models', name)
        if os.path.isfile(src):
            shutil.copy2(src, str(share / 'models' / name))
    for name in ('params.yaml',):
        shutil.copy2(os.path.join(PKG_ROOT, 'config', name),
                     str(share / 'config' / name))
    shutil.copy2(os.path.join(PKG_ROOT, 'launch', 'main.launch.py'),
                 str(share / 'launch' / 'main.launch.py'))
    # get_package_share_directory 的返回值是 <ws>/install/share/<pkg>，
    # 所以「假 share 根」就是它的上一级 <ws>/install/share
    return str(share), str(share.parents[0])


def _load_yaml_params():
    with io.open(os.path.join(PKG_ROOT, 'config', 'params.yaml'), encoding='utf-8') as f:
        return pyyaml.safe_load(f)


def _import_launch_module():
    path = os.path.join(PKG_ROOT, 'launch', 'main.launch.py')
    spec = importlib.util.spec_from_file_location('uav_main_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# ----------------------------------------------------------------------
# 1. 参数文件与节点声明必须一一对上
# ----------------------------------------------------------------------
def test_params_yaml_keys_are_all_declared():
    """YAML 里写错参数名时真实 ROS 会静默忽略——这里必须炸出来。"""
    raw = _load_yaml_params()
    overrides = {name: body['ros__parameters'] for name, body in raw.items()}

    _boot()
    from uav_camera_tracking.controller_node import ControllerNode
    from uav_camera_tracking.detector_node import DetectorNode
    from uav_camera_tracking.sim_node import SimNode

    unused = {}
    for cls in (SimNode, DetectorNode, ControllerNode):
        fake_ros.set_param_overrides(overrides)
        node = cls()
        unused[node.node_name] = node.unused_overrides()

    assert set(overrides) == {'uav_sim', 'uav_detector', 'uav_controller'}, \
        'params.yaml 的节点名必须与 launch 里的 name= 一致'
    assert unused == {'uav_sim': [], 'uav_detector': [], 'uav_controller': []}, \
        '以下参数节点根本没有声明（真实 ROS 会静默忽略）：%r' % unused


# ----------------------------------------------------------------------
# 2. 三节点闭环：图像 -> 检测 -> 指令 -> 动力学
# ----------------------------------------------------------------------
def _run_closed_loop(tmp_path, scene_key, mode, duration=None):
    share, share_parent = _install_tree(tmp_path)
    fake_ros.reset()
    fake_ros.set_share_root(share_parent)
    import rclpy
    rclpy.init()

    from uav_camera_tracking import params as P
    from uav_camera_tracking.controller_node import ControllerNode
    from uav_camera_tracking.detector_node import DetectorNode
    from uav_camera_tracking.sim_node import SimNode

    sim_params = {'scene': scene_key, 'csv': str(tmp_path / 'metrics.csv')}
    if duration is not None:
        sim_params['duration'] = float(duration)
    fake_ros.set_param_overrides({
        'uav_sim': sim_params,
        'uav_controller': {'mode': mode},
    })
    sim = SimNode()
    det = DetectorNode()
    ctrl = ControllerNode()

    rig = fake_ros.Rig(dt=P.DT)
    rig.run(steps=int(sim.steps_total) + 20, stop_when=lambda: sim.done)

    with io.open(tmp_path / 'metrics.csv', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    return sim, det, ctrl, rows[-1]


def test_two_stage_closed_loop_on_moving_scene(tmp_path):
    """两阶段：ROS 侧的误差必须与离线 `sim.rollout` 同一量级（都是 0.1x）。"""
    sim, det, ctrl, row = _run_closed_loop(tmp_path, 'moving', 'two_stage')

    assert sim.done, '仿真节点没跑完回合'
    assert det.pub.count > 100, '感知节点没有持续出检测结果'
    assert ctrl.pub.count >= sim.steps_total - 5, '控制节点没有按周期发指令'
    assert sim.published_on('/uav/image') == sim.steps_total
    assert sim.published_on('/uav/odom') == sim.steps_total

    assert row['scene'] == 'moving'
    assert row['success'] == '1', '闭环未通过：%r' % row
    err = float(row['err_rms'])
    assert err < 0.20, '闭环误差 RMS %.4f 偏大（离线结果是 0.136）' % err


def test_expert_mode_closed_loop_short_run(tmp_path):
    """专家模式不加载任何权重，验证「不依赖训练产物也能跑」。"""
    sim, det, ctrl, row = _run_closed_loop(tmp_path, 'static', 'expert',
                                           duration=10.0)
    assert row['success'] == '1', row
    assert float(row['err_rms']) < 0.15
    assert ctrl.control is None and ctrl.e2e is None


def test_missing_weight_gives_actionable_error(tmp_path):
    _boot()
    from uav_camera_tracking.detector_node import DetectorNode
    fake_ros.set_param_overrides(
        {'uav_detector': {'model': str(tmp_path / 'not_trained.npz')}})
    with pytest.raises(FileNotFoundError) as info:
        DetectorNode()
    assert '请先训练' in str(info.value)


# ----------------------------------------------------------------------
# 3. launch 必须真正把参数传给三个节点
# ----------------------------------------------------------------------
def test_launch_builds_three_nodes_with_install_paths(tmp_path):
    share, share_parent = _install_tree(tmp_path)
    fake_ros.reset()
    fake_ros.set_share_root(share_parent)

    module = _import_launch_module()
    desc = module.generate_launch_description()

    context, nodes = fake_ros.expand_launch(
        desc, {'scene': 'occlusion', 'mode': 'expert', 'duration': '8'})
    assert context['scene'] == 'occlusion'

    by_exec = {n.kwargs['executable']: n.kwargs for n in nodes}
    assert set(by_exec) == {'sim', 'detector', 'controller'}

    params_path = by_exec['sim']['parameters'][0]
    assert os.path.isfile(params_path), 'launch 引用的 params.yaml 不存在：%s' % params_path
    assert params_path.startswith(share), 'params.yaml 应取自 install 后的 share 目录'

    sim_override = by_exec['sim']['parameters'][1]
    assert sim_override['scene'] == 'occlusion'
    assert isinstance(sim_override['duration'], float), \
        'duration 必须显式转成 float，否则节点声明 double 参数时类型不匹配'
    assert sim_override['duration'] == 8.0
    assert by_exec['controller']['parameters'][1]['mode'] == 'expert'


def test_setup_py_data_files_and_entry_points_are_consistent():
    """install 之后缺文件 / 入口点写错，是评审最常挑的两类问题。"""
    with io.open(os.path.join(PKG_ROOT, 'setup.py'), encoding='utf-8') as f:
        tree = ast.parse(f.read())

    setup_call = None
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, 'id', '') == 'setup':
            setup_call = node
    assert setup_call is not None, 'setup.py 里没有 setup() 调用'

    pkg_name = os.path.basename(PKG_ROOT)
    data_files = entries = None
    for kw in setup_call.keywords:
        if kw.arg == 'name' and isinstance(kw.value, ast.Constant):
            pkg_name = kw.value.value
        elif kw.arg == 'data_files':
            data_files = kw.value
        elif kw.arg == 'entry_points':
            entries = kw.value
    assert data_files is not None, 'setup.py 里没有 data_files'
    assert entries is not None, 'setup.py 里没有 entry_points'

    # --- data_files：glob 不能落空，字面量文件必须存在 ---
    checked = 0
    for tup in data_files.elts:
        dest = _eval_str(tup.elts[0], pkg_name)
        spec = tup.elts[1]
        if isinstance(spec, ast.Call):                       # glob(...)
            pattern = spec.args[0].value
            hits = glob.glob(os.path.join(PKG_ROOT, pattern))
            assert hits, 'data_files 模式 %r（安装到 %s）匹配不到任何文件' % (pattern, dest)
            checked += len(hits)
        else:                                                # ['resource/<pkg>'] 等
            for item in spec.elts:
                rel = _eval_str(item, pkg_name)
                assert os.path.exists(os.path.join(PKG_ROOT, rel)), \
                    'data_files 要安装的 %r 不存在（%s）' % (rel, dest)
                checked += 1
    assert checked >= 6, '只检查到 %d 个待安装文件，setup.py 结构可能变了' % checked

    # --- entry_points：模块文件与 main 函数必须真的存在 ---
    for node in ast.walk(entries):
        if isinstance(node, ast.Constant) and isinstance(node.value, str) \
                and ':' in node.value:
            alias, target = node.value.split('=', 1)
            alias, target = alias.strip(), target.strip()
            module_path, func = target.split(':')
            rel = module_path.replace('.', os.sep) + '.py'
            path = os.path.join(PKG_ROOT, rel)
            assert os.path.isfile(path), \
                '入口点 %s 指向的模块 %r 不存在' % (alias, module_path)
            with io.open(path, encoding='utf-8') as f:
                body = ast.parse(f.read())
            funcs = {n.name for n in body.body if isinstance(n, ast.FunctionDef)}
            assert func in funcs, '入口点 %s 指向 %s.%s，但该模块里没有 %s()' % (
                alias, module_path, func, func)


def _eval_str(node, pkg_name):
    """把 data_files 里 'share/' + package_name + '/models' 这类拼接求值出来。"""
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.Name) and node.id == 'package_name':
        return pkg_name
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        return _eval_str(node.left, pkg_name) + _eval_str(node.right, pkg_name)
    raise AssertionError('无法解析的 data_files 目标表达式：%s' % ast.dump(node))


def setattr_nested(msg, path, value):
    parts = path.split('.')
    for part in parts[:-1]:
        msg = getattr(msg, part)
    setattr(msg, parts[-1], value)
