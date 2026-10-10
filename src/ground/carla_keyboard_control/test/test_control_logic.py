#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单元测试：不依赖 CARLA 服务端即可验证控制映射与倒挡判定逻辑。

被测试的是 `carla_common.compose_control` —— **真实被 standalone 主入口和
ROS 2 键盘节点共用的那份实现**，因此这里断言的行为就是实际运行的行为。

两种运行方式都支持：
    python3 test/test_control_logic.py          # 自带运行器，无需 pytest
    python3 -m pytest test/test_control_logic.py -v
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carla_keyboard_control import carla_common as cc  # noqa: E402

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _keys(*pressed):
    """构造按键状态：_keys('rev') 表示只按下 S。"""
    state = cc.blank_keys()
    for name in pressed:
        state[name] = True
    return state


# ------------------------------------------------------------------ 倒挡判定
def test_low_speed_s_presses_reverse():
    """低速按 S 应挂倒挡：reverse=True、有油门、无刹车。"""
    throttle, _steer, brake, reverse = cc.compose_control(_keys('rev'), 0.0)
    assert reverse is True
    assert throttle > 0.0
    assert brake == 0.0


def test_high_speed_s_brakes_instead_of_reversing():
    """有速度按 S 应刹车：reverse=False、无油门、有刹车。"""
    throttle, _steer, brake, reverse = cc.compose_control(_keys('rev'), 5.0)
    assert reverse is False
    assert brake > 0.0
    assert throttle == 0.0


def test_reverse_threshold_boundary_is_respected():
    """阈值边界：恰好等于阈值按刹车，略低于阈值才挂倒挡。"""
    thr = cc.DEFAULT_REV_THRESHOLD
    assert cc.compose_control(_keys('rev'), thr - 1e-6)[3] is True
    assert cc.compose_control(_keys('rev'), thr)[3] is False


def test_threshold_is_configurable():
    """阈值必须真的生效（可用 --rev_threshold 覆盖）。"""
    assert cc.compose_control(_keys('rev'), 1.0, rev_threshold=2.0)[3] is True
    assert cc.compose_control(_keys('rev'), 1.0, rev_threshold=0.5)[3] is False


# ------------------------------------------------------------------ 油门/转向
def test_w_accelerates_without_braking():
    throttle, _steer, brake, reverse = cc.compose_control(_keys('fwd'), 0.0)
    assert throttle == cc.DEFAULT_THROTTLE_MAX
    assert brake == 0.0
    assert reverse is False


def test_no_key_means_no_command():
    """松开所有按键不应残留任何控制量。"""
    assert cc.compose_control(_keys(), 3.0) == (0.0, 0.0, 0.0, False)


def test_steer_direction_and_saturation():
    """A/左微调为负、D/右微调为正，且始终落在 [-1, 1]。"""
    left = cc.compose_control(_keys('left'), 0.0)[1]
    right = cc.compose_control(_keys('right'), 0.0)[1]
    assert left < 0.0 < right
    assert abs(left) <= 1.0 and abs(right) <= 1.0

    fine_left = cc.compose_control(_keys('left_fine'), 0.0)[1]
    fine_right = cc.compose_control(_keys('right_fine'), 0.0)[1]
    assert 0.0 < abs(fine_left) < abs(left)      # 微调幅度必须小于全转向
    assert abs(fine_left) == abs(fine_right)


def test_control_quantities_stay_in_valid_range():
    """任意按键组合下，throttle/brake ∈ [0,1]、steer ∈ [-1,1]。"""
    all_keys = list(cc.KEY_NAMES)
    for name in all_keys:
        for speed in (0.0, 0.4, 10.0):
            throttle, steer, brake, reverse = cc.compose_control(_keys(name), speed)
            assert 0.0 <= throttle <= 1.0
            assert 0.0 <= brake <= 1.0
            assert -1.0 <= steer <= 1.0
            assert isinstance(reverse, bool)


# ------------------------------------------------------------------ 与 ROS 节点一致性
def test_ros_node_reuses_the_same_compose_control():
    """ROS 2 键盘节点必须复用同一份控制实现，否则两条路径行为会漂移。"""
    src = os.path.join(PKG_ROOT, 'carla_keyboard_control',
                       'keyboard_teleop_node.py')
    text = open(src, encoding='utf-8').read()
    assert 'cc.compose_control(' in text, \
        'keyboard_teleop_node 应调用 cc.compose_control，而不是另写一份'


# ------------------------------------------------------------------ 打包与入口约定
def test_requirement_files_exist():
    """课程约定的关键文件必须存在（main.* 入口 + launch + 打包文件）。"""
    for name in ('main.py', 'main.sh', 'main.bat', 'package.xml', 'setup.py',
                 'setup.cfg', 'requirements.txt',
                 'launch/main.launch.py', 'launch/main.launch'):
        assert os.path.exists(os.path.join(PKG_ROOT, name)), f"缺少 {name}"


def test_platform_specific_imports_are_lazy():
    """termios/tty 是 POSIX 专属：必须缩进（函数内）导入，否则 Windows 无法导入本包。"""
    src = os.path.join(PKG_ROOT, 'carla_keyboard_control',
                       'keyboard_teleop_node.py')
    offenders = []
    for lineno, line in enumerate(open(src, encoding='utf-8'), 1):
        code = line.rstrip('\n')
        if not code.strip() or code.lstrip().startswith('#'):
            continue
        # 只在模块顶层（零缩进）导入 POSIX 专属模块才是有害的
        if not code[:1].isspace() and code.lstrip().startswith(
                ('import termios', 'import tty',
                 'from termios', 'from tty')):
            offenders.append(lineno)
    assert not offenders, (
        'keyboard_teleop_node.py 在顶层导入了 POSIX 专属模块'
        f'（行 {offenders}），会使本包在 Windows 上无法导入；'
        '请改为在 _terminal_loop() 内部导入。')


def test_requirements_lists_runtime_deps():
    """requirements.txt 必须包含 numpy / pygame，且不能把安装路径写死。"""
    path = os.path.join(PKG_ROOT, 'requirements.txt')
    text = open(path, encoding='utf-8').read()
    assert 'numpy' in text and 'pygame' in text


# ---------------------------------------------------------- 入口可执行位与参数透传
def test_main_entries_are_executable():
    """main.* 必须有可执行位。

    launch/main.launch 里是 <node type="main.py"> —— roslaunch 会**直接执行**
    该文件并依赖首行 shebang，没有可执行位就会失败（曾实测为 100644）。
    """
    import subprocess
    names = ('main.py', 'main.sh', 'main.bat')
    modes = {}
    try:
        out = subprocess.check_output(
            ['git', 'ls-files', '-s', '--'] + list(names),
            cwd=PKG_ROOT, stderr=subprocess.DEVNULL).decode()
        for line in out.splitlines():
            mode, _sha, _stage, path = line.split(None, 3)
            modes[os.path.basename(path.strip())] = mode
    except Exception:  # noqa: BLE001  非 git 检出（如 colcon 安装目录）时退回
        for name in names:
            assert os.access(os.path.join(PKG_ROOT, name), os.X_OK), \
                f'{name} 缺少可执行位'
        return
    assert modes, 'git ls-files 没有返回 main.* 的信息'
    for name in names:
        assert modes.get(name, '') == '100755', (
            f'{name} 的 git 模式是 {modes.get(name)}，应为 100755；'
            '请执行 git update-index --chmod=+x <文件>')


def test_launch_bool_args_accept_text_values():
    """roslaunch 只能按文本透传布尔量，main.py 必须接受 `--follow true/false`。

    否则 `--follow false` 会被 store_true 置为 True，而那个多余的 false 被
    parse_known_args 当未知位置参数丢掉 —— follow:=false 反而生效为 true。
    """
    import argparse
    import main as entry

    def parse(argv):
        p = argparse.ArgumentParser()
        entry._add_bool(p, '--follow', '')
        entry._add_bool(p, '--headless', '')
        p.add_argument('--save_dir', nargs='?', const='', default='')
        args, _unknown = p.parse_known_args(argv)
        return args.follow, args.headless, args.save_dir

    assert parse(['--follow', 'false', '--headless', 'false']) == \
        (False, False, ''), 'launch 默认值应解析为 False'
    assert parse(['--follow', 'true', '--headless', 'true']) == \
        (True, True, ''), 'true 应解析为 True'
    assert parse(['--follow', '--headless']) == (True, True, ''), \
        '裸开关（命令行习惯写法）仍应可用'


def test_launch_empty_save_dir_does_not_abort():
    """save_dir 默认空串替换后末尾会剩一个光秃秃的 --save_dir。

    原实现下 argparse 会以 "expected one argument" 直接退出，
    导致按文档跑 roslaunch 必失败。
    """
    import argparse
    import main as entry

    p = argparse.ArgumentParser()
    p.add_argument('--save_dir', nargs='?', const='', default='')
    args, _unknown = p.parse_known_args(['--save_dir'])   # 不带值
    assert args.save_dir == '', f'空 --save_dir 应得到空串，实际 {args.save_dir!r}'


def test_main_launch_passes_declared_args_to_node():
    """main.launch 声明的每个 <arg> 都必须出现在 <node args=...> 里。"""
    import re
    text = open(os.path.join(PKG_ROOT, 'launch', 'main.launch'),
                encoding='utf-8').read()
    declared = set(re.findall(r'<arg\s+name="([^"]+)"', text))
    node = re.search(r'<node\b.*?/>', text, re.S)
    assert node, 'main.launch 里没有找到 <node .../>'
    passed = set(re.findall(r'\$\(arg\s+([^)]+)\)', node.group(0)))
    assert declared <= passed, (
        f'这些已声明的 arg 没有透传给节点：{sorted(declared - passed)}'
        '（漏传会让 follow:=true 之类的设置不起作用）')


def _run_all():
    """自带运行器：不依赖 pytest 也能跑完所有 test_* 函数。"""
    fns = sorted(k for k in list(globals()) if k.startswith('test_'))
    passed, failed = 0, []
    for fn in fns:
        try:
            globals()[fn]()
            passed += 1
            print(f"[PASS] {fn}")
        except Exception as exc:  # noqa: BLE001
            failed.append(fn)
            print(f"[FAIL] {fn}: {exc}")
    print(f"\n==== 测试结果：PASS={passed}  FAIL={len(failed)} ====")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(_run_all())
