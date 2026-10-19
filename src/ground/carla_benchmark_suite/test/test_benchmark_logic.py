#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 作业综合整合与性能评价 —— 单元测试。

不依赖 CARLA、不依赖 ROS，验证：
  1. 指标计算正确（RMSE / MAE / AoS / 相关系数 / 方向一致率）
  2. 指标计算对空输入与单元素输入不崩
  3. AoS 衡量转向平滑度的语义正确
  4. **各功能包内 nn_models.py 副本一致性**（回归测试）

     `nn_models.py` 在每个功能包内各有一份副本（为了让每个包都能独立编译运行）。
     多份副本带来一个真实风险：修了 A 包的 bug 却忘了修 B 包，
     导致同一个算法在不同包里行为不一致。本测试把这一风险制度化卡住——
     若副本出现分歧就失败，提示同步。

     实例：作业四修复了 maxpool 反向传播与推理 tanh 两处缺陷后，
     性能评价包仍用旧副本，基准测试中端到端 MAE 一度高达 36.12（正常应 <0.3）。

  5. 基准评测套件能被导入且评测项齐全
  6. 调度器能正确定位兄弟功能包
  7. 指标对比图能正常导出

运行：  python3 test/test_benchmark_logic.py
"""

import glob
import hashlib
import os
import sys
import tempfile

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

import importlib.util  # noqa: E402

# 实现在 Python 包里；包根的 main.py 只是一层薄壳（课程要求入口叫 main.*）。
# 静态检查必须读**实现**那份，否则 `from ... import *` 不会带出下划线开头的函数。
_IMPL = os.path.join(_PKG, "carla_benchmark_suite", "main.py")

_spec = importlib.util.spec_from_file_location("bench_main", _IMPL)
bench = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(bench)


# ---------------------------------------------------------------- 指标
def test_compute_metrics_lateral_rmse():
    """横向误差 RMSE 应为平方均值开根。"""
    e = np.array([0.0, 3.0, 4.0])          # 平方和=25，均值=25/3
    m = bench.compute_metrics(lateral_err=e)
    expect = float(np.sqrt(25.0 / 3.0))
    assert abs(m["lateral_rmse"] - expect) < 1e-9, \
        f"RMSE 计算错误: {m['lateral_rmse']} != {expect}"
    assert abs(m["lateral_max"] - 4.0) < 1e-9


def test_compute_metrics_aos():
    """AoS = 相邻转向变化量的平均绝对值，衡量转向平滑度。"""
    smooth = np.array([0.1, 0.1, 0.1, 0.1])       # 无变化
    jerky = np.array([0.1, -0.1, 0.1, -0.1])      # 抖动
    ms = bench.compute_metrics(steer=smooth)
    mj = bench.compute_metrics(steer=jerky)
    assert abs(ms["aos"]) < 1e-12, f"平滑序列 AoS 应为 0，实际 {ms['aos']}"
    assert abs(mj["aos"] - 0.2) < 1e-9, f"抖动序列 AoS 应为 0.2，实际 {mj['aos']}"
    assert mj["aos"] > ms["aos"], "AoS 应能区分平滑与抖动"


def test_compute_metrics_regression():
    """回归指标：MAE / RMSE / 相关系数 / 方向一致率。"""
    y = np.array([-1.0, -0.5, 0.5, 1.0])
    p = np.array([-0.9, -0.4, 0.6, 0.9])
    m = bench.compute_metrics(pred=p, target=y)
    assert abs(m["reg_mae"] - 0.1) < 1e-9, f"MAE 错误: {m['reg_mae']}"
    assert abs(m["reg_rmse"] - 0.1) < 1e-9, f"RMSE 错误: {m['reg_rmse']}"
    assert m["reg_corr"] > 0.99, f"完全同向的序列相关系数应接近 1，实际 {m['reg_corr']}"
    assert abs(m["reg_sign_acc"] - 1.0) < 1e-9, f"方向应全对，实际 {m['reg_sign_acc']}"


def test_compute_metrics_handles_empty_and_single():
    """空输入与单元素输入不应抛异常（基准套件里可能出现零长度轨迹）。"""
    assert bench.compute_metrics() == {}
    m = bench.compute_metrics(steer=[0.5], speed=[3.0], lateral_err=[0.0], latency_ms=[1.0])
    assert m["aos"] == 0.0, "单元素序列的 AoS 应为 0"
    assert m["steer_mean"] == 0.5


def test_steer_saturation_ratio():
    """转向饱和比例：被打到 ±1 边界的比例。"""
    s = np.array([1.0, -1.0, 0.5, 0.0])
    m = bench.compute_metrics(steer=s)
    assert abs(m["steer_saturation_ratio"] - 0.5) < 1e-9, \
        f"饱和比例应为 0.5，实际 {m['steer_saturation_ratio']}"


# ---------------------------------------------------------------- 副本一致性
def test_sibling_nn_models_are_consistent():
    """各功能包内的 `nn_models.py` 副本必须保持一致。

    每个包自带一份副本是为了"独立可编译/可运行"，但多副本会导致
    "修了一个包、忘了另一个包"的缺陷。此测试在副本分歧时失败并给出差异提示。
    """
    ground = os.path.dirname(_PKG)
    pattern = os.path.join(ground, "carla_*", "carla_*", "nn_models.py")
    files = sorted(glob.glob(pattern))
    if len(files) < 2:
        # 只拿到本包（其他功能包未随本分支检出）时无法比较，跳过
        print("    （仅发现 %d 份副本，跳过一致性比较）" % len(files))
        return

    digests = {}
    for f in files:
        with open(f, "rb") as fh:
            digests[f] = hashlib.md5(fh.read()).hexdigest()

    uniq = set(digests.values())
    assert len(uniq) == 1, (
        "nn_models.py 副本出现分歧，请同步以下文件：\n  "
        + "\n  ".join(f"{os.path.relpath(f, ground)}  {d[:12]}" for f, d in digests.items())
        + "\n（历史教训：性能评价包用了旧副本，端到端 MAE 一度高达 36.12）")


def test_nn_models_has_tanh_and_argmax_pool():
    """本包的 nn_models.py 必须含两处关键修复（回归测试）。"""
    src_path = os.path.join(_PKG, "carla_benchmark_suite", "nn_models.py")
    with open(src_path, encoding="utf-8") as f:
        src = f.read()
    assert "np.tanh(fc_out)" in src, \
        "nn_models.py 缺少推理 tanh（会导致输出无界、MAE 高达数十）"
    assert "(xc == am)" in src or "argmax" in src, \
        "nn_models.py 的 maxpool 反向未按 argmax 回传（会导致网络学不动）"


# ---------------------------------------------------------------- 套件
def test_benchmark_registry_complete():
    """基准评测项注册表应包含三个可离线评测的模块。"""
    for key in ("perception", "navigation", "end_to_end"):
        assert key in bench.BENCHMARKS, f"缺少评测项 {key}"
    for key in ("control", "perception", "navigation", "end_to_end"):
        assert key in bench.MODULES, f"缺少模块登记 {key}"
    # 每个模块都要有功能包名、任务描述与说明
    for key, info in bench.MODULES.items():
        for field in ("package", "task", "desc"):
            assert info.get(field), f"模块 {key} 缺少字段 {field}"


def test_module_path_resolution():
    """调度器应能定位到磁盘上存在的兄弟功能包，缺失时返回 None 而非抛异常。"""
    p = bench._module_main_path("end_to_end")
    if p is not None:
        assert os.path.isfile(p), f"返回的路径不存在: {p}"
        assert p.endswith("main.py")
    assert bench._module_main_path("不存在的模块") is None


def test_render_metric_bars_exports_png():
    """指标对比图应能导出为有效 PNG。"""
    fake = {
        "perception": {"percept_acc": 0.995, "control_mse": 0.003, "lateral_rmse": 0.231},
        "navigation": {"plan_mse": 0.006, "known_ratio": 0.29, "occupied_cells": 1380,
                       "nav_min_dist": 1.458},
        "end_to_end": {"reg_mae": 0.094, "reg_sign_acc": 0.98},
    }
    with tempfile.TemporaryDirectory() as d:
        p = bench.render_metric_bars(fake, os.path.join(d, "m.png"))
        assert os.path.isfile(p), "对比图未生成"
        with open(p, "rb") as f:
            head = f.read(8)
        assert head == b"\x89PNG\r\n\x1a\n", "导出的不是有效 PNG"


def test_render_metric_bars_handles_empty():
    """全部评测项失败时，对比图应导出空白图而不是崩溃。"""
    with tempfile.TemporaryDirectory() as d:
        p = bench.render_metric_bars({}, os.path.join(d, "empty.png"))
        assert os.path.isfile(p), "空数据也应产出文件"


def test_perception_replay_heading_matches_route():
    """perception 基准的回放初始航向必须由轨迹首段推导（回归测试）。

    历史缺陷：`x, y, yaw, v = wps[0][0], wps[0][1], 0.0, 0.0` 把车头写死为
    +x，而 DEMO_ROUTE 首段指向 -x。回放先掉头再追线，横向误差被这段
    无效机动污染——实测 lateral_rmse 达 44.59 m、reached_goal 恒为 False，
    把"控制器完全失效"的假结论写进了评测报告。
    修复：由 wps[0]→wps[1] 方向推导，修后 lateral_rmse 0.219 m、reached_goal True。
    """
    src = open(_IMPL, encoding="utf-8").read()
    i_fn = src.index("def bench_perception(")
    seg = src[i_fn:i_fn + 3000]
    assert "yaw = math.atan2(wps[1][1] - wps[0][1], wps[1][0] - wps[0][0])" in seg, \
        "perception 回放未由轨迹首段推导初始航向"
    assert "wps[0][1], 0.0, 0.0" not in seg, "初始航向仍被写死为 0"


def test_navigation_replay_heading_faces_goal():
    """navigation 基准的回放初始航向必须朝目标（回归测试）。

    历史缺陷：`x, y, yaw, v = mod.START[0], mod.START[1], 0.0, 0.0`，
    而默认目标在起点左后方，回放开头先背离目标再掉头。
    """
    src = open(_IMPL, encoding="utf-8").read()
    i_fn = src.index("def bench_navigation(")
    seg = src[i_fn:i_fn + 4000]
    assert "yaw = math.atan2(mod.DEFAULT_GOAL[1] - mod.START[1]," in seg, \
        "navigation 回放未朝目标推导初始航向"
    assert "mod.START[1], 0.0, 0.0" not in seg, "初始航向仍被写死为 0"


def test_perception_time_budget_covers_route():
    """perception 基准的时间预算必须按轨迹长度推导（回归测试）。

    历史缺陷：回放固定 `range(int(40.0 / mod.DT))`，而 DEMO_ROUTE 长约 400 m、
    车速约 7 m/s，跑完全程需 55 s 以上。回放在离终点 60 多米处被截断，
    reached_goal 因此恒为 False——并非控制失败，而是预算不足造成的假结论。
    修复：按轨迹长度推导预算（5 m/s 保守估计，上下限 40~180 s）。
    """
    src = open(_IMPL, encoding="utf-8").read()
    i_fn = src.index("def bench_perception(")
    i_next = src.index("def bench_navigation(")
    seg = src[i_fn:i_next]          # 只截取 bench_perception 本体
    assert "route_len" in seg, "未按轨迹长度计算时间预算"
    assert "budget" in seg, "未使用推导出的时间预算"
    assert "range(int(40.0 / mod.DT))" not in seg, \
        "perception 回放仍固定为 40 s，无法跑完约 400 m 的轨迹"


def test_no_dead_carla_common():
    """本包不应携带未被引用的 carla_common.py（回归测试）。

    历史缺陷：包内有一份从未被任何代码 import 的 carla_common.py，
    且它保留着作业二修复前的缺陷（20 s 超时、出生朝向逆行）。
    本包自身不连接 CARLA（它评测兄弟包），留着这份死代码只会误导使用者。
    """
    p = os.path.join(_PKG, "carla_benchmark_suite", "carla_common.py")
    assert not os.path.isfile(p), \
        "包内仍存在无人引用的 carla_common.py（死代码）"
    # 全包范围内也不应再有对它的引用（测试文件自身会提到该名字，故跳过 test/）
    name = "carla_" + "common"      # 拼接以避免本文件被自己的检查命中
    for root, _dirs, files in os.walk(_PKG):
        if "__pycache__" in root or os.sep + "test" in root:
            continue
        for fn in files:
            if not fn.endswith((".py", ".launch", ".yaml", ".xml")):
                continue
            fp = os.path.join(root, fn)
            try:
                txt = open(fp, encoding="utf-8").read()
            except (OSError, UnicodeDecodeError):
                continue
            assert name not in txt, f"{fp} 仍引用已删除的 {name}"


# ------------------------------------------------- 入口可执行位与 launch 参数透传
def _pkg_root():
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_main_entries_are_executable():
    """main.* 必须有可执行位。

    launch/main.launch 里是 <node type="main.py"> —— roslaunch 会直接执行该
    文件并依赖首行 shebang；没有可执行位（实测曾为 100644）就会启动失败。
    """
    import subprocess
    names = ("main.py", "main.sh", "main.bat")
    modes = {}
    try:
        out = subprocess.check_output(
            ["git", "ls-files", "-s", "--"] + list(names),
            cwd=_pkg_root(), stderr=subprocess.DEVNULL).decode()
        for line in out.splitlines():
            mode, _sha, _stage, path = line.split(None, 3)
            modes[os.path.basename(path.strip())] = mode
    except Exception:  # noqa: BLE001  非 git 检出（如 colcon 安装目录）时退回
        for name in names:
            assert os.access(os.path.join(_pkg_root(), name), os.X_OK), \
                f"{name} 缺少可执行位"
        return
    assert modes, "git ls-files 没有返回 main.* 的信息"
    for name in names:
        assert modes.get(name, "") == "100755", (
            f"{name} 的 git 模式是 {modes.get(name)}，应为 100755；"
            "请执行 git update-index --chmod=+x <文件>")


def _parse_bool_from_source():
    """从 main.py 取出 _parse_bool 来测，避免 import main 拉起 CARLA 依赖。"""
    import ast
    import argparse as _ap
    src = open(_IMPL, encoding="utf-8").read()
    tree = ast.parse(src)
    fn = next(n for n in tree.body
              if isinstance(n, ast.FunctionDef) and n.name == "_parse_bool")
    ns = {"argparse": _ap}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), "<main.py>", "exec"), ns)
    return ns["_parse_bool"]


def test_launch_bool_args_accept_text_values():
    """roslaunch 只能按文本透传参数，布尔开关必须接受 true / false。

    否则 `--follow false` 会被 store_true 置为 True，那个多余的 false 又被
    parse_known_args 当未知位置参数丢掉 —— follow:=false 反而生效为 true。
    """
    import argparse
    parse_bool = _parse_bool_from_source()

    def parse(argv):
        p = argparse.ArgumentParser()
        p.add_argument("--follow", type=parse_bool, nargs="?",
                       const=True, default=False)
        p.add_argument("--headless", type=parse_bool, nargs="?",
                       const=True, default=False)
        p.add_argument("--save_dir", nargs="?", const=None, default=None)
        a, _unknown = p.parse_known_args(argv)
        return a.follow, a.headless, a.save_dir

    assert parse(["--follow", "false", "--headless", "false",
                  "--save_dir"]) == (False, False, None), \
        "launch 传来的 false 应解析为 False，空 save_dir 应解析为 None"
    assert parse(["--follow", "true", "--headless", "true"]) == (True, True, None)
    assert parse(["--follow", "--headless"]) == (True, True, None), \
        "裸开关（命令行习惯写法）仍应可用"

    src = open(_IMPL, encoding="utf-8").read()
    assert "_add_bool(" in src, "main.py 应通过 _add_bool 注册布尔开关"
    assert 'action="store_true"' not in src, (
        'main.py 仍有 action="store_true"：launch 传来的 false 会被丢掉')


def test_empty_optional_value_does_not_abort():
    """$(arg save_dir) 为空时会剩一个光秃秃的 --save_dir。

    argparse 原会以 "expected one argument" 直接退出，导致 launch 起不来。
    """
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--save_dir", nargs="?", const=None, default=None)
    a, _unknown = p.parse_known_args(["--save_dir"])
    assert a.save_dir is None


def test_main_launch_passes_declared_args_to_node():
    """main.launch 声明的每个 <arg> 都必须被节点用到。

    漏传会让 follow:=true 之类的设置不起作用。
    """
    import re
    text = open(os.path.join(_pkg_root(), "launch", "main.launch"),
                encoding="utf-8").read()
    declared = set(re.findall(r'<arg\s+name="([^"]+)"', text))
    nodes = re.findall(r"<node\b.*?/>", text, re.S)
    assert nodes, "main.launch 里没有找到 <node .../>"
    passed = set()
    for n in nodes:
        passed |= set(re.findall(r"\$\(arg\s+([^)]+)\)", n))
    missing = sorted(declared - passed)
    assert not missing, f"这些已声明的 arg 没有透传给节点：{missing}"


def test_main_sh_launch_dispatches_per_ros_version():
    """main.sh 的 --launch 分支要按 ROS 版本分别调 ros2 launch / roslaunch。

    原写法 `exec ros2 launch <pkg> main.launch.py "${@/--launch/}"` 有两个坑：
      1) 只传 --launch 时，${@/--launch/} 会留下一个**空字符串**参数，
         `ros2 launch <pkg> main.launch.py ""` 会以 Invalid launch argument 失败；
      2) 不管装的是 ROS 1 还是 ROS 2 都调 ros2，Noetic 上直接 command not found。
    """
    sh = open(os.path.join(_pkg_root(), "main.sh"), encoding="utf-8").read()
    assert "${@/--launch/}" not in sh, \
        "旧的 --launch 展开写法会留下空参数，应改成收集到数组再展开"
    assert "ros2 launch" in sh, "main.sh 应支持 ROS 2 的 ros2 launch"
    assert "roslaunch" in sh, "main.sh 应支持 ROS 1 的 roslaunch"
    assert "ROS_VERSION" in sh, "应按 ROS_VERSION 选择 launch 命令"
    assert "launch_args" in sh, "其余参数应原样透传给 launch"


def _lzw_decode(blob, min_code, npix):
    """极简 GIF LZW 解码器，只用来验证我们自己编码器的输出。

    码长增长用 `==`：解码端每读一个码才建一条表项、且第一个码不建表，所以比
    编码端慢一条（编码端对应写 `>`）。两边不共用同一套假设，才能真判错。
    """
    clear = 1 << min_code
    end = clear + 1
    state = {"bits": 0, "code_size": min_code + 1}
    table = {i: [i] for i in range(clear)}
    next_code = end + 1
    total = len(blob) * 8

    def read():
        v = 0
        for k in range(state["code_size"]):
            p = state["bits"] + k
            v |= ((blob[p >> 3] >> (p & 7)) & 1) << k
        state["bits"] += state["code_size"]
        return v

    out = []
    prev = None
    while state["bits"] + state["code_size"] <= total:
        code = read()
        if code == clear:
            table = {i: [i] for i in range(clear)}
            next_code = end + 1
            state["code_size"] = min_code + 1
            prev = None
            continue
        if code == end:
            break
        if code in table:
            entry = table[code]
        elif prev is not None:
            entry = prev + [prev[0]]
        else:
            raise AssertionError(f"非法码 {code}")
        out.extend(entry)
        if prev is not None and next_code < 4096:
            table[next_code] = prev + [entry[0]]
            next_code += 1
            if next_code == (1 << state["code_size"]) and state["code_size"] < 12:
                state["code_size"] += 1
        prev = entry
    return np.array(out[:npix], dtype=np.uint8)


def _gif_read(path):
    """读回我们自己写的 GIF：返回 (调色板, [每帧索引数组], (宽, 高))。"""
    import struct

    data = open(path, "rb").read()
    assert data[:6] in (b"GIF87a", b"GIF89a"), "文件头不是 GIF"
    w, h, packed = struct.unpack("<HHB", data[6:11])
    pos = 13
    pal = []
    if packed & 0x80:
        ncol = 1 << ((packed & 0x07) + 1)
        pal = [tuple(data[pos + 3 * i: pos + 3 * i + 3]) for i in range(ncol)]
        pos += 3 * ncol
    frames = []
    while pos < len(data):
        tag = data[pos]
        if tag == 0x3B:
            break
        if tag == 0x21:
            pos += 2
            while data[pos]:
                pos += data[pos] + 1
            pos += 1
            continue
        assert tag == 0x2C, f"期望图像描述符，遇到 {tag:#x}"
        _l, _t, fw, fh, ipacked = struct.unpack("<HHHHB", data[pos + 1: pos + 10])
        pos += 10
        if ipacked & 0x80:
            pos += 3 * (1 << ((ipacked & 0x07) + 1))
        min_code = data[pos]
        pos += 1
        blob = bytearray()
        while data[pos]:
            n = data[pos]
            blob += data[pos + 1: pos + 1 + n]
            pos += n + 1
        pos += 1
        frames.append(_lzw_decode(blob, min_code, fw * fh).reshape(fh, fw))
    return pal, frames, (w, h)


def test_gif_writer_roundtrip():
    """GIF 编码必须能被独立解码器原样读回（像素级一致）。

    文档要求放 gif 动图，而本模块是纯 numpy 的、不装 Pillow，所以编码器是自己写的。
    解码器按 GIF 规范实现（码长增长用 `==`，与编码端 `>` 相差一位），能独立判错。
    该编码器写出的文件另外用 Pillow 验证过（纯色 / 渐长 / 噪声 / 两色 / 256 色 / RGB 自动调色板）。
    """
    from carla_benchmark_suite.gif_writer import write_gif

    frames = []
    for i in range(5):
        f = np.zeros((24, 32), dtype=np.uint8)
        f[:, : max(1, i * 6)] = 1
        f[6:12, 3:3 + max(1, i * 5)] = 2
        frames.append(f)
    palette = [(255, 255, 255), (66, 133, 244), (219, 68, 55)]

    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, "t.gif")
        write_gif(p, frames, palette, delay_cs=20)
        pal, got, size = _gif_read(p)

    assert size == (32, 24), f"尺寸应保持 (32,24)，实际 {size}"
    assert len(got) == len(frames), f"帧数应 {len(frames)}，实际 {len(got)}"
    for i, (a, b) in enumerate(zip(got, frames)):
        assert np.array_equal(a, b), f"第 {i} 帧像素不一致"
    assert [tuple(c) for c in pal[:len(palette)]] == \
        [tuple(c) for c in palette], "调色板被改动了"


def test_metric_bars_canvas_grows_with_progress():
    """对比图动图的每一帧必须颜色数 ≤256，且 progress 越大条越长。"""
    results = {"perception": {"percept_acc": 0.99, "control_mse": 0.0035, "lateral_rmse": 0.22},
               "navigation": {"plan_mse": 0.0079, "known_ratio": 0.265, "nav_min_dist": 1.42},
               "end_to_end": {"reg_mae": 0.144, "reg_sign_acc": 0.93}}
    small = bench.metric_bars_canvas(results, progress=0.2)
    full = bench.metric_bars_canvas(results, progress=1.0)
    assert small.shape == full.shape, "两帧尺寸应一致"
    for f in (small, full):
        n = len({tuple(c) for c in f.reshape(-1, 3)})
        assert n <= 256, f"一帧用了 {n} 种颜色，GIF 索引化不了"
    # 条形区域里"非白"像素应随 progress 增多
    small_ink = int((small != 255).any(axis=2).sum())
    full_ink = int((full != 255).any(axis=2).sum())
    assert full_ink > small_ink, f"条长没随 progress 增长: {small_ink} -> {full_ink}"


def test_node_loads_benchmark_from_inside_package():
    """回归：节点必须能通过**包内绝对导入**拿到评测实现。

    早期 `benchmark_node.py` 用 `spec_from_file_location` 按路径加载功能包根目录的
    `main.py`。但 `setup.py` 只声明 `packages=[package_name]`，包根目录下的文件
    根本不会被安装，`ros2 launch` 之后加载必然失败、指标全空。
    现在实现放在 `carla_benchmark_suite/main.py`，节点用
    `from carla_benchmark_suite import main` 导入，本用例把这个约定钉住。
    """
    impl = os.path.join(_PKG, "carla_benchmark_suite", "main.py")
    assert os.path.isfile(impl), f"实现文件不存在: {impl}"

    node_txt = open(os.path.join(_PKG, "carla_benchmark_suite", "benchmark_node.py"),
                    encoding="utf-8").read()
    assert "from carla_benchmark_suite import main" in node_txt, \
        "benchmark_node.py 应通过包内绝对导入拿实现"
    assert "spec_from_file_location" not in node_txt, \
        "benchmark_node.py 不应再按文件路径加载 main.py（包根目录不会被安装）"

    # 包根那份 main.py 必须还在（课程要求入口叫 main.*），并且是转发用的薄壳
    shim = os.path.join(_PKG, "main.py")
    assert os.path.isfile(shim), "包根 main.py（课程约定入口）丢了"
    assert "from carla_benchmark_suite.main import" in open(shim, encoding="utf-8").read(), \
        "包根 main.py 应是转发到包内实现的薄壳"


def _run_all():
    fns = sorted(k for k in list(globals()) if k.startswith("test_"))
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
