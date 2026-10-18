#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 建图 + 导航（神经网络规划）—— 单元测试。

不依赖 CARLA 服务端、不依赖 ROS，验证建图与规划逻辑：
  1. 占用栅格以起点为中心，车辆出生点必须落在栅格内（回归测试：早期版本越界）
  2. 世界坐标 ↔ 栅格索引互转自洽
  3. 贝叶斯对数几率更新方向正确（命中→占据，穿过→空闲）
  4. 建图函数能标记占据与空闲格
  5. 状态向量的所有维度都归一化到相近尺度
  6. 规划标签满足：无障碍时转向由航向差决定、左侧障碍多则右转
  7. 规划神经网络能收敛
  8. 模型保存/加载往返一致
  9. 离线取证模式跑通并导出地图
  10. 目标解析：命令行 / ROS 1 / ROS 2 三种 "x,y" 写法统一，非法输入回退默认值
  11. 文档里出现的 launch 参数都必须在 launch 文件里真的声明过（回归测试：
      ROS 2 的 main.launch.py 一度漏声明 goal，导致 goal:="x,y" 被静默忽略）
  12. 状态第 5 维是最近障碍方向角，不是第 4 维（距离）的副本
  13. 监督标签真的用到方向角：障碍偏左则右转、偏右则左转、正前方不偏转
  14. 合成数据集的第 4、5 列不相同（第 5 维没有退化成距离副本）
  15. sim_params.yaml 里的每个参数键都被节点声明过（否则改了不生效）

运行：  python3 test/test_mapping_logic.py
       或 pytest test/
"""

import os
import sys
import tempfile

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

import importlib.util  # noqa: E402

_spec = importlib.util.spec_from_file_location("mn_main", os.path.join(_PKG, "main.py"))
main_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(main_mod)


# ---------------------------------------------------------------- 建图
def test_grid_contains_vehicle_start():
    """回归测试：车辆出生点必须落在栅格覆盖范围内。

    早期版本栅格固定以世界原点为中心、只覆盖 ±30 m，而默认出生点在 (36,-5)，
    车辆落在栅格之外，所有雷达命中点被丢弃，建图结果恒为空。
    """
    g = main_mod.OccupancyGrid(center=main_mod.START)
    r, c = g.world_to_grid(*main_mod.START)
    assert g.in_bounds(r, c), (
        f"车辆出生点 {main_mod.START} 映射到栅格 ({r},{c}) 越界，"
        f"有效范围 0..{g.n - 1}")
    # 栅格中心应恰好对应起点
    assert abs(r - g.n / 2.0) < 1e-6, f"起点应为栅格中心，实际 row={r}"
    assert abs(c - g.n / 2.0) < 1e-6, f"起点应为栅格中心，实际 col={c}"


def test_grid_coordinate_roundtrip():
    """世界坐标 → 栅格 → 世界坐标应往返一致。"""
    g = main_mod.OccupancyGrid(center=main_mod.START)
    x, y = 30.0, 12.0
    r, c = g.world_to_grid(x, y)
    x2, y2 = g.grid_to_world(int(r), int(c))
    assert abs(x2 - x) <= g.m, f"x 往返误差过大: {x} -> {x2}"
    assert abs(y2 - y) <= g.m, f"y 往返误差过大: {y} -> {y2}"


def test_log_odds_update_direction():
    """命中应提高占据概率，穿过应降低。"""
    g = main_mod.OccupancyGrid(center=(0.0, 0.0), n=40)
    g.update_hits(np.array([0.0]), np.array([0.0]), hit=True)
    p_hit = float(g.probability()[g.n // 2, g.n // 2])
    assert p_hit > 0.5, f"命中后占据概率应 >0.5，实际 {p_hit}"

    g2 = main_mod.OccupancyGrid(center=(0.0, 0.0), n=40)
    g2.update_hits(np.array([0.0]), np.array([0.0]), hit=False)
    p_miss = float(g2.probability()[g2.n // 2, g2.n // 2])
    assert p_miss < 0.5, f"穿过后占据概率应 <0.5，实际 {p_miss}"


def test_build_map_marks_cells():
    """建图函数应同时产生占据格与空闲格。"""
    g = main_mod.OccupancyGrid(center=(0.0, 0.0), n=80)
    # 车在原点，正前方 x=5 处有一堵墙
    pc = np.array([[5.0, y, 0.0, 1.0] for y in np.linspace(-2, 2, 21)], dtype=np.float32)
    n_hit, n_miss = main_mod.build_map_from_scan(g, pc, 0.0, 0.0, 0.0)
    assert n_hit > 0, "应产生占据格"
    assert n_miss > 0, "应产生空闲格（射线沿途）"
    assert g.occupied_count > 0, "占据格计数应 >0"


def test_build_map_rejects_out_of_range():
    """超出雷达量程的点不应写入栅格。"""
    g = main_mod.OccupancyGrid(center=(0.0, 0.0), n=80)
    pc = np.array([[100.0, 0.0, 0.0, 1.0], [0.1, 0.0, 0.0, 1.0]], dtype=np.float32)
    main_mod.build_map_from_scan(g, pc, 0.0, 0.0, 0.0, max_range=35.0)
    assert g.occupied_count == 0, "量程外的点不应产生占据格"


# ---------------------------------------------------------------- 规划
def test_state_features_normalized():
    """状态向量的每一维都必须落在相近尺度（回归测试：早期第 5 维是原始米数）。"""
    pc = np.array([[2.0, 1.0, 0.0, 1.0], [3.0, -1.0, 0.0, 1.0]], dtype=np.float32)
    state, _raw = main_mod.obs_features(pc, goal_ang_diff=0.5)
    assert state.shape == (5,), f"状态维度应为 5，实际 {state.shape}"
    assert np.all(np.abs(state) <= 1.5), f"状态存在未归一化的大数值: {state}"


def test_state_bearing_dim_is_not_a_duplicate():
    """第 5 维必须是最近障碍的方向角，不能是第 4 维的副本。

    回归测试：早期 state = [..., d_n, d_n]，第 4、5 维完全相同，等于只有 4 维
    有效输入——网络分不清"障碍在正前方"和"障碍在侧前方"，而这两种情况该往
    哪边打方向恰好相反。
    """
    ahead = np.array([[2.0, 0.0, 0.0, 1.0]], dtype=np.float32)   # 正前方
    left = np.array([[2.0, 2.0, 0.0, 1.0]], dtype=np.float32)    # 左前方
    s_ahead, raw_ahead = main_mod.obs_features(ahead, 0.0)
    s_left, raw_left = main_mod.obs_features(left, 0.0)

    assert not np.allclose(s_ahead[3], s_ahead[4]), \
        "第 4、5 维不应相同（第 5 维是方向角，不是距离的副本）"
    assert abs(s_ahead[3] - s_left[3]) < 1e-6, "两点距离相同，第 4 维应一致"
    assert abs(s_ahead[4]) < 1e-6, f"正前方障碍的方位角应为 0，实际 {s_ahead[4]}"
    assert s_left[4] > 0, f"左侧障碍的方位角应为正（左为正），实际 {s_left[4]}"
    assert abs(raw_ahead[4]) < 1e-6 and raw_left[4] > 0, "raw 第 5 项应是方向角（弧度）"


def test_planning_label_uses_obstacle_bearing():
    """最近障碍偏左 → 右转；偏右 → 左转；正前方 → 不额外偏转。

    第 5 维（方向角）必须真的被监督标签用到，否则网络从这一维学不到东西，
    加了等于没加。
    """
    import math
    _th, st_left = main_mod.planning_label(0.0, 0.0, 0.0, 1.0, bearing=+math.pi / 4)
    _th, st_right = main_mod.planning_label(0.0, 0.0, 0.0, 1.0, bearing=-math.pi / 4)
    _th, st_ahead = main_mod.planning_label(0.0, 0.0, 0.0, 1.0, bearing=0.0)
    _th, st_far = main_mod.planning_label(0.0, 0.0, 0.0, 8.0, bearing=+math.pi / 4)

    assert st_left < 0, f"障碍在左应右转（负），实际 {st_left}"
    assert st_right > 0, f"障碍在右应左转（正），实际 {st_right}"
    assert abs(st_ahead) < 1e-9, f"障碍在正前方不应额外偏转，实际 {st_ahead}"
    assert abs(st_left) > abs(st_far), "障碍越近，横向排斥应越强"


def test_synth_dataset_uses_five_distinct_dims():
    """合成数据集的第 5 列必须随方向角变化，不能是第 4 列的复制。"""
    X, _Y = main_mod.synth_dataset(400, seed=0)
    assert X.shape[1] == 5, f"状态应为 5 维，实际 {X.shape[1]}"
    assert not np.allclose(X[:, 3], X[:, 4]), \
        "第 4、5 列完全相同，说明第 5 维又退化成距离的副本了"
    assert np.all(X[:, 4] >= -1.0) and np.all(X[:, 4] <= 1.0), \
        f"方向角应归一化到 [-1,1]，实际 [{X[:, 4].min()}, {X[:, 4].max()}]"


def test_planning_label_symmetry():
    """无障碍时（左右障碍对称）转向只由航向差决定。"""
    th0_l, st0 = main_mod.planning_label(0.0, 0.0, 0.0, 8.0)
    assert abs(st0) < 1e-9, f"正对目标且无障碍时转向应为 0，实际 {st0}"
    # 左侧障碍多 → 右转（负）
    _th, st_left = main_mod.planning_label(0.0, 0.9, 0.0, 8.0)
    assert st_left < 0, f"左侧障碍多应右转（负），实际 {st_left}"
    # 右侧障碍多 → 左转（正）
    _th, st_right = main_mod.planning_label(0.0, 0.0, 0.9, 8.0)
    assert st_right > 0, f"右侧障碍多应左转（正），实际 {st_right}"
    # 正前方航向差为正（目标在左）→ 左转
    _th, st_goal = main_mod.planning_label(0.5, 0.0, 0.0, 8.0)
    assert st_goal > 0, f"目标在左应左转（正），实际 {st_goal}"


def test_planning_throttle_in_unit_range():
    """油门标签必须落在 [0,1]，且障碍越近越小。"""
    th_far, _ = main_mod.planning_label(0.0, 0.0, 0.0, 8.0)
    th_near, _ = main_mod.planning_label(0.0, 0.0, 0.0, 0.3)
    assert 0.0 <= th_near <= 1.0 and 0.0 <= th_far <= 1.0, "油门应在 [0,1]"
    assert th_near < th_far, f"障碍更近时油门应更小: {th_near} vs {th_far}"


def test_planning_network_converges():
    """规划神经网络：5 维状态 → (油门, 转向)，MSE 应足够小。"""
    X, Y = main_mod.synth_dataset(500, seed=0)
    net = main_mod.MLPPolicy([5, 64, 2], seed=0)
    net.train(X, Y, epochs=300, lr=0.15, verbose=0)
    mse = float(np.mean((net.predict(X) - Y) ** 2))
    assert mse < 0.02, f"规划 NN MSE 过大: {mse}"


def test_model_save_load_roundtrip():
    """规划网络存取后预测应一致。"""
    X, Y = main_mod.synth_dataset(200, seed=1)
    net = main_mod.MLPPolicy([5, 64, 2], seed=0)
    net.train(X, Y, epochs=50, lr=0.15, verbose=0)
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "plan.json")
        net.save(p)
        loaded = main_mod.MLPPolicy.load(p)
    assert np.allclose(loaded.predict(X), net.predict(X), atol=1e-4), \
        "规划 NN 存取后预测不一致"


def test_bicycle_model_moves_toward_heading():
    """自行车模型：转向为 0 时应沿当前航向直行。"""
    x0, y0, yaw0, v0 = 0.0, 0.0, 0.0, 3.0
    x1, y1, _yaw1, _v1 = main_mod.step_bicycle(x0, y0, yaw0, v0, 0.5, 0.0)
    assert x1 > x0, "航向为 0 时应沿 +x 前进"
    assert abs(y1 - y0) < 1e-6, "无转向时不应产生侧向位移"


def test_offline_demo_runs():
    """离线取证模式应能在无 CARLA 环境下跑通并导出地图与曲线。"""
    with tempfile.TemporaryDirectory() as d:
        rc = main_mod.run_offline_demo(d, epochs=30, sim_time=15.0)
        assert rc == 0
        produced = sorted(os.listdir(d))
        assert "occupancy_map.png" in produced, f"缺少栅格地图: {produced}"
        assert "plan_loss.png" in produced, f"缺少损失曲线: {produced}"


def test_run_carla_source_guards_missing_model():
    """run_carla 必须在加载模型前检查文件是否存在（回归测试）。

    历史缺陷：`net = MLPPolicy.load(model_path)` 是 run_carla 的第一句，
    模型不存在时直接抛 FileNotFoundError。而 ROS 节点
    （mapping_navigation_node.py）本就有 os.path.isfile 守卫并回退到现场训练，
    两者行为不一致。修复：CLI 也在缺失时现场训练后再加载。
    """
    src = open(os.path.join(_PKG, "main.py"), encoding="utf-8").read()
    i_guard = src.index("if not os.path.isfile(model_path):")
    i_load = src.index("net = MLPPolicy.load(model_path)")
    assert i_guard < i_load, "模型存在性检查必须在 MLPPolicy.load 之前"
    # 缺失时应现场训练（与 ROS 节点一致），而不是直接崩
    assert "train_planning(epochs=300, out=model_path)" in src, \
        "模型缺失时未回退到现场训练"


def test_run_carla_keeps_sensor_references():
    """run_carla 必须保留 LiDAR 句柄，否则回调失效（回归测试）。

    历史缺陷：`cc.make_lidar(...)` 的返回值被丢弃，sensor 无引用被 Python
    垃圾回收，Actor 留在仿真里但回调不再触发——表现为栅格恒为空、
    感知特征全为 0，并伴随 CARLA 警告：
      sensor object went out of the scope but the sensor is still alive in the simulation
    """
    src = open(os.path.join(_PKG, "main.py"), encoding="utf-8").read()
    i_sensors = src.index("sensors = []")
    i_make = src.index("cc.make_lidar(")
    assert i_make > i_sensors, "make_lidar 的返回值未收集到 sensors 列表"
    assert "sensors.append(cc.make_lidar(" in src, \
        "make_lidar 的返回值必须 append 到 sensors 以保持引用"
    assert "for s in sensors:" in src, "缺少传感器清理循环"


def test_run_carla_restores_async_mode():
    """run_carla 退出前必须清理并恢复异步模式（回归测试）。

    历史缺陷：connect() 打开了 synchronous_mode，此时服务端只在客户端
    world.tick() 时推进；脚本结束或节点退出后没人再 tick，世界永久静止
    （CARLA 窗口看起来"卡住不动"）。且清理原先裸放在函数末尾，
    中途抛异常会被跳过，留下残留 actor 与同步模式。
    """
    src = open(os.path.join(_PKG, "main.py"), encoding="utf-8").read()
    assert "cc.restore_async(" in src, "退出时未恢复异步模式"
    assert "finally:" in src, "缺少 finally 块，异常时不会清理"
    i_fin = src.index("finally:")
    assert src.index("cc.restore_async(") > i_fin, "restore_async 不在 finally 内"
    for call in ("s.stop()", "s.destroy()", "vehicle.destroy()"):
        assert src.index(call) > i_fin, f"{call} 不在 finally 内"

    # carla_common 必须真的实现 restore_async 并写入同步开关
    common = open(os.path.join(_PKG, "carla_mapping_navigation",
                               "carla_common.py"), encoding="utf-8").read()
    assert "def restore_async(" in common, "carla_common 未实现 restore_async"
    i_fn = common.index("def restore_async(")
    assert "synchronous_mode = False" in common[i_fn:i_fn + 900], \
        "restore_async 未把 synchronous_mode 置为 False"


def test_connect_timeout_and_map_skip():
    """connect() 应有足够超时，且已在地图上时跳过重载（回归测试）。

    历史缺陷：set_timeout(20.0) 且无条件 load_world(town)。
    load_world 实测本机需 7 s 以上，虚拟机过网络更久——20 s 在虚拟机侧
    实测直接超时失败（RuntimeError: time-out of 20000ms）。
    """
    common = open(os.path.join(_PKG, "carla_mapping_navigation",
                               "carla_common.py"), encoding="utf-8").read()
    assert "set_timeout(20.0)" not in common, "超时仍为 20 s，虚拟机侧会超时"
    assert "timeout=60.0" in common, "未提供 60 s 默认超时"
    assert "force_reload" in common, "未提供 force_reload 开关"
    assert "get_map().name" in common, "未比较当前地图，会无条件重载"
    # 出生朝向也必须与车道一致（(36,-5) 车道实测 yaw=-181.2°）
    assert "0.0, 0.0, 178.8" in common, "DEFAULT_SPAWN 的 yaw 未按车道朝向修正"


def test_offline_demo_heading_faces_goal():
    """离线回放的初始航向应朝向目标（回归测试）。

    历史缺陷：`x, y, yaw = START[0], START[1], 0.0` 把车头写死为 +x，
    而默认目标 (20,8) 在起点 (36,-5) 的左后方（方位角约 +141°），
    自车会先背离目标行驶再掉头，污染"距目标最近距离"等指标。
    """
    src = open(os.path.join(_PKG, "main.py"), encoding="utf-8").read()
    assert "yaw = math.atan2(goal[1] - START[1], goal[0] - START[0])" in src, \
        "离线回放未由起点→目标方向推导初始航向"
    assert "x, y, yaw = START[0], START[1], 0.0" not in src, \
        "仍存在把初始航向写死为 0 的代码"


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
    src = open(os.path.join(_pkg_root(), "main.py"), encoding="utf-8").read()
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

    src = open(os.path.join(_pkg_root(), "main.py"), encoding="utf-8").read()
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


def test_parse_goal_accepts_documented_forms():
    """命令行 --goal、ROS 1 / ROS 2 launch 的 goal:="x,y" 走同一个解析函数。

    解析失败必须**回退到默认值**而不是抛异常：launch 启动时因为一个笔误就
    整个起不来，比直接用默认目标难排查得多。
    """
    assert main_mod.parse_goal("20,8") == (20.0, 8.0)
    assert main_mod.parse_goal(" 30 , 5 ") == (30.0, 5.0)
    assert main_mod.parse_goal("-1.5,2") == (-1.5, 2.0)
    assert main_mod.parse_goal(None) == main_mod.DEFAULT_GOAL
    for bad in ("", "abc", "1", "1,2,3", "x,y", "1;"):
        assert main_mod.parse_goal(bad) == main_mod.DEFAULT_GOAL, \
            f"{bad!r} 应回退到默认目标"


def test_documented_launch_args_are_declared():
    """文档里出现的每个 `name:=value` 都必须在对应 launch 文件里声明过。

    回归测试：ROS 2 的 main.launch.py 一度只声明 goal_x/goal_y，而 6 处文档
    都在教用户传 goal:="20,8"。ROS 2 对未声明的 launch 参数**不报错、直接忽略**，
    又因为默认目标恰好就是 (20,8)，这个错误一直看不出来——换成别的目标就失效。
    """
    import re

    ros2 = open(os.path.join(_pkg_root(), "launch", "main.launch.py"),
                encoding="utf-8").read()
    ros1 = open(os.path.join(_pkg_root(), "launch", "main.launch"),
                encoding="utf-8").read()
    decl2 = set(re.findall(r"DeclareLaunchArgument\(\s*'([^']+)'", ros2))
    decl1 = set(re.findall(r'<arg\s+name="([^"]+)"', ros1))

    assert "goal" in decl2, "ROS 2 launch 必须声明 goal（与 ROS 1 的接口对齐）"
    assert "goal" in decl1, "ROS 1 launch 必须声明 goal"

    sources = [os.path.join(_pkg_root(), "README.md"),
               os.path.join(_pkg_root(), "main.sh"),
               os.path.join(_pkg_root(), "launch", "main.launch.py"),
               os.path.join(_pkg_root(), "launch", "main.launch")]
    repo = os.path.abspath(os.path.join(_pkg_root(), "..", "..", ".."))
    for rel in ("docs/ground/carla_mapping_navigation.md",
                "docs/ground/pcl_recorder.md"):
        path = os.path.join(repo, rel)
        if os.path.exists(path):      # 单测在 colcon 安装目录下跑时没有 docs
            sources.append(path)

    problems = []
    for path in sources:
        for line in open(path, encoding="utf-8").read().splitlines():
            # ros2 launch <pkg> <file> key:=value ...
            m = re.search(r"ros2\s+launch\s+\S+\s+\S+(.*)", line)
            if m:
                for k in re.findall(r"([A-Za-z_]\w*):=", m.group(1)):
                    if k not in decl2:
                        problems.append(f"{os.path.basename(path)} 的 ros2 launch "
                                        f"用了未声明的 {k}:=")
            # roslaunch <pkg> <file> key:=value ...
            m = re.search(r"(?<!ros2 )\broslaunch\s+\S+\s+\S+(.*)", line)
            if m:
                for k in re.findall(r"([A-Za-z_]\w*):=", m.group(1)):
                    if k not in decl1:
                        problems.append(f"{os.path.basename(path)} 的 roslaunch "
                                        f"用了未声明的 {k}:=")
            # bash main.sh --launch key:=value ...（两个 launch 都要认）
            m = re.search(r"main\.sh\s+--launch\s+(.*)", line)
            if m:
                for k in re.findall(r"([A-Za-z_]\w*):=", m.group(1)):
                    if k not in decl1 or k not in decl2:
                        problems.append(f"{os.path.basename(path)} 的 main.sh --launch "
                                        f"用了并非两个 launch 都声明的 {k}:=")

    assert not problems, "；".join(sorted(set(problems)))


def test_sim_params_keys_are_declared_by_node():
    """sim_params.yaml 里的每个参数键都必须被节点 declare_parameter 过。

    回归测试：早期 yaml 里写了 16 个键，节点只声明 8 个，剩下的一半
    （grid_size / log_odds_hit / lidar_channels / ego_blueprint ...）改了完全不生效，
    却看起来像可配置项——rclpy 对未声明的参数覆盖项直接忽略。
    """
    import re

    yaml_txt = open(os.path.join(_pkg_root(), "config", "sim_params.yaml"),
                    encoding="utf-8").read()
    node_txt = open(os.path.join(_pkg_root(), "carla_mapping_navigation",
                                 "mapping_navigation_node.py"),
                    encoding="utf-8").read()

    keys = []
    for line in yaml_txt.splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        m = re.match(r"^\s+([a-z_]+):", line)
        if m and m.group(1) != "ros__parameters":
            keys.append(m.group(1))

    declared = set(re.findall(r'declare_parameter\(\s*"([a-z_]+)"', node_txt))
    dead = sorted(k for k in keys if k not in declared)
    assert not dead, f"这些参数节点没声明，改了不生效：{dead}"
    assert keys, "没解析出任何参数键，yaml 结构可能变了"


def _lzw_decode(blob, min_code, npix):
    """极简 GIF LZW 解码器，只用来验证我们自己编码器的输出。

    注意码长增长用 `==`：解码端每读一个码才建一条表项、且第一个码不建表，
    所以它比编码端慢一条；编码端那边对应的写法是 `>`。
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
        if tag == 0x3B:                       # trailer
            break
        if tag == 0x21:                       # 扩展块
            pos += 2
            while data[pos]:
                pos += data[pos] + 1
            pos += 1
            continue
        assert tag == 0x2C, f"期望图像描述符，遇到 {tag:#x}"
        _l, _t, fw, fh, ipacked = struct.unpack("<HHHHB", data[pos + 1: pos + 10])
        pos += 10
        if ipacked & 0x80:                    # 局部色表
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

    文档要求放 gif 动图，而模块是纯 numpy 的、不装 Pillow，所以编码器是自己写的。
    自己写的编码器 + 自己写的解码器一起测会「互相迁就」，所以这里的解码器按
    GIF 规范实现（码长增长用 `==`，与编码端的 `>` 相差一位），能独立判错。
    已验证它写出的文件 Pillow 也能正确读回。
    """
    frames = []
    for i in range(6):
        f = np.zeros((40, 50), dtype=np.uint8)
        f[:, : max(1, i * 8)] = 1
        f[10:20, 5:5 + max(1, i * 6)] = 2
        f[0, :] = 5
        frames.append(f)

    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, "t.gif")
        main_mod.write_gif(p, frames, main_mod.MAP_PALETTE, delay_cs=8)
        pal, got, size = _gif_read(p)

    assert size == (50, 40), f"尺寸应保持 (50,40)，实际 {size}"
    assert len(got) == len(frames), f"帧数应 {len(frames)}，实际 {len(got)}"
    for i, (a, b) in enumerate(zip(got, frames)):
        assert np.array_equal(a, b), f"第 {i} 帧像素不一致"

    # 调色板前 6 项必须与 MAP_PALETTE 完全相同，否则颜色会变
    assert [tuple(c) for c in pal[:len(main_mod.MAP_PALETTE)]] == \
        [tuple(c) for c in main_mod.MAP_PALETTE], "调色板被改动了"


def test_prob_canvas_colors_are_all_in_palette():
    """快照画面的每种颜色都必须在 MAP_PALETTE 里，否则转索引时会丢像素。

    GIF 只有索引色，`rgb_to_indices` 把调色板外的颜色记成 0（未知灰）。
    如果渲染函数以后加了新颜色却忘了进调色板，动图会悄悄变色。
    """
    grid = main_mod.OccupancyGrid(center=main_mod.START)
    prob = grid.probability()
    # 造出全部 4 种状态 + 轨迹 + 目标 + 进度条
    prob[10:20, 10:20] = 0.1
    prob[30:40, 30:40] = 0.9
    pts = [(main_mod.START[0] + i * 0.5, main_mod.START[1]) for i in range(5)]
    img = main_mod.prob_canvas(prob, grid, path_pts=pts,
                               goal=main_mod.DEFAULT_GOAL, scale=1, title_t=6.0)
    colors = {tuple(c) for c in img.reshape(-1, 3)}
    allowed = {tuple(c) for c in main_mod.MAP_PALETTE}
    extra = sorted(colors - allowed)
    assert not extra, f"这些颜色不在 MAP_PALETTE 里：{extra}"

    idx = main_mod.rgb_to_indices(img)
    # 每个下标都要能反查回原色，说明没有像素被误判成「未知」
    back = np.array(main_mod.MAP_PALETTE, dtype=np.uint8)[idx]
    assert np.array_equal(back, img), "rgb_to_indices 有像素映射错了"


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
