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
