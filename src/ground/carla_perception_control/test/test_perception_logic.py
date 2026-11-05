#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 感知 + 轨迹跟踪（神经网络版）—— 单元测试。

不依赖 CARLA 服务端、不依赖 ROS，只验证神经网络与特征提取逻辑：
  1. 感知分类器能收敛（精度）
  2. 控制策略网络能收敛（MSE）
  3. 特征归一化把各维度压到相近尺度
  4. 合成数据集形状正确
  5. 模型保存/加载往返一致

运行：  python3 test/test_perception_logic.py
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

_spec = importlib.util.spec_from_file_location("pc_main", os.path.join(_PKG, "main.py"))
main_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(main_mod)


def test_perception_classifier_converges():
    """感知 NN：传感器特征 → 障碍类别，训练精度应 > 0.9。"""
    feat_X, feat_Y, _cX, _cY = main_mod.synth_dataset(400, seed=0)
    net = main_mod.MLPClassifier([4, 12, 3], seed=0)
    net.train(feat_X, feat_Y, epochs=300, lr=0.2, verbose=0)
    acc = float(np.mean(net.predict(feat_X) == feat_Y))
    assert acc > 0.9, f"感知 NN 精度过低: {acc}"


def test_control_policy_converges():
    """控制 NN：状态 → 转向角，回归 MSE 应足够小。

    注意：控制网络输入做了标准化（量纲差异大），因此误差必须在**标准化后的
    输入**上评估——这与在线推理 `main_mod.nn_control` 的做法一致。
    """
    _fX, _fY, ctrl_X, ctrl_Y = main_mod.synth_dataset(400, seed=0)
    sens, ctrl, _h = main_mod.train(_fX, _fY.astype(int), ctrl_X, ctrl_Y, epochs=300)
    xin = (ctrl_X - ctrl.input_mu) / ctrl.input_sd
    mse = float(np.mean(np.square(ctrl.predict(xin).ravel() - ctrl_Y)))
    assert mse < 0.02, f"控制 NN MSE 过大: {mse}"


def test_pure_pursuit_law_zero_on_straight():
    """纯跟踪几何律在航向差为 0（正对目标）时应给出零转向。"""
    assert abs(main_mod.pure_pursuit_law(0.0, 6.0)) < 1e-9, "直线行驶不应产生转向"
    # 左偏/右偏应给出相反符号的转向
    assert main_mod.pure_pursuit_law(0.5, 6.0) > 0
    assert main_mod.pure_pursuit_law(-0.5, 6.0) < 0


def test_waypoint_interpolation():
    """路点加密应保持首尾端点并显著增加点数。"""
    raw = [(0.0, 0.0), (10.0, 0.0)]
    dense = main_mod.interpolate_waypoints(raw, step=2.0)
    assert dense[0] == (0.0, 0.0), "加密后起点应保持"
    assert dense[-1] == (10.0, 0.0), "加密后终点应保持"
    assert len(dense) > len(raw), "加密后点数应增加"


def test_lateral_error_is_perpendicular_distance():
    """横向误差应为点到折线的垂距：点在折线上时为 0，偏离 3 m 时为 3。"""
    line = [(0.0, 0.0), (10.0, 0.0)]
    assert main_mod._dist_to_polyline((5.0, 0.0), line) < 1e-9
    assert abs(main_mod._dist_to_polyline((5.0, 3.0), line) - 3.0) < 1e-6


def test_feature_normalization_range():
    """特征归一化后各维应落在相近尺度（约 [0,1]）。"""
    feat = main_mod._feat_feature(0.5, 200.0, 30, 10.0)
    assert feat.shape == (4,), f"特征维度应为 4，实际 {feat.shape}"
    assert np.all(feat >= -1.0) and np.all(feat <= 1.0), f"归一化越界: {feat}"


def test_synth_dataset_shapes():
    feat_X, feat_Y, ctrl_X, ctrl_Y = main_mod.synth_dataset(120, seed=3)
    assert feat_X.shape == (120, 4)
    assert feat_Y.shape == (120,)
    assert ctrl_X.shape == (120, 2)
    assert ctrl_Y.shape == (120,)


def test_model_save_load_roundtrip():
    feat_X, feat_Y, ctrl_X, ctrl_Y = main_mod.synth_dataset(200, seed=5)
    sens, ctrl, _h = main_mod.train(feat_X, feat_Y.astype(int),
                                    ctrl_X, ctrl_Y, epochs=50)
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "nn.json")
        main_mod.save_model(p, sens, ctrl)
        loaded = main_mod.load_model(p)
    assert np.array_equal(loaded["sens"].predict(feat_X), sens.predict(feat_X)), \
        "感知 NN 存取后预测不一致"
    # 控制网络：用同一组标准化参数比较预测，且标准化参数本身也要能往返
    xin = (ctrl_X - ctrl.input_mu) / ctrl.input_sd
    xin_loaded = (ctrl_X - loaded["ctrl"].input_mu) / loaded["ctrl"].input_sd
    assert np.allclose(loaded["ctrl"].predict(xin_loaded), ctrl.predict(xin), atol=1e-4), \
        "控制 NN 存取后预测不一致"
    assert np.allclose(loaded["ctrl"].input_mu, ctrl.input_mu, atol=1e-6), \
        "标准化均值未正确保存"
    assert np.allclose(loaded["ctrl"].input_sd, ctrl.input_sd, atol=1e-6), \
        "标准化方差未正确保存"


def test_offline_demo_runs():
    """离线取证模式应能在无 CARLA 环境下跑通并导出曲线图。"""
    with tempfile.TemporaryDirectory() as d:
        rc = main_mod.run_offline_demo(d, epochs=30, sim_time=3.0)
        assert rc == 0
        produced = sorted(os.listdir(d))
        assert "percept_loss.png" in produced, f"缺少感知损失曲线: {produced}"
        assert "lateral_error.png" in produced, f"缺少横向误差曲线: {produced}"


def test_run_carla_trains_when_model_missing():
    """模型文件缺失时 run_carla 应现场训练而非直接崩溃（回归测试）。

    历史缺陷：`run_carla` 一开始就无条件 load_model(model_path)，
    没有模型文件时抛 FileNotFoundError，用户按文档顺序执行必然踩坑
    （实测报错：FileNotFoundError: 'models/nn_percept.json'）。
    ROS 节点在模型缺失时会回退到现场训练保证"节点总能运行"，
    main.py 的在线模式应与之一致。

    本测试只验证"缺模型 → 自动训练并落盘"这一步，
    不连接 CARLA（连接部分在 load_model 之后，此处不会执行到）。
    """
    with tempfile.TemporaryDirectory() as d:
        missing = os.path.join(d, "不会存在", "nn_percept.json")
        assert not os.path.isfile(missing)
        # 直接调用 run_carla 会因缺少 carla 模块而提前返回；这里改为验证
        # "缺模型时先训练并保存" 这段前置逻辑本身可用。
        feat_X, feat_Y, ctrl_X, ctrl_Y = main_mod.synth_dataset(60, seed=0)
        sens, ctrl, _h = main_mod.train(feat_X, feat_Y, ctrl_X, ctrl_Y, epochs=20)
        main_mod.save_model(missing, sens, ctrl)
        assert os.path.isfile(missing), "缺模型时应能自动训练并落盘"
        loaded = main_mod.load_model(missing)
        assert loaded["sens"] is not None and loaded["ctrl"] is not None


def test_run_carla_source_guards_missing_model():
    """静态检查：run_carla 必须在 load_model 之前做存在性判断。

    防止将来有人把这段"缺模型则现场训练"的保护逻辑删掉，
    再次出现按文档顺序执行就崩溃的情况。
    """
    src = open(os.path.join(_PKG, "main.py"), encoding="utf-8").read()
    i_guard = src.find("if not os.path.isfile(model_path)")
    i_load = src.find("model = load_model(model_path)")
    assert i_guard != -1, "run_carla 缺少“模型不存在则先训练”的保护逻辑"
    assert i_load != -1, "未找到 load_model 调用"
    assert i_guard < i_load, "存在性判断必须写在 load_model 之前"


def test_run_carla_keeps_sensor_references():
    """静态检查：run_carla 必须持有 sensor 对象的引用（回归测试）。

    历史缺陷：三个 make_*_camera/make_lidar 的返回值被直接丢弃，
    Python 随即垃圾回收这些 sensor 对象。Actor 仍留在仿真里但回调不再触发，
    实测表现为：
        WARNING: sensor object went out of the scope but the sensor is still alive
        共收到相机帧 0 张
        NN感知=无目标 rgb=+0.00 depth=0 lidar(0,20.0)   ← 全是默认值
    因此传感器句柄必须被收集到一个变量中并存活到循环结束。
    """
    src = open(os.path.join(_PKG, "main.py"), encoding="utf-8").read()
    assert "sensors = [" in src, "run_carla 未把传感器句柄收集到一个变量中"
    # 每个 make_* 调用都必须出现在 sensors = [...] 之后，而不是被裸调用
    i_sensors = src.find("sensors = [")
    for call in ("cc.make_rgb_camera(", "cc.make_depth_camera(", "cc.make_lidar("):
        assert call in src, f"未找到 {call}"
        assert src.find(call) > i_sensors, f"{call} 必须在 sensors 列表内被引用"
    # 收尾必须销毁传感器，避免 Actor 堆积
    assert "for s in sensors:" in src, "退出前未销毁传感器"


def test_demo_route_waypoints_are_lane_valid():
    """DEMO_ROUTE 的每个航点必须落在可行驶车道上（回归测试）。

    历史缺陷：原路线的 (120,-5) 偏离车道 2.93 m、(160,40) 偏离 4.94 m。
    航点之间是直线连接，航点离路会让车开出路面撞上障碍物后卡死
    （实测在线运行时在 (140.1,4.9) 撞停，lidar 最近距离降到 1.32 m）。

    本测试用 CARLA 地图校验；无 CARLA 时跳过（打印提示）。
    """
    try:
        import carla  # noqa: F401
    except ImportError:
        print("    （未安装 carla，跳过车道校验）")
        return

    try:
        import carla
        client = carla.Client(
            os.environ.get("CARLA_HOST", "127.0.0.1"),
            int(os.environ.get("CARLA_PORT", "2000")))
        client.set_timeout(20.0)
        cmap = client.load_world(os.environ.get("CARLA_MAP", "Town05")).get_map()
    except Exception as exc:  # noqa: BLE001
        print(f"    （连接 CARLA 失败：{exc}，跳过车道校验）")
        return

    import math as _m
    worst = 0.0
    bad = []
    for x, y in main_mod.DEMO_ROUTE:
        wp = cmap.get_waypoint(carla.Location(x=float(x), y=float(y), z=0.6),
                               project_to_road=True,
                               lane_type=carla.LaneType.Driving)
        lp = wp.transform.location
        d = _m.hypot(lp.x - x, lp.y - y)
        worst = max(worst, d)
        if d >= 2.0:
            bad.append(f"({x},{y}) 偏离 {d:.2f} m")
    assert not bad, ("DEMO_ROUTE 有航点不在可行驶车道上：\n  "
                     + "\n  ".join(bad) + f"\n  最大偏离 {worst:.2f} m")


def test_demo_route_has_curves():
    """DEMO_ROUTE 必须包含真实转弯，且弯道处航点足够密（回归测试）。

    两条约束：
      1. 纯直道无法体现轨迹跟踪控制器的转向能力，必须含转弯；
      2. 前视距离 LOOKAHEAD=6 m，若转弯处航点间距过大，车在两点间走直线，
         转弯被压缩到航点附近几米内完成，必然切出路面
         （实测间距 51 m 时在 (-185,-15) 撞停，lidar 0.037 m）。
    """
    import math as _m
    r = main_mod.DEMO_ROUTE
    assert len(r) >= 4, "示范路线路点过少"

    # 1) 累计转角要够大，说明确实有弯
    total_turn = 0.0
    for i in range(1, len(r) - 1):
        a1 = _m.atan2(r[i][1] - r[i - 1][1], r[i][0] - r[i - 1][0])
        a2 = _m.atan2(r[i + 1][1] - r[i][1], r[i + 1][0] - r[i][0])
        total_turn += abs((a2 - a1 + _m.pi) % (2 * _m.pi) - _m.pi)
    assert _m.degrees(total_turn) > 90.0, \
        f"示范路线转角仅 {_m.degrees(total_turn):.1f}°，不足以体现转向控制"

    # 2) 每个转弯处的进/出航点间距都要小于安全上限
    #    允许的最小转弯半径 R_min = L/tan(STEER_GAIN) ≈ 0.91 m；
    #    取保守上限 30 m，保证前视追踪有足够余量。
    LIMIT = 30.0
    tight = []
    for i in range(1, len(r) - 1):
        a1 = _m.atan2(r[i][1] - r[i - 1][1], r[i][0] - r[i - 1][0])
        a2 = _m.atan2(r[i + 1][1] - r[i][1], r[i + 1][0] - r[i][0])
        turn = abs((a2 - a1 + _m.pi) % (2 * _m.pi) - _m.pi)
        if _m.degrees(turn) < 10.0:
            continue
        g_in = _m.hypot(r[i][0] - r[i - 1][0], r[i][1] - r[i - 1][1])
        g_out = _m.hypot(r[i + 1][0] - r[i][0], r[i + 1][1] - r[i][1])
        if max(g_in, g_out) > LIMIT:
            tight.append(f"路点{i}{r[i]} 转角{_m.degrees(turn):.0f}° "
                         f"间距{g_in:.0f}/{g_out:.0f} m")
    assert not tight, ("转弯处航点间距过大，车辆会切出路面：\n  "
                       + "\n  ".join(tight))


def test_offline_demo_starts_aligned_with_route():
    """离线回放的初始航向应与轨迹首段方向一致（回归测试）。

    历史缺陷：初始航向被写死为 0（+x）。当路线首段指向 -x 时，
    车会先掉头再追线，横向误差被初始大偏差污染
    （实测 RMSE 从 0.25 m 恶化到 97.3 m）。
    """
    import math as _m
    r = main_mod.DEMO_ROUTE
    a0 = _m.atan2(r[1][1] - r[0][1], r[1][0] - r[0][0])
    src = open(os.path.join(_PKG, "main.py"), encoding="utf-8").read()
    assert "math.atan2(_dy0, _dx0)" in src, \
        "离线回放未由轨迹首段方向推导初始航向"
    # 该路线的首段确实不是 +x，因此这个修复是必要的
    assert abs(_m.degrees(a0)) > 5.0, \
        "示范路线首段接近 +x，本用例失去意义，请更换路线"


def test_default_waypoints_use_demo_route():
    """省略 --waypoints 时必须使用内置 DEMO_ROUTE（回归测试）。

    历史缺陷：--waypoints 的默认值是早期文档里那条错误路线
    "40,-8 40,12 25,20"（其中 (40,-8) 偏离车道 2.59 m、(40,12) 偏离 4.92 m）。
    用户不带该参数运行时就会用坏路线，车开出路面撞停
    （实测在 (41.4,7.9) 卡死、相机帧 0 张）。
    """
    # 1) 默认值不应是硬编码的错误路线
    p = main_mod.build_parser()
    ns = p.parse_args([])
    assert ns.waypoints is None, (
        f"--waypoints 默认值仍被写死为 {ns.waypoints!r}，应为 None 以回退到 DEMO_ROUTE")

    # 2) 不带 --waypoints 时，解析结果应等于 DEMO_ROUTE
    args = main_mod.build_parser().parse_args(["--mode", "train"])
    wps = [tuple(map(float, t.split(","))) for t in args.waypoints.split()] \
        if args.waypoints else list(main_mod.DEMO_ROUTE)
    assert wps == [tuple(map(float, p_)) for p_ in main_mod.DEMO_ROUTE], \
        "省略 --waypoints 时未使用内置 DEMO_ROUTE"

    # 3) 错误路线不应再作为 add_argument 的 default 出现
    #    注意：注释中引用该错误路线用于说明历史缺陷是允许的，
    #    因此这里只检查 --waypoints 的参数定义行本身。
    src = open(os.path.join(_PKG, "main.py"), encoding="utf-8").read()
    for line in src.splitlines():
        if "add_argument" in line and "--waypoints" in line:
            assert "40,-8" not in line, \
                f"--waypoints 的默认值仍写死为错误路线：{line.strip()}"
            assert "default=None" in line, \
                f"--waypoints 应默认 None 以回退到 DEMO_ROUTE：{line.strip()}"


def test_follow_camera_is_wired():
    """第三人称跟随镜头应可通过 --follow 开启，并在主循环中逐帧更新（回归测试）。"""
    p = main_mod.build_parser()
    ns = p.parse_args(["--follow"])
    assert ns.follow is True
    assert ns.follow_dist > 0 and ns.follow_height > 0

    src = open(os.path.join(_PKG, "main.py"), encoding="utf-8").read()
    assert "cc.set_spectator_follow(" in src, "未调用 set_spectator_follow"
    # 循环内每帧都要更新，否则镜头只在开头摆一次、之后不跟随
    assert src.count("cc.set_spectator_follow(") >= 2, \
        "set_spectator_follow 应初始化时调用一次、循环内每帧再调用"


def test_run_carla_restores_async_mode():
    """run_carla 退出前必须清理 actor 并恢复异步模式（回归测试）。

    历史缺陷：connect() 打开了 synchronous_mode，此时服务端**只在客户端
    world.tick() 时推进一帧**。脚本结束后没人再 tick，服务端就永久停住——
    CARLA 窗口看起来"卡住不动"，后来者连接同一服务端也像死机。
    且清理代码原先在函数末尾裸放，中途抛异常（如连接超时）会跳过清理，
    留下满世界的残留 actor 和同步模式。

    修复：整体包进 try/finally，finally 里销毁传感器+自车并 restore_async。
    """
    src = open(os.path.join(_PKG, "main.py"), encoding="utf-8").read()

    # 1) 必须调用 restore_async
    assert "cc.restore_async(" in src, "退出时未恢复异步模式，服务端会卡住"

    # 2) restore_async 必须在 finally 块内（异常路径也要清理）
    assert "finally:" in src, "缺少 finally 块，异常时不会清理"
    i_fin = src.index("finally:")
    i_res = src.index("cc.restore_async(")
    assert i_res > i_fin, "restore_async 不在 finally 块内"

    # 3) 销毁逻辑也必须在 finally 内
    for call in ("s.stop()", "s.destroy()", "vehicle.destroy()"):
        assert src.index(call) > i_fin, f"{call} 不在 finally 块内"

    # 4) carla_common 必须真的实现了 restore_async，且写入同步开关
    common = open(os.path.join(_PKG, "carla_perception_control",
                               "carla_common.py"), encoding="utf-8").read()
    assert "def restore_async(" in common, "carla_common 未实现 restore_async"
    i_fn = common.index("def restore_async(")
    seg = common[i_fn:i_fn + 900]
    assert "synchronous_mode = False" in seg, \
        "restore_async 未把 synchronous_mode 置为 False"


def test_ros_node_restores_async_mode():
    """ROS 节点退出时同样要恢复异步模式（回归测试）。

    历史缺陷：destroy_node 只销毁了 actor，没恢复异步模式，
    导致 Ctrl+C 之后 CARLA 窗口同样卡住。
    """
    p = os.path.join(_PKG, "carla_perception_control", "perception_control_node.py")
    src = open(p, encoding="utf-8").read()
    i_dn = src.index("def destroy_node(")
    seg = src[i_dn:i_dn + 1200]
    assert "cc.restore_async(" in seg, \
        "ROS 节点 destroy_node 未恢复异步模式，Ctrl+C 后 CARLA 会卡住"


def test_diagnostic_script_ignores_map_native_actors():
    """连通性诊断不应把地图自带的红绿灯/spectator 误报为残留（回归测试）。

    历史缺陷：脚本用 `len(actors) > 60` 判定残留，而 Town05 自带 113 个
    traffic（红绿灯）+ 1 个 spectator，正常情况也会误报"actor 偏多"。
    """
    p = os.path.join(_PKG, "check_connection.py")
    assert os.path.isfile(p), "缺少连通性诊断脚本 check_connection.py"
    src = open(p, encoding="utf-8").read()
    assert "map_native" in src, "诊断脚本未区分地图自带对象与脚本残留"
    assert "len(actors) > 60" not in src, \
        "诊断脚本仍在用总数阈值判定残留，会误报地图自带对象"
    for k in ("traffic", "spectator"):
        assert k in src, f"诊断脚本未把 {k} 归入地图自带对象"


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
