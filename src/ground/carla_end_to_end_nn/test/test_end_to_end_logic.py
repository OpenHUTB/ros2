#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 端到端神经网络（图像 → 控制）—— 单元测试。

不依赖 CARLA、不依赖 ROS、不依赖 TensorFlow：
  1. 合成图像确实**含有**转向信息（车道线位置与标签强相关）
  2. maxpool 反向传播：梯度只回传给窗口内最大值位置（回归测试：早期版本均摊到全部位置）
  3. maxpool 梯度与数值梯度一致（有限差分校验）
  4. 推理输出与训练目标同值域 [-1,1]（回归测试：早期 predict 返回未过 tanh 的无界值）
  5. CNN 能真正学到"图像 → 转向"（相关系数与方向一致率）
  6. 模型保存/加载往返一致
  7. 预测在图像被加噪时稳定（不依赖单一像素）
  8. 专家控制器几何律正确
  9. 离线取证模式跑通并导出图
 10. CARLA 函数缺模型时有友好提示、回退训练预算有上限、传感器引用不被回收
 11. main.* 有可执行位（roslaunch 靠 shebang 执行 main.py）
 12. launch 的布尔参数接受文本值、空的可选值不会崩
 13. **入口与文档一致性**：sim_params.yaml 的键都被节点声明过；
     文档/脚本里写的 `名字:=值` 都被对应 launch 声明过（回归测试：曾有 5 个
     假开关与一个在 Humble 上无效的 `mode:=test`）

运行：  python3 test/test_end_to_end_logic.py
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

_spec = importlib.util.spec_from_file_location("e2e_main", os.path.join(_PKG, "main.py"))
main_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(main_mod)

from carla_end_to_end_nn.nn_models import SimpleCNN  # noqa: E402


# ---------------------------------------------------------------- 数据
def test_synth_images_contain_steering_info():
    """合成图像必须真的含有"路往哪弯"的信息。

    这是端到端学习的前提：图像与标签要有因果关系。用"车道线亮像素的横向重心"
    作为图像中转向信息的代理量，它应与 steer 标签强相关。
    """
    X, Y = main_mod.synth_dataset(n=60, seed=0)
    cents = []
    for i in range(len(X)):
        gray = X[i].astype(np.float32).mean(axis=2)
        bright = gray > 200
        if bright.sum() > 0:
            _ys, xs = np.nonzero(bright)
            cents.append(xs.mean())
        else:
            cents.append(np.nan)
    cents = np.asarray(cents)
    r = float(np.corrcoef(cents, Y.ravel())[0, 1])
    assert abs(r) > 0.9, (
        f"车道线位置与转向标签相关性过弱 (r={r:.3f})，"
        f"说明图像里看不出转向，网络不可能学到映射")


def test_synth_dataset_shapes():
    X, Y = main_mod.synth_dataset(n=10, seed=0)
    assert X.shape == (10, main_mod.IMG_H, main_mod.IMG_W, 3), f"图像形状错误: {X.shape}"
    assert Y.shape == (10, 1), f"标签形状错误: {Y.shape}"
    assert X.dtype == np.uint8, f"图像应为 uint8，实际 {X.dtype}"


# ---------------------------------------------------------------- 梯度
def test_maxpool_backward_only_max_position():
    """maxpool 反向：梯度只应回传给窗口内最大值所在位置。

    回归测试：早期实现把 dout 赋给窗口内全部 p×p 个位置，等价于把梯度放大 p² 倍
    并把梯度错分给未被选中的元素，导致网络无法学习（输出塌缩为常数）。
    """
    net = SimpleCNN(in_channels=3, hidden=4, out=1, img=(10, 12), seed=0)
    # 构造 4×4 单通道输入，2×2 maxpool → 2×2 输出
    x = np.zeros((1, 4, 4, 1), dtype=np.float32)
    x[0, 0, 0, 0] = 5.0      # 窗口(0,0)的最大值
    x[0, 2, 3, 0] = 7.0      # 窗口(1,1)的最大值
    x[0, 0, 1, 0] = 1.0
    x[0, 1, 0, 0] = 2.0
    dout = np.ones((1, 2, 2, 1), dtype=np.float32)
    dx = net._pool_backward(dout, x)

    assert dx[0, 0, 0, 0] == 1.0, f"最大值位置应收到梯度 1，实际 {dx[0, 0, 0, 0]}"
    assert dx[0, 2, 3, 0] == 1.0, f"最大值位置应收到梯度 1，实际 {dx[0, 2, 3, 0]}"
    # 非最大值位置不应收到梯度
    assert dx[0, 0, 1, 0] == 0.0, f"非最大值位置不应收到梯度，实际 {dx[0, 0, 1, 0]}"
    assert dx[0, 1, 0, 0] == 0.0, f"非最大值位置不应收到梯度，实际 {dx[0, 1, 0, 0]}"
    # 总梯度不应被放大
    assert abs(dx.sum() - dout.sum()) < 1e-6, \
        f"回传梯度总和应守恒（{dout.sum()}），实际 {dx.sum()}"


def test_pool_gradient_matches_finite_difference():
    """用有限差分校验 maxpool 反向传播正确性。"""
    net = SimpleCNN(in_channels=2, hidden=3, out=1, img=(8, 8), seed=0)
    rng = np.random.RandomState(0)
    x = rng.rand(2, 8, 8, 2).astype(np.float32) * 255
    p1, _ = net._pool(net._conv_forward(x, net.c1[0], net.c1[1]))  # 池化前的激活

    def f(a):
        pr, _ = net._pool(a)
        return float(np.sum(pr ** 2))

    dout = 2.0 * net._pool(p1)[0]              # d(sum(pool^2))/d(pool)
    ana = net._pool_backward(dout, p1)

    eps = 1e-3
    num = np.zeros_like(p1)
    idx = rng.choice(p1.size, size=12, replace=False)
    flat = p1.reshape(-1)
    for k in idx:
        orig = flat[k]
        flat[k] = orig + eps
        fp = f(p1)
        flat[k] = orig - eps
        fm = f(p1)
        flat[k] = orig
        num.reshape(-1)[k] = (fp - fm) / (2 * eps)
    rel = np.abs(ana.reshape(-1)[idx] - num.reshape(-1)[idx]) / (
        np.abs(ana.reshape(-1)[idx]) + np.abs(num.reshape(-1)[idx]) + 1e-8)
    assert rel.max() < 1e-2, f"maxpool 梯度与数值梯度不符，最大相对误差 {rel.max():.3e}"


# ---------------------------------------------------------------- 推理一致性
def test_predict_output_in_tanh_range():
    """推理输出必须与训练目标同值域（[-1,1]）。

    回归测试：早期 `predict()` 返回未过 tanh 的线性输出，导致预测值可达 ±13，
    而真实转向标签在 [-1,1]，训练/推理不一致。
    """
    net = SimpleCNN(in_channels=3, hidden=8, out=1, img=(main_mod.IMG_H, main_mod.IMG_W), seed=0)
    X, _Y = main_mod.synth_dataset(n=8, seed=0)
    p = net.predict(X.astype(np.float32))
    assert np.all(np.abs(p) <= 1.0), f"推理输出超出 [-1,1]: {p.ravel()}"


def test_cnn_learns_image_to_steer():
    """端到端 CNN 应真正学到"图像 → 转向"：预测与标签强相关、方向判对率高。

    这是端到端模块的核心验收项。为控制纯 numpy 训练耗时，使用较小的样本量。
    """
    X, Y = main_mod.synth_dataset(n=100, seed=0)
    net = SimpleCNN(in_channels=3, hidden=8, out=1,
                    img=(main_mod.IMG_H, main_mod.IMG_W), seed=0)
    net.train(X, Y.ravel(), epochs=45, lr=0.15, batch=16, verbose=0)

    pred = net.predict(X.astype(np.float32)).ravel()
    y = Y.ravel()
    mae = float(np.mean(np.abs(pred - y)))
    base = float(np.mean(np.abs(y)))                # 恒输出 0 的基线
    corr = float(np.corrcoef(pred, y)[0, 1])
    sign_acc = float(np.mean(np.sign(pred) == np.sign(y)))

    assert mae < base * 0.6, f"MAE({mae:.4f}) 未明显优于零输出基线({base:.4f})，网络没学到映射"
    assert corr > 0.8, f"预测与标签相关性过低 (r={corr:.3f})"
    assert sign_acc > 0.8, f"转向方向判对率过低 ({sign_acc * 100:.1f}%)"


def test_prediction_stable_under_noise():
    """加入轻微噪声后预测不应剧烈跳变（说明网络学到的是结构而非单像素）。"""
    X, Y = main_mod.synth_dataset(n=60, seed=0)
    net = SimpleCNN(in_channels=3, hidden=8, out=1,
                    img=(main_mod.IMG_H, main_mod.IMG_W), seed=0)
    net.train(X, Y.ravel(), epochs=45, lr=0.15, batch=16, verbose=0)

    rng = np.random.RandomState(1)
    noisy = np.clip(X.astype(np.float32) + rng.normal(0, 6.0, X.shape), 0, 255).astype(np.uint8)
    p0 = net.predict(X.astype(np.float32)).ravel()
    p1 = net.predict(noisy.astype(np.float32)).ravel()
    max_jump = float(np.max(np.abs(p1 - p0)))
    assert max_jump < 0.25, f"轻微噪声导致预测跳变过大: {max_jump:.3f}"


def test_model_save_load_roundtrip():
    """端到端 CNN 存取后预测应一致。"""
    X, Y = main_mod.synth_dataset(n=20, seed=2)
    net = SimpleCNN(in_channels=3, hidden=8, out=1,
                    img=(main_mod.IMG_H, main_mod.IMG_W), seed=0)
    net.train(X, Y.ravel(), epochs=5, lr=0.15, batch=16, verbose=0)
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "cnn.json")
        net.save(p)
        loaded = main_mod.load_cnn(p, backend="numpy")
    assert np.allclose(loaded.predict(X.astype(np.float32)),
                       net.predict(X.astype(np.float32)), atol=1e-6), \
        "端到端 CNN 存取后预测不一致"


# ---------------------------------------------------------------- 专家控制器
def test_expert_steer_geometry():
    """专家控制器：正对目标转向为 0；目标在左则左转（正）。"""
    st0, diff0 = main_mod.expert_steer((0.0, 0.0), 0.0, [(10.0, 0.0)])
    assert abs(st0) < 1e-6 and abs(diff0) < 1e-6, f"正对目标应零转向，实际 {st0}"

    st_left, _ = main_mod.expert_steer((0.0, 0.0), 0.0, [(5.0, 5.0)])
    assert st_left > 0, f"目标在左应左转（正），实际 {st_left}"

    st_right, _ = main_mod.expert_steer((0.0, 0.0), 0.0, [(5.0, -5.0)])
    assert st_right < 0, f"目标在右应右转（负），实际 {st_right}"


def test_expert_steer_clipped():
    """专家转向必须落在 [-1,1]。"""
    for wp in ([(0.0, 50.0)], [(0.5, -1.0)], [(100.0, 0.2)]):
        s, _ = main_mod.expert_steer((0.0, 0.0), 0.0, wp)
        assert -1.0 <= s <= 1.0, f"专家转向越界: {s}"


# ---------------------------------------------------------------- 离线取证
def test_offline_demo_runs():
    """离线取证模式应能在无 CARLA / 无 GPU / 无 TensorFlow 环境下跑通并导出图。"""
    with tempfile.TemporaryDirectory() as d:
        rc = main_mod.run_offline_demo(d, epochs=6, n_samples=30)
        assert rc == 0
        produced = sorted(os.listdir(d))
        for need in ("cnn_loss.png", "steer_pred_vs_true.png", "road_samples.png", "cnn_steer.json"):
            assert need in produced, f"缺少产物 {need}: {produced}"


def test_test_carla_source_guards_missing_model():
    """test_carla 必须在加载模型前检查文件是否存在（回归测试）。

    历史缺陷：`net = load_cnn(model_path, ...)` 无存在性检查，模型缺失时
    直接抛 FileNotFoundError；而 ROS 节点（end_to_end_node.py:80）本就有
    os.path.isfile 守卫并回退现场训练，两者行为不一致。
    """
    src = open(os.path.join(_PKG, "main.py"), encoding="utf-8").read()
    i_fn = src.index("def test_carla(")
    seg = src[i_fn:i_fn + 3000]
    i_guard = seg.index("if not os.path.isfile(model_path):")
    i_load = seg.index("net = load_cnn(model_path")
    assert i_guard < i_load, "模型存在性检查必须在 load_cnn 之前"
    assert "train_cnn(" in seg, "模型缺失时未回退到现场训练"


def test_fallback_training_budget_is_bounded():
    """现场训练回退的预算必须受控（回归测试）。

    历史缺陷：回退直接用 400 样本 × 60 轮，而纯 numpy CNN 实测约 15 s/epoch，
    合计约 15 分钟——使用者会以为程序卡死（本机实测直接撞上 10 分钟超时）。
    修复：回退改为小样本 + 少轮次，并打印预计耗时。
    """
    src = open(os.path.join(_PKG, "main.py"), encoding="utf-8").read()
    i_fn = src.index("def test_carla(")
    seg = src[i_fn:i_fn + 3000]
    assert "epochs=60, backend=\"numpy\")" not in seg, \
        "回退训练仍使用 60 轮，耗时会达到十几分钟"
    assert "ep_fb" in seg, "回退训练未使用受控的轮次预算"

    # ROS 节点的回退同样要压小预算且不能静默（verbose=0 会几分钟无输出）
    node = open(os.path.join(_PKG, "carla_end_to_end_nn",
                             "end_to_end_node.py"), encoding="utf-8").read()
    assert "epochs=40" not in node, "ROS 节点回退仍是 40 轮，耗时过长"
    assert "verbose=0)" not in node, \
        "ROS 节点回退训练静默进行，几分钟无输出会被当成卡死"


def test_carla_funcs_keep_sensor_references():
    """collect_carla 与 test_carla 都必须保留相机句柄（回归测试）。

    历史缺陷：`cc.make_rgb_camera(...)` 的返回值被丢弃，sensor 无引用被
    Python 垃圾回收，Actor 留在仿真里但回调失效——表现为采集到 0 帧、
    steer 恒为 0，并伴随 CARLA 警告：
      sensor object went out of the scope but the sensor is still alive in the simulation
    """
    src = open(os.path.join(_PKG, "main.py"), encoding="utf-8").read()
    for fn in ("collect_carla", "test_carla"):
        i_fn = src.index(f"def {fn}(")
        seg = src[i_fn:i_fn + 3500]
        assert "sensors.append(cc.make_rgb_camera(" in seg, \
            f"{fn} 未保留相机句柄（会被 GC 回收）"
        assert "for s in sensors:" in seg, f"{fn} 缺少传感器清理循环"
        assert "cc.restore_async(" in seg, f"{fn} 退出未恢复异步模式"


def test_carla_funcs_wrap_cleanup_in_finally():
    """两个 CARLA 函数的清理都必须在 finally 内（回归测试）。

    历史缺陷：清理裸放在函数末尾，中途抛异常（如连接超时）会被跳过，
    留下残留 actor 与同步模式。
    """
    src = open(os.path.join(_PKG, "main.py"), encoding="utf-8").read()
    for fn in ("collect_carla", "test_carla"):
        i_fn = src.index(f"def {fn}(")
        i_next = src.find("\ndef ", i_fn + 1)
        seg = src[i_fn:i_next if i_next > 0 else len(src)]
        assert "try:" in seg and "finally:" in seg, f"{fn} 缺少 try/finally"
        i_fin = seg.index("finally:")
        for call in ("s.stop()", "s.destroy()", "vehicle.destroy()",
                     "cc.restore_async("):
            assert seg.index(call) > i_fin, f"{fn}: {call} 不在 finally 内"


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


def test_sim_params_keys_are_declared_by_node():
    """config/sim_params.yaml 里的每个键，节点都必须 declare_parameter 过。

    rclpy 对**未声明**的参数覆盖项是**静默忽略**的（不报错、不警告）。
    所以 yaml 里写一个节点没声明的键，等于给使用者一个假开关：
    改了它不会有任何效果，却会让人以为改了。实测该文件里曾有 5 个这样的键
    （ego_blueprint / fixed_delta_seconds / image_width / image_height / camera_fov）。
    """
    import re
    cfg = open(os.path.join(_PKG, "config", "sim_params.yaml"),
               encoding="utf-8").read()
    node = open(os.path.join(_PKG, "carla_end_to_end_nn", "end_to_end_node.py"),
                encoding="utf-8").read()

    # yaml：只取 ros__parameters: 之后、缩进更深的那一段里的 "key:"
    body = cfg.split("ros__parameters:", 1)
    assert len(body) == 2, "sim_params.yaml 里找不到 ros__parameters:"
    keys = set()
    for line in body[1].splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if not line.startswith("    "):        # 缩进退回到顶层 → 该段结束
            break
        m = re.match(r"\s+([A-Za-z_]\w*)\s*:", line)
        if m:
            keys.add(m.group(1))
    assert keys, "没有从 sim_params.yaml 里解析出任何键"

    declared = set(re.findall(r'declare_parameter\(\s*"([^"]+)"', node))
    assert declared, "没有从 end_to_end_node.py 里解析出任何 declare_parameter"
    undeclared = sorted(keys - declared)
    assert not undeclared, (
        f"sim_params.yaml 里这些键节点没有声明，改了不会生效：{undeclared}；"
        f"节点实际声明的是 {sorted(declared)}")


def test_documented_launch_args_are_declared():
    """文档/脚本里出现的 `名字:=值`，必须真的被对应 launch 声明过。

    ROS 2 对未声明的 launch 参数**不报错**，直接忽略；ROS 1 同理。
    所以文档里写一个 launch 不认识的参数，使用者照抄后什么也没发生，
    排查起来极其费劲。实测 main.sh 的示例里曾出现 `mode:=test`
    ——它只有 ROS 1 的 main.launch 认，在 Humble 上是空操作。
    """
    import re

    def declared(path):
        text = open(path, encoding="utf-8").read()
        if path.endswith(".py"):
            return set(re.findall(r"DeclareLaunchArgument\(\s*['\"]([^'\"]+)", text))
        return set(re.findall(r'<arg\s+name="([^"]+)"', text))

    ros2 = declared(os.path.join(_PKG, "launch", "main.launch.py"))
    ros1 = declared(os.path.join(_PKG, "launch", "main.launch"))
    assert ros2 and ros1, "没有解析出 launch 参数"

    targets = [
        os.path.join(_PKG, "main.sh"),
        os.path.join(_PKG, "launch", "main.launch.py"),
        os.path.join(_PKG, "launch", "main.launch"),
        os.path.join(_PKG, "README.md"),
        os.path.join(_PKG, "..", "..", "..", "docs", "ground",
                     "carla_end_to_end_nn.md"),
        os.path.join(_PKG, "..", "..", "..", "docs", "ground", "ad_agent.md"),
    ]

    problems = []
    for path in targets:
        if not os.path.isfile(path):
            continue
        for lineno, line in enumerate(open(path, encoding="utf-8"), 1):
            if ":=" not in line:
                continue
            # 只看真正的启动命令行；解释某个参数坑的注释行不算命令
            if "ros2 launch" in line:
                allowed, tag = ros2, "ROS 2"
            elif "roslaunch" in line:
                allowed, tag = ros1, "ROS 1"
            elif "main.sh" in line and "--launch" in line:
                # main.sh 按装的是 ROS 1 还是 ROS 2 分派，参数必须两边都认
                allowed, tag = ros2 & ros1, "ROS 1 与 ROS 2 都"
            else:
                continue
            for n in re.findall(r"([A-Za-z_]\w*):=", line):
                if n not in allowed:
                    problems.append(
                        f"{os.path.basename(path)}:{lineno} 的 {n}:= "
                        f"在 {tag} 未声明（可用的有 {sorted(allowed)}）")
    assert not problems, "文档/脚本里的 launch 参数没被声明：\n  " + "\n  ".join(problems)


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
