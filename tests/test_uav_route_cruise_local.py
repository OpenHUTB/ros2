#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_route_cruise 的本地测试（不需要 ROS / CARLA / torch / sb3）.

覆盖重点
--------
1. 航线几何：闭合、长度、最近点投影、前方航点环绕、完成度统计
2. 环境：观测/动作维度、单步推进、完成一圈判定、障碍摆放（含压线障碍）
3. 策略：权重文件形状与观测/动作维度一致、输出落在 [-1,1]
4. **包装正确性**（这是端到端视觉控制在别的 PR 里翻过车的地方）：
   - 入口 `main.py` 在 CMakeLists 里走 catkin_install_python
   - 被 `route_node.py` import 的模块（policy/route/route_env）必须以普通文件安装，
     否则 catkin 会包 wrapper，import 到的模块里没有 main 等符号
   - route_node.py 能被 `import` 一次（语法/依赖层面自洽）
5. 配置与代码一致性：`route_node.py` 里 read 的每个参数都在 config/*.yaml 里出现
"""
from __future__ import annotations

import os
import re
import sys

import numpy as np
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
# 仓库布局：tests/test_uav_route_cruise_local.py  与  src/air/uav_route_cruise/
PKG = os.path.normpath(os.path.join(_HERE, "..", "src", "air", "uav_route_cruise"))
if not os.path.isdir(os.path.join(PKG, "scripts")):
    # 兼容「测试文件临时放在包内」的情况
    PKG = os.path.normpath(os.path.join(_HERE, "uav_route_cruise"))
SCRIPTS = os.path.join(PKG, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)


def _read(path: str) -> str:
    with open(path, encoding="utf-8") as fh:
        return fh.read()


# ============================================================ 1. 航线几何
def test_default_route_is_closed_and_has_expected_length():
    from route import default_route
    pts = default_route()
    assert pts.ndim == 2 and pts.shape[1] == 3
    assert np.allclose(pts[0], pts[-1]), "航线必须首尾闭合"
    assert abs(pts[0, 2] - 1.5) < 1e-9, "默认高度应为 1.5 m"


def test_route_length_matches_perimeter():
    from route import Route, default_route
    r = Route(default_route(width=20.0, depth=14.0))
    # 20×14 圆角矩形（角半径 3）的总长 ≈ 2*(20-2*3) + 2*(14-2*3) + 2*pi*3
    expect = 2 * (20 - 6) + 2 * (14 - 6) + 2 * np.pi * 3
    assert abs(r.length - expect) < 0.5, "航线总长 %s 与解析值 %s 不符" % (r.length, expect)


def test_projection_returns_signed_cross_track():
    from route import Route, default_route
    r = Route(default_route())
    # 取航线第一段的中点，沿法向偏移 1 m，横向偏差应约为 1
    p0 = r.points[0, :2]
    d = r.seg[0, :2]
    d = d / (np.linalg.norm(d) or 1.0)
    n = np.array([-d[1], d[0]])          # 左法向
    probe = p0 + d * 0.5 + n * 1.0
    _, _s, _, cross = r.project(probe)
    assert abs(abs(cross) - 1.0) < 0.15, "横向偏差应约为 1 m，实际 %s" % cross


def test_waypoint_ahead_wraps_around():
    from route import Route, default_route
    r = Route(default_route())
    # 在接近终点处前视，应当环绕到起点附近，而不是返回终点
    near_end = r.length - 0.5
    wp = r.waypoint_ahead(r.points[0, :2], near_end, 4.0)
    start_xy = r.points[0, :2]
    assert np.linalg.norm(wp - start_xy) < 5.0, "前视应环绕到航线起点附近"


def test_completion_counts_visited_arc():
    from route import Route, default_route
    r = Route(default_route())
    half = np.linspace(0.0, r.length * 0.5, 40)
    c = r.completion(half, bins=36)
    assert 0.4 <= c <= 0.6, "走了一半弧长，完成度应约 50%%，实际 %s" % c


# ============================================================ 2. 环境
def test_env_obs_and_action_dims():
    import route_env as RE
    env = RE.RouteEnv(seed=0)
    obs, _ = env.reset(seed=0)
    assert obs.shape == (RE.OBS_DIM,)
    assert RE.OBS_DIM == RE.N_SECTORS + 6
    assert env.action_space.shape == (RE.ACT_DIM,)
    assert np.all(np.abs(obs) < 10.0), "观测应在合理范围内"


def test_env_step_advances_and_reports():
    import route_env as RE
    env = RE.RouteEnv(seed=0, num_obstacles=0)
    env.reset(seed=0)
    p0 = env.pos.copy()
    for _ in range(5):
        _o, rew, term, trunc, info = env.step(np.array([0.5, 0.0, 0.0, 0.0]))
        if term or trunc:
            break
    # 注意：不用弧长 s 判断，因为起终点闭合、投影在首尾附近可能落在不同段上
    assert np.linalg.norm(env.pos - p0) > 0.05, "给速度指令后位置应当变化"
    assert "cross" in info and "progress" in info


def test_env_on_route_obstacles_really_sit_on_route():
    """配置了 on_route_frac 后，必须有障碍离航线中心 < 障碍半径。"""
    import route_env as RE
    env = RE.RouteEnv(seed=0, num_obstacles=14, on_route_frac=0.35)
    env.reset(seed=0)
    ds = sorted(abs(env.route.project(o)[3]) for o in env.obstacles_xy)
    on_route = [d for d in ds if d < env.obstacle_radius]
    assert len(on_route) >= 1, "应当有压在航线上的障碍，实际最近距离 %s" % ds[:4]


def test_env_detects_lap_completion():
    """用纯循迹控制器跑完一圈，应判定为成功。"""
    import route_env as RE
    env = RE.RouteEnv(seed=0, num_obstacles=0, max_steps=900)
    env.reset(seed=0)
    done = False
    info = {}
    steps = 0
    while not done and steps < 900:
        _, s, _, _ = env.route.project(env.pos[:2])
        wp = env.route.waypoint_ahead(env.pos[:2], s, 4.0)
        d = wp - env.pos[:2]
        n = np.linalg.norm(d) or 1.0
        a = np.array([d[0] / n, d[1] / n,
                      float(np.clip(-(env.pos[2] - env.route.points[0, 2]), -1, 1)), 0.0])
        _o, _r, term, trunc, info = env.step(a)
        done = bool(term or trunc)
        steps += 1
    assert done and info.get("success"), "无障碍时循迹应能完成一圈"


# ============================================================ 3. 策略
@pytest.mark.parametrize("fname", ["route_ppo_s0.npz"])
def test_policy_weights_match_dims(fname):
    import policy as P
    import route_env as RE
    path = os.path.join(PKG, "models", fname)
    if not os.path.isfile(path):
        pytest.skip("权重文件不存在：%s" % path)
    pol = P.MlpPolicy.from_npz(path)
    d = np.load(path)
    in_dim = d["h0_w"].shape[1]
    assert in_dim == RE.OBS_DIM, "权重输入维度 %d 与观测维度 %d 不一致" % (in_dim, RE.OBS_DIM)
    assert d["out_w"].shape[0] == RE.ACT_DIM, "权重输出维度与动作维度不一致"
    obs = np.zeros(RE.OBS_DIM, dtype=np.float32)
    a = pol.forward(obs)
    assert a.shape == (RE.ACT_DIM,)
    assert np.all(np.abs(a) <= 1.0 + 1e-6), "动作必须落在 [-1,1]"


# ============================================================ 4. 包装正确性
def test_main_entry_is_installed_by_catkin_install_python():
    txt = _read(os.path.join(PKG, "CMakeLists.txt"))
    m = re.search(r"catkin_install_python\(PROGRAMS(.*?)\)", txt, re.S)
    assert m, "CMakeLists.txt 里应有 catkin_install_python(PROGRAMS ...)"
    assert "scripts/main.py" in m.group(1), "入口 main.py 必须由 catkin_install_python 安装"


def test_imported_modules_are_plain_installed():
    """route_node.py 里 import 的本包模块，必须在 install(FILES) 里。

    否则 catkin 会给它们包 wrapper，import 到的模块里没有函数，
    这正是「装出来能 build、一 launch 就 ModuleNotFound / AttributeError」的根因。
    """
    txt = _read(os.path.join(PKG, "CMakeLists.txt"))
    m = re.search(r"install\(FILES(.*?)\)", txt, re.S)
    assert m, "CMakeLists.txt 里应有 install(FILES ...) 安装被 import 的模块"
    installed = set(re.findall(r"scripts/([a-zA-Z_][\w]*\.py)", m.group(1)))

    node = _read(os.path.join(PKG, "scripts", "route_node.py"))
    # 本包模块名（不含第三方/标准库）
    siblings = {os.path.splitext(f)[0] for f in os.listdir(SCRIPTS) if f.endswith(".py")}
    needed = set()
    for line in node.splitlines():
        mm = re.match(r"^\s*(?:import|from)\s+([a-zA-Z_][\w]*)", line)
        if mm and mm.group(1) in siblings:
            needed.add(mm.group(1) + ".py")
    missing = needed - installed
    assert not missing, "这些被 import 的模块没以普通文件安装：%s" % sorted(missing)


def test_route_node_defines_class_and_main():
    txt = _read(os.path.join(PKG, "scripts", "route_node.py"))
    assert "class RouteCruiseNode" in txt
    assert re.search(r"^def main\(", txt, re.M), "route_node.py 必须有 main()"


def test_no_undefined_module_level_names():
    """静态检查：脚本里用到的「导入名 / 全大写常量名」必须真的有定义。

    为什么需要这条：route_node.py 依赖 rospy，pytest 里 import 不了，
    所以「运行时会 NameError」这类问题跑测试是发现不了的。
    实测踩过：把 route_env 里的 LOOKAHEAD_DISTS 删掉后，node 里仍在用它 ——
    当时 17 项测试全绿，但一 launch 就是 NameError。
    """
    import ast
    import builtins

    problems = []
    for fname in ("route_node.py", "main.py", "policy.py", "route.py",
                  "route_env.py", "train.py", "eval.py", "compare.py",
                  "multiseed_eval.py", "make_gif.py"):
        path = os.path.join(SCRIPTS, fname)
        tree = ast.parse(_read(path), filename=path)

        defined, imported = set(), set()
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                defined.add(node.name)
                for a in list(node.args.args) + list(node.args.kwonlyargs):
                    defined.add(a.arg)
            elif isinstance(node, ast.ClassDef):
                defined.add(node.name)
            elif isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
                defined.add(node.id)
            elif isinstance(node, ast.Import):
                for a in node.names:
                    imported.add((a.asname or a.name).split(".")[0])
            elif isinstance(node, ast.ImportFrom):
                for a in node.names:
                    imported.add(a.asname or a.name)
            elif isinstance(node, ast.arg):
                defined.add(node.arg)
            elif isinstance(node, ast.ExceptHandler) and node.name:
                defined.add(node.name)

        for node in ast.walk(tree):
            if not isinstance(node, ast.Name) or not isinstance(node.ctx, ast.Load):
                continue
            nm = node.id
            is_const = nm.isupper() and len(nm) > 2
            if not (is_const or nm in imported):
                continue
            if nm in defined or nm in imported or hasattr(builtins, nm):
                continue
            problems.append("%s:%d 使用了未定义的名字 %s" % (fname, node.lineno, nm))
    assert not problems, "\n".join(sorted(set(problems)))


def test_route_node_imports_exist_in_route_env():
    """route_node.py 从 route_env import 的每个名字，route_env 里必须真的有。"""
    import route_env as RE

    txt = _read(os.path.join(PKG, "scripts", "route_node.py"))
    m = re.search(r"from\s+route_env\s+import\s+([^\n#]+)", txt)
    assert m, "route_node.py 应当从 route_env 导入常量"
    names = [n.strip().split(" as ")[0].strip()
             for n in m.group(1).split(",") if n.strip()]
    missing = [n for n in names if n and not hasattr(RE, n)]
    assert not missing, "route_env 里没有这些名字：%s" % missing


def test_entry_script_prepares_import_path():
    """main.py 必须能把源码目录插到 sys.path（绕开 catkin wrapper）。"""
    txt = _read(os.path.join(PKG, "scripts", "main.py"))
    assert "UAV_ROUTE_SCRIPT_DIR" in txt, "launch 会通过该环境变量传真实源码目录"
    assert "sys.path.insert" in txt
    assert "class RouteCruiseNode" in txt, "应据此判定「哪份才是真源码」"


# ============================================================ 5. 配置一致性
def test_node_params_are_documented_in_config():
    """route_node.py 里 get_param("~k") 的每个 k，都应出现在 config/*.yaml 里。"""
    node = _read(os.path.join(PKG, "scripts", "route_node.py"))
    names = set(re.findall(r'get_param\(\s*"~([a-zA-Z_][\w]*)"', node))
    assert names, "route_node.py 里应当有 get_param 调用"
    cfg_dir = os.path.join(PKG, "config")
    blobs = "".join(_read(os.path.join(cfg_dir, f))
                    for f in os.listdir(cfg_dir) if f.endswith((".yaml", ".yml")))
    missing = [n for n in sorted(names) if n not in blobs]
    assert not missing, "这些参数在 config 里没有说明：%s" % missing


def test_launch_declares_what_it_passes():
    txt = _read(os.path.join(PKG, "launch", "main.launch"))
    args = set(re.findall(r'<arg\s+name="([a-zA-Z_][\w]*)"', txt))
    used = set(re.findall(r'\$\(arg\s+([a-zA-Z_][\w]*)\)', txt))
    missing = used - args
    assert not missing, "launch 里用了未声明的 arg：%s" % sorted(missing)
    assert 'type="main.py"' in txt, "入口必须是 main.py（作业要求入口为 main.*）"
    assert "UAV_ROUTE_SCRIPT_DIR" in txt, "launch 要把真实源码目录通过环境变量传进去"


def test_files_have_no_bom_and_no_crlf_dependency():
    """仓库要求：源码/文档用 UTF-8 无 BOM（带 BOM 会让 shebang 失效）。"""
    targets = []
    for root, _dirs, files in os.walk(PKG):
        if "__pycache__" in root or "/models" in root.replace(os.sep, "/"):
            continue
        for f in files:
            if f.endswith((".py", ".launch", ".yaml", ".xml", ".md", ".txt")):
                targets.append(os.path.join(root, f))
    assert targets
    bad = []
    for p in targets:
        with open(p, "rb") as fh:
            if fh.read(3) == b"\xef\xbb\xbf":
                bad.append(p)
    assert not bad, "以下文件带 UTF-8 BOM：%s" % bad
