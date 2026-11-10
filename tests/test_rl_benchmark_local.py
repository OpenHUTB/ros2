#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_rl_benchmark 本地测试（不需要 ROS / 仿真器 / torch）.

覆盖：
    A. 参数解析与配置加载（_parse_arch / _parse_seeds / load_config）
    B. 环境定位（能根据源码布局 / devel 布局找到 uav_ppo_nav 的 env.py）
    C. 算法工厂的纯函数（arch_str）与算法名校验
    D. **评估指标计算**：用桩策略驱动真实 NavEnv，验证成功率/超时率/路径效率/
       动作平滑度算得对不对（这是本包最核心的逻辑，不需要训练框架即可验证）
    E. 报告汇总（有 pandas 才算；缺库则跳过）

用法: python3 tests/test_rl_benchmark_local.py
"""
from __future__ import annotations

import os
import sys

import numpy as np

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
SCRIPTS = os.path.join(ROOT, "src", "air", "uav_rl_benchmark", "scripts")
CONFIG = os.path.join(ROOT, "src", "air", "uav_rl_benchmark", "config", "benchmark.yaml")
sys.path.insert(0, SCRIPTS)

import algos as A            # noqa: E402
import benchmark as B        # noqa: E402
import main as M             # noqa: E402
import nav_env as NE         # noqa: E402


# ----------------------------------------------------------------- 桩策略
class HoverPolicy(object):
    """永远悬停：到不了目标，回合必然靠 max_steps 截断 -> 全是"超时"."""

    def predict(self, obs, deterministic=True):
        return np.zeros(4, dtype=np.float32), None


class GoalSeekPolicy(object):
    """最简单的比例控制器（直接读观测里的世界系目标偏移，不依赖任何网络）.

    观测第 16~18 维就是 goal_rel / GOAL_SCALE，所以可以反解出世界系目标方向。
    """

    def predict(self, obs, deterministic=True):
        g = np.asarray(obs, dtype=np.float64).reshape(-1)[16:19] * 8.0
        a = np.clip(g / 3.0, -1.0, 1.0)
        return np.array([a[0], a[1], a[2], 0.0], dtype=np.float32), None


# ================================================================= A. 参数解析
def test_parse():
    print("== A. 参数解析 / 配置加载 ==")
    check(M._parse_arch("64,64") == (64, 64), "_parse_arch('64,64')")
    check(M._parse_arch("64x64") == (64, 64), "_parse_arch('64x64')（x 分隔也支持）")
    check(M._parse_arch([256, 256]) == (256, 256), "_parse_arch(list)")
    check(M._parse_seeds("0,1,2") == (0, 1, 2), "_parse_seeds('0,1,2')")
    check(M._parse_seeds([3]) == (3,), "_parse_seeds(list)")

    cfg = M.load_config(CONFIG)
    if cfg:
        check("algos" in cfg.get("matrix", {}), "配置文件含 matrix.algos")
        check(cfg["matrix"]["steps"] == 100000, "配置文件 matrix.steps = 100000")
        check(cfg["eval"]["episodes"] == 100, "配置文件 eval.episodes = 100")
        check(len(cfg["matrix"]["archs"]) == 3, "配置文件列出 3 种网络结构")
    else:
        print("  [跳过] 未安装 PyYAML 或配置缺失")


# ================================================================= B. 环境定位
def test_nav_env():
    print("== B. 环境定位（复用 uav_ppo_nav 的 NavEnv）==")
    d = NE.find_nav_scripts()
    check(os.path.isfile(os.path.join(d, "env.py")), "定位到含 env.py 的目录")
    check(os.path.isfile(os.path.join(d, "policy.py")), "同目录含 policy.py")
    NavEnv, policy = NE.load_nav_env()
    env = NavEnv(num_obstacles=4, seed=0)
    obs, _ = env.reset()
    check(obs.shape == (22,), "复用到的 NavEnv 观测维度 22")
    check(policy.OBS_DIM == 22 and policy.ACT_DIM == 4, "policy 接口常量一致")
    env.close()


# ================================================================= C. 算法工厂
def test_algos():
    print("== C. 算法工厂（纯函数部分）==")
    check(A.arch_str([64, 64]) == "64x64", "arch_str([64,64]) -> '64x64'")
    check(A.arch_str((256, 256)) == "256x256", "arch_str((256,256)) -> '256x256'")
    check(A.arch_str([64, 64, 64]) == "64x64x64", "arch_str([64,64,64]) -> '64x64x64'")
    check(set(A.ALGOS) == {"ppo", "sac", "td3"}, "ALGOS = ppo / sac / td3")
    try:
        A.make_model("dqn", None)
        check(False, "未知算法名应抛 ValueError")
    except ValueError:
        check(True, "未知算法名抛 ValueError（与是否安装 sb3 无关）")
    except ImportError:
        check(False, "未知算法名不应走到 ImportError 分支")
    # 已知算法 + 无 sb3 时才应抛 ImportError（原写法把这条分支写在 ValueError 之后，永远走不到）
    try:
        import stable_baselines3  # noqa: F401,WPS433
        have_sb3 = True
    except Exception:                                       # noqa: BLE001
        have_sb3 = False
    if have_sb3:
        print("  [跳过] 本机已装 sb3，无法验证「无 sb3 时抛 ImportError」这一分支")
    else:
        try:
            A.make_model("ppo", None)
            check(False, "无 sb3 时应抛 ImportError")
        except ImportError:
            check(True, "无 sb3 时 make_model 抛 ImportError")


# ================================================================= C2. 参数透传
def test_run_sweep_wiring(tmp_dir):
    """回归测试：run_sweep 必须接收并透传 eval_freq / eval_episodes_during.

    之前正是这里出过 bug —— main.py 给 run_sweep 传了 eval_freq，但 run_sweep 签名里
    没有这个参数，`python3 main.py --sweep` 直接 TypeError 崩溃（评审发现的第 1 条）；
    而且就算不崩，配置里的 eval.freq 也会被 run_one 的默认值 20000 顶掉，
    导致文档里"配置文件是真读的"这句话对 eval.freq 不成立。
    """
    print("== C2. run_sweep 参数透传（回归：此前 --sweep 直接 TypeError）==")
    import inspect
    params = inspect.signature(B.run_sweep).parameters
    check("eval_freq" in params and "eval_episodes_during" in params,
          "run_sweep 签名含 eval_freq / eval_episodes_during")

    calls = []
    orig_run_one = B.run_one

    def _fake_run_one(algo, **kw):
        calls.append((algo, kw))
        return {}

    B.run_one = _fake_run_one
    try:
        B.run_sweep(algos=("ppo",), archs=((64, 64),), steps=1234, seeds=(7,),
                    obstacles=5, out_dir=tmp_dir, eval_episodes=9,
                    eval_freq=4321, eval_episodes_during=3)
    finally:
        B.run_one = orig_run_one

    check(len(calls) == 1, "run_sweep 调用了 1 次 run_one")
    if calls:
        algo, kw = calls[0]
        check(algo == "ppo", "算法名透传")
        check(kw.get("eval_freq") == 4321, "eval_freq 透传（此前被默认值 20000 顶掉）")
        check(kw.get("eval_episodes_during") == 3, "eval_episodes_during 透传")
        check(kw.get("steps") == 1234 and kw.get("obstacles") == 5
              and kw.get("eval_episodes") == 9, "steps / obstacles / eval_episodes 也透传")

    # 再走一遍 main.py 的 --sweep 分支：这正是当初崩溃的那条调用路径
    captured = {}
    orig_sweep = B.run_sweep

    def _fake_sweep(**kw):
        captured.update(kw)
        return "stub"

    B.run_sweep = _fake_sweep
    argv_bak = list(sys.argv)
    try:
        sys.argv = ["main.py", "--sweep", "--steps", "1000", "--seeds", "0",
                    "--out", tmp_dir,
                    "--config", os.path.join(tmp_dir, "no_such_config.yaml")]
        M.main()
    finally:
        B.run_sweep = orig_sweep
        sys.argv = argv_bak
    check("eval_freq" in captured,
          "main.py --sweep 调用 run_sweep 不再 TypeError（eval_freq 被接受）")
    check(captured.get("eval_episodes_during") == 20,
          "eval 的 during_episodes 从配置默认值透传（=20）")


# ================================================================= C3. 默认值一致性
def test_eval_freq_defaults_consistent():
    """五处 eval_freq 默认值必须一致。

    回归背景：这五处曾经不一致 —— config / launch / run_one 都是 20000，
    而 main.py 里的兜底值是 25000，会出现「配置写一个数、实际按另一个数跑」
    这种要查半天的问题。评审是靠人眼发现的，说明缺测试覆盖。
    """
    print("== C3. eval_freq 默认值五处一致性（回归：main.py 兜底曾是 25000）==")
    import inspect
    import re
    pkg = os.path.join(ROOT, "src", "air", "uav_rl_benchmark")

    def read(*parts):
        with open(os.path.join(pkg, *parts), encoding="utf-8") as fh:
            return fh.read()

    cfg = int(re.search(r"^\s*freq:\s*(\d+)", read("config", "benchmark.yaml"), re.M).group(1))
    launch = int(re.search(r'name="eval_freq"\s+default="(\d+)"',
                           read("launch", "main.launch")).group(1))
    # 函数默认值用 inspect 读，别用正则 —— run_sweep 签名里有 archs=(DEFAULT_ARCH,)
    # 这种带右括号的默认值，[^)]*? 会提前截断
    run_one = int(inspect.signature(B.run_one).parameters["eval_freq"].default)
    run_sweep = int(inspect.signature(B.run_sweep).parameters["eval_freq"].default)
    main_fb = int(re.search(r'v\.get\("freq",\s*(\d+)\)', read("scripts", "main.py")).group(1))

    check(len({cfg, launch, run_one, run_sweep, main_fb}) == 1,
          "五处默认值一致（config=%d launch=%d run_one=%d run_sweep=%d main兜底=%d）"
          % (cfg, launch, run_one, run_sweep, main_fb))
    check(main_fb == 20000, "main.py 兜底值 = 20000（曾是 25000）")


# ================================================================= D. 评估指标
def test_evaluate_metrics():
    print("== D. 评估指标计算（桩策略 + 真实 NavEnv）==")
    NavEnv, _ = NE.load_nav_env()

    # D1. 悬停策略：必然超时
    m = B.evaluate(HoverPolicy(), NavEnv, episodes=5, obstacles=0, seed=7)
    check(m["success_rate"] == 0.0, "悬停策略成功率 = 0")
    check(abs(m["timeout_rate"] - 1.0) < 1e-9, "悬停策略超时率 = 100%")
    check(abs(m["success_rate"] + m["crash_rate"] + m["timeout_rate"] - 1.0) < 1e-9,
          "三类结果占比之和 = 1")
    check(m["mean_steps"] > 100, "悬停策略平均步数很大（%0.0f 步）" % m["mean_steps"])
    check(m["action_smoothness"] < 1e-9, "悬停策略动作恒定 -> 平滑度 = 0")
    check(m["path_efficiency"] == 0.0,
          "悬停策略路径效率 = 0（原地不动不能算满分）")

    # D2. 比例控制策略：无障碍环境下应能到目标
    m2 = B.evaluate(GoalSeekPolicy(), NavEnv, episodes=30, obstacles=0, seed=7)
    check(m2["success_rate"] > 0.5,
          "比例控制策略成功率 > 50%%（实测 %.0f%%）" % (m2["success_rate"] * 100))
    check(m2["success_rate"] > m["success_rate"], "比例控制优于悬停（指标有区分度）")
    # 平滑度必须能区分"动作恒定"与"动作变化"——原来写的 `>= 0.0` 是恒真断言，没有验证能力
    check(m["action_smoothness"] == 0.0 and m2["action_smoothness"] > 0.0,
          "平滑度能区分「动作恒定」(%.3f) 与「动作变化」(%.3f)"
          % (m["action_smoothness"], m2["action_smoothness"]))
    check(0.0 < m2["path_efficiency"] <= 1.0,
          "比例控制路径效率落在 (0,1] 内（实测 %.2f）" % m2["path_efficiency"])
    print("      悬停:  成功率 %.0f%%  步数 %.0f  效率 %.2f  平滑 %.3f"
          % (m["success_rate"] * 100, m["mean_steps"], m["path_efficiency"],
             m["action_smoothness"]))
    print("      比例控: 成功率 %.0f%%  步数 %.0f  效率 %.2f  平滑 %.3f"
          % (m2["success_rate"] * 100, m2["mean_steps"], m2["path_efficiency"],
             m2["action_smoothness"]))

    # D3. 指标字段齐全
    need = set(B.RESULT_FIELDS) - {"algo", "arch", "seed", "steps", "obstacles",
                                   "eval_episodes", "train_time_s", "train_fps"}
    check(need <= set(m2.keys()), "评估返回包含全部指标字段：%s" % sorted(need))


# ================================================================= E. 报告汇总
def test_report(tmp_dir):
    print("== E. 报告汇总与绘图 ==")
    try:
        import pandas  # noqa: F401,WPS433
    except Exception:                                       # noqa: BLE001
        print("  [跳过] 缺少 pandas")
        return
    # 造一份最小的 results.csv
    import csv
    rows = []
    for algo in ("ppo", "sac"):
        for seed in (0, 1):
            r = {k: 0.0 for k in B.RESULT_FIELDS}
            r.update({"algo": algo, "arch": "64x64", "seed": seed, "steps": 1000,
                      "obstacles": 16, "eval_episodes": 5,
                      "train_time_s": 10.0 + seed, "train_fps": 100.0,
                      "success_rate": 0.5 if algo == "ppo" else 0.25,
                      "crash_rate": 0.2, "timeout_rate": 0.3,
                      "mean_steps": 40.0, "mean_reward": 1.0,
                      "mean_path_len": 5.0, "path_efficiency": 0.7,
                      "action_smoothness": 0.1})
            rows.append(r)
    os.makedirs(tmp_dir, exist_ok=True)
    with open(os.path.join(tmp_dir, "results.csv"), "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=B.RESULT_FIELDS)
        w.writeheader()
        w.writerows(rows)

    md = B.__dict__  # noqa: F841  (占位，避免 linter 抱怨)
    import report as R  # noqa: E402,WPS433
    path = R.build_report(tmp_dir, make_plots=False)
    check(path is not None and os.path.isfile(path), "生成了 report.md")
    txt = open(path, encoding="utf-8").read()
    low = txt.lower()
    check("ppo" in low and "sac" in low, "报告表格含两种算法")
    check("成功率" in txt, "报告含成功率列")
    check("结论要点" in txt, "报告含结论要点小节")


# ================================================================= F. 仓库卫生
def test_repo_hygiene():
    """仓库卫生检查：.gitignore 与源码文件.

    两条都来自评审意见。以前只是一次性脚本，评审指出"仓库里搜不到那个检测脚本"——
    现在固化成测试，任何人跑桩测试都会顺手校验：

    * **孤儿注释**：.gitignore 里「注释后面没跟任何规则」的段落
      （曾在尾部留下 4 段：追加脚本跳过了已存在的规则，却仍写了注释）；
    * **UTF-8 BOM**：`report.py` 曾被写入 BOM，而它是 755 可执行 ——
      Linux 下 `./report.py` 的 shebang 会直接失效。
    """
    print("== F. 仓库卫生（.gitignore 孤儿注释 / 源码 BOM）==")

    gi = os.path.join(ROOT, ".gitignore")
    if not os.path.isfile(gi):
        print("  [跳过] 找不到仓库根 .gitignore（可能未在仓库内运行）")
    else:
        lines = open(gi, encoding="utf-8").read().splitlines()
        # 把「连续注释（中间可有空行）」当成一个注释块，检查块后是否真的跟着规则；
        # 注意不能把多行注释块误判成孤儿（注释块后面跟规则是正常的）
        orphans = []
        i = 0
        while i < len(lines):
            if lines[i].lstrip().startswith("#"):
                j = i
                while j < len(lines) and (not lines[j].strip()
                                          or lines[j].lstrip().startswith("#")):
                    j += 1
                if j >= len(lines):
                    orphans.append("行%d: %s" % (i + 1, lines[i].strip()))
                i = j
            else:
                i += 1
        check(not orphans, "无孤儿注释" + ("" if not orphans else "：%s" % orphans[:3]))
        check("**/results/runs/**/*.zip" in lines, ".gitignore 含本模块的跑次产物规则")
        check(open(gi, "rb").read(3) != b"\xef\xbb\xbf", ".gitignore 无 UTF-8 BOM")

    # 包内 .py 不允许 BOM：这些脚本是 755 可执行，BOM 会让 shebang 失效
    bad = []
    for d in (os.path.join(ROOT, "src", "air", "uav_rl_benchmark"),
              os.path.join(ROOT, "tests")):
        for dp, _dn, fn in os.walk(d):
            for x in fn:
                if x.endswith(".py"):
                    p = os.path.join(dp, x)
                    if open(p, "rb").read(3) == b"\xef\xbb\xbf":
                        bad.append(os.path.relpath(p, ROOT))
    check(not bad, "包内 .py 均无 UTF-8 BOM" + ("" if not bad else "：%s" % bad))


def main():
    tmp_dir = os.path.join(HERE, "_tmp_bench_report")
    try:
        test_parse()
        test_nav_env()
        test_algos()
        test_run_sweep_wiring(tmp_dir)
        test_eval_freq_defaults_consistent()
        test_evaluate_metrics()
        test_report(tmp_dir)
        test_repo_hygiene()
    finally:
        import shutil
        shutil.rmtree(tmp_dir, ignore_errors=True)

    print("\n==================== 结果 ====================")
    print("通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    if FAIL:
        for f in FAIL:
            print("  FAIL: %s" % f)
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
