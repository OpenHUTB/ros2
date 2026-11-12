"""入口与节点层的轻量 smoke test。

覆盖评审提到的四件事：
1. 参数真源：`config/landing.yaml` 的键必须与 `ros_node.DEFAULT_PARAMETERS` 完全一致；
2. 权重发现：`_resolve_checkpoint()` 能在包目录里找到 `results/*.pt`，找不到时给补救提示；
3. 入口能起能退：`python main.py --selftest` 必须返回 0；
4. launch 文件能正常展开（缺 launch 环境时跳过）。

这些用例都不需要 MuJoCo 之外的重依赖；需要 torch 的用例用 `importorskip`。
"""

import importlib.util
import re
import subprocess
import sys
from pathlib import Path

import pytest

PKG_ROOT = Path(__file__).resolve().parent.parent
PKG_DIR = PKG_ROOT / "uav_fault_emergency_landing"
CONFIG = PKG_ROOT / "config" / "landing.yaml"


def _yaml_parameter_keys() -> list[str]:
    """从 landing.yaml 里抠出 `ros__parameters:` 下的键（不依赖 PyYAML）。"""
    text = CONFIG.read_text(encoding="utf-8")
    body = text.split("ros__parameters:", 1)[1]
    keys = []
    for line in body.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if line.startswith("    ") and not line.startswith("      "):
            match = re.match(r"([A-Za-z_][A-Za-z0-9_]*):", stripped)
            if match:
                keys.append(match.group(1))
        elif line and not line.startswith(" "):
            break
    return keys


def test_yaml_keys_match_node_parameters():
    """改了一处忘了另一处，会在这里直接失败。"""
    from uav_fault_emergency_landing import ros_node as RN

    declared = list(RN.DEFAULT_PARAMETERS)
    from_yaml = _yaml_parameter_keys()
    assert from_yaml, "config/landing.yaml 里没解析到参数键"
    assert sorted(from_yaml) == sorted(declared), (
        f"yaml 有而节点没有：{sorted(set(from_yaml) - set(declared))}；"
        f"节点有而 yaml 没有：{sorted(set(declared) - set(from_yaml))}")


def test_default_publish_rate_matches_config():
    """评审指出的 50 Hz / 20 Hz 不一致：文档与配置都对齐到 20 Hz。"""
    from uav_fault_emergency_landing import ros_node as RN

    text = CONFIG.read_text(encoding="utf-8")
    match = re.search(r"publish_rate:\s*([0-9.]+)", text)
    assert match, "landing.yaml 里没有 publish_rate"
    assert float(match.group(1)) == pytest.approx(RN.DEFAULT_PARAMETERS["publish_rate"])
    assert RN.DEFAULT_PARAMETERS["publish_rate"] == pytest.approx(20.0)


def test_resolve_checkpoint_finds_package_weights():
    from uav_fault_emergency_landing import ros_node as RN

    for name in (RN.DEFAULT_SCORE_CHECKPOINT, RN.DEFAULT_GUIDANCE_CHECKPOINT):
        path = Path(RN._resolve_checkpoint(name))
        assert path.is_file(), f"{name} 没找到"
        assert path.stat().st_size > 0


def test_resolve_checkpoint_missing_gives_hint():
    from uav_fault_emergency_landing import ros_node as RN

    with pytest.raises(RuntimeError) as excinfo:
        RN._resolve_checkpoint("results/definitely_missing.pt")
    message = str(excinfo.value)
    assert "先训练一次" in message
    assert "--gen-data" in message and "--train" in message


def test_launch_file_expands():
    """launch 文件能 import 且能产出描述对象（缺 launch / ament / 未安装时跳过）。"""
    pytest.importorskip("launch")
    ament = pytest.importorskip("ament_index_python")
    try:
        ament.get_package_share_directory("uav_fault_emergency_landing")
    except Exception as exc:  # 包还没 colcon build，属于环境前提而非代码问题
        pytest.skip(f"包尚未安装到 ament 索引：{exc}")
    path = PKG_ROOT / "launch" / "landing.launch.py"
    spec = importlib.util.spec_from_file_location("_landing_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert description is not None
    assert len(description.entities) > 0


def test_cli_selftest_entry_returns_zero():
    """真正跑一次入口：能启动、能跑完自检、能以 0 退出。"""
    result = subprocess.run(
        [sys.executable, str(PKG_ROOT / "main.py"), "--selftest"],
        cwd=str(PKG_ROOT), capture_output=True, text=True, timeout=900,
    )
    tail = (result.stdout or "")[-400:] + (result.stderr or "")[-400:]
    assert result.returncode == 0, f"--selftest 返回 {result.returncode}\n{tail}"
    assert "自检" in result.stdout or "OK" in result.stdout


def test_checkpoint_restores_arch_from_file(tmp_path):
    """结构必须从检查点里恢复，而不是用模块默认值。"""
    torch = pytest.importorskip("torch")
    from uav_fault_emergency_landing import model as Md

    score = Md.build_score_module((8, 12))
    path = Md.save_checkpoint(tmp_path / "score.pt", "score", score,
                              {"note": "narrow channels"})
    loaded, meta = Md.load_checkpoint(path)
    assert tuple(loaded.channels) == (8, 12)
    assert Md.n_parameters(loaded) == Md.n_parameters(score)
    assert meta["note"] == "narrow channels"

    guidance = Md.build_guidance_module((32,))
    gpath = Md.save_checkpoint(tmp_path / "guidance.pt", "guidance", guidance)
    gloaded, _ = Md.load_checkpoint(gpath)
    assert tuple(gloaded.hidden) == (32,)
    assert Md.n_parameters(gloaded) == Md.n_parameters(guidance)


def test_checkpoint_rejects_newer_arch_version(tmp_path):
    torch = pytest.importorskip("torch")
    from uav_fault_emergency_landing import model as Md

    payload = {
        "arch": {"kind": "guidance", "arch_version": Md.ARCH_VERSION + 1,
                 "hidden": [64]},
        "state_dict": Md.build_guidance_module().state_dict(),
        "meta": {},
    }
    path = tmp_path / "future.pt"
    torch.save(payload, str(path))
    with pytest.raises(ValueError) as excinfo:
        Md.load_checkpoint(path)
    assert "版本" in str(excinfo.value)
