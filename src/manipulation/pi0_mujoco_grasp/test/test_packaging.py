# -*- coding: utf-8 -*-
"""Packaging + CLI compatibility tests (no ROS 2 required)."""
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def test_package_import():
    import pi0_mujoco_grasp
    assert pi0_mujoco_grasp.__version__


def test_required_files_exist():
    for rel in (
        "models/franka_emika_panda/scene_pickplace.xml",
        "models/franka_emika_panda/panda_pick.xml",
        "models/franka_emika_panda/assets.sha256",
        "models/franka_emika_panda/LICENSE",
        "scripts/setup_model.py",
        "scripts/make_gif.py",
        "scripts/record_demo.sh",
        "launch/main.launch.py",
        "requirements.txt",
    ):
        assert (ROOT / rel).is_file(), rel


def test_duplicate_policy_stub_removed():
    """pi0_policy_stub.py 只应存在于包内，模块根目录不应再有副本。"""
    root_copy = ROOT / "pi0_policy_stub.py"
    assert not root_copy.exists(), "duplicate %s should be removed" % root_copy


def test_asset_manifest_matches_scene():
    """assets.sha256 的条目应覆盖 panda_pick.xml 引用的全部网格。"""
    import re
    text = (ROOT / "models/franka_emika_panda/panda_pick.xml").read_text(encoding="utf-8")
    referenced = set(re.findall(r'file="([^"]+)"', text))
    lines = (ROOT / "models/franka_emika_panda/assets.sha256").read_text(encoding="utf-8").splitlines()
    manifest = {line.split(None, 1)[1].strip() for line in lines if line.strip()}
    missing = referenced - manifest
    assert referenced, "no mesh references found"
    assert not missing, "manifest missing: %s" % sorted(missing)


def test_shell_syntax():
    for rel in ("scripts/record_demo.sh",):
        subprocess.run(["bash", "-n", str(ROOT / rel)], check=True)


def test_cli_ignores_ros_args():
    """ros2 launch 的 --ros-args 不能被 argparse 当成未知参数报错。"""
    pytest.importorskip("mujoco")  # app.py 依赖 mujoco（ROS 2 环境无此依赖时跳过）
    from pi0_mujoco_grasp.app import build_parser
    args, unknown = build_parser().parse_known_args(
        ["--ros-args", "-r", "__node:=pi0_mujoco_grasp"])
    assert args.record == ""
    assert args.fps == 30
    assert args.policy == "stub"
    assert unknown == ["--ros-args", "-r", "__node:=pi0_mujoco_grasp"]
