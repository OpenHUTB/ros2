# -*- coding: utf-8 -*-
"""Packaging smoke tests."""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def test_package_import():
    import dave_underwater_sim
    assert dave_underwater_sim.__version__


def test_entry_files_exist():
    for rel in ("run_demo.sh", "main.sh", "install_dave.sh",
                "launch/main.launch.py", "scripts/dave_arm_move.py",
                "scripts/dave_save_camera.py", "scripts/dave_camseq.py"):
        assert (ROOT / rel).is_file(), rel


def test_shell_syntax():
    for rel in ("run_demo.sh", "main.sh", "install_dave.sh"):
        subprocess.run(["bash", "-n", str(ROOT / rel)], check=True)


def test_demo_args_filter_ros_args():
    from dave_underwater_sim.app import demo_args
    assert demo_args([]) == ["demo"]
    assert demo_args(["valve"]) == ["valve"]
    assert demo_args(["demo", "headless"]) == ["demo", "headless"]
    # ros2 launch / Node 追加的参数应被忽略
    assert demo_args(["--ros-args", "-r", "__node:=dave_underwater_sim"]) == ["demo"]
    assert demo_args(["valve", "--ros-args", "-r", "__node:=x"]) == ["valve"]
