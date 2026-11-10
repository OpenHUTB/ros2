# -*- coding: utf-8 -*-
"""main.py CLI / 离线闭环 / 模块源码卫生 的单测。"""

from __future__ import annotations

import os
import subprocess
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _path  # noqa: E402

PKG = os.path.dirname(os.path.dirname(_path.model_path()))   # .../humanoid_embodied_control
MAIN = os.path.join(PKG, "main.py")
SCRIPTS_MAIN = os.path.join(PKG, "scripts", "main.py")


class TestEntryPoints:
    """仓库约定：入口必须 main 开头，且真的能跑起来。"""

    @pytest.mark.parametrize("name", ["main.py", "main.sh", "main.bat"])
    def test_entry_exists(self, name):
        assert os.path.isfile(os.path.join(PKG, name)), name

    def test_model_and_requirements(self):
        assert os.path.isfile(_path.model_path())
        assert os.path.isfile(os.path.join(PKG, "requirements.txt"))

    def test_launch_exists(self):
        assert os.path.isfile(os.path.join(PKG, "launch", "main.launch"))


class TestCLI:
    def _run(self, *args):
        return subprocess.run(
            [sys.executable, MAIN, *args],
            cwd=PKG, capture_output=True, text=True, timeout=300)

    def test_help(self):
        r = self._run("--help")
        assert r.returncode == 0
        assert "--mode" in r.stdout

    def test_offline_runs_and_emits_json(self):
        r = self._run("--mode", "offline", "--seconds", "12")
        assert r.returncode in (0, 1)   # 1 = 未到达目标，属正常退出
        assert '"mode": "offline"' in r.stdout
        assert '"final_state"' in r.stdout

    def test_offline_reaches_close_goal(self):
        """目标放近一点：离线运动学应能到达（链路正确的最强证据）。"""
        r = self._run("--mode", "offline", "--seconds", "60",
                      "--goal_x", "1.0")
        assert '"success": true' in r.stdout, r.stdout[-800:]

    def test_json_out_written(self, tmp_path):
        out = tmp_path / "r.json"
        r = self._run("--mode", "offline", "--seconds", "8",
                      "--json_out", str(out))
        assert out.is_file()
        assert '"mode"' in out.read_text(encoding="utf-8")

    def test_scripts_main_is_implementation(self):
        assert os.path.isfile(SCRIPTS_MAIN)


class TestSourceHygiene:
    def test_flake8_clean(self):
        """仓库用 ament_flake8，这里先本地把关。"""
        try:
            import flake8  # noqa: F401
        except ImportError:
            pytest.skip("flake8 未安装")
        r = subprocess.run(
            [sys.executable, "-m", "flake8", "--max-line-length", "120",
             _path.model_path().rsplit("models", 1)[0]],
            capture_output=True, text=True, timeout=180)
        assert r.returncode == 0, r.stdout + r.stderr

    def test_no_conflicting_unicode_in_launch(self):
        txt = open(os.path.join(PKG, "launch", "main.launch"), encoding="utf-8").read()
        assert txt.lstrip().startswith("<launch>")
