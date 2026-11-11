"""Installed pipeline and launch argument contracts / 安装入口与启动参数。"""

import runpy
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace


def test_installed_configuration_uses_package_share(tmp_path, monkeypatch):
    import main

    package = ModuleType("ament_index_python")
    packages = ModuleType("ament_index_python.packages")
    packages.get_package_share_directory = lambda name: str(tmp_path / "share" / name)
    monkeypatch.setitem(sys.modules, "ament_index_python", package)
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", packages)
    monkeypatch.setattr(main, "MODULE_ROOT", tmp_path / "lib")
    assert main.default_config() == tmp_path / "share" / "mujoco_uav_rooftop_pv_coverage_survey" / "config" / "survey.yaml"


def test_workflow_launch_passes_only_pipeline_arguments(monkeypatch):
    launch = ModuleType("launch")
    actions = ModuleType("launch.actions")
    substitutions = ModuleType("launch.substitutions")
    launch.LaunchDescription = list
    actions.DeclareLaunchArgument = lambda name, **kwargs: SimpleNamespace(name=name, **kwargs)
    actions.ExecuteProcess = lambda **kwargs: SimpleNamespace(**kwargs)
    substitutions.LaunchConfiguration = lambda name: {"mode": "train-control", "output": "/tmp/survey data", "scenario": "dense"}[name]
    for name, module in (("launch", launch), ("launch.actions", actions), ("launch.substitutions", substitutions)):
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).resolve().parents[1] / "launch" / "rooftop_pv_workflow.launch.py"
    description = runpy.run_path(str(path))["generate_launch_description"]()
    assert [arg.name for arg in description[:3]] == ["mode", "output", "scenario"]
    assert description[-1].cmd == ["ros2", "run", "mujoco_uav_rooftop_pv_coverage_survey", "rooftop_pv_pipeline",
                                   "--mode", "train-control", "--output", "/tmp/survey data", "--scenario", "dense"]
    assert "--ros-args" not in description[-1].cmd


def test_setup_installs_both_program_entry_points(monkeypatch):
    import setuptools

    captured = {}
    monkeypatch.setattr(setuptools, "setup", lambda **kwargs: captured.update(kwargs))
    runpy.run_path(str(Path(__file__).resolve().parents[1] / "setup.py"))
    assert set(captured["py_modules"]) == {"main", "ros2_uav_controller"}
    assert "rooftop_pv_pipeline = main:main" in captured["entry_points"]["console_scripts"]
