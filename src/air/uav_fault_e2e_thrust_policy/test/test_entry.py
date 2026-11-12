"""入口与节点参数层的 smoke test。

对应 #1341 评审提出的两件事：
1. 三处 fault 默认值必须一致（``config/policy.yaml`` / ``launch/policy.launch.py`` /
   ``ros_node.DEFAULT_PARAMETERS``），这里做三方比对；
2. 节点与入口要能被自动测试覆盖，至少验证参数读取、权重发现、入口能否启动再退出。

这些用例不需要 MuJoCo；需要 torch 的用 ``importorskip``。
"""

import importlib.util
import re
import subprocess
import sys
from pathlib import Path

import pytest

PKG_ROOT = Path(__file__).resolve().parent.parent
CONFIG = PKG_ROOT / "config" / "policy.yaml"
LAUNCH = PKG_ROOT / "launch" / "policy.launch.py"


def _yaml_parameters() -> dict:
    """读 landing 风格的 yaml：返回 key -> 原始字符串值（不依赖 PyYAML）。"""
    text = CONFIG.read_text(encoding="utf-8")
    body = text.split("ros__parameters:", 1)[1]
    values = {}
    for line in body.splitlines():
        if line and not line.startswith(" "):
            break
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        key, _, raw = stripped.partition(":")
        values[key.strip()] = raw.strip().strip('"')
    return values


def _launch_defaults() -> dict:
    """从 launch 文件里抠出 DeclareLaunchArgument 的默认值。"""
    text = LAUNCH.read_text(encoding="utf-8")
    found = re.findall(
        r'DeclareLaunchArgument\(\s*"([a-z_]+)"\s*,\s*default_value="([^"]*)"', text)
    return {name: value for name, value in found}


def test_yaml_keys_match_node_parameters():
    """yaml 里写的键必须都被节点声明，反之亦然。"""
    from uav_fault_e2e_thrust_policy import ros_node as RN

    declared = set(RN.DEFAULT_PARAMETERS)
    from_yaml = set(_yaml_parameters())
    assert from_yaml == declared, (
        f"yaml 有而节点没有：{sorted(from_yaml - declared)}；"
        f"节点有而 yaml 没有：{sorted(declared - from_yaml)}")


def test_fault_default_consistent_in_three_places():
    """评审 #1341 第 2 条：三处 fault 默认值必须一样。"""
    from uav_fault_e2e_thrust_policy import ros_node as RN

    node_default = str(RN.DEFAULT_PARAMETERS["fault"])
    yaml_default = _yaml_parameters()["fault"]
    launch_default = _launch_defaults()["fault"]
    assert node_default == yaml_default == launch_default, (
        f"node={node_default} / yaml={yaml_default} / launch={launch_default}")


def test_launch_defaults_match_node_parameters():
    """launch 暴露的那几个参数，默认值也要和节点一致（转子号、时长、时刻、严重度）。"""
    from uav_fault_e2e_thrust_policy import ros_node as RN

    defaults = _launch_defaults()
    assert defaults, "launch 文件里没解析到 DeclareLaunchArgument"
    for name, raw in defaults.items():
        if name not in RN.DEFAULT_PARAMETERS:
            continue
        expected = RN.DEFAULT_PARAMETERS[name]
        if isinstance(expected, bool):
            continue
        if isinstance(expected, (int, float)):
            assert float(raw) == pytest.approx(float(expected)), (
                f"{name}: launch={raw} vs node={expected}")
        else:
            assert raw == str(expected), f"{name}: launch={raw} vs node={expected}"


def test_publish_rate_is_parameter_not_constant():
    """评审 #1341 第 4 条：发布频率只保留参数这一处真源，不再有额外的模块常量。"""
    from uav_fault_e2e_thrust_policy import ros_node as RN

    assert "PUBLISH_RATE" not in dir(RN), "PUBLISH_RATE 常量应该已删除"
    assert float(RN.DEFAULT_PARAMETERS["publish_rate"]) == pytest.approx(20.0)
    assert float(_yaml_parameters()["publish_rate"]) == pytest.approx(20.0)


def test_resolve_checkpoint_finds_package_weights():
    from uav_fault_e2e_thrust_policy import ros_node as RN

    path = Path(RN._resolve_checkpoint(RN.DEFAULT_CHECKPOINT))
    assert path.is_file() and path.stat().st_size > 0


def test_resolve_checkpoint_missing_gives_hint():
    from uav_fault_e2e_thrust_policy import ros_node as RN

    with pytest.raises(RuntimeError) as excinfo:
        RN._resolve_checkpoint("results/definitely_missing.pt")
    message = str(excinfo.value)
    assert "--gen-data" in message and "--train" in message


def test_launch_file_expands():
    """launch 文件能 import 且能产出描述对象（缺 launch / ament / 未安装时跳过）。"""
    pytest.importorskip("launch")
    ament = pytest.importorskip("ament_index_python")
    try:
        ament.get_package_share_directory("uav_fault_e2e_thrust_policy")
    except Exception as exc:  # 还没 colcon build，属于环境前提
        pytest.skip(f"包尚未安装到 ament 索引：{exc}")
    spec = importlib.util.spec_from_file_location("_policy_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert description is not None and len(description.entities) > 0


def test_node_class_has_public_finished_flag():
    """评审 #1341 第 7 条：退出判据不再访问私有成员 ``_finished``。"""
    source = (PKG_ROOT / "uav_fault_e2e_thrust_policy" / "ros_node.py").read_text(
        encoding="utf-8")
    assert "_finished" not in source
    assert "node.finished" in source


def test_cli_selftest_entry_returns_zero():
    """真正跑一次入口：能启动、能跑完自检、能以 0 退出。"""
    result = subprocess.run(
        [sys.executable, str(PKG_ROOT / "main.py"), "--selftest"],
        cwd=str(PKG_ROOT), capture_output=True, text=True, timeout=1800,
    )
    tail = (result.stdout or "")[-400:] + (result.stderr or "")[-400:]
    assert result.returncode == 0, f"--selftest 返回 {result.returncode}\n{tail}"
