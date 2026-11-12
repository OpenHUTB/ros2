#!/usr/bin/env python3
"""离线单测：正解、雅可比、力矩映射与产物目录，全部不依赖图形界面。

    cd src/chap8/mujoco_2link_impedance_control
    python3 -m pytest test -q                       # 需要 pytest
    python3 test/test_impedance_controller.py       # 不装 pytest 也能跑（内置 __main__）

模型一致性测试需要 pip3 install mujoco；未安装时自动跳过，其余数学测试照常跑。
"""
import math
import os
import sys

import numpy as np
import pytest

from impedance_controller import ImpedanceController
from output_paths import resolve_model_path, resolve_out_dir

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(HERE)
MODEL = os.path.join(PKG_ROOT, "models", "two_link_soft_arm.xml")

L1, L2 = 0.3, 0.28
KP = [80.0, 50.0]
BD = [8.0, 5.0]


def test_fk_zero_pose():
    """q=(0,0)：两连杆都沿 +x 伸直，末端在 (L1+L2, 0)。"""
    ctrl = ImpedanceController()
    x = ctrl.fk([0.0, 0.0])
    assert x[0] == pytest.approx(L1 + L2)
    assert x[1] == pytest.approx(0.0)


def test_fk_bent_pose():
    """q=(0.5,-0.5)：q1+q2=0，第二段回正，末端与文档初始位姿一致。"""
    ctrl = ImpedanceController()
    x = ctrl.fk([0.5, -0.5])
    assert x[0] == pytest.approx(L1 * math.cos(0.5) + L2, abs=1e-12)
    assert x[1] == pytest.approx(L1 * math.sin(0.5), abs=1e-12)
    # 与文档「初始 tip (0.543, 0.144)」对得上（四舍五入到 3 位）
    assert round(float(x[0]), 3) == 0.543
    assert round(float(x[1]), 3) == 0.144


@pytest.mark.parametrize("q", [
    (0.0, 0.0),
    (0.5, -0.5),
    (-1.2, 0.7),
    (2.0, -2.6),
    (0.9, 1.1),
])
def test_jacobian_matches_finite_difference(q):
    """解析雅可比与中心差分一致（odef=4 阶截断，容差 1e-6）。"""
    ctrl = ImpedanceController()
    J = ctrl.jac(q)
    eps = 1e-6
    J_fd = np.zeros((2, 2))
    for j in range(2):
        dq = np.zeros(2)
        dq[j] = eps
        J_fd[:, j] = (ctrl.fk(np.array(q) + dq) - ctrl.fk(np.array(q) - dq)) / (2 * eps)
    assert np.max(np.abs(J - J_fd)) < 1e-6


def test_torque_is_jacobian_transpose_of_force():
    """tau = J^T F，且 F = Kp(x_des-x) + Bd(xdot_des - J qdot)（test.py 里 ctrl 输出就是这两步）。"""
    ctrl = ImpedanceController(Kp=KP, Bd=BD)
    q = np.array([0.4, -0.3])
    qdot = np.array([0.2, -0.1])
    x_des = np.array([0.62, 0.0])
    tau, F, x = ctrl.compute_torque(q, qdot, x_des)

    J = ctrl.jac(q)
    F_ref = np.diag(KP) @ (x_des - ctrl.fk(q)) - np.diag(BD) @ (J @ qdot)
    assert np.allclose(F, F_ref)
    assert np.allclose(tau, J.T @ F)


def test_zero_force_when_target_is_current_tip():
    """目标就是当前末端位置、且不运动时，笛卡尔力为零、关节力矩为零（无力控不产生漂移）。"""
    ctrl = ImpedanceController(Kp=KP, Bd=BD)
    q = np.array([0.3, -0.6])
    tau, F, _ = ctrl.compute_torque(q, np.zeros(2), ctrl.fk(q))
    assert np.allclose(F, 0.0, atol=1e-12)
    assert np.allclose(tau, 0.0, atol=1e-12)


def test_force_points_outward_when_target_inside_wall():
    """x_des 在墙内、末端在墙左侧时，x 向力应为正（持续把末端压向墙面）。"""
    ctrl = ImpedanceController(Kp=KP, Bd=BD)
    q = np.array([0.5, -0.5])
    _, F, x = ctrl.compute_torque(q, np.zeros(2), np.array([0.62, 0.0]))
    assert x[0] < 0.62
    assert F[0] > 0.0


def test_resolve_out_dir_prefers_cli(tmp_path):
    """--out 显式指定时直接用它，不落到默认仓库目录。"""
    out = resolve_out_dir(PKG_ROOT, str(tmp_path / "cli"))
    assert out == str(tmp_path / "cli")
    assert os.path.isdir(out)


def test_resolve_out_dir_env(tmp_path, monkeypatch):
    """没有 --out 时读环境变量 IMPEDANCE_OUT_DIR。"""
    monkeypatch.setenv("IMPEDANCE_OUT_DIR", str(tmp_path / "env"))
    out = resolve_out_dir(PKG_ROOT)
    assert out == str(tmp_path / "env")


def test_resolve_out_dir_skips_readonly(tmp_path, monkeypatch):
    """首选目录不可写时跳到下一个候选（这里用环境变量指向只读目录验证探测逻辑）。"""
    ro = tmp_path / "readonly"
    ro.mkdir()
    os.chmod(ro, 0o500)
    monkeypatch.setenv("IMPEDANCE_OUT_DIR", str(ro))
    monkeypatch.setenv("HOME", str(tmp_path))
    try:
        out = resolve_out_dir(PKG_ROOT)
    finally:
        os.chmod(ro, 0o700)
    assert not out.startswith(str(ro))


def test_resolve_out_dir_accepts_sparse_repo(tmp_path, monkeypatch):
    """稀疏检出只有 docs/img/chap8、没有 docs/index.md 时也要认成仓库（写回仓库该目录）。"""
    pkg = tmp_path / "repo" / "src" / "chap8" / "pkg"
    pkg.mkdir(parents=True)
    (tmp_path / "repo" / "docs" / "img" / "chap8").mkdir(parents=True)
    monkeypatch.delenv("IMPEDANCE_OUT_DIR", raising=False)
    out = resolve_out_dir(str(pkg))
    assert out == str(tmp_path / "repo" / "docs" / "img" / "chap8")


def test_controller_fk_matches_model_fk():
    """控制器的解析正解与 MuJoCo 模型 FK 一致（同一套 L1/L2、同一个平面）。"""
    mujoco = pytest.importorskip("mujoco")
    import numpy as np_local  # noqa: F401  (保持与 mujoco 版本无关)

    m = mujoco.MjModel.from_xml_path(MODEL)
    d = mujoco.MjData(m)
    ctrl = ImpedanceController()
    site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
    assert site >= 0

    for q in ([0.0, 0.0], [0.5, -0.5], [1.0, -1.4]):
        d.qpos[:] = q
        mujoco.mj_forward(m, d)
        x_model = np.array([d.site_xpos[site][0], d.site_xpos[site][1]])
        x_ctrl = ctrl.fk(np.array(q))
        assert np.allclose(x_model, x_ctrl, atol=1e-6), (q, x_model, x_ctrl)


def test_model_loads_with_expected_size():
    """模型可加载、规模与文档一致（nq=2 nv=2 nu=2），执行器 gear=1。"""
    mujoco = pytest.importorskip("mujoco")
    m = mujoco.MjModel.from_xml_path(MODEL)
    assert (m.nq, m.nv, m.nu) == (2, 2, 2)
    assert list(m.actuator_gear[:, 0]) == [1.0, 1.0]
    assert m.opt.timestep == pytest.approx(0.002)


def test_resolve_model_path_env_override(tmp_path, monkeypatch):
    """$IMPEDANCE_MODEL_PATH 显式指定时优先使用，便于装在只读 share 时指到可读副本。"""
    xml = tmp_path / "any.xml"
    xml.write_text("<mujoco/>", encoding="utf-8")
    monkeypatch.setenv("IMPEDANCE_MODEL_PATH", str(xml))
    assert resolve_model_path(str(tmp_path)) == str(xml)


def test_resolve_model_path_devel_layout(tmp_path, monkeypatch):
    """devel/install 布局：<ws>/lib/<pkg> 下的脚本要能找 <ws>/share/<pkg>/models/<xml>。

    旧版用 dirname(dirname(__file__)) 拼路径，装完会指向 <ws>/lib/models/...（不存在），
    mjModel.from_xml_path 直接抛 mjParseXML resource not found。
    """
    monkeypatch.delenv("IMPEDANCE_MODEL_PATH", raising=False)
    ws = tmp_path / "ws"
    pkg_lib = ws / "lib" / "mujoco_2link_impedance_control"
    pkg_lib.mkdir(parents=True)
    models = ws / "share" / "mujoco_2link_impedance_control" / "models"
    models.mkdir(parents=True)
    xml = models / "two_link_soft_arm.xml"
    xml.write_text("<mujoco/>", encoding="utf-8")
    assert resolve_model_path(str(pkg_lib)) == str(xml)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
