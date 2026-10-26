# -*- coding: utf-8 -*-
"""
人形机器人单目图像端到端控制 —— 自动化测试
==========================================
覆盖模块核心指标：
  1. 站姿稳定性：零指令 1000 步保持直立、根高落在合理区间；
  2. 感知图像：运行时跟随相机渲染尺寸为 96×128×3；
  3. 网络前向：回归输出线速度 / 偏航角速度均在限幅内；
  4. 仿真性能：跟随相机渲染帧率 > 30 FPS，单帧 CNN 推理 < 50 ms；
  5. 权重加载：随包权重存在时可正常载入（缺失则跳过，不阻断基础测试）。

运行方式（包根目录）：
    python3 test/test_standalone.py        # 独立运行，失败时退出码为 1
    python3 -m pytest test/ -v              # pytest / colcon 收集
"""

import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from humanoid_camera_e2e_control.sim_core import HumanoidSim, IMG_H, IMG_W
from humanoid_camera_e2e_control.network import E2ECNN

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEIGHTS = os.path.join(HERE, "weights", "humanoid_cnn.pt")

try:
    import pytest

    @pytest.fixture(name="sim")
    def _sim_fixture():
        """pytest 用的仿真实例；脚本模式下由 main() 显式传入。"""
        return HumanoidSim()

except ImportError:                       # 未安装 pytest（直接跑脚本）时不需要 fixture
    pass


def test_standing(sim, steps=1000):
    """站姿稳定性：零指令保持直立，根高落在合理区间。"""
    sim.reset()
    for _ in range(steps):
        sim.step(0.0, 0.0)
    od = sim.read_odom()
    assert sim.standing(), f"站姿失稳，根高 z={od['z']:.3f} m"
    assert 0.85 < od["z"] < 1.15, f"根高异常: z={od['z']:.3f}"
    print(f"[PASS] 站姿稳定性: {steps} 步零指令，根高 z={od['z']:.3f} m")


def test_render_front(sim):
    """跟随相机渲染尺寸与网络输入尺寸一致。"""
    img = sim.render_front()
    assert img.shape == (IMG_H, IMG_W, 3), f"渲染尺寸异常: {img.shape}"
    assert img.dtype == np.uint8
    print(f"[PASS] 跟随相机渲染: {IMG_W}×{IMG_H} RGB，像素范围 "
          f"[{img.min()}, {img.max()}]")


def test_network_bounds(sim):
    """网络前向输出物理指令均在限幅内。"""
    import torch
    net = E2ECNN(sim.V_MAX, sim.W_MAX)
    net.eval()
    x = torch.zeros(1, 3, IMG_H, IMG_W)
    with torch.no_grad():
        out = net(x)[0]
    v, w = float(out[0]), float(out[1])
    assert abs(v) <= sim.V_MAX + 1e-6, f"线速度越限: v={v:.3f}"
    assert abs(w) <= sim.W_MAX + 1e-6, f"偏航角速度越限: w={w:.3f}"
    print(f"[PASS] 网络输出限幅: v={v:.3f} m/s (±{sim.V_MAX}), "
          f"w={w:.3f} rad/s (±{sim.W_MAX})")


def test_performance(sim, seconds=1.0):
    """渲染帧率与单帧推理耗时。"""
    import torch
    sim.reset()
    t0 = time.time()
    count = 0
    while time.time() - t0 < seconds:
        sim.render_front()
        count += 1
    render_fps = count / (time.time() - t0)

    net = E2ECNN(sim.V_MAX, sim.W_MAX)
    net.eval()
    x = torch.zeros(1, 3, IMG_H, IMG_W)
    t0 = time.time()
    n_infer = 50
    with torch.no_grad():
        for _ in range(n_infer):
            net(x)
    infer_ms = (time.time() - t0) / n_infer * 1000

    assert render_fps > 30, f"渲染帧率过低: {render_fps:.0f} FPS"
    assert infer_ms < 50, f"单帧推理过慢: {infer_ms:.1f} ms"
    print(f"[PASS] 仿真性能: 渲染 {render_fps:.0f} FPS，单帧推理 {infer_ms:.1f} ms")


def test_weights_loadable():
    """随包权重存在时可正常载入；缺失则跳过。"""
    if not os.path.isfile(WEIGHTS):
        print("[SKIP] 未发现权重文件（先运行 --teacher 再 --train），跳过权重校验。")
        return
    net = E2ECNN.load(WEIGHTS)
    n_params = sum(p.numel() for p in net.parameters())
    print(f"[PASS] 权重加载成功: {WEIGHTS}，参数量 {n_params:,}")


def main():
    print("=" * 56)
    print("  人形机器人单目图像端到端控制 自动化测试")
    print("=" * 56)
    sim = HumanoidSim()
    test_standing(sim)
    test_render_front(sim)
    test_network_bounds(sim)
    test_performance(sim)
    test_weights_loadable()
    print("=" * 56)
    print("  全部测试通过")
    print("=" * 56)


if __name__ == "__main__":
    main()
