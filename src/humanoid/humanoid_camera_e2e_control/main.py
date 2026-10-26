#!/usr/bin/env python3
"""
人形机器人单目图像端到端控制主入口（standalone，无 ROS 依赖）
================================================================
课程要求：入口为 main.py，能够直接运行整个模块。

支持模式：
  --teacher    规则教师自动驾驶，采集跟随视角图像—指令数据集（默认采集路径）
  --collect    键盘驾驶，采集跟随视角图像—指令数据集（备选路径）
  --train      在已采集数据集上训练 E2ECNN，保存权重与评测指标
  --run        加载权重端到端推理演示，录制 GIF / 关键帧与性能指标（默认）
  --test       自动化测试：模型加载、站姿、渲染、网络前向、限幅与性能
  --ros2       通过 ros2 launch 启动 ROS2 节点（mode:=run 或 mode:=collect）
  --no-viewer  关闭可视化窗口（无图形环境使用）

示例（包根目录）：
    python3 main.py --teacher                  # 规则教师自动采集数据
    python3 main.py --collect                  # 键盘驾驶采集数据
    python3 main.py --train                    # 训练 CNN
    python3 main.py                           # 端到端推理演示并录制素材
    python3 main.py --test                     # 自动化测试
    python3 main.py --ros2 -- mode:=run       # ROS2 launch 推理模式
"""

import argparse
import os
import subprocess
import sys

_PKG_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _PKG_ROOT)
sys.path.insert(0, os.path.join(_PKG_ROOT, "scripts"))

from humanoid_camera_e2e_control.sim_core import HumanoidSim, IMG_H, IMG_W
from humanoid_camera_e2e_control.network import E2ECNN
from humanoid_camera_e2e_control.data_io import load_dataset

HERE = _PKG_ROOT
DEFAULT_DATA = os.path.join(HERE, "data", "training")
DEFAULT_WEIGHTS = os.path.join(HERE, "weights", "humanoid_cnn.pt")


# ----------------------------------------------------------------------
# 自动化测试
# ----------------------------------------------------------------------
def run_test(viewer=False):
    import time
    import numpy as np
    import torch

    results = []
    def check(name, ok, detail=""):
        results.append((name, ok, detail))
        print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")

    # 1) 仿真模型加载 + 站姿保持
    sim = HumanoidSim()
    for _ in range(1000):
        sim.step(0.0, 0.0)
    od = sim.read_odom()
    check("站姿保持（根高 > 0.9 m）", sim.standing(),
          f"根高 z={od['z']:.3f} m")
    check("根高在合理区间 [0.85, 1.15]",
          0.85 < od["z"] < 1.15, f"z={od['z']:.3f}")

    # 2) 跟随视角相机渲染尺寸
    img = sim.render_front()
    check("跟随视角图像尺寸 96×128×3",
          img.shape == (IMG_H, IMG_W, 3), f"shape={img.shape}")

    # 3) 网络前向 + 输出限幅
    net = E2ECNN(sim.V_MAX, sim.W_MAX)
    x = torch.zeros(1, 3, IMG_H, IMG_W)
    with torch.no_grad():
        out = net(x)[0]
    v, w = float(out[0]), float(out[1])
    check("网络输出维度 (v, w)", True, f"v={v:.3f}, w={w:.3f}")
    check("速度指令在限幅内",
          abs(v) <= sim.V_MAX and abs(w) <= sim.W_MAX,
          f"|v|={abs(v):.3f}<={sim.V_MAX}, |w|={abs(w):.3f}<={sim.W_MAX}")

    # 4) 性能：渲染帧率 + 单帧推理耗时
    t0 = time.time()
    for _ in range(100):
        sim.render_front()
    render_fps = 100 / (time.time() - t0)
    t0 = time.time()
    with torch.no_grad():
        for _ in range(50):
            net(x)
    infer_ms = (time.time() - t0) / 50 * 1000
    check("跟随视角渲染帧率 > 30 FPS", render_fps > 30, f"{render_fps:.0f} FPS")
    check("单帧推理 < 50 ms", infer_ms < 50, f"{infer_ms:.1f} ms")

    # 5) 数据集 / 权重存在性（存在才校验，缺失不阻断基础测试）
    packed = load_dataset(DEFAULT_DATA)
    if packed is not None:
        images, labels = packed
        check("数据集可加载", True, f"{len(images)} 帧")
    else:
        print("[INFO] 未发现数据集（先运行 --collect），跳过数据集校验。")

    if os.path.isfile(DEFAULT_WEIGHTS):
        net2 = E2ECNN.load(DEFAULT_WEIGHTS)
        check("训练权重可加载", True, DEFAULT_WEIGHTS)
    else:
        print("[INFO] 未发现权重文件（先运行 --collect 再 --train），跳过权重校验。")

    passed = sum(1 for _, ok, _ in results if ok)
    print(f"\n==== 测试汇总：{passed}/{len(results)} 项通过 ====")
    return 0 if passed == len(results) else 1


# ----------------------------------------------------------------------
# 主入口
# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="人形机器人单目图像端到端控制")
    ap.add_argument("--collect", action="store_true", help="键盘驾驶采集数据集")
    ap.add_argument("--teacher", action="store_true", help="规则教师自动驾驶采集数据集")
    ap.add_argument("--train", action="store_true", help="训练端到端 CNN")
    ap.add_argument("--run", action="store_true", help="加载权重推理演示（默认）")
    ap.add_argument("--test", action="store_true", help="运行自动化测试")
    ap.add_argument("--ros2", action="store_true", help="ros2 launch 启动 ROS2 节点")
    ap.add_argument("--data-dir", default=DEFAULT_DATA)
    ap.add_argument("--weights", default=DEFAULT_WEIGHTS)
    ap.add_argument("--duration", type=float, default=25.0)
    ap.add_argument("--max-frames", type=int, default=2000)
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--no-viewer", dest="viewer", action="store_false")
    ap.set_defaults(viewer=True)
    # ros2 透传参数（如 --ros2 -- mode:=run）
    ap.add_argument("ros2_args", nargs="*")
    args = ap.parse_args()

    if args.test:
        sys.exit(run_test(viewer=args.viewer))

    if args.ros2:
        mode = "run"
        for a in args.ros2_args:
            if a.startswith("mode:="):
                mode = a.split("=", 1)[1]
        print(f"[INFO] ros2 launch humanoid_camera_e2e_control mode:={mode}")
        sys.exit(subprocess.run(
            ["ros2", "launch", "humanoid_camera_e2e_control", "main.launch.py",
             f"mode:={mode}"]).returncode)

    if args.collect:
        import collect_data
        collect_data.collect(args.data_dir, args.max_frames, viewer=args.viewer)
        return

    if args.teacher:
        import teacher_collect
        teacher_collect.collect(args.data_dir, args.max_frames)
        return

    if args.train:
        import train_e2e
        sys.argv = ["train_e2e.py", "--data-dir", args.data_dir,
                    "--weights", args.weights, "--epochs", str(args.epochs)]
        train_e2e.main()
        return

    # 默认：端到端推理演示
    import record_demo
    out_dir = os.path.join(os.getcwd(), "demo_assets")
    record_demo.run(args.weights, out_dir, args.duration, viewer=args.viewer)


if __name__ == "__main__":
    main()
