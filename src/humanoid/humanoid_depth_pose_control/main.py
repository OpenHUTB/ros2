#!/usr/bin/env python3
"""main.py —— 深度图姿态估计与平衡控制 standalone 入口

用法：
    python3 main.py --check                       # 自检（用临时数据集，不覆盖正式数据）
    python3 main.py --collect --samples 1500      # 采集真实 MuJoCo 数据
    python3 main.py --train                       # 训练并回存验证集最优权重
    python3 main.py --demo --gif demo.gif         # 仿真演示，逐帧真实渲染录成 gif
"""
from __future__ import annotations
import argparse, json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from humanoid_depth_pose_control.sim import DepthPoseSim
from humanoid_depth_pose_control.dataset import collect
from humanoid_depth_pose_control.train import train
from humanoid_depth_pose_control.predictor import DepthPosePredictor
from humanoid_depth_pose_control.controller import ProportionalController

MODEL = os.path.join(HERE, "models", "humanoid_depth.xml")
DATASET = os.path.join(HERE, "data", "depth_pose_dataset.npz")
WEIGHTS = os.path.join(HERE, "models_trained", "depth_pose_mlp.npz")
_CHECK_DS = os.path.join(HERE, "data", "_check_ds.npz")
_CHECK_W = os.path.join(HERE, "data", "_check_w.npz")


def run_check():
    print("="*66); print("  深度图姿态估计与平衡控制 自检"); print("="*66)
    ok = True
    def check(name, passed, detail=""):
        nonlocal ok
        ok = ok and bool(passed)
        print(f"[{'PASS' if passed else 'FAIL'}] {name} {detail}")

    sim = DepthPoseSim(MODEL)
    check("模型加载", sim.model.nu == 15, f"(nu={sim.model.nu})")

    # 站姿必须真的站得住：只跑 0.6 s 看到"还没倒"是假判据 —— 原实现就是在这个
    # 窗口里通过的，而实际 1.4 s 内自身前倾 86.9° 倒地。这里跑满 3 s 并检查倾角。
    sim.reset()
    max_tilt = 0.0
    for _ in range(1500):
        sim.step()
        p, r, _ = sim.torso_pose()
        max_tilt = max(max_tilt, abs(np.degrees(p)), abs(np.degrees(r)))
    check("站姿稳定(3s)", (not sim.fallen()) and max_tilt < 5.0,
          f"(max_tilt={max_tilt:.2f}°, root_z={sim.root_z:.3f})")

    depth = sim.render_depth()
    check("深度图渲染", depth.shape == (64, 64) and 0.0 <= float(depth.min())
          and float(depth.max()) <= 1.0,
          f"(shape={depth.shape}, range=[{depth.min():.3f}, {depth.max():.3f}])")

    # 自检用临时数据集/权重，不覆盖 data/depth_pose_dataset.npz 与正式权重
    os.makedirs(os.path.dirname(_CHECK_DS), exist_ok=True)
    collect(MODEL, _CHECK_DS, n_samples=300, verbose=False)
    check("数据集采集", os.path.isfile(_CHECK_DS))

    log = train(_CHECK_DS, _CHECK_W, epochs=40, verbose=False)
    # 判据按角度给：val_rmse 8° 相当于 MSE≈0.0195 rad²。只看 MSE 的绝对阈值
    # 对回归任务没有直觉，也说不清"多差算差"。
    check("训练收敛", log["best_val_rmse_deg"] < 8.0,
          f"(val_rmse={log['best_val_rmse_deg']:.2f}°, mse={log['best_val_mse']:.4f}, "
          f"best_epoch={log['best_epoch']})")

    pred = DepthPosePredictor(_CHECK_W)
    p, r, y = pred.predict_from_depth(depth)
    check("推理输出", abs(p) < 0.4 and abs(r) < 0.4,
          f"(pitch={p:.3f}, roll={r:.3f}, yaw={y:.3f})")

    ctrl = ProportionalController()
    t = ctrl.target_for(p, r, y)
    check("控制器输出", t.shape == (15,))

    sim.close()
    for f in (_CHECK_DS, _CHECK_W):
        try: os.remove(f)
        except OSError: pass
    print(f"\n结论：自检{'通过' if ok else '存在问题'}。")
    return 0 if ok else 1


def run_demo(force=120, direction=0, gif_path=None, steps=1500):
    sim = DepthPoseSim(MODEL)
    pred = DepthPosePredictor(WEIGHTS)
    ctrl = ProportionalController()
    sim.reset()
    for _ in range(250):
        sim.step()
    rad = np.radians(direction)
    sim.apply_push((force*np.cos(rad), force*np.sin(rad), 0), duration_steps=40)

    frames, max_tilt, fallen_at = [], 0.0, None
    for i in range(steps):
        depth = sim.render_depth()
        p, r, y = pred.predict_from_depth(depth)
        target = ctrl.target_for(p, r, y)
        sim.step(target=target)
        tp, tr, _ = sim.torso_pose()
        max_tilt = max(max_tilt, abs(np.degrees(tp)), abs(np.degrees(tr)))
        # 动图必须是真实仿真逐帧渲染，每隔几帧取一帧；320×240 足够看清姿态，
        # 640×480 会让 gif 涨到 13 MB 以上。用 overview 机位（跟随躯干），
        # 深度图那个 front_camera 在头顶朝下，录出来只能看到躯干顶面。
        if gif_path and i % 3 == 0:
            frames.append(sim.render_rgb(240, 320, camera="overview"))
        if sim.fallen():
            fallen_at = i
            print(f"  [demo] 跌倒 @ step {i} ({sim.data.time:.2f}s)")
            break
        if i % 100 == 0:
            print(f"  t={sim.data.time:.2f}s  pred_pitch={p:.3f}  pred_roll={r:.3f}  "
                  f"z={sim.root_z:.3f}  tilt={np.degrees(tp):.1f}°")

    result = {
        "fallen": bool(sim.fallen()),
        "fallen_at_step": fallen_at,
        "survived_s": round(float(sim.data.time), 2),
        "final_z": round(float(sim.root_z), 3),
        "max_tilt_deg": round(float(max_tilt), 2),
        "frames": len(frames),
    }
    if gif_path and frames:
        import imageio
        os.makedirs(os.path.dirname(os.path.abspath(gif_path)), exist_ok=True)
        imageio.mimsave(gif_path, frames, duration=0.06, loop=0)
        print(f"[demo] gif saved: {gif_path} ({len(frames)} 帧, 真实 MuJoCo 渲染)")
    sim.close()
    print(json.dumps(result, indent=2))
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--check", action="store_true")
    p.add_argument("--collect", action="store_true")
    p.add_argument("--train", action="store_true")
    p.add_argument("--demo", action="store_true")
    p.add_argument("--samples", type=int, default=2000)
    p.add_argument("--force", type=float, default=120)
    p.add_argument("--direction", type=float, default=0)
    p.add_argument("--gif", type=str, default=None, help="demo 模式下录制动图的输出路径")
    p.add_argument("--epochs", type=int, default=80)
    a = p.parse_args()
    if a.check: sys.exit(run_check())
    if a.collect:
        os.makedirs(os.path.dirname(DATASET), exist_ok=True)
        collect(MODEL, DATASET, n_samples=a.samples)
    elif a.train:
        os.makedirs(os.path.dirname(WEIGHTS), exist_ok=True)
        train(DATASET, WEIGHTS, epochs=a.epochs)
    elif a.demo:
        run_demo(a.force, a.direction, a.gif)
    else:
        p.print_help()


if __name__ == "__main__":
    main()
