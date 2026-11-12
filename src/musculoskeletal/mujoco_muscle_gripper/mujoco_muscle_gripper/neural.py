"""真实接触力反馈与相机图像策略：监督学习、独立测试及闭环仿真。"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from . import core, runs

CONTROL_DT = 0.01
SUBSTEPS = round(CONTROL_DT / core.DT)


class MLP:
    """标准化输入和输出的单隐层 tanh 网络，使用 Adam 最小化均方误差。"""

    def __init__(self, n_in, hidden=32, seed=21):
        rng = np.random.default_rng(seed)
        self.w1 = rng.normal(0, np.sqrt(1 / n_in), (n_in, hidden))
        self.b1 = np.zeros(hidden)
        self.w2 = rng.normal(0, np.sqrt(1 / hidden), (hidden, 1))
        self.b2 = np.zeros(1)
        self.x_mean = np.zeros(n_in)
        self.x_std = np.ones(n_in)
        self.y_mean = np.zeros(1)
        self.y_std = np.ones(1)

    def predict(self, x):
        x = np.atleast_2d(np.asarray(x, dtype=float))
        z = np.clip((x - self.x_mean) / self.x_std, -6, 6)
        return (np.tanh(z @ self.w1 + self.b1) @ self.w2 + self.b2) * self.y_std + self.y_mean

    def fit(self, x, y, validation, epochs=350, lr=0.003, seed=21):
        x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float).reshape(-1, 1)
        if len(x) < 2 or not np.isfinite(x).all() or not np.isfinite(y).all():
            raise ValueError("训练数据必须包含有限数值和至少两个样本")
        self.x_mean = x.mean(axis=0)
        self.x_std = np.maximum(x.std(axis=0), 0.04)
        self.y_mean = y.mean(axis=0)
        self.y_std = np.maximum(y.std(axis=0), 0.005)
        z = np.clip((x - self.x_mean) / self.x_std, -6, 6)
        target = (y - self.y_mean) / self.y_std
        vx, vy = validation
        vy = np.asarray(vy).reshape(-1, 1)
        rng = np.random.default_rng(seed)
        params = [self.w1, self.b1, self.w2, self.b2]
        moment = [np.zeros_like(p) for p in params]
        variance = [np.zeros_like(p) for p in params]
        history, step = [], 0
        for epoch in range(epochs):
            order = rng.permutation(len(x))
            for begin in range(0, len(x), 128):
                idx = order[begin:begin + 128]
                xb = z[idx]
                h = np.tanh(xb @ self.w1 + self.b1)
                err = (h @ self.w2 + self.b2 - target[idx]) * (2 / len(idx))
                hidden_err = (err @ self.w2.T) * (1 - h * h)
                gradients = [xb.T @ hidden_err, hidden_err.sum(axis=0), h.T @ err, err.sum(axis=0)]
                step += 1
                for p, g, m, v in zip(params, gradients, moment, variance):
                    m *= 0.9
                    m += 0.1 * np.clip(g, -10, 10)
                    v *= 0.999
                    v += 0.001 * np.clip(g, -10, 10) ** 2
                    p -= lr * (m / (1 - 0.9 ** step)) / (np.sqrt(v / (1 - 0.999 ** step)) + 1e-8)
            if epoch % 5 == 0 or epoch == epochs - 1:
                history.append({"epoch": epoch + 1,
                                "train_mse": float(np.mean((self.predict(x) - y) ** 2)),
                                "validation_mse": float(np.mean((self.predict(vx) - vy) ** 2))})
        return history

    def save(self, path):
        np.savez_compressed(path, **{name: getattr(self, name) for name in
                                    ("w1", "b1", "w2", "b2", "x_mean", "x_std", "y_mean", "y_std")})

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as data:
            net = cls(data["w1"].shape[0], data["w1"].shape[1])
            for name in data.files:
                if name in vars(net):
                    setattr(net, name, data[name].copy())
        return net


def split_episodes(groups, seed=21):
    """按完整仿真回合切分，避免相邻帧进入不同数据集。"""
    unique = np.unique(groups)
    if len(unique) < 5:
        raise ValueError("至少需要五个独立回合")
    order = np.random.default_rng(seed).permutation(unique)
    n_test = max(1, len(order) // 5)
    n_val = max(1, len(order) // 5)
    test, val, train = order[:n_test], order[n_test:n_test + n_val], order[n_test + n_val:]
    return tuple(np.flatnonzero(np.isin(groups, part)) for part in (train, val, test))


class GripEnvironment:
    """复用现有肌肉和接触模型；观测值直接来自当前物理仿真。"""

    def __init__(self, mass=0.03, friction=1.5, initial_travel=0.0):
        self.model, self.data = runs._make(obj_mass=mass, friction=friction)
        self.muscles = [core.HillMuscle(), core.HillMuscle()]
        self.qids = [int(self.model.joint(n).qposadr[0]) for n in ("flj", "frj")]
        self.dids = [int(self.model.joint(n).dofadr[0]) for n in ("flj", "frj")]
        for qid in self.qids:
            self.data.qpos[qid] = initial_travel
        mujoco.mj_forward(self.model, self.data)
        self.obj_zid = int(self.model.jnt_qposadr[self.model.joint("objfree").id]) + 2
        self.start_z = float(self.data.qpos[self.obj_zid])
        self.activation = 0.30
        self.renderer = None

    def observe(self):
        count, force = runs._finger_contact(self.model, self.data)
        return float(force), float(np.mean(self.data.qpos[self.qids])), float(np.mean(self.data.qvel[self.dids])), count

    def step(self, activation, lift=0.0, substeps=SUBSTEPS):
        self.activation = float(np.clip(activation, 0, 1))
        for _ in range(substeps):
            self.data.ctrl[0] = lift
            for muscle, qid, did in zip(self.muscles, self.qids, self.dids):
                muscle.update_activation(self.activation, core.DT)
                self.data.qfrc_applied[did] = core.muscle_force(muscle, self.data.qpos[qid], self.data.qvel[did])
            mujoco.mj_step(self.model, self.data)
        return self.observe()

    def render(self):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=240, width=320)
            self.camera = mujoco.MjvCamera()
            self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            self.camera.lookat[:] = [0.0, 0.0, -0.03]
            self.camera.distance = 0.34
            self.camera.elevation = -12
            self.camera.azimuth = 135
        self.renderer.update_scene(self.data, camera=self.camera)
        return Image.fromarray(self.renderer.render().copy())

    def close(self):
        if self.renderer is not None:
            self.renderer.close()


def force_features(target, force, travel, velocity, previous):
    return np.array([target / 10, force / 15, travel / 0.02, velocity / 0.10, previous], dtype=float)


def teacher_delta(target, force):
    """监督标签来自独立积分控制器，网络本身通过监督学习获得增量。"""
    return float(np.clip(0.30 * (target - force) * CONTROL_DT, -0.02, 0.02))


def collect_force_data(episodes=24, seed=21):
    rng = np.random.default_rng(seed)
    xs, ys, groups = [], [], []
    for episode in range(episodes):
        env = GripEnvironment(mass=float(rng.uniform(0.02, 0.06)), friction=float(rng.uniform(0.7, 1.5)))
        target = float(rng.uniform(1, 10))
        u = float(rng.uniform(0.1, 0.5))
        for tick in range(240):
            force, q, v, _ = env.observe()
            delta = teacher_delta(target, force)
            xs.append(force_features(target, force, q, v, u))
            ys.append(delta)
            groups.append(episode)
            u = float(np.clip(u + delta, 0, 1))
            env.step(u)
        env.close()
    return np.asarray(xs), np.asarray(ys), np.asarray(groups)


def force_rollout(target, network=None, seconds=3.0):
    env = GripEnvironment()
    rows, u, integral = [], 0.30, 0.0
    for tick in range(round(seconds / CONTROL_DT)):
        force, q, velocity, contacts = env.observe()
        feature = force_features(target, force, q, velocity, u)
        if network is None:
            # 独立 PI 基准：比例增益 0.025，积分增益 0.30。
            error = target - force
            integral = float(np.clip(integral + 0.30 * error * CONTROL_DT, -0.30, 0.70))
            u = float(np.clip(0.30 + 0.025 * error + integral, 0, 1))
        else:
            delta = float(network.predict(feature)[0, 0])
            u = float(np.clip(u + np.clip(delta, -0.02, 0.02), 0, 1))
        env.step(u)
        rows.append([tick * CONTROL_DT, target, force, u, q, contacts])
    env.close()
    trace = np.asarray(rows)
    steady = trace[-50:, 2]
    result = {"target_n": float(target), "steady_force_n": float(steady.mean()),
              "steady_mae_n": float(np.mean(np.abs(steady - target))),
              "full_rmse_n": float(np.sqrt(np.mean((trace[:, 2] - target) ** 2))),
              "final_activation": float(u)}
    return result, trace


def image_features(frame):
    return np.asarray(frame.convert("L").resize((16, 16), Image.Resampling.BOX), dtype=float).reshape(-1) / 255


def collect_visual_data(episodes=18, seed=21):
    """从实际渲染帧采样；教师看状态生成动作，视觉策略只接收像素。"""
    rng = np.random.default_rng(seed)
    xs, ys, groups = [], [], []
    for episode in range(episodes):
        env = GripEnvironment(initial_travel=float(rng.uniform(0, 0.008)),
                              mass=float(rng.uniform(0.02, 0.04)), friction=float(rng.uniform(1, 1.5)))
        try:
            for tick in range(24):
                _, q, _, _ = env.observe()
                action = 0.60 + 0.20 * float(np.clip((0.009 - q) / 0.009, 0, 1))
                xs.append(image_features(env.render()))
                ys.append(action)
                groups.append(episode)
                env.step(action)
        finally:
            env.close()
    return np.asarray(xs), np.asarray(ys), np.asarray(groups)


def visual_rollout(network, out):
    env = GripEnvironment()
    frames, rows = [], []
    try:
        for tick in range(280):
            t = tick * 0.02
            frame = env.render()
            action = float(np.clip(network.predict(image_features(frame))[0, 0], 0, 1))
            lift = 0.10 * np.clip((t - 1.0) / 4, 0, 1)
            env.step(action, lift=float(lift), substeps=2 * SUBSTEPS)
            force, q, _, _ = env.observe()
            rows.append([t, action, force, float(env.data.qpos[env.obj_zid] - env.start_z)])
            if tick % 3 == 0:
                frames.append(frame)
        frames[len(frames) // 2].save(out / "camera_frame.png")
        frames[0].save(out / "visual_control.gif", save_all=True, append_images=frames[1:], duration=60,
                       loop=0, optimize=True)
        trace = np.asarray(rows)
        result = {"input": "16x16 rendered grayscale pixels", "frames": len(rows),
                  "lift_mm": float(trace[:, 3].max() * 1000),
                  "final_force_n": float(trace[-1, 2]), "mean_activation": float(trace[:, 1].mean()),
                  "camera_png": "camera_frame.png", "gif": "visual_control.gif"}
        return result, trace
    finally:
        env.close()


def _font(size):
    for name in ("C:/Windows/Fonts/msyh.ttc", "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
                 "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if Path(name).exists():
            return ImageFont.truetype(name, size)
    return ImageFont.load_default()


def plot_lines(path, series, title, xlabel, ylabel):
    """数据曲线按真实采样坐标绘制，不将标注图混作仿真截图。"""
    image = Image.new("RGB", (1000, 580), "white")
    draw = ImageDraw.Draw(image)
    font, small = _font(23), _font(18)
    x0, y0, x1, y1 = 85, 65, 960, 480
    all_x = np.concatenate([np.asarray(s[1]) for s in series])
    all_y = np.concatenate([np.asarray(s[2]) for s in series])
    xmin, xmax = float(all_x.min()), float(all_x.max())
    ymin, ymax = float(all_y.min()), float(all_y.max())
    ymax += max(1e-9, 0.08 * (ymax - ymin))
    if xmax == xmin:
        xmax += 1
    if ymax == ymin:
        ymax += 1
    draw.text((85, 18), title, font=font, fill="black")
    for i in range(6):
        frac = i / 5
        yy = y1 - frac * (y1 - y0)
        draw.line((x0, yy, x1, yy), fill="#dddddd")
        draw.text((5, yy - 10), f"{ymin + frac * (ymax - ymin):.3g}", font=small, fill="black")
        xx = x0 + frac * (x1 - x0)
        draw.text((xx - 16, y1 + 10), f"{xmin + frac * (xmax - xmin):.3g}", font=small, fill="black")
    colors = ("#1769aa", "#e3701a", "#25863c", "#9b38a6", "#c12c3b", "#007d7c")
    for i, (label, xs, ys) in enumerate(series):
        pts = [(x0 + (float(x) - xmin) / (xmax - xmin) * (x1 - x0),
                y1 - (float(y) - ymin) / (ymax - ymin) * (y1 - y0)) for x, y in zip(xs, ys)]
        if len(pts) > 1:
            draw.line(pts, fill=colors[i % len(colors)], width=2)
        draw.text((85 + (i % 3) * 290, 535 + (i // 3) * 20), label,
                  font=small, fill=colors[i % len(colors)])
    draw.text((415, 508), xlabel, font=small, fill="black")
    draw.text((85, 43), ylabel, font=small, fill="black")
    image.save(path)


def run_experiments(out, seed=21, include_vision=True, epochs=300):
    """训练、独立测试、闭环对比及可交付记录的单一入口。"""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    x, y, groups = collect_force_data(seed=seed)
    train, val, test = split_episodes(groups, seed)
    net = MLP(x.shape[1], hidden=24, seed=seed)
    history = net.fit(x[train], y[train], (x[val], y[val]), epochs=epochs, seed=seed)
    net.save(out / "force_policy.npz")
    np.savez_compressed(out / "force_dataset.npz", x=x, y=y, episode=groups, train=train, validation=val, test=test)
    metrics = {"seed": seed, "mujoco_version": mujoco.__version__, "control_dt_s": CONTROL_DT,
               "force_training": {"samples": len(x), "train_episodes": np.unique(groups[train]).tolist(),
                                  "validation_episodes": np.unique(groups[val]).tolist(),
                                  "test_episodes": np.unique(groups[test]).tolist(),
                                  "held_out_mse": float(np.mean((net.predict(x[test]).ravel() - y[test]) ** 2)),
                                  "loss": history}, "force_control": []}
    plot_lines(out / "force_training_loss.png", [("训练集", [h["epoch"] for h in history], [h["train_mse"] for h in history]),
                                                ("验证集", [h["epoch"] for h in history], [h["validation_mse"] for h in history])],
               "力反馈网络训练与验证损失", "训练轮次", "激活增量均方误差")
    curves = []
    for target in (2.0, 4.0, 8.0):
        for name, policy in (("神经网络", net), ("PI基准", None)):
            result, trace = force_rollout(target, policy)
            result["controller"] = name
            metrics["force_control"].append(result)
            curves.append((f"{name} {target:g}N", trace[:, 0], trace[:, 2]))
            np.savetxt(out / f"force_{'nn' if policy else 'baseline'}_{target:g}n.csv", trace, delimiter=",",
                       header="time_s,target_n,contact_force_n,activation,travel_m,contact_count", comments="")
    plot_lines(out / "force_tracking.png", curves, "真实接触力闭环跟踪", "时间（秒）", "双指接触法向力（牛顿）")
    if include_vision:
        vx, vy, vg = collect_visual_data(seed=seed)
        vtrain, vval, vtest = split_episodes(vg, seed)
        visual = MLP(vx.shape[1], hidden=24, seed=seed)
        vhistory = visual.fit(vx[vtrain], vy[vtrain], (vx[vval], vy[vval]), epochs=epochs, seed=seed)
        visual.save(out / "visual_policy.npz")
        np.savez_compressed(out / "visual_dataset.npz", x=vx, y=vy, episode=vg, train=vtrain, validation=vval, test=vtest)
        metrics["visual_training"] = {"samples": len(vx), "input_shape": [16, 16],
                                      "train_episodes": np.unique(vg[vtrain]).tolist(),
                                      "validation_episodes": np.unique(vg[vval]).tolist(),
                                      "test_episodes": np.unique(vg[vtest]).tolist(),
                                      "held_out_mse": float(np.mean((visual.predict(vx[vtest]).ravel() - vy[vtest]) ** 2)),
                                      "loss": vhistory}
        plot_lines(out / "visual_training_loss.png", [("训练集", [h["epoch"] for h in vhistory], [h["train_mse"] for h in vhistory]),
                                                     ("验证集", [h["epoch"] for h in vhistory], [h["validation_mse"] for h in vhistory])],
                   "图像到肌肉激活网络损失", "训练轮次", "激活指令均方误差")
        metrics["visual_control"], vtrace = visual_rollout(visual, out)
        np.savetxt(out / "visual_closed_loop.csv", vtrace, delimiter=",",
                   header="time_s,activation,contact_force_n,lift_m", comments="")
        plot_lines(out / "visual_lift.png", [("方块抬升", vtrace[:, 0], vtrace[:, 3] * 1000)],
                   "视觉策略闭环抓取的方块抬升", "时间（秒）", "抬升高度（毫米）")
    (out / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    return metrics


def benchmark(out, seed=21, include_vision=True, epochs=200):
    """供主模块 CLI 调用的完整训练、测试与仿真入口。"""
    return run_experiments(out, seed=seed, include_vision=include_vision, epochs=epochs)


def main(argv=None):
    parser = argparse.ArgumentParser(description="肌肉夹爪神经网络感知控制与真实图像策略")
    parser.add_argument("--out", default="out/neural")
    parser.add_argument("--seed", type=int, default=21)
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--no-vision", action="store_true")
    args = parser.parse_args(argv)
    result = run_experiments(args.out, seed=args.seed, include_vision=not args.no_vision, epochs=args.epochs)
    print(json.dumps({k: v for k, v in result.items() if k not in ("force_training", "visual_training")},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
