# -*- coding: utf-8 -*-
"""runner.py —— 单次跌倒实验的执行器

standalone（main.py）、ROS2 节点（ros_node.py）和自检脚本都调用这里，
保证「一次实验」的定义只有一份：先站 1 s、推 0.08 s、再跑到 3.5 s。
"""

import os

from . import config
from .metrics import RunRecorder, reduction_vs_best_passive
from .protection import FallController
from .sim_core import FallSim, default_model_path


def run_episode(strategy="rigid", push_force=None, push_duration=None, push_delay=None,
                duration=None, params=None, model_path=None, record_csv=None,
                frame_callback=None):
    """跑一次「站立 → 被推 → 倒地」的完整过程。

    参数
    ----
    strategy : "rigid" / "limp" / "protect"
    push_force : 推扰力 (N)，+x 为向前推（人向前扑）
    frame_callback : 可选 callable(sim, t, state)，每帧调用，用于渲染/发布话题

    返回
    ----
    (summary: dict, recorder: RunRecorder)
    """
    p = dict(config.DEFAULTS)
    if params:
        p.update(params)
    force = float(p["push_force"] if push_force is None else push_force)
    push_dur = float(p["push_duration"] if push_duration is None else push_duration)
    push_t0 = float(p["push_delay"] if push_delay is None else push_delay)
    total = float(p["settle_time"] if duration is None else duration)

    sim = FallSim(model_path or default_model_path())
    sim.reset()
    controller = FallController(p)
    controller.reset(strategy)
    recorder = RunRecorder(strategy, force, push_t0, push_dur)

    steps = int(total / sim.dt)
    for i in range(steps):
        t = i * sim.dt
        active = push_t0 <= t < push_t0 + push_dur
        sim.apply_push(force, active)
        state = controller.step(sim, t)
        sim.step()
        state["tilt"] = sim.tilt()
        state["tilt_rate"] = sim.tilt_rate()
        recorder.sample(sim, t, state)
        if frame_callback is not None:
            frame_callback(sim, t, state)

    summary = recorder.summary()
    summary["tilt_threshold_deg"] = float(p["tilt_threshold_deg"])
    summary["detector_mode"] = p["detector_mode"]
    if record_csv:
        os.makedirs(os.path.dirname(os.path.abspath(record_csv)), exist_ok=True)
        recorder.write_csv(record_csv)
    return summary, recorder


def run_comparison(strategies=None, params=None, model_path=None, export_dir=None, **kwargs):
    """依次跑多种策略，返回 (summaries, reduction)。"""
    strategies = strategies or list(config.STRATEGIES)
    summaries = []
    for strategy in strategies:
        csv_path = None
        if export_dir:
            csv_path = os.path.join(export_dir, "{}_run.csv".format(strategy))
        summary, _ = run_episode(strategy=strategy, params=params,
                                 model_path=model_path, record_csv=csv_path, **kwargs)
        summaries.append(summary)
    return summaries, reduction_vs_best_passive(summaries)
