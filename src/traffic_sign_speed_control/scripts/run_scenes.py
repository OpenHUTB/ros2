# -*- coding: utf-8 -*-
"""端到端多场景评测（运行真实 ROS 系统）。

对每个预置场景：
1. 以 render_mode=off 启动 world/detector/driver 三个节点；
2. 注入场景配置，采集 /world/odom、/speed_limit、/cmd_vel（50 Hz 采样）；
3. 以世界几何真值计算每块牌的「视野区间」，统计：
   - 感知：是否在区间内识别出正确限速值、识别延迟；
   - 控制：区间内稳态误差、达标时间、区间内最大速度、超速时间占比；
4. 写入 artifacts/scene_eval.json 并输出每场景速度曲线图。

用法：python3 scripts/run_scenes.py [--duration 70] [--scene default degraded dense]
"""
import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from traffic_sign_speed_control import scene as S  # noqa: E402

SETTLE_TOL_ABS = 0.05     # 达标容差：max(0.05, 5%·v_lim) m/s
SETTLE_SUSTAIN = 1.0      # 需持续保持达标的时间 s


def in_view(x, y, theta, s):
    sx, sy = S.sign_xy(s.angle_deg)
    d = math.hypot(sx - x, sy - y)
    if d > S.CAM_RANGE_M:
        return False, d
    alpha = math.atan2(sy - y, sx - x) - theta
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))
    return abs(alpha) <= math.radians(S.CAM_FOV_DEG / 2.0), d


class Collector:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        if not rclpy.ok():
            rclpy.init()
        self.rclpy = rclpy
        self.node = Node("scene_collector")
        self.t0 = None
        self.rows = []          # (t, x, y, theta, v_cmd, v_odom, limit)
        self.node.create_subscription(
            self._mk_odom(), "world/odom", self.on_odom, 20)
        self.node.create_subscription(self._mk_int(), "speed_limit",
                                      self.on_limit, 10)
        self.pub_cfg = self.node.create_publisher(
            self._mk_str(), "world/scene_config", 10)

    def _mk_odom(self):
        from nav_msgs.msg import Odometry
        return Odometry

    def _mk_int(self):
        from std_msgs.msg import Int32
        return Int32

    def _mk_str(self):
        from std_msgs.msg import String
        return String

    def on_odom(self, m):
        import rclpy.time
        now = self.node.get_clock().now().nanoseconds
        if self.t0 is None:
            self.t0 = now
        q = m.pose.pose.orientation
        theta = 2.0 * math.atan2(q.z, q.w)
        self.rows.append((now, m.pose.pose.position.x, m.pose.pose.position.y,
                          theta, m.twist.twist.linear.x, self.last_limit))

    def on_limit(self, m):
        self.last_limit = int(m.data)

    last_limit = -1

    def inject(self, data):
        import time as _t
        msg = self._mk_str()()
        msg.data = json.dumps(data)
        for _ in range(30):                 # 等待 world 订阅建立
            self.pub_cfg.publish(msg)
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
            _t.sleep(0.1)


def _kill_stack(proc):
    """彻底终止 launch 及其子节点（进程组 SIGTERM -> SIGKILL -> pkill 兜底）。"""
    import os
    import signal
    import subprocess as sp
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        pass
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
        proc.wait(timeout=8)
    sp.run(["pkill", "-9", "-f",
            "install/traffic_sign_speed_control/lib/"],
           check=False, stdout=sp.DEVNULL, stderr=sp.DEVNULL)
    time.sleep(1.0)


def evaluate_scene(name, cfg, duration, log):
    proc = subprocess.Popen(
        ["ros2", "launch", "traffic_sign_speed_control",
         "traffic_sign_speed_control.launch.py", "render_mode:=none"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)
    time.sleep(4.0)                        # 等待系统起来
    col = Collector()
    signs = [S.Sign(sd["angle"], sd["value"], sd.get("degrade", cfg.get("degrade")))
             for sd in cfg["signs"]]
    col.inject({"name": name,
                "signs": [{"angle": s.angle_deg, "value": s.value_kmh} for s in signs],
                "degrade": cfg.get("degrade", {})})
    t_start = time.time()
    while time.time() - t_start < duration:
        col.rclpy.spin_once(col.node, timeout_sec=0.2)
    _kill_stack(proc)

    # ---- 真值区间与指标 ----
    rows = col.rows
    if not rows:
        log(f"[{name}] 无数据")
        return None
    t_base = rows[0][0]
    cols = list(zip(*rows))                       # (t_ns, x, y, theta, v, limit)
    ts = [(t - t_base) * 1e-9 for t in cols[0]]
    xs, ys, ths, vs, lims = cols[1], cols[2], cols[3], cols[4], cols[5]
    results = {"signs": []}
    for s in signs:
        v_lim = s.value_kmh * S.KMH2MS
        tol = max(SETTLE_TOL_ABS, 0.05 * v_lim)
        enter = leave = detect_t = None
        for i, t in enumerate(ts):
            view, _ = in_view(xs[i], ys[i], ths[i], s)
            if view and enter is None:
                enter = t
            if enter is not None and not view:
                leave = t
                break
            if view and lims[i] == s.value_kmh and detect_t is None:
                detect_t = t
        if enter is None:
            results["signs"].append({"value": s.value_kmh, "seen": False})
            log(f"[{name}] 牌 {s.value_kmh}: 未进入视野")
            continue
        zone_end = leave if leave is not None else ts[-1]
        zt = [t for t, v, l in zip(ts, vs, lims) if enter <= t <= zone_end]
        zv = [v for t, v, l in zip(ts, vs, lims) if enter <= t <= zone_end]
        # 达标时间：从时刻 t 起 1 秒内持续 |v-v_lim|<=tol
        settle = None
        for i, t in enumerate(zt):
            win = [v for v, tt in zip(zv[i:], zt[i:]) if tt <= t + SETTLE_SUSTAIN]
            if win and zt[i] + SETTLE_SUSTAIN <= zone_end and \
                    all(abs(v - v_lim) <= tol for v in win) and \
                    max(zt[i:]) >= t + SETTLE_SUSTAIN:
                settle = t - enter
                break
        # 稳态误差：区间最后 2 秒
        tail = [v for t, v in zip(zt, zv) if t >= zone_end - 2.0]
        steady = float(np.mean([abs(v - v_lim) for v in tail])) if tail else None
        base_t = settle if settle is not None else 0.0
        over = sum(1 for v, t in zip(zv, zt) if t - enter >= base_t
                   and v > v_lim + tol)
        results["signs"].append({
            "value": s.value_kmh, "seen": True,
            "detected": detect_t is not None,
            "detect_latency_s": (detect_t - enter) if detect_t else None,
            "zone_dur_s": zone_end - enter,
            "settle_s": settle,
            "steady_err_ms": steady,
            "max_v_ms": max(zv),
            "over_frac": over / max(len(zv), 1),
        })
        log(f"[{name}] 牌 {s.value_kmh}: detected={detect_t is not None} "
            f"latency={results['signs'][-1]['detect_latency_s']} "
            f"settle={settle} steady={steady} max_v={max(zv):.2f}")
    # 汇总 + 速度曲线
    seen = [r for r in results["signs"] if r.get("seen")]
    results["summary"] = {
        "n_signs": len(signs),
        "detect_rate": (sum(1 for r in seen if r["detected"]) / max(len(seen), 1)),
        "mean_latency_s": None if not any(r["detect_latency_s"] is not None for r in seen)
        else float(np.mean([r["detect_latency_s"] for r in seen
                            if r["detect_latency_s"] is not None])),
        "mean_settle_s": None if not any(r["settle_s"] is not None for r in seen)
        else float(np.mean([r["settle_s"] for r in seen if r["settle_s"] is not None])),
        "mean_steady_err_ms": None if not any(r["steady_err_ms"] is not None for r in seen)
        else float(np.mean([r["steady_err_ms"] for r in seen
                            if r["steady_err_ms"] is not None])),
    }
    # 曲线图
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 3))
        ax.plot(ts, [r[4] for r in rows], label="v_cmd (m/s)", lw=1.2)
        ax.plot(ts, [r[5] * S.KMH2MS if r[5] > 0 else None for r in rows],
                "g--", label="detected limit (m/s)", lw=1.0)
        for s in signs:
            v_lim = s.value_kmh * S.KMH2MS
            ax.axhline(v_lim, ls=":", c="gray", lw=0.7)
        ax.set_xlabel("time (s)"); ax.set_ylabel("speed (m/s)")
        ax.set_title(f"scene: {name}")
        ax.legend(fontsize=8)
        fig.tight_layout()
        art = PKG / "artifacts"
        art.mkdir(exist_ok=True)
        fig.savefig(art / f"scene_speed_{name}.png", dpi=120)
    except Exception as e:  # noqa: BLE001
        log(f"[{name}] 绘图失败: {e}")
    return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=70.0)
    ap.add_argument("--scene", nargs="+", default=["default", "degraded", "dense"])
    args = ap.parse_args()

    all_res = {}
    for name in args.scene:
        print(f"===== 场景 {name}（{args.duration}s） =====")
        res = evaluate_scene(name, S.SCENES[name], args.duration, print)
        if res:
            all_res[name] = res
            print(f"  汇总: {json.dumps(res['summary'], ensure_ascii=False)}")

    art = PKG / "artifacts"
    art.mkdir(exist_ok=True)
    (art / "scene_eval.json").write_text(
        json.dumps(all_res, indent=2, ensure_ascii=False))
    print(f"[done] 已写入 {art / 'scene_eval.json'}")


if __name__ == "__main__":
    main()
