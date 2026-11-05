# -*- coding: utf-8 -*-
"""指标记录：爬升高度、落脚误差、脚与台阶的间距、关节限位。"""

import math

import humanoid_stair_climb.config as config


class RunRecorder:
    """按控制周期采样，结束时给出汇总。"""

    def __init__(self):
        self.samples = []
        self.landings = []       # 每次支撑相开始时的落脚点记录
        self._prev_swing = None  # 首次调用只初始化，不记落脚

    # ------------------------------------------------------------------
    def add(self, t, base, joints, clearance, support_height):
        x, y, z = base
        self.samples.append({
            "t": float(t),
            "x": float(x), "y": float(y), "z": float(z),
            "clearance": float(clearance),
            "support_height": float(support_height),
            "hips": (float(joints[0]), float(joints[3])),
            "knees": (float(joints[1]), float(joints[4])),
            "ankles": (float(joints[2]), float(joints[5])),
        })

    def add_landing(self, side, t, target_x, actual_x, target_z, actual_z):
        self.landings.append({
            "side": side,
            "t": float(t),
            "err_x": float(actual_x - target_x),
            "err_z": float(actual_z - target_z),
        })

    def detect_landings(self, t, foot_states, sim=None, foot_offset=0.06):
        """摆动 → 支撑的瞬间记为一次落脚。

        foot_states: {"l": (x, z, swing), ...}，x/z 是**踝**的目标位置；
        sim 给定时用实际脚底中心做对比（脚底中心比踝前伸 0.06 m）。
        """
        if self._prev_swing is None:
            self._prev_swing = {k: v[2] for k, v in foot_states.items()}
            return
        for side, (fx, fz, swing) in foot_states.items():
            if self._prev_swing[side] and not swing:
                if sim is not None:
                    bottom = sim.foot_bottom(side)
                    self.add_landing(side, t, fx + foot_offset, bottom[0],
                                     fz, bottom[2])
                else:
                    self.add_landing(side, t, fx, fx, fz, fz)
            self._prev_swing[side] = swing

    # ------------------------------------------------------------------
    def summary(self):
        s = self.samples
        if not s:
            return {
                "reached": False, "time": 0.0, "steps": 0,
                "climb": 0.0, "max_base_z": 0.0, "final_x": 0.0,
                "min_clearance": 0.0, "max_penetration": 0.0,
                "landing_err_max": 0.0, "joint_violations": 0,
                "samples": 0,
            }

        max_z = max(p["z"] for p in s)
        min_clr = min(p["clearance"] for p in s)
        penetration = max(0.0, -min_clr)
        landing_err = max([math.hypot(l["err_x"], l["err_z"]) for l in self.landings] or [0.0])

        # 六个关节全查：hip / knee / ankle 各两条腿。
        # 限位取自 config.JOINT_LIMITS，与 models/stair_scene.xml 的 joint range 一致。
        violations = 0
        pairs = (("hip", "hips"), ("knee", "knees"), ("ankle", "ankles"))
        for p in s:
            for name, key in pairs:
                lo, hi = config.JOINT_LIMITS[name]
                if any(v < lo or v > hi for v in p[key]):
                    violations += 1
                    break

        return {
            "reached": bool(max_z >= config.REACH_Z and s[-1]["x"] >= config.REACH_X),
            "time": s[-1]["t"],
            "steps": len(self.landings),
            "climb": max_z - s[0]["z"],
            "max_base_z": max_z,
            "final_x": s[-1]["x"],
            "min_clearance": min_clr,
            "max_penetration": penetration,
            "landing_err_max": landing_err,
            "joint_violations": violations,
            "samples": len(s),
        }

    # ------------------------------------------------------------------
    def write_csv(self, path):
        import csv
        fields = ["t", "x", "y", "z", "clearance", "support_height",
                  "hip_l", "knee_l", "ankle_l", "hip_r", "knee_r", "ankle_r"]
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(fields)
            for p in self.samples:
                w.writerow([
                    "%.5f" % p["t"], "%.5f" % p["x"], "%.5f" % p["y"], "%.5f" % p["z"],
                    "%.5f" % p["clearance"], "%.5f" % p["support_height"],
                    "%.5f" % p["hips"][0], "%.5f" % p["knees"][0], "%.5f" % p["ankles"][0],
                    "%.5f" % p["hips"][1], "%.5f" % p["knees"][1], "%.5f" % p["ankles"][1],
                ])
        return path


def format_summary(s):
    """一行摘要，供终端与测试断言复用。"""
    return ("到达=%s | 用时=%.2fs | 步数=%d | 爬升=%.3fm | 最高基座=%.3fm | "
            "终点x=%.2f | 最小间距=%.3fm | 最大穿透=%.3fm | 落脚误差=%.3fm | 关节越界=%d"
            % (s["reached"], s["time"], s["steps"], s["climb"], s["max_base_z"],
               s["final_x"], s["min_clearance"], s["max_penetration"],
               s["landing_err_max"], s["joint_violations"]))
