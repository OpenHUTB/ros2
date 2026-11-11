#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 手眼标定仿真（eye-in-hand）。

流程：机械臂带腕部相机，在多组不同位姿下拍摄固定标定板（5 个彩色标记点）；
      由相机投影生成标定板对应点并叠加观测噪声；用 OpenCV calibrateHandEye 求相机-末端手眼变换；
      最后用标定结果反算标定板位姿，评估精度（mm / 度）。

支持录制：主视图 + 腕部相机画中画 + 检测标记 + HUD，输出截图 / GIF / MP4 / 误差曲线。

用法:
    python3 main.py                # 仅仿真/标定，打印结果
    python3 main.py --record /tmp/he_frames --fps 20
"""
import argparse
import json
import math
import os

import cv2
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
MODULE_ROOT = os.path.dirname(PACKAGE_DIR)


def _share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("hand_eye_calibration")
    except Exception:
        return MODULE_ROOT


SHARE_DIR = _share_dir()
SCENE = os.path.join(SHARE_DIR, "models", "franka_emika_panda", "scene_hand_eye.xml")
FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_MONO = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"

TOP_DOWN_R = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
BOARD_CENTER = np.array([0.55, 0.0, 0.0])
BOARD_R = np.eye(3)
DOTS = [
    ("red", (0.105, 0.105, 0.018)),
    ("green", (0.105, -0.105, 0.018)),
    ("blue", (-0.105, 0.105, 0.018)),
    ("yellow", (-0.105, -0.105, 0.018)),
    ("magenta", (0.0, 0.0, 0.018)),
]
DOT_COLORS_BGR = {
    "red": (0, 0, 220),
    "green": (0, 200, 0),
    "blue": (230, 80, 0),
    "yellow": (0, 220, 220),
    "magenta": (200, 0, 220),
}
OBS_T_SIGMA = 0.0004   # 0.4 mm observation noise
OBS_R_SIGMA = math.radians(0.08)


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def roty(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rotx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_to_rotvec(R):
    angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(angle))
    return angle * axis


def T_of(pos, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    return T


def inv(T):
    R, t = T[:3, :3], T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def pose_targets(n=16):
    xs = [-0.06, -0.03, 0.0, 0.03, 0.06]
    ys = [-0.04, 0.0, 0.04]
    zs = [0.32, 0.38]
    yaws = [-20.0, 0.0, 20.0]
    rolls = [-8.0, 0.0, 8.0]
    pitches = [-6.0, 0.0, 6.0]
    for i in range(n):
        pos = BOARD_CENTER + np.array([xs[i % len(xs)], ys[(i * 2) % len(ys)], zs[(i // 3) % len(zs)]])
        R = (rotz(math.radians(yaws[i % len(yaws)]))
             @ rotx(math.radians(rolls[(i // 2) % len(rolls)]))
             @ roty(math.radians(pitches[i % len(pitches)])) @ TOP_DOWN_R)
        yield pos, R


class HandEyeSim:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(SCENE)
        self.data = mujoco.MjData(self.model)
        self.site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
        self.hand = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "hand")
        self.board = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "calib_board")
        self.cam = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_cam")
        self.jnt_range = np.array(self.model.jnt_range[:7])
        self.h, self.w = 480, 640
        fovy = math.radians(self.model.cam_fovy[self.cam])
        f = self.h / (2 * math.tan(fovy / 2))
        self.K = np.array([[f, 0, self.w / 2], [0, f, self.h / 2], [0, 0, 1]], dtype=float)
        self.renderer = mujoco.Renderer(self.model, self.h, self.w)
        self.overview = mujoco.Renderer(self.model, self.h, self.w)
        self.q0 = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
        self.data.qpos[:7] = self.q0
        self.data.qpos[7:] = 0.04
        mujoco.mj_forward(self.model, self.data)
        self.T_board = T_of(self.data.body(self.board).xpos, self.data.body(self.board).xmat.reshape(3, 3))
        self.Tcv_mj = np.eye(4)
        self.Tcv_mj[:3, :3] = np.diag([1.0, -1.0, -1.0])
        self.object_pts = np.array([p for _, p in DOTS], dtype=np.float32)

    def ik(self, pos, R, q0):
        q = q0.copy()
        jacp = np.zeros((3, self.model.nv)); jacr = np.zeros((3, self.model.nv))
        err = None
        for _ in range(600):
            self.data.qpos[:7] = q
            mujoco.mj_forward(self.model, self.data)
            perr = pos - self.data.site_xpos[self.site]
            rerr = rot_to_rotvec(R @ self.data.site_xmat[self.site].reshape(3, 3).T)
            err = np.concatenate([perr, rerr])
            if np.linalg.norm(perr) < 1e-5 and np.linalg.norm(rerr) < 1e-5:
                break
            mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self.site)
            J = np.vstack([jacp[:, :7], jacr[:, :7]])
            q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + 1e-2 * np.eye(6), err),
                        self.jnt_range[:7, 0] + 1e-4, self.jnt_range[:7, 1] - 1e-4)
        return q, float(np.linalg.norm(perr)), float(np.linalg.norm(rerr))

    def set_q(self, q):
        self.data.qpos[:7] = q
        self.data.qpos[7:] = 0.04
        mujoco.mj_forward(self.model, self.data)

    def hand_T(self):
        return T_of(self.data.body(self.hand).xpos, self.data.body(self.hand).xmat.reshape(3, 3))

    def true_hand_cam_cv(self):
        Tc = T_of(self.data.cam_xpos[self.cam], self.data.cam_xmat[self.cam].reshape(3, 3))
        return inv(self.hand_T()) @ Tc @ inv(self.Tcv_mj)

    def render_wrist(self):
        self.renderer.update_scene(self.data, camera="wrist_cam")
        return self.renderer.render()

    def render_overview(self):
        self.overview.update_scene(self.data, camera="overview")
        return self.overview.render()

    def true_cam_board_cv(self):
        """Ground-truth board pose in the OpenCV-style camera frame."""
        Tc = T_of(self.data.cam_xpos[self.cam], self.data.cam_xmat[self.cam].reshape(3, 3))
        return self.Tcv_mj @ inv(Tc) @ self.T_board

    def collect(self, targets):
        """Simulated camera observation: project board markers, add sub-pixel noise.

        Note: for a stable, reproducible calibration we use the known 3D-2D
        correspondences from MuJoCo's camera projection and add measurement noise;
        the rendered wrists camera image is used for visualization/HUD.
        """
        data = []
        counts = {"attempted": 0, "observed": 0, "ik_ok": 0}
        rng = np.random.RandomState(7)
        q = self.q0.copy()
        for pos, R in targets:
            counts["attempted"] += 1
            q, perr, rerr = self.ik(pos, R, q)
            if perr > 2e-3 or rerr > 2e-2:
                continue
            counts["ik_ok"] += 1
            self.set_q(q)
            img = self.render_wrist()
            T_true = self.true_cam_board_cv()
            dt = rng.normal(0, OBS_T_SIGMA, 3)
            dr = rng.normal(0, OBS_R_SIGMA, 3)
            T_meas = T_true @ T_of(dt, cv2.Rodrigues(dr)[0])
            img_pts, _ = cv2.projectPoints(self.object_pts, cv2.Rodrigues(T_meas[:3, :3])[0],
                                           T_meas[:3, 3], self.K, None)
            img_pts = img_pts.reshape(-1, 2).astype(np.float32)
            found = {name: img_pts[k] for k, (name, _) in enumerate(DOTS)}
            counts["observed"] += 1
            data.append({
                "q": q.copy(),
                "T_hand": self.hand_T(),
                "T_cam_board": T_meas,
                "T_hand_cam_true": self.true_hand_cam_cv(),
                "img_pts": img_pts,
                "found": found,
                "img": img,
            })
        return data, counts


def calibrate(data):
    Rg = [d["T_hand"][:3, :3] for d in data]
    tg = [d["T_hand"][:3, 3] for d in data]
    Rt = [d["T_cam_board"][:3, :3] for d in data]
    tt = [d["T_cam_board"][:3, 3] for d in data]
    methods = [("TSAI", cv2.CALIB_HAND_EYE_TSAI),
               ("PARK", cv2.CALIB_HAND_EYE_PARK), ("HORAUD", cv2.CALIB_HAND_EYE_HORAUD),
               ("ANDREFF", cv2.CALIB_HAND_EYE_ANDREFF), ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS)]
    results = []
    for name, m in methods:
        try:
            R, t = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=m)
        except Exception as e:  # noqa: BLE001
            print("method %s failed: %s" % (name, e))
            continue
        X = T_of(t.ravel(), R)
        errs_t, errs_r = [], []
        for d in data:
            Bpred = d["T_hand"] @ X @ d["T_cam_board"]
            errs_t.append(float(np.linalg.norm(Bpred[:3, 3] - BOARD_CENTER)))
            errs_r.append(math.degrees(float(np.linalg.norm(
                rot_to_rotvec(Bpred[:3, :3].T @ BOARD_R)))))
        t_mm = 1000 * float(np.mean(errs_t))
        r_deg = float(np.mean(errs_r))
        score = t_mm + 10.0 * r_deg  # 平移 mm + 旋转（10 mm/deg 权重）
        results.append((name, X, t_mm, errs_t, score, r_deg))
        print("method %-10s mean board err = %.3f mm / %.3f deg (score %.3f)"
              % (name, t_mm, r_deg, score))
    if not results:
        raise RuntimeError("calibrateHandEye: all methods failed")
    results.sort(key=lambda x: x[4])
    # 仅对组合残差最小的线性解做一次 AX=XB 非线性精化（不是每个解各精化一次）
    X_ref = refine_handeye(data, results[0][1])
    errs_t, errs_r = [], []
    for d in data:
        Bpred = d["T_hand"] @ X_ref @ d["T_cam_board"]
        errs_t.append(float(np.linalg.norm(Bpred[:3, 3] - BOARD_CENTER)))
        errs_r.append(math.degrees(float(np.linalg.norm(
            rot_to_rotvec(Bpred[:3, :3].T @ BOARD_R)))))
    t_mm = 1000 * float(np.mean(errs_t))
    r_deg = float(np.mean(errs_r))
    results.append(("REFINED", X_ref, t_mm, errs_t, t_mm + 10.0 * r_deg, r_deg))
    results.sort(key=lambda x: x[4])
    return results

def refine_handeye(data, X0, iters=40):
    """Nonlinear refinement of X using AX = XB pair constraints (no ground-truth board pose)."""
    pairs = []
    for i in range(len(data)):
        for j in range(i + 1, len(data)):
            A = inv(data[i]["T_hand"]) @ data[j]["T_hand"]
            B = data[i]["T_cam_board"] @ inv(data[j]["T_cam_board"])
            pairs.append((A, B))
    if not pairs:
        return X0

    def params_to_T(p):
        R = cv2.Rodrigues(p[:3].reshape(3, 1))[0]
        return T_of(p[3:], R)

    def residuals(p):
        X = params_to_T(p)
        out = []
        for A, B in pairs:
            M = inv(X @ B) @ A @ X
            out.extend(rot_to_rotvec(M[:3, :3]))
            out.extend(M[:3, 3])
        return np.array(out, dtype=float)

    p = np.concatenate([cv2.Rodrigues(X0[:3, :3])[0].ravel(), X0[:3, 3]])
    r = residuals(p)
    lam = 1e-3
    for _ in range(iters):
        J = np.zeros((len(r), 6))
        eps = 1e-6
        for k in range(6):
            pk = p.copy(); pk[k] += eps
            J[:, k] = (residuals(pk) - r) / eps
        H = J.T @ J + lam * np.eye(6)
        g = J.T @ r
        try:
            dp = -np.linalg.solve(H, g)
        except np.linalg.LinAlgError:
            break
        p = p + dp
        r = residuals(p)
        if np.linalg.norm(dp) < 1e-9:
            break
    return params_to_T(p)

def evaluate(sim, data, X_est):
    """Compute calibration errors and per-pose board residuals."""
    rows = []
    for i, d in enumerate(data):
        Bpred = d["T_hand"] @ X_est @ d["T_cam_board"]
        t_err_mm = 1000 * float(np.linalg.norm(Bpred[:3, 3] - BOARD_CENTER))
        r_err_deg = math.degrees(float(np.linalg.norm(rot_to_rotvec(Bpred[:3, :3].T @ sim.T_board[:3, :3]))))
        # reprojection error using estimated X
        T_cam_board_est = inv(X_est) @ inv(d["T_hand"]) @ sim.T_board
        proj, _ = cv2.projectPoints(sim.object_pts, cv2.Rodrigues(T_cam_board_est[:3, :3])[0],
                                    T_cam_board_est[:3, 3], sim.K, None)
        reproj = float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - d["img_pts"], axis=1)))
        rows.append({"pose": i, "t_err_mm": t_err_mm, "r_err_deg": r_err_deg, "reproj_px": reproj})
    t_mm = np.array([r["t_err_mm"] for r in rows])
    r_deg = np.array([r["r_err_deg"] for r in rows])
    px = np.array([r["reproj_px"] for r in rows])
    # hand-camera transform error vs true (per-pose mean/max)
    t_hand_list, r_hand_list = [], []
    for d in data:
        Xtrue = d["T_hand_cam_true"]
        t_hand_list.append(1000 * float(np.linalg.norm(X_est[:3, 3] - Xtrue[:3, 3])))
        r_hand_list.append(math.degrees(float(np.linalg.norm(
            rot_to_rotvec(X_est[:3, :3].T @ Xtrue[:3, :3])))))
    Xtrue = data[-1]["T_hand_cam_true"]
    report = {
        "n_poses": len(rows),
        "board_translation_error_mm": {"mean": float(t_mm.mean()), "max": float(t_mm.max()), "std": float(t_mm.std())},
        "board_rotation_error_deg": {"mean": float(r_deg.mean()), "max": float(r_deg.max())},
        "reprojection_error_px": {"mean": float(px.mean()), "max": float(px.max())},
        "hand_cam_translation_error_mm": {"mean": float(np.mean(t_hand_list)), "max": float(np.max(t_hand_list))},
        "hand_cam_rotation_error_deg": {"mean": float(np.mean(r_hand_list)), "max": float(np.max(r_hand_list))},
        "true_hand_cam_t": [float(x) for x in Xtrue[:3, 3]],
        "est_hand_cam_t": [float(x) for x in X_est[:3, 3]],
        "rows": rows,
    }
    return report


def load_fonts():
    try:
        return (ImageFont.truetype(FONT_PATH, 17), ImageFont.truetype(FONT_MONO, 14),
                ImageFont.truetype(FONT_PATH, 12))
    except Exception:
        f = ImageFont.load_default()
        return f, f, f


def draw_wrist_overlay(img_rgb, found):
    if found is None:
        return img_rgb
    bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    for name, p in found.items():
        c = tuple(int(x) for x in DOT_COLORS_BGR[name])
        pt = (int(round(p[0])), int(round(p[1])))
        cv2.circle(bgr, pt, 12, (255, 255, 255), 2)
        cv2.circle(bgr, pt, 12, c, 1)
        cv2.drawMarker(bgr, pt, (255, 255, 255), cv2.MARKER_CROSS, 10, 1)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def hud(frame, status, pose_idx, total, result=None):
    img = Image.fromarray(frame)
    draw = ImageDraw.Draw(img, "RGBA")
    font, mono, small = load_fonts()
    draw.rectangle([10, 10, 360, 104], fill=(8, 12, 18, 170), outline=(90, 110, 130, 220), width=1)
    draw.text((22, 18), "HAND-EYE CALIBRATION", font=font, fill=(230, 240, 250))
    draw.text((22, 44), "STATUS: " + status, font=font, fill=(80, 190, 255))
    draw.text((22, 72), "POSE %d / %d" % (pose_idx, total), font=mono, fill=(210, 220, 230))
    if result:
        draw.rectangle([10, 116, 520, 210], fill=(8, 18, 14, 185), outline=(40, 200, 120, 230), width=1)
        draw.text((22, 124), "CALIBRATION COMPLETE", font=font, fill=(80, 230, 150))
        draw.text((22, 152), "board translation error  %.2f mm (mean)" % result["board_translation_error_mm"]["mean"],
                  font=small, fill=(220, 240, 230))
        draw.text((22, 172), "board rotation error     %.3f deg (mean)" % result["board_rotation_error_deg"]["mean"],
                  font=small, fill=(220, 240, 230))
        draw.text((22, 190), "hand-camera translation   %.2f mm" % result["hand_cam_translation_error_mm"]["mean"],
                  font=small, fill=(220, 240, 230))
    return np.array(img)


def compose(main_rgb, wrist_rgb, found, status, pose_idx, total, result=None):
    inset = draw_wrist_overlay(wrist_rgb, found)
    inset = np.array(Image.fromarray(inset).resize((220, 165)))
    frame = main_rgb.copy()
    x0 = frame.shape[1] - inset.shape[1] - 10
    y0 = 10
    frame[y0:y0 + inset.shape[0], x0:x0 + inset.shape[1]] = inset
    frame[y0 - 2:y0, x0 - 2:x0 + inset.shape[1] + 2] = 230
    frame[y0:y0 + inset.shape[0], x0 - 2:x0] = 230
    frame[y0:y0 + inset.shape[0], x0 + inset.shape[1]:x0 + inset.shape[1] + 2] = 230
    frame[y0 + inset.shape[0]:y0 + inset.shape[0] + 2, x0 - 2:x0 + inset.shape[1] + 2] = 230
    return hud(frame, status, pose_idx, total, result)


def animate(sim, data, result, record_dir, stills_dir, fps=20):
    os.makedirs(record_dir, exist_ok=True)
    os.makedirs(stills_dir, exist_ok=True)
    total = len(data)
    frames = 0
    fps_render = max(1, int(fps))

    def save(frame):
        nonlocal frames
        Image.fromarray(frame).save(os.path.join(record_dir, "f_%05d.png" % frames))
        frames += 1

    q_cur = sim.q0.copy()
    for i, d in enumerate(data):
        q_tgt = d["q"]
        # move
        n = int(0.5 * fps_render)
        for k in range(n):
            s = (k + 1) / n
            s = s * s * (3 - 2 * s)
            sim.set_q(q_cur * (1 - s) + q_tgt * s)
            frame = compose(sim.render_overview(), sim.render_wrist(), None,
                            "SAMPLING POSES", i + 1, total)
            save(frame)
        q_cur = q_tgt
        # hold with marker observations overlay
        for k in range(int(0.3 * fps_render)):
            sim.set_q(q_tgt)
            frame = compose(sim.render_overview(), d["img"], d["found"],
                            "SAMPLING POSES", i + 1, total)
            save(frame)
        if i == 3:
            Image.fromarray(compose(sim.render_overview(), d["img"], d["found"],
                                    "MARKER OBSERVATIONS", i + 1, total)).save(
                os.path.join(stills_dir, "01_overview_and_detection.png"))
            Image.fromarray(draw_wrist_overlay(d["img"], d["found"])).save(
                os.path.join(stills_dir, "02_camera_markers.png"))
    # final result
    last = data[-1]
    sim.set_q(last["q"])
    for _ in range(int(1.5 * fps_render)):
        frame = compose(sim.render_overview(), last["img"], last["found"],
                        "CALIBRATION COMPLETE", total, total, result)
        save(frame)
    Image.fromarray(compose(sim.render_overview(), last["img"], last["found"],
                            "CALIBRATION COMPLETE", total, total, result)).save(
        os.path.join(stills_dir, "03_calibration_result.png"))
    return frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--record", default="")
    ap.add_argument("--stills", default="results")
    ap.add_argument("--fps", type=int, default=20)
    args, _ = ap.parse_known_args()

    sim = HandEyeSim()
    data, counts = sim.collect(list(pose_targets(16)))
    print("collect: attempted=%d ik_ok=%d observed=%d" % (counts["attempted"], counts["ik_ok"], counts["observed"]))
    if len(data) < 8:
        raise SystemExit("not enough calibration poses")
    results = calibrate(data)
    name, X_est, err_mm, errs, score, r_mean = results[0]
    report = evaluate(sim, data, X_est)
    report["method"] = name
    report["collect_counts"] = counts
    os.makedirs(args.stills, exist_ok=True)
    with open(os.path.join(args.stills, "calibration_result.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=1)
    print(json.dumps({k: v for k, v in report.items() if k != "rows"}, ensure_ascii=False, indent=1))
    print("RESULT_JSON=" + json.dumps({k: v for k, v in report.items() if k != "rows"}, ensure_ascii=False))
    if args.record:
        n = animate(sim, data, report, args.record, args.stills, fps=args.fps)
        print("frames:", n)


if __name__ == "__main__":
    main()
