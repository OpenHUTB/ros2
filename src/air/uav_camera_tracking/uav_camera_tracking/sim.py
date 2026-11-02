# -*- coding: utf-8 -*-
"""sim.py —— 闭环仿真：单回合推演、示教数据采集、批量性能评价。

所有控制器都通过同一个 rollout 接口跑同一批场景，保证对比条件一致。
"""
import numpy as np

from . import params as P
from .camera import Camera
from .controllers import (ServoState, detection_from_gt, expert_command,
                          pixel_errors, pointing_yaw)
from .quadrotor import Quadrotor


def detection_error_px(gt):
    return float(np.hypot(gt['u'] - (P.IMG_W - 1) / 2.0,
                          gt['v'] - (P.IMG_H - 1) / 2.0))


def rollout(scene, controller, camera, rng, record_every=0):
    """跑一个回合，返回日志字典（含真值误差与可见性，用于评价）。"""
    quad = Quadrotor(scene.uav0, scene.yaw0)
    controller.reset()
    n = scene.steps()
    log = {
        't': [], 'p': [], 'target': [], 'err': [], 'visible': [],
        'cmd': [], 'depth': [], 'frames': [], 'cls_frames': [],
    }
    for k in range(n):
        t = k * P.DT
        tp = scene.target_fn(t)
        cam_pos = quad.camera_pose()
        img = camera.render(cam_pos, quad.yaw, tp, scene.distractors,
                            scene.walls, rng)
        gt = camera.observe(cam_pos, quad.yaw, tp, scene.walls)
        cmd = controller.step(img, quad, P.DT, gt)
        lost = controller.lost_steps > 0
        quad.set_cmd(cmd[0:2], cmd[2], controller.yaw_des(quad, lost))
        quad.step(scene.wind(t))

        log['t'].append(t)
        log['p'].append(quad.p.copy())
        log['target'].append(np.array(tp, dtype=float))
        log['err'].append(detection_error_px(gt) / (P.IMG_W / 2.0))
        log['visible'].append(1.0 if gt['visible'] else 0.0)
        log['cmd'].append(cmd.copy())
        log['depth'].append(gt['depth'])
        if record_every and (k % record_every == 0):
            log['frames'].append(img.copy())
            log['cls_frames'].append({
                'gt': gt,
                'det': detection_from_gt(gt),
                'err': log['err'][-1],
                't': t,
            })
    for key in ('p', 'target', 'cmd'):
        log[key] = np.array(log[key])
    for key in ('err', 'visible', 'depth', 't'):
        log[key] = np.array(log[key])
    return log


def evaluate(log):
    """把回合日志压成指标：成功率、像素误差 RMS、丢失占比、平均速度。"""
    vis = log['visible'] > 0.5
    err = log['err']
    vis_frac = float(vis.mean())
    err_vis = err[vis] if vis.any() else np.array([1.0])
    rms = float(np.sqrt(np.mean(err_vis ** 2)))
    success = bool(vis_frac >= P.LOST_FRAC_OK and rms <= P.ERR_OK)
    speed = float(np.mean(np.linalg.norm(log['cmd'][:, 0:2], axis=1)))
    return {
        'visible_frac': vis_frac,
        'err_rms': rms,
        'err_mean': float(err_vis.mean()),
        'lost_frac': 1.0 - vis_frac,
        'success': success,
        'mean_speed': speed,
    }


def _e2e_feat(camera, img, v_prev):
    """端到端网络输入：图像特征 + 上一步指令(可选)。"""
    f = camera.downsample(img, P.E2E_DS).ravel()
    if P.E2E_CMD_FEEDBACK:
        f = np.concatenate([f, np.asarray(v_prev, float)
                            / np.array([P.V_MAX, P.V_MAX, P.VZ_MAX])])
    return f


def collect_demo(scenes, expert, camera, episodes=1, seed=0, feat_jitter=0.01):
    """用几何专家飞若干回合，采集 (特征, 指令) 与 (图像, 指令) 两套示教数据。"""
    rng = np.random.default_rng(seed)
    Xf, Yf, Xi, Yi = [], [], [], []
    for scene in scenes:
        for ep in range(episodes):
            quad = Quadrotor(scene.uav0, scene.yaw0)
            servo = ServoState()
            v_prev = np.zeros(3)
            lost_ct = 0
            for k in range(scene.steps()):
                t = k * P.DT
                tp = scene.target_fn(t)
                cam_pos = quad.camera_pose()
                img = camera.render(cam_pos, quad.yaw, tp, scene.distractors,
                                    scene.walls, rng)
                gt = camera.observe(cam_pos, quad.yaw, tp, scene.walls)
                det = detection_from_gt(gt)
                ex, ey = pixel_errors(det)
                servo.update(ey, P.DT)
                feat = servo.features(ex, ey, quad.p[2],
                                      v_prev[0], v_prev[1],
                                      det['vis'] >= 0.5)
                cmd, _lost = expert_command(feat, det['vis'] >= 0.5)
                if det['vis'] >= 0.5:      # 只对"看得见"的时刻监督控制量
                    noisy = feat + rng.normal(0.0, feat_jitter, feat.shape)
                    label = [cmd[0] / P.V_MAX, cmd[1] / P.V_MAX,
                             cmd[2] / P.VZ_MAX]
                    Xf.append(noisy)
                    Yf.append(label)
                    Xi.append(_e2e_feat(camera, img, v_prev))
                    Yi.append(label)
                cmd = np.asarray(cmd, float)
                v_prev = cmd
                lost_ct = lost_ct + 1 if det['vis'] < 0.5 else 0
                quad.set_cmd(cmd[0:2], cmd[2],
                             pointing_yaw(quad.yaw, ex, lost_ct))
                quad.step(scene.wind(t))
    return (np.array(Xf), np.array(Yf), np.array(Xi), np.array(Yi))


def dagger_collect(scenes, learner, camera, seed=0, feat_jitter=0.02):
    """DAgger：让学习控制器自己飞，专家对"它访问到的状态"重新标注。

    行为克隆只在专家轨迹附近有效，一旦闭环偏离进入未见状态就会外推发散；
    这里把学习器实际遇到的 (特征/图像, 专家指令) 收回来再训练，正是
    #529 机械臂模块里用过的同一套思路。
    """
    rng = np.random.default_rng(seed)
    Xf, Yf, Xi, Yi = [], [], [], []
    for scene in scenes:
        quad = Quadrotor(scene.uav0, scene.yaw0)
        learner.reset()
        for k in range(scene.steps()):
            t = k * P.DT
            tp = scene.target_fn(t)
            cam_pos = quad.camera_pose()
            img = camera.render(cam_pos, quad.yaw, tp, scene.distractors,
                                scene.walls, rng)
            gt = camera.observe(cam_pos, quad.yaw, tp, scene.walls)
            cmd = learner.step(img, quad, P.DT, gt)
            feat = getattr(learner, 'last_feat', None)
            v_prev_learner = learner.v_prev.copy()
            if feat is not None and gt['visible']:
                expert_feat = feat.copy()
                expert_cmd, _ = expert_command(expert_feat, True)
                label = [expert_cmd[0] / P.V_MAX, expert_cmd[1] / P.V_MAX,
                         expert_cmd[2] / P.VZ_MAX]
                Xf.append(expert_feat + rng.normal(0.0, feat_jitter,
                                                   expert_feat.shape))
                Yf.append(label)
                if getattr(learner, 'image_input', False):
                    Xi.append(learner.features_from_img(img, v_prev_learner).ravel())
                    Yi.append(label)
            lost = learner.lost_steps > 0
            quad.set_cmd(cmd[0:2], cmd[2], learner.yaw_des(quad, lost))
            quad.step(scene.wind(t))
    return (np.array(Xf), np.array(Yf), np.array(Xi), np.array(Yi))


def random_training_scenes(seed=0, count=8):
    """示教用的随机化场景（目标轨迹/初始位姿不同），提升泛化性。"""
    from .scenes import Scene
    rng = np.random.default_rng(seed)
    out = []
    for i in range(count):
        cx, cy = rng.uniform(6, 14), rng.uniform(-6, 6)
        r = rng.uniform(3.0, 6.0)
        w = rng.uniform(0.12, 0.30) * rng.choice([-1, 1])
        phase = rng.uniform(0, 2 * np.pi)
        start = np.array([cx - 6.0 + rng.uniform(-2, 2),
                          cy + rng.uniform(-3, 3), P.H_DES])

        def fn(t, cx=cx, cy=cy, r=r, w=w, phase=phase):
            a = phase + w * t
            return (cx + r * np.cos(a), cy + r * np.sin(a))

        out.append(Scene(
            key='train_%d' % i, name='示教场景%d' % i, desc='随机圆周/线性目标',
            duration=16.0, uav0=tuple(start), yaw0=rng.uniform(-0.6, 0.6),
            target_fn=fn,
            distractors=((cx + rng.uniform(-5, 5), cy + rng.uniform(-5, 5)),),
        ))
    return out
