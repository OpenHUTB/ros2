"""
humanoid_visual_servo_grasp standalone 入口
===========================================
自建带单臂的简化人形，用前视相机感知桌面目标物（红色立方体），
通过 IK 反解肩/肘关节角，视觉伺服驱动手臂伸向目标并合拢两指夹具完成抓取。

用法：
    python3 main.py                     # 可视化窗口演示抓取
    python3 main.py --headless          # 无窗口，仅命令行输出
    python3 main.py --gif               # 离屏录制演示动图 doc/humanoid_visual_servo_grasp.gif
    python3 main.py --shots             # 渲染关键帧截图
    python3 main.py --test              # 自动化测试
    python3 main.py --version           # 版本号
"""

import argparse
import time
import numpy as np
import mujoco

from humanoid_visual_servo_grasp.sim_core import (
    ServoGraspController, CONTROL_HZ, REACH_TOL)


def run_sim(ctrl, steps, dt, n_sub, callback=None):
    """推进 steps 个控制周期，回调用于录制。"""
    for i in range(steps):
        err, reached, grasped = ctrl.servo_step(dt)
        for _ in range(n_sub):
            mujoco.mj_step(ctrl.model, ctrl.data)
        if callback:
            callback(i, err, reached, grasped)
        if reached and grasped:
            return i, err, reached, grasped
    return steps, None, ctrl.reached, ctrl.grasped


def main():
    ap = argparse.ArgumentParser(description='人形机器人视觉伺服抓取')
    ap.add_argument('--headless', action='store_true', help='无窗口模式')
    ap.add_argument('--gif', action='store_true', help='录制演示动图')
    ap.add_argument('--gif-width', type=int, default=520, help='动图宽度')
    ap.add_argument('--gif-colors', type=int, default=48, help='动图调色板大小')
    ap.add_argument('--shots', action='store_true', help='渲染关键帧截图')
    ap.add_argument('--test', action='store_true', help='自动化测试')
    ap.add_argument('--version', action='store_true', help='版本号')
    args = ap.parse_args()

    if args.version:
        print('humanoid_visual_servo_grasp 1.0.0')
        return
    if args.test:
        return run_test()

    ctrl = ServoGraspController()
    dt = 1.0 / CONTROL_HZ
    n_sub = max(1, int(round(dt / ctrl.model.opt.timestep)))

    frames = []
    shots = []
    if args.shots:
        shots.append(_frame(ctrl, 0))  # 初始

    def cb(i, err, reached, grasped):
        if args.gif and len(frames) < 45:
            frames.append(_frame(ctrl, i))
        if args.shots and i in (int(0.06*CONTROL_HZ), int(0.25*CONTROL_HZ), 0):
            pass  # 截图在 run 后单独处理

    if not args.headless and not args.gif and not args.shots:
        viewer = mujoco.viewer.launch_passive(ctrl.model, ctrl.data)
    else:
        viewer = None

    max_steps = int(4 * CONTROL_HZ)
    start = time.time()
    steps_done = max_steps
    hold_frames = 0
    for i in range(max_steps):
        err, reached, grasped = ctrl.servo_step(dt)
        for _ in range(n_sub):
            mujoco.mj_step(ctrl.model, ctrl.data)
        if viewer is not None:
            viewer.sync()
        if args.gif and len(frames) < 45:
            frames.append(_frame(ctrl, i))
        if reached and grasped:
            steps_done = i
            hold_frames += 1
            if hold_frames >= 10:   # 抓取后继续录 10 帧展示握持
                break
        time.sleep(max(0.0, dt - (time.time() - start) / (i + 1)))

    if viewer is not None:
        viewer.close()

    if args.gif:
        _save_gif(frames, args.gif_width, args.gif_colors)
        print('动图已生成: doc/humanoid_visual_servo_grasp.gif')
    if args.shots:
        # 用全新控制器渲染关键帧，避免复用已抓取的 ctrl
        _render_shots(ServoGraspController(), dt, n_sub)
    print('抓取完成：第 %d 步末端到达目标，夹具合拢完成抓取' % (steps_done + 1))


def run_test():
    dt = 1.0 / CONTROL_HZ
    n_sub = max(1, int(round(dt / 0.002)))
    ok = True

    # 测试1：末端最终到达目标物
    ctrl = ServoGraspController()
    err_final = None
    reached = False
    for _ in range(int(4 * CONTROL_HZ)):
        err, reached, grasped = ctrl.servo_step(dt)
        for _ in range(n_sub):
            mujoco.mj_step(ctrl.model, ctrl.data)
        err_final = err
        if reached:
            break
    arrive = reached and err_final is not None and err_final < REACH_TOL
    ok &= arrive
    print('[%s] 末端到达目标物: err=%.3f (阈值 %.3f)'
          % ('PASS' if arrive else 'FAIL', err_final, REACH_TOL))

    # 测试2：抓取后夹具合拢（两指角度非零）
    ctrl2 = ServoGraspController()
    for _ in range(int(4 * CONTROL_HZ)):
        err, reached, grasped = ctrl2.servo_step(dt)
        for _ in range(n_sub):
            mujoco.mj_step(ctrl2.model, ctrl2.data)
        if reached:
            break
    fl = ctrl2._joint_angle('finger_L_j')
    fr = ctrl2._joint_angle('finger_R_j')
    grasp_closed = ctrl2.grasped and (abs(fl) > 0.01 or abs(fr) > 0.01)
    ok &= grasp_closed
    print('[%s] 抓取后夹具合拢: finger_L=%.3f finger_R=%.3f'
          % ('PASS' if grasp_closed else 'FAIL', fl, fr))
    return 0 if ok else 1


def _frame(ctrl, idx):
    if not hasattr(_frame, 'renderer'):
        _frame.renderer = mujoco.Renderer(ctrl.model, 700, 560)
    mujoco.mj_forward(ctrl.model, ctrl.data)
    _frame.renderer.update_scene(ctrl.data)
    return _frame.renderer.render()


def _save_gif(frames, width, colors):
    import os
    from PIL import Image
    os.makedirs('doc', exist_ok=True)
    imgs = [Image.fromarray(f).resize((width, int(width * 560 / 700))) for f in frames]
    imgs[0].save('doc/humanoid_visual_servo_grasp.gif',
                 save_all=True, append_images=imgs[1:],
                 duration=100, loop=0, optimize=True, palette=colors)


def _render_shots(ctrl, dt, n_sub):
    """渲染关键帧：初始 / 伸向中 / 追近 / 抓取完成（默认机位）。"""
    from PIL import Image
    import os
    renderer = mujoco.Renderer(ctrl.model, 700, 560)
    os.makedirs('shot', exist_ok=True)

    def shot(tag):
        mujoco.mj_forward(ctrl.model, ctrl.data)
        renderer.update_scene(ctrl.data)
        Image.fromarray(renderer.render()).save('shot/%s.png' % tag)
        print('saved shot/%s.png' % tag)

    shot('scene_start')
    total = int(4 * CONTROL_HZ)
    for i in range(total):
        err, reached, grasped = ctrl.servo_step(dt)
        for _ in range(n_sub):
            mujoco.mj_step(ctrl.model, ctrl.data)
        # 固定时间点截中间帧
        if i == int(0.15 * CONTROL_HZ):
            shot('scene_reaching')
        if i == int(0.30 * CONTROL_HZ) and not reached:
            shot('scene_near')
        if reached:
            shot('scene_grasp')
            break
    # 若一直未抓取，结束时也截一帧
    if not ctrl.grasped:
        shot('scene_final')


if __name__ == '__main__':
    main()
