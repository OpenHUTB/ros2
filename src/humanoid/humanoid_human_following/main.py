"""
humanoid_human_following standalone 入口
=======================================
自建简化双足人形在 MuJoCo 场景中实时跟随移动目标（红色球体），保持设定距离。

用法：
    python3 main.py                  # 弹出可视化窗口，红球走 L 形，人形跟随
    python3 main.py --headless       # 无窗口，仅命令行输出
    python3 main.py --gif            # 离屏录制演示动图 doc/humanoid_human_following.gif
    python3 main.py --test           # 自动化测试
    python3 main.py --version        # 版本号
"""

import argparse
import time
import numpy as np
import mujoco

from humanoid_human_following.sim_core import FollowController, CONTROL_HZ, D_KEEP, V_MAX, W_MAX


def main():
    ap = argparse.ArgumentParser(description='人形机器人人体目标跟随')
    ap.add_argument('--headless', action='store_true', help='无窗口模式')
    ap.add_argument('--gif', action='store_true', help='录制演示动图')
    ap.add_argument('--gif-width', type=int, default=480, help='动图宽度')
    ap.add_argument('--gif-colors', type=int, default=48, help='动图调色板大小')
    ap.add_argument('--test', action='store_true', help='自动化测试')
    ap.add_argument('--version', action='store_true', help='版本号')
    args = ap.parse_args()

    if args.version:
        print('humanoid_human_following 1.0.0')
        return
    if args.test:
        return run_test()

    ctrl = FollowController()
    dt = 1.0 / CONTROL_HZ
    n_sub = max(1, int(round(dt / ctrl.model.opt.timestep)))
    sim_duration = 30.0
    steps = int(sim_duration * CONTROL_HZ)
    frames = [] if args.gif else None

    if not args.headless and not args.gif:
        viewer = mujoco.viewer.launch_passive(ctrl.model, ctrl.data)
    else:
        viewer = None

    start = time.time()
    for i in range(steps):
        ctrl.follow_step(dt)
        for _ in range(n_sub):
            mujoco.mj_step(ctrl.model, ctrl.data)
        if viewer is not None:
            viewer.sync()
        if args.gif and len(frames) < 40:
            frames.append(_frame(ctrl))
        time.sleep(max(0.0, dt - (time.time() - start) / (i + 1)))

    if viewer is not None:
        viewer.close()
    if args.gif:
        _save_gif(frames, args.gif_width, args.gif_colors)
        print('动图已生成: doc/humanoid_human_following.gif')
    print('跟随完成：人形沿 L 形轨迹保持 %.2f m 距离跟随目标' % D_KEEP)


def run_test():
    dt = 1.0 / CONTROL_HZ
    ok = True

    # 测试1：目标静止时距离收敛到 D_KEEP
    ctrl = FollowController()
    jid = ctrl.model.body_jntadr[ctrl.target_body.id]
    j0 = ctrl.model.jnt_qposadr[jid]
    ctrl.data.qpos[j0:j0+3] = [3.0, 0.0, 0.3]
    ctrl.freeze = True
    n_sub = max(1, int(round(dt / ctrl.model.opt.timestep)))
    d = 0.0
    for _ in range(int(8 * CONTROL_HZ)):
        d, _, _, _ = ctrl.follow_step(dt)
        for _ in range(n_sub):
            mujoco.mj_step(ctrl.model, ctrl.data)
    converge = abs(d - D_KEEP) < 0.25
    ok &= converge
    print('[%s] 目标静止时距离收敛到期望值: d=%.2f (期望 %.2f)'
          % ('PASS' if converge else 'FAIL', d, D_KEEP))

    # 测试2：控制量限幅
    ctrl2 = FollowController()
    _, v, w, _ = ctrl2.follow_step(dt)
    bounded = abs(v) <= V_MAX + 1e-6 and abs(w) <= W_MAX + 1e-6
    ok &= bounded
    print('[%s] 控制量限幅: v=%.2f (≤%.2f), w=%.2f (≤%.2f)'
          % ('PASS' if bounded else 'FAIL', v, V_MAX, w, W_MAX))
    return 0 if ok else 1


def _frame(ctrl):
    if not hasattr(_frame, 'renderer'):
        _frame.renderer = mujoco.Renderer(ctrl.model, 640, 480)
    mujoco.mj_forward(ctrl.model, ctrl.data)
    _frame.renderer.update_scene(ctrl.data)
    return _frame.renderer.render()


def _save_gif(frames, width, colors):
    import os
    from PIL import Image
    os.makedirs('doc', exist_ok=True)
    imgs = [Image.fromarray(f).resize((width, int(width * 480 / 640))) for f in frames]
    imgs[0].save('doc/humanoid_human_following.gif',
                 save_all=True, append_images=imgs[1:],
                 duration=100, loop=0, optimize=True, palette=colors)


if __name__ == '__main__':
    main()
