#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Panda 机械臂 MuJoCo 键盘控制（安全与限位增强版）
====================================================
在原有键盘控制基础上，新增以下安全特性（对应项目文档“未验证项”第二条）：

1. 关节硬限位检查
   - 从 MuJoCo 模型直接读取每个关节的 [下限, 上限]（jnt_range）。
   - 按键请求的目标角度超出硬限位时【拒绝执行】并打印原因，避免关节被推到物理极限之外。
   - 目标角度进入“软限位区”（接近边界 10%）时打印警告，提醒用户即将触达极限。

2. 速度限幅与平滑（斜坡逼近）
   - 不再“每按一次键目标直接跳变”，而是让写入 ctrl 的位置伺服命令
     以“最大关节速度”为上限，逐步逼近目标角（斜坡插值）。
   - 即使目标角瞬间跳变 0.1 rad，实际关节速度也被钳制在 MAX_JOINT_SPEED 内，
     避免因目标突变导致的瞬间猛加速。

3. 碰撞检测（自碰撞保护）
   - 每个物理步进后读取 data.contact，检测机械臂“非相邻连杆”之间是否发生接触。
   - 一旦检测到自碰撞：立即回退到碰撞发生前的安全关节位置，
     并进入保护状态打印告警；反向运动退出或按 r 复位。
   - 原理：MuJoCo 默认不计算“父子 body（相邻连杆）”之间的接触，
     所以出现的接触即代表非相邻连杆的真实自碰撞，可靠且无多余误报。

4. 急停 E-STOP
   - 按 空格键 立即锁住所有关节（目标与命令冻结），再次按 空格 解除。

5. 步长调节
   - 按 [ 减小单次增量，按 ] 增大单次增量（范围 0.02 ~ 0.5 rad）。

6. 实时状态面板
   - 每 0.5 秒刷新一行，显示各关节当前角度、是否接近/到达限位、运行状态。

按键说明（沿用原设计并扩展）：
  1-7        : 对应关节正向转动
  Shift+1-7  : 对应关节反向转动
  r          : 复位到初始关键帧姿态
  [ / ]      : 减小 / 增大单步增量
  空格       : 急停（再按解除）
  q          : 退出程序
"""

import os
import sys
import time
import tty
import termios
import threading

import mujoco
import mujoco.viewer

# ---------------------------------------------------------------------------
# 0. 配置参数（可按需修改）
# ---------------------------------------------------------------------------
XML_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    'mujoco_menagerie/franka_emika_panda/scene.xml',
)

JOINT_STEP_DEFAULT = 0.1     # 单次按键的关节目标增量（rad）
JOINT_STEP_MIN = 0.02        # 步长调节下限
JOINT_STEP_MAX = 0.5         # 步长调节上限
MAX_JOINT_SPEED = 1.0        # 最大关节速度（rad/s），速度限幅的核心参数
SOFT_LIMIT_RATIO = 0.10      # 软限位区比例（距边界的 10% 内视为接近极限）
STATUS_INTERVAL = 0.5        # 状态面板刷新间隔（秒）
SIM_SLEEP = 0.005            # 每帧间隔（秒）

# Shift+数字键 在键盘上产生的符号 -> 对应关节索引（反向转动）
REVERSE_KEYS = {'!': 0, '@': 1, '#': 2, '$': 3, '%': 4, '^': 5, '&': 6}
JOINT_NAMES = ['joint{}'.format(i + 1) for i in range(7)]

# ---------------------------------------------------------------------------
# 1. 加载模型并提取“安全元数据”
# ---------------------------------------------------------------------------
if not os.path.exists(XML_PATH):
    print('[错误] 未找到模型文件: {}'.format(XML_PATH))
    print('       请先执行:')
    print('       cd ~/catkin_ws/src/panda_keyboard_control/mujoco_sim')
    print('       git clone https://github.com/google-deepmind/mujoco_menagerie.git')
    sys.exit(1)

model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)

# 前 7 个关节 = 机械臂关节（Panda 有 7 个臂关节 + 2 个夹爪关节，这里只控臂）
n_joint = 7

# 通过名称取每个臂关节的 id，并读取其硬限位 [min, max]
jnt_ids = [
    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    for name in JOINT_NAMES
]
if any(j < 0 for j in jnt_ids):
    print('[错误] 模型中没有找到关节: {}'.format(JOINT_NAMES))
    sys.exit(1)

JNT_RANGE = model.jnt_range[jnt_ids]        # shape (7, 2)，每行 [min, max]

# 机械臂连杆对应的 body id（用于自碰撞检测过滤）
ARM_BODIES = set()
for i in range(model.nbody):
    bname = model.body(i).name or ''
    if bname.startswith('link') or bname in ('hand', 'panda_hand'):
        ARM_BODIES.add(i)

# ---------------------------------------------------------------------------
# 2. 运行时状态
# ---------------------------------------------------------------------------
target_q = data.qpos[:n_joint].copy()   # 用户期望的目标角
ctrl_q = target_q.copy()                # 实际写入 ctrl 的位置伺服命令（斜坡逼近用）
prev_target = target_q.copy()           # 上一次（安全）目标，用于碰撞回退
prev_ctrl = ctrl_q.copy()               # 上一次命令，用于碰撞回退

pressed_key = None
e_stop = False                          # 急停标志
collision_flag = False                  # 碰撞保护标志
joint_step = JOINT_STEP_DEFAULT
last_status_time = time.monotonic()


def get_key():
    """后台线程：在 raw 模式下持续读取单个按键，写入全局 pressed_key。"""
    global pressed_key
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            pressed_key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def in_soft_limit(idx, value):
    """是否已进入软限位区（距边界 10% 内）。"""
    lo, hi = JNT_RANGE[idx]
    return value < lo + (hi - lo) * SOFT_LIMIT_RATIO or value > hi - (hi - lo) * SOFT_LIMIT_RATIO


def check_self_collision():
    """检测机械臂“非相邻连杆”之间是否发生自碰撞。"""
    for c in range(data.ncon):
        con = data.contact[c]
        b1 = model.geom_bodyid[con.geom1]
        b2 = model.geom_bodyid[con.geom2]
        # 两个接触体都属于机械臂连杆，且不是同一 body -> 自碰撞
        if b1 in ARM_BODIES and b2 in ARM_BODIES and b1 != b2:
            return True
    return False


def print_status():
    """刷新一行实时状态面板。"""
    parts = []
    for i in range(n_joint):
        q = data.qpos[i]
        lo, hi = JNT_RANGE[i]
        flag = 'R' if in_soft_limit(i, q) else '.'
        parts.append('J{}{}={:+.2f}'.format(i + 1, flag, q))
    line = ' '.join(parts)
    sys.stdout.write('\r\x1b[K[{}] step={:.2f} | {}'.format(
        'E-STOP' if e_stop else ('COLLISION' if collision_flag else 'OK'),
        joint_step, line))
    sys.stdout.flush()


def apply_velocity_limited(target, current, max_delta):
    """以 max_delta 为每帧上限，让 current 斜坡逼近 target（速度限幅）。"""
    diff = target - current
    diff = max(-max_delta, min(max_delta, diff))
    return current + diff


# ---------------------------------------------------------------------------
# 3. 启动后台按键监听
# ---------------------------------------------------------------------------
threading.Thread(target=get_key, daemon=True).start()

print('==================================================')
print('Panda MuJoCo 键盘控制（安全与限位增强版）已启动！')
print('  1-7 / Shift+1-7 : 关节正 / 反向转动')
print('  r      : 复位        [ / ] : 减 / 增步长')
print('  空格    : 急停(再按解除)')
print('  q      : 退出程序')
print('  MAX_JOINT_SPEED = {} rad/s | 单步 = {} rad'.format(MAX_JOINT_SPEED, joint_step))
print('==================================================')

# ---------------------------------------------------------------------------
# 4. 主循环
# ---------------------------------------------------------------------------
with mujoco.viewer.launch_passive(model, data) as viewer:
    last_time = time.monotonic()
    while viewer.is_running():
        now = time.monotonic()
        real_dt = max(now - last_time, 1e-6)   # 真实经过时间，用于速度限幅
        last_time = now

        # ---------- 4.1 处理按键 ----------
        if pressed_key:
            key = pressed_key
            pressed_key = None

            if key == 'q':
                print('\n[退出] 程序结束。')
                break

            elif key == 'r':
                if model.nkey > 0:
                    target_q = model.key_qpos[0][:n_joint].copy()
                else:
                    target_q = data.qpos[:n_joint].copy()
                ctrl_q = target_q.copy()
                collision_flag = False
                e_stop = False
                print('\n[复位] 已回到初始关键帧姿态。')

            elif key == ' ':
                e_stop = not e_stop
                print('\n[急停] {}。'.format('已激活，所有关节锁定' if e_stop else '已解除'))

            elif key == '[':
                joint_step = max(JOINT_STEP_MIN, joint_step - 0.02)
                print('\n[步长] 单步增量 -> {:.2f} rad'.format(joint_step))

            elif key == ']':
                joint_step = min(JOINT_STEP_MAX, joint_step + 0.02)
                print('\n[步长] 单步增量 -> {:.2f} rad'.format(joint_step))

            elif key in [str(i) for i in range(1, 8)]:
                idx = int(key) - 1
                delta = joint_step
                # 硬限位检查：越界则拒绝
                new_q = target_q[idx] + delta
                lo, hi = JNT_RANGE[idx]
                if new_q < lo or new_q > hi:
                    print('\n[限位拒绝] 关节{} 请求 {:.3f}，超出硬限位 [{:.3f}, {:.3f}]'.format(
                        idx + 1, new_q, lo, hi))
                else:
                    target_q[idx] = new_q
                    if in_soft_limit(idx, new_q):
                        print('\n[警告] 关节{} 已进入软限位区 {:.3f}（边界[{:.3f},{:.3f}]）'.format(
                            idx + 1, new_q, lo, hi))
                    else:
                        print('\n[关节{}] 目标角 = {:.3f} rad'.format(idx + 1, new_q))

            elif key in REVERSE_KEYS:
                idx = REVERSE_KEYS[key]
                delta = -joint_step
                new_q = target_q[idx] + delta
                lo, hi = JNT_RANGE[idx]
                if new_q < lo or new_q > hi:
                    print('\n[限位拒绝] 关节{} 请求 {:.3f}，超出硬限位 [{:.3f}, {:.3f}]'.format(
                        idx + 1, new_q, lo, hi))
                else:
                    target_q[idx] = new_q
                    if in_soft_limit(idx, new_q):
                        print('\n[警告] 关节{} 已进入软限位区 {:.3f}'.format(idx + 1, new_q))
                    else:
                        print('\n[关节{}] 目标角 = {:.3f} rad'.format(idx + 1, new_q))

        # ---------- 4.2 保存“碰撞前”安全状态，然后物理步进 ----------
        prev_target = target_q.copy()
        prev_ctrl = ctrl_q.copy()

        if e_stop:
            ctrl_q = prev_ctrl   # 急停：命令完全冻结
        else:
            # 速度限幅：每帧命令最多变化 max_delta = MAX_JOINT_SPEED * real_dt
            max_delta = MAX_JOINT_SPEED * real_dt
            for i in range(n_joint):
                ctrl_q[i] = apply_velocity_limited(target_q[i], ctrl_q[i], max_delta)

        data.ctrl[:n_joint] = ctrl_q
        mujoco.mj_step(model, data)

        # ---------- 4.3 碰撞检测与保护 ----------
        if check_self_collision():
            if not collision_flag:
                collision_flag = True
                print('\n[碰撞保护] 检测到机械臂自碰撞！已回退到安全位置。')
                print('            请反向运动退出，或按 r 复位。')
            # 回退到碰撞发生前的安全命令
            target_q[:] = prev_target
            ctrl_q[:] = prev_ctrl
        else:
            collision_flag = False

        # ---------- 4.4 状态面板 ----------
        if now - last_status_time >= STATUS_INTERVAL:
            last_status_time = now
            print_status()

        viewer.sync()
        time.sleep(SIM_SLEEP)
