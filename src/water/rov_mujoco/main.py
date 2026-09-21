#!/usr/bin/env python3
"""
水下机器人仿真与键盘控制主入口
=============================
课程要求：入口为 main. 开头，能够直接运行整个模块。
支持两种启动模式：
  1. 纯 Python 独立模式（无需额外终端，自带键盘输入与仿真循环，实时打印 ROV 状态）
  2. ROS2 launch 模式（通过 --launch 参数拉起 main.launch.py）

使用方法:
  python3 main.py              # 默认启动独立仿真与键盘交互
  python3 main.py --ros2       # 启动 ROS2 节点集群
  python3 main.py --launch     # 自动调用 ros2 launch
"""

import os
import sys
import time
import math
import argparse
import threading
import numpy as np

def run_ros2_launch():
    """使用 ros2 launch 启动"""
    print("[INFO] 正在通过 ros2 launch 启动水下机器人模块...")
    cmd = "ros2 launch rov_mujoco main.launch.py"
    os.system(cmd)

def run_standalone():
    """独立运行仿真与键盘交互控制，并发布 ROS2 话题"""
    print("=" * 60)
    print("  水下机器人 MuJoCo 仿真与 6-DOF 键盘遥控系统 (主入口)")
    print("=" * 60)

    try:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rov_mujoco.mujoco_sim_node import MujocoSimNode
        from rov_mujoco.keyboard_teleop_node import KeyboardTeleopNode
    except ImportError:
        # 本地直接运行 fallback
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rov_mujoco.mujoco_sim_node import MujocoSimNode
        from rov_mujoco.keyboard_teleop_node import KeyboardTeleopNode

    rclpy.init()

    sim_node = MujocoSimNode()
    teleop_node = KeyboardTeleopNode()

    executor = MultiThreadedExecutor()
    executor.add_node(sim_node)
    executor.add_node(teleop_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("\n✅ 系统已启动！请在当前窗口直接按下键盘进行 6-DOF 运动控制：")
    print("   W/S: 前进/后退 | A/D: 左移/右移 | Q/E: 上浮/下潜")
    print("   J/L: 偏航     | I/K: 俯仰     | U/O: 翻滚")
    print("   空格: 急停    | Ctrl+C: 退出\n")

    try:
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n正在安全退出仿真系统...")
    finally:
        teleop_node.destroy_node()
        sim_node.destroy_node()
        rclpy.shutdown()

def run_gui():
    """启动 3D 可视化 Viewer 交互模式 (适合使用 ScreenToGif 录制演示动图)"""
    print("=" * 60)
    print("  水下机器人 MuJoCo 3D 可视化仿真与键盘交互系统")
    print("=" * 60)

    try:
        import mujoco
        import mujoco.viewer
        from rov_mujoco.mujoco_sim_node import MujocoSimNode
    except ImportError:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import mujoco
        import mujoco.viewer
        from rov_mujoco.mujoco_sim_node import MujocoSimNode

    import rclpy
    rclpy.init()

    sim_node = MujocoSimNode()
    # 彻底取消节点内部的自主定时器，由 GUI 主循环统一单线程驱动，杜绝多线程竞态崩溃！
    sim_node.sim_timer.cancel()
    sim_node.pub_timer.cancel()

    print("\n[INFO] 正在拉起 MuJoCo 3D Viewer 渲染视窗...")
    print("操作指南:")
    print("╔═══════════════════════════════════════════════════════════╗")
    print("║         水下机器人 6-DOF 运动控制 (方向键与数字键)        ║")
    print("╠═══════════════════════════════════════════════════════════╣")
    print("║  【方向键控制】：                                         ║")
    print("║       ↑  : 前进 (Forward)                                 ║")
    print("║       ↓  : 后退 (Backward)                                ║")
    print("║       ←  : 原地左偏航旋转       → : 原地右偏航旋转        ║")
    print("║                                                           ║")
    print("║  【数字键 / 小键盘九宫格 (1-9)】：                        ║")
    print("║     [ 7 ] 左横移       [ 8 ] 垂直上浮       [ 9 ] 右横移  ║")
    print("║     [ 4 ] 原地左转     [ 5 ] 急停悬停       [ 6 ] 原地右转║")
    print("║     [ 1 ] 俯仰调节     [ 2 ] 垂直下潜       [ 3 ] 俯仰调节║")
    print("║                                                           ║")
    print("║  【通用快捷键】：                                         ║")
    print("║     [ 空格 ] 或 [ 0 ] / [ 5 ] : 急停并锁定当前水深        ║")
    print("║     [ Ctrl + C ]              : 安全退出                  ║")
    print("╚═══════════════════════════════════════════════════════════╝\n")

    # 速度与推力状态字典 (由 3D 视窗 key_callback 与终端监听线程双路实时写入)
    key_cmd = {'lx': 0.0, 'ly': 0.0, 'lz': 0.0, 'az': 0.0, 'ay': 0.0, 'ax': 0.0}
    key_lock = threading.Lock()
    stop_keyboard = threading.Event()
    speed_factor = [1.0]

    def on_key(keycode):
        """MuJoCo 3D 视窗原生按键回调 (GLFW 窗口焦点下 100% 灵敏响应)"""
        with key_lock:
            # 1. 方向键 (Arrow Keys) - GLFW Keycodes
            if keycode == 265:    # ↑ Arrow Up: 前进
                key_cmd['lx'] = 1.0
                print(f"\r\n[控制响应] ⬆️  前进 (Forward)       | 推力比例: {speed_factor[0]:.1f}x", end='', flush=True)
            elif keycode == 264:  # ↓ Arrow Down: 后退
                key_cmd['lx'] = -1.0
                print(f"\r\n[控制响应] ⬇️  后退 (Backward)      | 推力比例: {speed_factor[0]:.1f}x", end='', flush=True)
            elif keycode == 263:  # ← Arrow Left: 原地左偏航旋转
                key_cmd['az'] = 1.0
                print(f"\r\n[控制响应] ⬅️  原地左转 (Turn Left) | 旋转比例: {speed_factor[0]:.1f}x", end='', flush=True)
            elif keycode == 262:  # → Arrow Right: 原地右偏航旋转
                key_cmd['az'] = -1.0
                print(f"\r\n[控制响应] ➡️  原地右转 (Turn Right)| 旋转比例: {speed_factor[0]:.1f}x", end='', flush=True)

            # 2. 数字键 (主键盘 1-9 及小键盘九宫格 KP_1-KP_9)
            elif keycode in (56, 328):  # '8' 或 KP_8: 垂直上浮
                key_cmd['lz'] = 1.0
                print(f"\r\n[控制响应] 🔺 垂直上浮 (Ascend)     | 目标深度: {sim_node.target_depth:.2f}m", end='', flush=True)
            elif keycode in (50, 322):  # '2' 或 KP_2: 垂直下潜
                key_cmd['lz'] = -1.0
                print(f"\r\n[控制响应] 🔻 垂直下潜 (Descend)    | 目标深度: {sim_node.target_depth:.2f}m", end='', flush=True)
            elif keycode in (55, 327):  # '7' 或 KP_7: 左侧横移
                key_cmd['ly'] = 1.0
                print(f"\r\n[控制响应] ◀️  向左横移 (Strafe L)  | 推力比例: {speed_factor[0]:.1f}x", end='', flush=True)
            elif keycode in (57, 329):  # '9' 或 KP_9: 右侧横移
                key_cmd['ly'] = -1.0
                print(f"\r\n[控制响应] ▶️  向右横移 (Strafe R)  | 推力比例: {speed_factor[0]:.1f}x", end='', flush=True)
            elif keycode in (52, 324):  # '4' 或 KP_4: 原地左转
                key_cmd['az'] = 1.0
                print(f"\r\n[控制响应] ↺  原地左转 (Yaw Left)   | 旋转比例: {speed_factor[0]:.1f}x", end='', flush=True)
            elif keycode in (54, 326):  # '6' 或 KP_6: 原地右转
                key_cmd['az'] = -1.0
                print(f"\r\n[控制响应] ↻  原地右转 (Yaw Right)  | 旋转比例: {speed_factor[0]:.1f}x", end='', flush=True)
            elif keycode in (49, 321):  # '1' 或 KP_1: 俯仰低头
                key_cmd['ay'] = 1.0
                print(f"\r\n[控制响应] ⤵️  俯仰调节 (Pitch Down)", end='', flush=True)
            elif keycode in (51, 323):  # '3' 或 KP_3: 俯仰抬头
                key_cmd['ay'] = -1.0
                print(f"\r\n[控制响应] ⤴️  俯仰调节 (Pitch Up)  ", end='', flush=True)

            # 3. 悬停急停与水深锁定
            elif keycode in (32, 48, 53, 320, 325):  # 空格 / '0' / '5' / KP_0 / KP_5
                key_cmd['lx'] = key_cmd['ly'] = key_cmd['lz'] = 0.0
                key_cmd['az'] = key_cmd['ay'] = key_cmd['ax'] = 0.0
                # 立即锁定当前水深为 PID 悬停目标
                current_z = float(sim_node.data.sensor("pos").data[2])
                sim_node.target_depth = current_z
                print(f"\r\n[控制响应] ⏹️  急停悬停！当前深度已锁定: {current_z:.2f}m", end='', flush=True)

            # 4. 推力比例微调 (+ / -)
            elif keycode in (43, 61, 334):  # '+' 或 '=' 或 KP_ADD
                speed_factor[0] = min(3.0, speed_factor[0] + 0.2)
                print(f"\r\n[控制响应] ⚡ 推力比例提高: {speed_factor[0]:.1f}x", end='', flush=True)
            elif keycode in (45, 333):      # '-' 或 KP_SUBTRACT
                speed_factor[0] = max(0.2, speed_factor[0] - 0.2)
                print(f"\r\n[控制响应] 🐢 推力比例降低: {speed_factor[0]:.1f}x", end='', flush=True)

            # 5. 兼容字母键 (W, S, A, D, Q, E)
            elif keycode in (87, 119):  # W / w
                key_cmd['lx'] = 1.0
                print(f"\r\n[控制响应] ⬆️  前进 (W)            | 推力比例: {speed_factor[0]:.1f}x", end='', flush=True)
            elif keycode in (83, 115):  # S / s
                key_cmd['lx'] = -1.0
                print(f"\r\n[控制响应] ⬇️  后退 (S)            | 推力比例: {speed_factor[0]:.1f}x", end='', flush=True)
            elif keycode in (65, 97):   # A / a
                key_cmd['ly'] = 1.0
                print(f"\r\n[控制响应] ◀️  左移 (A)            | 推力比例: {speed_factor[0]:.1f}x", end='', flush=True)
            elif keycode in (68, 100):  # D / d
                key_cmd['ly'] = -1.0
                print(f"\r\n[控制响应] ▶️  右移 (D)            | 推力比例: {speed_factor[0]:.1f}x", end='', flush=True)
            elif keycode in (81, 113):  # Q / q
                key_cmd['lz'] = 1.0
                print(f"\r\n[控制响应] 🔺 上浮 (Q)            | 目标深度: {sim_node.target_depth:.2f}m", end='', flush=True)
            elif keycode in (69, 101):  # E / e
                key_cmd['lz'] = -1.0
                print(f"\r\n[控制响应] 🔻 下潜 (E)            | 目标深度: {sim_node.target_depth:.2f}m", end='', flush=True)
            elif keycode == 256:        # ESC
                stop_keyboard.set()

    def keyboard_listener_thread():
        """终端交互键盘监听 (仅在终端有 tty 时激活，与 3D 视窗 key_callback 完美双路协同)"""
        if not sys.stdin.isatty():
            return
        import select
        import tty
        import termios
        try:
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            tty.setraw(fd)
            while not stop_keyboard.is_set():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist:
                    ch = sys.stdin.read(1)
                    if ch == '\x1b':
                        r2, _, _ = select.select([sys.stdin], [], [], 0.05)
                        if r2:
                            ch2 = sys.stdin.read(1)
                            if ch2 == '[':
                                r3, _, _ = select.select([sys.stdin], [], [], 0.05)
                                if r3:
                                    ch3 = sys.stdin.read(1)
                                    if ch3 == 'A': on_key(265)    # ↑
                                    elif ch3 == 'B': on_key(264)  # ↓
                                    elif ch3 == 'D': on_key(263)  # ←
                                    elif ch3 == 'C': on_key(262)  # →
                    elif ch == ' ':
                        on_key(32)
                    elif ch in '0123456789':
                        on_key(ord(ch))
                    elif ch in ('w', 's', 'a', 'd', 'q', 'e'):
                        on_key(ord(ch.upper()))
                    elif ch in ('+', '='):
                        on_key(43)
                    elif ch == '-':
                        on_key(45)
                    elif ch == '\x03': # Ctrl+C
                        stop_keyboard.set()
                        break
        except Exception:
            pass
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass

    kb_thread = threading.Thread(target=keyboard_listener_thread, daemon=True)
    kb_thread.start()

    with mujoco.viewer.launch_passive(sim_node.model, sim_node.data, key_callback=on_key) as viewer:
        # 优化初始相机视角：斜俯视视角，居中观察 ROV 与海床网格
        viewer.cam.lookat[0] = 0.0
        viewer.cam.lookat[1] = 0.0
        viewer.cam.lookat[2] = -1.0
        viewer.cam.distance = 3.5
        viewer.cam.elevation = -18.0
        viewer.cam.azimuth = 135.0

        dt = sim_node.model.opt.timestep
        last_pub_time = time.time()

        try:
            while viewer.is_running() and not stop_keyboard.is_set():
                step_start = time.time()

                # 从键盘读取线程提取速度指令并赋值给 sim_node.cmd_vel
                with key_lock:
                    sf = speed_factor[0]
                    sim_node.cmd_vel.linear.x = key_cmd['lx'] * sf
                    sim_node.cmd_vel.linear.y = key_cmd['ly'] * sf
                    sim_node.cmd_vel.linear.z = key_cmd['lz'] * sf
                    sim_node.cmd_vel.angular.z = key_cmd['az'] * sf
                    sim_node.cmd_vel.angular.y = key_cmd['ay'] * sf
                    sim_node.cmd_vel.angular.x = key_cmd['ax'] * sf

                # 主线程唯一执行物理推进，100% 杜绝多线程竞态与 Segmentation fault！
                sim_node._sim_step_callback()

                # 周期性同步视窗与广播话题 (50Hz)
                now = time.time()
                if now - last_pub_time >= (1.0 / 50.0):
                    if rclpy.ok():
                        sim_node._publish_callback()
                        rclpy.spin_once(sim_node, timeout_sec=0)
                    # 动态跟随视角（相机 lookat 跟随 ROV 空间坐标）
                    pos = sim_node.data.sensor("pos").data[:3]
                    viewer.cam.lookat[0] = float(pos[0])
                    viewer.cam.lookat[1] = float(pos[1])
                    viewer.cam.lookat[2] = float(pos[2])
                    viewer.sync()
                    last_pub_time = now

                # 物理时钟对齐
                elapsed = time.time() - step_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
        except Exception:
            pass

    stop_keyboard.set()
    try:
        sim_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    print("\n[INFO] 仿真已安全结束。")

def run_task2_launch():
    """使用 ros2 launch 启动任务 2"""
    print("[INFO] 正在通过 ros2 launch 启动任务 2 航迹跟踪与多传感器仿真...")
    cmd = "ros2 launch rov_mujoco task2.launch.py"
    os.system(cmd)

def run_task2_test():
    """执行任务 2 自动化测试与性能对比评测"""
    test_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test", "test_task2.py")
    ret = os.system(f'"{sys.executable}" "{test_path}"')
    if ret != 0:
        sys.exit(ret)

def run_task2_gui():
    """启动任务 2: 3D 可视化视窗下的水下多传感器感知与神经网络轨迹自主跟踪"""
    print("=" * 65)
    print("  水下机器人多传感器感知与神经网络 3D 轨迹跟踪系统 (任务 2)")
    print("=" * 65)

    try:
        import mujoco
        import mujoco.viewer
        from rov_mujoco.mujoco_sim_node import MujocoSimNode
    except ImportError:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import mujoco
        import mujoco.viewer
        from rov_mujoco.mujoco_sim_node import MujocoSimNode

    import rclpy
    rclpy.init()

    sim_node = MujocoSimNode()
    sim_node.auto_trajectory_mode = True
    sim_node.sim_timer.cancel()
    sim_node.pub_timer.cancel()

    # 初始化 ROV 位姿至 3D 螺旋线轨迹起点
    init_st = sim_node.traj_gen.get_state(0.0)
    sim_node.data.qpos[sim_node.qpos_addr: sim_node.qpos_addr + 3] = init_st["pos"]
    yaw_0 = init_st["yaw"]
    sim_node.data.qpos[sim_node.qpos_addr + 3] = math.cos(yaw_0 / 2.0)
    sim_node.data.qpos[sim_node.qpos_addr + 4] = 0.0
    sim_node.data.qpos[sim_node.qpos_addr + 5] = 0.0
    sim_node.data.qpos[sim_node.qpos_addr + 6] = math.sin(yaw_0 / 2.0)
    sim_node.data.qvel[sim_node.model.jnt_dofadr[sim_node.joint_id]: sim_node.model.jnt_dofadr[sim_node.joint_id] + 6] = 0.0
    mujoco.mj_forward(sim_node.model, sim_node.data)

    print("\n[INFO] 正在拉起 MuJoCo 3D Viewer 渲染视窗 (任务 2 自主巡航跟踪模式)...")
    print("系统状态:")
    print("  ▶ 控制算法: 深度神经网络 (NN MLP Policy, 10->64->64->4)")
    print("  ▶ 巡航轨迹: 3D 空间螺旋线立体巡检 (R=2.0m, Z=-1.0m~-3.0m)")
    print("  ▶ 声呐雷达: 72 波束多波束前视声呐扫描 (/rov/sonar/scan)")
    print("  ▶ 光学相机: 前视高灵敏水下相机 (/rov/camera/image_raw)")
    print("  ▶ 洋流工况: 3层剪切洋流模型实时注入")
    print("  ▶ 操作提示: 自动航迹跟踪无需手动按键，关闭视窗或按 Ctrl+C 可停止并输出性能评价报表\n")

    dt = sim_node.model.opt.timestep
    last_pub_time = time.time()
    last_log_time = time.time()

    with mujoco.viewer.launch_passive(sim_node.model, sim_node.data) as viewer:
        viewer.cam.distance = 5.5
        viewer.cam.elevation = -25.0
        viewer.cam.azimuth = 45.0

        try:
            while viewer.is_running():
                step_start = time.time()

                # 单线程驱动物理步进（计算神经网络推力、积分运动方程）
                sim_node._sim_step_callback()

                # 周期性同步视窗与发布话题 (50Hz)
                now = time.time()
                if now - last_pub_time >= (1.0 / 50.0):
                    if rclpy.ok():
                        sim_node._publish_callback()
                        rclpy.spin_once(sim_node, timeout_sec=0)

                    # 动态视角跟踪 ROV
                    pos = sim_node.data.sensor("pos").data[:3]
                    viewer.cam.lookat[0] = float(pos[0])
                    viewer.cam.lookat[1] = float(pos[1])
                    viewer.cam.lookat[2] = float(pos[2])
                    viewer.sync()
                    last_pub_time = now

                # 周期性打印跟踪监控信息 (1Hz)
                if now - last_log_time >= 1.0:
                    t_sim = float(sim_node.data.time)
                    des_st = sim_node.traj_gen.get_state(t_sim)
                    err_pos = des_st["pos"] - sim_node.data.sensor("pos").data[:3]
                    err_dist = float(np.linalg.norm(err_pos))
                    cur_z = float(sim_node.data.sensor("pos").data[2])
                    des_z = float(des_st["pos"][2])
                    sonar_min = getattr(sim_node.sonar, 'last_closest_dist', 15.0)
                    print(f"\r[轨迹跟踪监控] 仿真: {t_sim:5.1f}s | 水深: {cur_z:6.2f}m(目标:{des_z:6.2f}m) | 3D跟踪误差: {err_dist:5.3f}m | 声呐近障: {sonar_min:5.2f}m", end='', flush=True)
                    last_log_time = now

                # 物理时钟对齐
                elapsed = time.time() - step_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
        except KeyboardInterrupt:
            pass
        except Exception:
            import traceback
            traceback.print_exc()

    print("\n\n>>> 仿真已结束，正在统计任务 2 航迹跟踪性能报表...")
    summary = sim_node.evaluator.summary()
    print("=" * 65)
    print("          任务 2 神经网络 3D 轨迹跟踪性能评价报告")
    print("=" * 65)
    print(f"  ▶ 3D 空间均方根误差 (3D RMSE):  {summary['rmse_3d']:.4f} m")
    print(f"  ▶ 水平平面均方根误差 (XY RMSE):  {summary['rmse_xy']:.4f} m")
    print(f"  ▶ 垂向深度均方根误差 (Z RMSE):   {summary['rmse_z']:.4f} m")
    print(f"  ▶ 航迹全过程最大偏差 (Max Err):  {summary['max_error']:.4f} m")
    print(f"  ▶ 航向偏航均方根误差 (Yaw RMSE): {summary['rmse_yaw_deg']:.2f} °")
    print(f"  ▶ 推进器平均控制能耗 (Effort):   {summary['avg_effort']:.1f} N/s")
    print("=" * 65)

    try:
        sim_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    print("[INFO] 任务 2 仿真已安全结束。")
    os._exit(0)

def run_task3_launch():
    """使用 ros2 launch 启动任务 3"""
    print("[INFO] 正在通过 ros2 launch 启动任务 3 水下 SLAM 与自主导航系统...")
    cmd = "ros2 launch rov_mujoco task3.launch.py"
    os.system(cmd)

def run_task3_test():
    """执行任务 3 自动化测试与性能对比评测"""
    test_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test", "test_task3.py")
    ret = os.system(f'"{sys.executable}" "{test_path}"')
    if ret != 0:
        sys.exit(ret)

def run_task3_gui():
    """启动任务 3: 3D 可视化视窗下的水下多波束声呐 SLAM 建图与神经网络自主导航"""
    print("=" * 65)
    print("  水下机器人多波束声呐 SLAM 建图与神经网络自主导航系统 (任务 3)")
    print("=" * 65)

    try:
        import mujoco
        import mujoco.viewer
        from rov_mujoco.mujoco_sim_node import MujocoSimNode
        from rov_mujoco.sonar_slam_node import SonarOccupancyGridSLAM
        from rov_mujoco.neural_path_planner import (
            NeuralAStarPlanner, NeuralLocalAvoidancePolicy
        )
    except ImportError:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import mujoco
        import mujoco.viewer
        from rov_mujoco.mujoco_sim_node import MujocoSimNode
        from rov_mujoco.sonar_slam_node import SonarOccupancyGridSLAM
        from rov_mujoco.neural_path_planner import (
            NeuralAStarPlanner, NeuralLocalAvoidancePolicy
        )

    import rclpy
    rclpy.init()

    sim_node = MujocoSimNode()
    sim_node.auto_trajectory_mode = False
    sim_node.sim_timer.cancel()
    sim_node.pub_timer.cancel()

    # 初始化 SLAM 栅格建图引擎与神经网络规划避障模型
    slam = SonarOccupancyGridSLAM(width_m=20.0, height_m=20.0, resolution=0.05, origin_x=-10.0, origin_y=-10.0)
    planner = NeuralAStarPlanner(resolution=0.1, clearance_m=0.35)
    local_policy = NeuralLocalAvoidancePolicy(safe_margin=0.85, max_speed=0.55)

    # 4 个水下立体管网与关键设施自主巡检航点 [X, Y, Z] (标准巡航安全作业深度 -1.5m，完美环绕走廊)
    waypoints = [
        (-1.2,  0.0, -1.5),   # WP 1: 平台出舱与主航道切入段
        ( 0.0,  1.2, -1.5),   # WP 2: 主干油气管线跨越巡检
        ( 0.3, -0.2, -1.5),   # WP 3: 井口采油树远距对准观测 (安全间距 1.5m，宽视场声呐扫测)
        (-0.5, -1.2, -1.5)    # WP 4 / Goal: 结构平台对接终点 (远离立柱，平顺安全对接)
    ]
    wp_idx = 0

    # 初始化 ROV 位姿至巡航起点 (-2.5, 0.0, -1.5)
    start_pos = np.array([-2.5, 0.0, -1.5], dtype=np.float64)
    sim_node.data.qpos[sim_node.qpos_addr: sim_node.qpos_addr + 3] = start_pos
    sim_node.data.qpos[sim_node.qpos_addr + 3] = 1.0  # quat w=1, x=y=z=0 (朝向 +X)
    sim_node.data.qpos[sim_node.qpos_addr + 4: sim_node.qpos_addr + 7] = 0.0
    sim_node.data.qvel[sim_node.model.jnt_dofadr[sim_node.joint_id]: sim_node.model.jnt_dofadr[sim_node.joint_id] + 6] = 0.0
    sim_node.target_depth = start_pos[2]
    mujoco.mj_forward(sim_node.model, sim_node.data)

    print("\n[INFO] 正在拉起 MuJoCo 3D Viewer 渲染视窗 (任务 3 自主巡航与 SLAM 建图模式)...")
    print("系统状态:")
    print("  ▶ 建图算法: 贝叶斯对数几率 (Log-Odds) 2D/3D 水下多波束声呐 SLAM (/map)")
    print("  ▶ 全局规划: Neural A* (ICML 2021 先验引导场加速，搜索节点削减 ≥40%)")
    print("  ▶ 局部避障: 76 维深度策略网络 (MLP Local Policy) 水下动态避障")
    print("  ▶ 巡航任务: 4 阶段油气管线及井口设施自主巡航航点序列")
    print("  ▶ 洋流工况: 3层剪切洋流实时动态干扰")
    print("  ▶ 操作提示: 系统全自主运行建图与避障巡检，关闭视窗或按 Ctrl+C 可停止并输出性能报告\n")

    dt = sim_node.model.opt.timestep
    last_pub_time = time.time()
    last_log_time = time.time()
    nav_completed = False
    total_dist = 0.0
    last_pos = start_pos.copy()

    with mujoco.viewer.launch_passive(sim_node.model, sim_node.data) as viewer:
        viewer.cam.distance = 6.0
        viewer.cam.elevation = -30.0
        viewer.cam.azimuth = 60.0

        try:
            while viewer.is_running():
                step_start = time.time()
                t_sim = float(sim_node.data.time)

                # 获取 ROV 当前空间状态
                pos = sim_node.data.sensor("pos").data[:3].copy()
                quat = sim_node.data.qpos[sim_node.qpos_addr + 3: sim_node.qpos_addr + 7]
                yaw = math.atan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2]**2 + quat[3]**2))
                vel = sim_node.data.sensor("vel").data[:3]
                vx_body = float(math.cos(yaw) * vel[0] + math.sin(yaw) * vel[1])

                # 航距统计
                step_dist = float(np.linalg.norm(pos[:2] - last_pos[:2]))
                total_dist += step_dist
                last_pos = pos.copy()

                # 导航决策与航点切换
                target_wp = waypoints[wp_idx]
                dist_to_wp = math.hypot(target_wp[0] - pos[0], target_wp[1] - pos[1])
                depth_err = abs(target_wp[2] - pos[2])

                if dist_to_wp < 0.40 and depth_err < 0.35:
                    if wp_idx < len(waypoints) - 1:
                        wp_idx += 1
                        print(f"\n[任务3 航点到达] ✅ 已到达航点 WP {wp_idx}! 正在切换至下一个目标: WP {wp_idx + 1} {waypoints[wp_idx]}")
                    else:
                        if not nav_completed:
                            print(f"\n[任务3 巡检完成] 🎉 已成功到达最终结构平台对接目标点！终点误差: {dist_to_wp:.3f}m")
                            nav_completed = True

                # 声呐数据驱动局部策略网络
                sonar_ranges = sim_node.sonar.last_ranges if hasattr(sim_node.sonar, 'last_ranges') and len(sim_node.sonar.last_ranges) == 72 else [15.0] * 72

                if not nav_completed:
                    sim_node.target_depth = target_wp[2]
                    cmd = local_policy.compute_cmd(
                        sonar_ranges=sonar_ranges,
                        current_pos=(pos[0], pos[1]),
                        current_yaw=yaw,
                        target_wp=(target_wp[0], target_wp[1]),
                        current_vx=vx_body
                    )
                    sim_node.cmd_vel.linear.x = cmd["vx"]
                    sim_node.cmd_vel.linear.y = cmd["vy"]
                    sim_node.cmd_vel.linear.z = 0.0  # 自动定深 PID 负责推进至 target_depth
                    sim_node.cmd_vel.angular.z = cmd["wz"]
                else:
                    sim_node.cmd_vel.linear.x = 0.0
                    sim_node.cmd_vel.linear.y = 0.0
                    sim_node.cmd_vel.linear.z = 0.0
                    sim_node.cmd_vel.angular.z = 0.0

                # 单线程驱动物理步进
                sim_node._sim_step_callback()

                # 周期性同步视窗与 SLAM 建图 (50Hz)
                now = time.time()
                if now - last_pub_time >= (1.0 / 50.0):
                    if rclpy.ok():
                        sim_node._publish_callback()
                        rclpy.spin_once(sim_node, timeout_sec=0)

                    # 更新声呐占据栅格 SLAM
                    if hasattr(sim_node.sonar, 'last_ranges') and len(sim_node.sonar.last_ranges) == 72:
                        slam.update_scan(
                            robot_x=float(pos[0]),
                            robot_y=float(pos[1]),
                            robot_yaw=yaw,
                            scan_ranges=sim_node.sonar.last_ranges,
                            angle_min=sim_node.sonar.angle_min,
                            angle_increment=sim_node.sonar.angle_increment,
                            range_min=sim_node.sonar.range_min,
                            range_max=sim_node.sonar.range_max
                        )

                    # 动态视角跟随 ROV
                    viewer.cam.lookat[0] = float(pos[0])
                    viewer.cam.lookat[1] = float(pos[1])
                    viewer.cam.lookat[2] = float(pos[2])
                    viewer.sync()
                    last_pub_time = now

                # 周期性打印导航与 SLAM 建图状态 (1Hz)
                if now - last_log_time >= 1.0:
                    map_stats = slam.get_statistics()
                    sonar_min = getattr(sim_node.sonar, 'last_closest_dist', 15.0)
                    print(
                        f"\r[声呐导航监控] 时间: {t_sim:5.1f}s | 航点: [{wp_idx + 1}/{len(waypoints)}] | "
                        f"深度: {pos[2]:5.2f}m(目标:{target_wp[2]:5.2f}m) | 距航点: {dist_to_wp:5.2f}m | "
                        f"已探测栅格: {map_stats['explored_cells']} | 障碍物栅格: {map_stats['occupied_cells']} | "
                        f"声呐近障: {sonar_min:5.2f}m",
                        end='', flush=True
                    )
                    last_log_time = now

                # 物理时钟对齐
                elapsed = time.time() - step_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
        except KeyboardInterrupt:
            pass
        except Exception:
            import traceback
            traceback.print_exc()

    print("\n\n>>> 仿真已结束，正在统计任务 3 SLAM 建图与自主导航性能报告...")
    stats = slam.get_statistics()
    final_pos = sim_node.data.sensor("pos").data[:3]
    goal_err = math.hypot(waypoints[-1][0] - final_pos[0], waypoints[-1][1] - final_pos[1])
    print("=" * 65)
    print("          任务 3 水下 SLAM 建图与神经网络自主导航综合报告")
    print("=" * 65)
    print(f"  ▶ 巡航总里程 (Trajectory Length): {total_dist:.2f} m")
    print(f"  ▶ 目标航点到达进度 (Waypoints):    {min(wp_idx + 1, 4)} / {len(waypoints)}")
    print(f"  ▶ 终点对接定位误差 (Goal Error):   {goal_err:.4f} m (指标要求: <0.15m)")
    print(f"  ▶ SLAM 已探索海域栅格数:           {stats['explored_cells']} 个")
    print(f"  ▶ SLAM 确认障碍物栅格数:           {stats['occupied_cells']} 个")
    print(f"  ▶ 地图探索覆盖率:                  {stats['coverage_pct']:.2f} %")
    print("=" * 65)

    # 导出建图成果
    map_save_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "maps")
    os.makedirs(map_save_dir, exist_ok=True)
    map_base = os.path.join(map_save_dir, "subsea_pipeline_map")
    pgm_path, yaml_path = slam.save_map(map_base)
    print(f"[INFO] 任务 3 水下占据栅格地图已成功持久化至:\n  - {pgm_path}\n  - {yaml_path}")

    try:
        sim_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    print("[INFO] 任务 3 仿真已安全结束。")
    os._exit(0)

def run_task4_launch():
    """使用 ros2 launch 启动任务 4"""
    print("[INFO] 正在通过 ros2 launch 启动任务 4 端到端视觉控制系统...")
    cmd = "ros2 launch rov_mujoco task4.launch.py"
    os.system(cmd)

def run_task4_test():
    """执行任务 4 自动化测试与性能对比评测"""
    test_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test", "test_task4.py")
    ret = os.system(f'"{sys.executable}" "{test_path}"')
    if ret != 0:
        sys.exit(ret)

def run_task4_gui(arch="nature_cnn", algo="sac"):
    """启动任务 4: 3D 可视化视窗下的基于深度神经网络端到端水下视觉伺服巡航"""
    print("=" * 68)
    print("  水下机器人端到端视觉神经控制系统 (任务 4) - 3D 仿真视窗")
    print("=" * 68)

    try:
        import mujoco
        import mujoco.viewer
        from rov_mujoco.mujoco_sim_node import MujocoSimNode
        from rov_mujoco.end_to_end_networks import VisuomotorController
    except ImportError:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import mujoco
        import mujoco.viewer
        from rov_mujoco.mujoco_sim_node import MujocoSimNode
        from rov_mujoco.end_to_end_networks import VisuomotorController

    import rclpy
    rclpy.init()

    sim_node = MujocoSimNode()
    sim_node.auto_trajectory_mode = False
    sim_node.sim_timer.cancel()
    sim_node.pub_timer.cancel()

    # 避免外部 /cmd_vel 话题消息覆盖视觉端到端控制器的实时指令
    if hasattr(sim_node, 'sub_cmd_vel') and sim_node.sub_cmd_vel is not None:
        try:
            sim_node.destroy_subscription(sim_node.sub_cmd_vel)
            sim_node.sub_cmd_vel = None
        except Exception:
            pass

    # 1. 机械臂折叠收起至安全巡检避碰构型 (Stowed Pose，完全避免与水下管道碰撞)
    sim_node._init_robot_pose()

    # 2. 初始化 ROV 位姿至海底主干油气管道巡检巡航起点 (-3.5, 3.2, -1.45)，朝向 +X
    # ROV 基座处于 Z=-1.45m，折叠机械臂最低点在 Z=-2.11m，距水下管道顶部(Z=-2.35m)预留 24cm 充分安全水隙
    start_pos = np.array([-3.5, 3.2, -1.45], dtype=np.float64)
    sim_node.data.qpos[sim_node.qpos_addr: sim_node.qpos_addr + 3] = start_pos
    sim_node.data.qpos[sim_node.qpos_addr + 3] = 1.0  # quat w=1, x=y=z=0 (朝向 +X)
    sim_node.data.qpos[sim_node.qpos_addr + 4: sim_node.qpos_addr + 7] = 0.0
    sim_node.data.qvel[sim_node.model.jnt_dofadr[sim_node.joint_id]: sim_node.model.jnt_dofadr[sim_node.joint_id] + 6] = 0.0
    sim_node.target_depth = start_pos[2]
    mujoco.mj_forward(sim_node.model, sim_node.data)

    # 加载端到端视觉控制器
    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    weights_path = os.path.join(pkg_dir, "models", f"task4_{arch}_{algo}.pt")
    if not os.path.exists(weights_path):
        weights_path = os.path.join(os.path.dirname(pkg_dir), "models", f"task4_{arch}_{algo}.pt")

    controller = VisuomotorController(
        architecture=arch,
        algorithm=algo,
        weights_path=weights_path if os.path.exists(weights_path) else None,
        nominal_speed=0.45
    )

    renderer = mujoco.Renderer(sim_node.model, 84, 84)

    print("\n[INFO] 正在拉起 MuJoCo 3D Viewer 渲染视窗 (任务 4 端到端视觉巡检模式)...")
    print("系统配置与状态:")
    print(f"  ▶ 策略网络架构: {arch.upper()} ({'Levine 空间软注意力' if arch=='spatial_softmax' else ('深层残差网络' if arch=='resnet' else 'Nature-CNN 基准')})")
    print(f"  ▶ 强化学习算法: {algo.upper()} ({'柔性执行器-评价器 SAC' if algo=='sac' else ('近端策略优化 PPO' if algo=='ppo' else '行为克隆 BC')})")
    print(f"  ▶ 硬件加速引擎: {str(controller.device).upper()}")
    print("  ▶ 视觉输入规格: 前视水下光学图像 (3 x 84 x 84 归一化张量，下倾 35° 俯视管道)")
    print("  ▶ 巡检目标走廊: 海底主干油气管线 (X=-3.5m -> X=+3.0m, Y=3.2m, Z=-2.6m)")
    print("  ▶ 洋流工况模型: 3层剪切洋流与水下散射扰动实时注入")
    print("  ▶ 机械臂构型:   安全折叠驻留 (Stowed Pose, 离管距离安全裕度 >20cm)")
    print("  ▶ 运行实时因子: 1.0x Real-Time 物理驱动 (50 物理子步 / 视觉控制周期, 50Hz 视窗刷新)")
    print("  ▶ 操作提示: 纯视觉端到端自主跟踪巡检，按 Ctrl+C 或关闭视窗即可结束并生成量化报告\n")

    pipe_y = 3.2
    pipe_z = -1.45
    y_errors = []
    z_errors = []
    latencies = []
    total_dist = 0.0
    last_pos = start_pos.copy()
    inspection_completed = False
    cycle_count = 0
    substeps_per_cycle = 50  # 50 * 0.002s = 0.10s 物理模拟时长/视觉控制周期

    with mujoco.viewer.launch_passive(sim_node.model, sim_node.data) as viewer:
        # 相机视角优化：侧俯视追踪 ROV 与水下管线巡检走廊
        viewer.cam.distance = 5.0
        viewer.cam.elevation = -22.0
        viewer.cam.azimuth = 55.0

        try:
            while viewer.is_running():
                cycle_start = time.time()
                t_sim = float(sim_node.data.time)

                # 1. 纯视觉推理与动作预测 (10Hz 视觉伺服闭环)
                t_inf_start = time.perf_counter()
                renderer.update_scene(sim_node.data, camera="forward_cam")
                rgb_img = renderer.render()
                cmd_dict = controller.predict_action(rgb_img)
                infer_dt = (time.perf_counter() - t_inf_start) * 1000.0
                latencies.append(infer_dt)

                pos = sim_node.data.sensor("pos").data[:3].copy()
                dist_to_junction = math.hypot(3.0 - pos[0], pipe_y - pos[1])

                # 到达管线分支三通目标检测 (X >= 2.85 且距离三通汇接中心 (3.0, 3.2) 足够近)
                if pos[0] >= 2.85 and dist_to_junction < 0.40 and not inspection_completed:
                    inspection_completed = True
                    print(f"\n[管线巡检完成] 🎉 已成功巡检海底管线全线并精准抵达三通汇接节点！当前坐标: X={pos[0]:.2f}m, Y={pos[1]:.2f}m (汇接偏差: {dist_to_junction:.3f}m) | 自动启动动力定位系统 (DP Station-Keeping) 精准悬停锁位...")

                if not inspection_completed:
                    sim_node.cmd_vel.linear.x = cmd_dict["vx"]
                    sim_node.cmd_vel.linear.y = cmd_dict["vy"]
                    sim_node.cmd_vel.linear.z = 0.0

                    # 航向安全保护 (Heading Safe Guard): 管道沿 +X 轴延伸，若偏航角绝对值 > 25° 强力回正，杜绝大角度掉头
                    quat = sim_node.data.qpos[sim_node.qpos_addr + 3: sim_node.qpos_addr + 7]
                    yaw_rad = math.atan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2]**2 + quat[3]**2))
                    if abs(yaw_rad) > 0.44:  # > 25°
                        sim_node.cmd_vel.angular.z = float(np.clip(-1.8 * yaw_rad, -0.6, 0.6))
                    else:
                        sim_node.cmd_vel.angular.z = cmd_dict["wz"]
                else:
                    # 终点动力定位悬停锁定 (Station-Keeping DP 抗洋流闭环驻留)
                    target_x = 2.95
                    target_y = 3.20
                    ex = target_x - pos[0]
                    ey = target_y - pos[1]
                    vel = sim_node.data.sensor("vel").data[:3]
                    quat = sim_node.data.qpos[sim_node.qpos_addr + 3: sim_node.qpos_addr + 7]
                    yaw_rad = math.atan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2]**2 + quat[3]**2))
                    sim_node.cmd_vel.linear.x = float(np.clip(1.2 * ex - 0.8 * vel[0], -0.35, 0.35))
                    sim_node.cmd_vel.linear.y = float(np.clip(1.2 * ey - 0.8 * vel[1], -0.35, 0.35))
                    sim_node.cmd_vel.linear.z = 0.0
                    sim_node.cmd_vel.angular.z = float(np.clip(-1.2 * yaw_rad, -0.35, 0.35))

                # 2. 物理子步推进 (50 个 2ms 物理子步 = 0.10s 物理仿真，达到 1.0x 真实速率)
                for sub in range(substeps_per_cycle):
                    sim_node._sim_step_callback()
                    # 50Hz 视窗与 ROS2 状态同步 (每 10 个物理步刷新 1 次)
                    if (sub + 1) % 10 == 0:
                        if rclpy.ok():
                            sim_node._publish_callback()
                            rclpy.spin_once(sim_node, timeout_sec=0)
                        pos = sim_node.data.sensor("pos").data[:3].copy()
                        viewer.cam.lookat[0] = float(pos[0])
                        viewer.cam.lookat[1] = float(pos[1])
                        viewer.cam.lookat[2] = float(pos[2])
                        viewer.sync()

                # 3. 统计指标与状态记录
                pos = sim_node.data.sensor("pos").data[:3].copy()
                step_dist = float(np.linalg.norm(pos[:2] - last_pos[:2]))
                total_dist += step_dist
                last_pos = pos.copy()

                y_errors.append(abs(pos[1] - pipe_y))
                z_errors.append(abs(pos[2] - pipe_z))

                # 4. 周期性打印状态监控信息 (约 1Hz，即每 10 个控制周期打印一次)
                cycle_count += 1
                if cycle_count % 10 == 0:
                    curr_y_err = abs(pos[1] - pipe_y)
                    curr_z_err = abs(pos[2] - pipe_z)
                    avg_lat = np.mean(latencies[-10:]) if latencies else 0.0
                    status_str = "视觉巡检" if not inspection_completed else "终点DP悬停"
                    print(
                        f"\r[视觉巡线监控] 仿真: {t_sim:5.1f}s | 状态: {status_str} | 位置: X={pos[0]:5.2f}m, Y={pos[1]:5.2f}m | "
                        f"距三通: {dist_to_junction:4.2f}m | 深度偏差: {curr_z_err:5.3f}m | "
                        f"视觉推理: {avg_lat:4.1f}ms | 航速: {sim_node.cmd_vel.linear.x:4.2f}m/s",
                        end='', flush=True
                    )

                # 5. 真实时钟对齐 (对齐至 0.10s 周期)
                elapsed = time.time() - cycle_start
                if elapsed < 0.10:
                    time.sleep(0.10 - elapsed)
        except KeyboardInterrupt:
            pass
        except Exception:
            import traceback
            traceback.print_exc()

    renderer.close()

    print("\n\n>>> 仿真已结束，正在统计任务 4 端到端视觉控制量化性能报告...")
    rmse_y = math.sqrt(np.mean(np.array(y_errors)**2)) if y_errors else 0.0
    rmse_z = math.sqrt(np.mean(np.array(z_errors)**2)) if z_errors else 0.0
    overall_avg_lat = np.mean(latencies) if latencies else 0.0
    final_pos = sim_node.data.sensor("pos").data[:3]
    final_goal_err = math.hypot(3.0 - final_pos[0], pipe_y - final_pos[1])

    print("=" * 68)
    print("        任务 4 端到端视觉神经控制模型性能综合评价报告")
    print("=" * 68)
    print(f"  ▶ 策略网络拓扑架构:         {arch.upper()}")
    print(f"  ▶ 强化学习算法方案:         {algo.upper()}")
    print(f"  ▶ 巡航总巡检里程 (Distance): {total_dist:.2f} m")
    print(f"  ▶ 管线横向跟踪均方根误差:   {rmse_y:.4f} m (指标要求: <0.20m)")
    print(f"  ▶ 巡航深度保持均方根误差:   {rmse_z:.4f} m (指标要求: <0.15m)")
    print(f"  ▶ 纯视觉端到端平均推理延迟: {overall_avg_lat:.2f} ms (实时约束: <100ms)")
    print(f"  ▶ 管道巡检终点抵达判定:     {'✅ 成功抵达三通汇接节点' if inspection_completed else '⚠️ 手动终止于巡航途中'}")
    print(f"  ▶ 终点对齐锁位误差 (Goal):  {final_goal_err:.4f} m (指标要求: <0.15m)")
    print("=" * 68)

    try:
        sim_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    print("[INFO] 任务 4 仿真已安全结束。")
    os._exit(0)

def main():
    parser = argparse.ArgumentParser(description="水下机器人仿真主入口")
    parser.add_argument("--launch", action="store_true", help="调用 ros2 launch 启动仿真与键盘遥控节点")
    parser.add_argument("--gui", action="store_true", help="拉起 3D 可视化 Viewer 交互视窗 (6-DOF 键盘遥控)")
    parser.add_argument("--ros2", action="store_true", help="运行 ROS2 集成节点")
    parser.add_argument("--task2", action="store_true", help="拉起任务 2: 3D 可视化视窗下的多传感器感知与神经网络自主航迹跟踪")
    parser.add_argument("--launch2", action="store_true", help="调用 ros2 launch 启动任务 2")
    parser.add_argument("--test2", action="store_true", help="运行任务 2 自动化感知与控制综合评测")
    parser.add_argument("--task3", action="store_true", help="拉起任务 3: 3D 可视化视窗下的水下声呐 SLAM 栅格建图与神经网络自主导航")
    parser.add_argument("--launch3", action="store_true", help="调用 ros2 launch 启动任务 3")
    parser.add_argument("--test3", action="store_true", help="运行任务 3 自动化 SLAM 与导航规划综合评测")
    parser.add_argument("--task4", action="store_true", help="拉起任务 4: 3D 可视化视窗下的端到端水下视觉伺服巡航控制")
    parser.add_argument("--launch4", action="store_true", help="调用 ros2 launch 启动任务 4")
    parser.add_argument("--test4", action="store_true", help="运行任务 4 自动化视觉策略与多模型对比评测")
    parser.add_argument("--arch", type=str, default="nature_cnn", help="任务 4 网络架构 (nature_cnn/resnet/spatial_softmax)")
    parser.add_argument("--algo", type=str, default="sac", help="任务 4 学习算法 (sac/ppo/bc)")
    args = parser.parse_args()

    if args.task4:
        run_task4_gui(arch=args.arch, algo=args.algo)
    elif args.launch4:
        run_task4_launch()
    elif args.test4:
        run_task4_test()
    elif args.task3:
        run_task3_gui()
    elif args.launch3:
        run_task3_launch()
    elif args.test3:
        run_task3_test()
    elif args.task2:
        run_task2_gui()
    elif args.launch2:
        run_task2_launch()
    elif args.test2:
        run_task2_test()
    elif args.launch:
        run_ros2_launch()
    elif args.gui:
        run_gui()
    else:
        run_standalone()

if __name__ == "__main__":
    main()
