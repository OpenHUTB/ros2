import mujoco
import mujoco.viewer
import time
import os
import sys
import tty
import termios
import threading

xml_path = os.path.join(os.path.dirname(__file__), 'mujoco_menagerie/franka_emika_panda/scene.xml')
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

target_q = data.qpos[:7].copy()
pressed_key = None

def get_key():
    global pressed_key
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            ch = sys.stdin.read(1)
            pressed_key = ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

threading.Thread(target=get_key, daemon=True).start()

print("========================================")
print("MuJoCo 键盘控制已启动！请点击终端窗口，然后按键：")
print("  1-7: 控制对应关节转动")
print("  r  : 复位")
print("  q  : 退出程序")
print("========================================")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        if pressed_key:
            if pressed_key == 'q':
                break
            elif pressed_key == 'r':
                target_q = model.key_qpos[0].copy() if model.nkey > 0 else [0, 0, 0, -1.57, 0, 1.57, 0.78]
                print("执行复位...")
            elif pressed_key in [str(i) for i in range(1, 8)]:
                idx = int(pressed_key) - 1
                target_q[idx] += 0.1 
                print(f"关节 {idx+1} 目标角度: {target_q[idx]:.2f}")
            
            pressed_key = None 
        data.ctrl[:7] = target_q
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.005)
