import pybullet as p
import pybullet_data
import time

# 连接 PyBullet 可视化仿真窗口（限制窗口尺寸，降低虚拟机软件渲染负担）
p.connect(p.GUI, options="--width=960 --height=720")
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# 关闭阴影，减少渲染开销，缓解画面卡顿
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

# 加载地面与 R2D2 小车
p.loadURDF("plane.urdf")
robotId = p.loadURDF("r2d2.urdf", [0, 0, 0.2])

# 开启实时仿真，仿真自动推进
p.setRealTimeSimulation(1)

# R2D2 四个车轮关节：[2,3] 为右侧轮，[6,7] 为左侧轮
right_wheels = [2, 3]
left_wheels = [6, 7]

# 方向键键码：使用 PyBullet 官方导出的 B3G 常量
ARROW_LEFT = p.B3G_LEFT_ARROW
ARROW_UP = p.B3G_UP_ARROW
ARROW_RIGHT = p.B3G_RIGHT_ARROW
ARROW_DOWN = p.B3G_DOWN_ARROW

# 控制参数
BASE_SPEED = 12.0   # 前进/后退基础速度
TURN_SPEED = 8.0    # 转向附加速度（左右轮差速）
force = 100         # 车轮驱动力

def set_wheel_speed(joints, speed):
    """设置一组车轮的目标速度"""
    for j in joints:
        p.setJointMotorControl2(robotId, j, p.VELOCITY_CONTROL,
                                targetVelocity=speed, force=force)

print("========== R2D2 键盘遥控 ==========")
print("W / ↑   前进")
print("S / ↓   后退")
print("A / ←   左转")
print("D / →   右转")
print("空格     急停")
print("Q / Esc  退出程序")
print("===================================")

while True:
    keys = p.getKeyboardEvents()

    fwd = 0.0   # 纵向：+1 前进，-1 后退
    turn = 0.0  # 转向：+1 左转，-1 右转
    stop = False

    for k, v in keys.items():
        # v 为 1（按下瞬间）或 3（持续按住）时才算按下；2（松开）忽略
        if v & 1:
            if k == ord('w') or k == ARROW_UP:
                fwd += 1.0
            elif k == ord('s') or k == ARROW_DOWN:
                fwd -= 1.0
            elif k == ord('a') or k == ARROW_LEFT:
                turn += 1.0
            elif k == ord('d') or k == ARROW_RIGHT:
                turn -= 1.0
            elif k == ord(' '):
                stop = True
            elif k == ord('q') or k == 27:  # q 或 Esc 退出
                print("退出遥控")
                p.disconnect()
                exit(0)

    if stop:
        set_wheel_speed(left_wheels, 0.0)
        set_wheel_speed(right_wheels, 0.0)
    else:
        # 差速转向：左转时左轮加速、右轮减速，小车向左转
        left_speed = BASE_SPEED * fwd + TURN_SPEED * turn
        right_speed = BASE_SPEED * fwd - TURN_SPEED * turn
        set_wheel_speed(left_wheels, left_speed)
        set_wheel_speed(right_wheels, right_speed)

    time.sleep(0.02)
