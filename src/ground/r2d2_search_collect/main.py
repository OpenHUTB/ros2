import pybullet as p
import pybullet_data
import time
import math

# ============ 参数 ============
LINEAR_SPEED = 12.0       # 前进速度
KP = 5.0                  # 转向 P 控制器比例系数
MAX_ANGULAR = 8.0         # 角速度限幅（rad/s）
WHEEL_FORCE = 100         # 轮子驱动力
COLLECT_DIST = 1.0        # 距球小于该距离 → 收集成功
TRAIL_INTERVAL = 0.15     # 轨迹取点间隔（秒）
LOG_INTERVAL = 0.5        # 状态打印间隔（秒）

# 场地中的目标球（x, y 分散摆放，颜色各不相同）
BALL_POSITIONS = [(-2.0, -2.0), (2.0, -1.5), (-1.5, 1.8), (2.5, 2.0)]
BALL_COLORS = [(0.9, 0.2, 0.2, 1.0), (0.2, 0.9, 0.2, 1.0),
               (0.95, 0.8, 0.1, 1.0), (0.8, 0.2, 0.9, 1.0)]

# ============ 连接仿真 ============
client = p.connect(p.GUI, options="--width=960 --height=720")
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

# 加载地面与 R2D2 小车
p.loadURDF("plane.urdf")
robot = p.loadURDF("r2d2.urdf", [0, 0, 0.2])

# 按关节名称建立车轮映射（官方 r2d2.urdf：right_* 为右轮，left_* 为左轮）
right_wheels = []
left_wheels = []
for i in range(p.getNumJoints(robot)):
    joint_name = p.getJointInfo(robot, i)[1].decode("utf-8")
    if joint_name in ("right_front_wheel_joint", "right_back_wheel_joint"):
        right_wheels.append(i)
    elif joint_name in ("left_front_wheel_joint", "left_back_wheel_joint"):
        left_wheels.append(i)

# 模型车头相对 base x 轴的固定偏角（用于计算车头朝向）
lf = p.getLinkState(robot, left_wheels[0])[0]
rf = p.getLinkState(robot, right_wheels[0])[0]
base_pos, _ = p.getBasePositionAndOrientation(robot)
mid = ((lf[0] + rf[0]) / 2, (lf[1] + rf[1]) / 2)
HEAD_ANGLE = math.atan2(mid[1] - base_pos[1], mid[0] - base_pos[0])
print("模型车头相对 base x 轴偏角: %.1f°" % math.degrees(HEAD_ANGLE))

# 放置目标球（半径 0.5m，醒目）
balls = []
for i, (bx, by) in enumerate(BALL_POSITIONS):
    bid = p.loadURDF("sphere_small.urdf", [bx, by, 0.5], globalScaling=2.0)
    p.changeVisualShape(bid, -1, rgbaColor=BALL_COLORS[i])
    balls.append({"id": bid, "x": bx, "y": by, "collected": False})
total = len(balls)

# 实时仿真 + 等 1 秒让小车落地
p.setRealTimeSimulation(1)
time.sleep(1.0)

# 可视化句柄
target_line = -1   # 黄色目标指示线（车 → 当前目标球）
trail_last = None
last_trail_time = 0.0
last_log = 0.0


def get_yaw():
    """读取小车车头实际朝向（弧度）= base 朝向 + 模型车头固定偏角"""
    _, orn = p.getBasePositionAndOrientation(robot)
    return p.getEulerFromQuaternion(orn)[2] + HEAD_ANGLE


def normalize(angle):
    """把角度归一化到 [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def set_wheel(joints, speed):
    """设置一组车轮的目标速度"""
    for j in joints:
        p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL,
                                targetVelocity=speed, force=WHEEL_FORCE)


def drive(linear, angular):
    """差速驱动：左轮 = 线速度 - 角速度，右轮 = 线速度 + 角速度
    角速度 > 0 → 右轮快 → 左转（逆时针，车头朝向目标方向）"""
    set_wheel(left_wheels, linear - angular)
    set_wheel(right_wheels, linear + angular)


def choose_target(cx, cy):
    """智能决策：选择离小车最近的未收集球作为目标"""
    best = None
    best_d = 1e9
    for b in balls:
        if b["collected"]:
            continue
        d = math.hypot(b["x"] - cx, b["y"] - cy)
        if d < best_d:
            best_d = d
            best = b
    return best, best_d


try:
    print("搜寻开始：自动选择最近的球，逐个收集（共 %d 个）" % total)
    collected_count = 0
    done_printed = False
    current_target = None   # 锁定目标：选定后一直追到收集完成，不乱切换

    while p.isConnected(client):
        # 小车当前位置
        pos, _ = p.getBasePositionAndOrientation(robot)
        cx, cy = pos[0], pos[1]

        # 无锁定目标时：选最近的未收集球（仅选一次，锁定）
        if current_target is None:
            current_target, _ = choose_target(cx, cy)
            if current_target is None:
                # 全部收集完成 → 停车
                drive(0.0, 0.0)
                if not done_printed:
                    print("全部 %d 个球收集完成！任务结束" % total)
                    done_printed = True
                time.sleep(0.02)
                continue

        # 朝锁定的目标前进
        target = current_target
        dist = math.hypot(target["x"] - cx, target["y"] - cy)
        target_angle = math.atan2(target["y"] - cy, target["x"] - cx)
        yaw = get_yaw()
        err = normalize(target_angle - yaw)
        angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, KP * err))
        drive(LINEAR_SPEED, angular)

        # 目标指示线（黄色，实时替换）
        target_line = p.addUserDebugLine(
            [cx, cy, 0.15], [target["x"], target["y"], 0.15],
            [1.0, 0.9, 0.1], 3.0, 0, replaceItemUniqueId=target_line)

        # 行驶轨迹（红色实线）
        if time.time() - last_trail_time >= TRAIL_INTERVAL:
            cur = (cx, cy, 0.05)
            if trail_last is not None:
                p.addUserDebugLine(trail_last, cur, [1.0, 0.1, 0.1], 3.0, 0)
            trail_last = cur
            last_trail_time = time.time()

        # 收集判定：靠近球 → 球消失，计数 +1，解锁选下一个
        if dist < COLLECT_DIST:
            p.removeBody(target["id"])
            target["collected"] = True
            collected_count += 1
            print("[收集] 第 %d/%d 个球收集完成！剩余 %d 个"
                  % (collected_count, total, total - collected_count))
            current_target = None

        # 状态打印
        if time.time() - last_log >= LOG_INTERVAL:
            print("[状态] 小车=(%.1f, %.1f) 目标=(%.1f, %.1f) 距离=%.2f 已收集=%d/%d"
                  % (cx, cy, target["x"], target["y"], dist,
                     collected_count, total))
            last_log = time.time()

        time.sleep(0.02)
except KeyboardInterrupt:
    pass
finally:
    p.disconnect(client)
    print("仿真已退出，连接已释放")
