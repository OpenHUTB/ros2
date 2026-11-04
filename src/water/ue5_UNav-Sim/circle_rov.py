"""
圆形轨迹控制器 v3（大圆版）
- 半径 3m、角速度 0.2 rad/s，一圈约 31 秒，运动幅度大、肉眼清晰可见
- PWM 计算完全复刻 HelloROV：pwm = A^+ * wrench / 40（0 = 零推力，无中心偏移）
- 深度控制与 HelloROV 相同约定，ROV 稳定在水下约 1.2m
"""
import airsim
import math
import time
import numpy as np

RADIUS, OMEGA = 2.5, 0.08     # 半径 2.5m，角速度 0.08 rad/s（一圈约 79 秒，平稳不饱和）
KP = 8.0                      # 水平 P 增益（放低，避免 PWM 饱和甩飞）
KZ = 20.0                    # 深度 P 增益（U=KZ*(1 - (-z))）
Z_REF = 1.0                  # 参考高度（UE 坐标约定，同 HelloROV）
RUN_SECONDS = 90             # 运行时长（约 3 圈）
SCALE = 40.0                 # PWM 缩放（同 HelloROV）

A = np.array([
    [0.0, 0.0, 0.0, 0.0,  0.5,  0.5, -0.5, -0.5],  # X 力
    [0.0, 0.0, 0.0, 0.0, -0.5,  0.5,  0.5, -0.5],  # Y 力
    [1.0, 1.0, 1.0, 1.0,  0.0,  0.0,  0.0,  0.0],  # Z 力
    [0.0, 0.0, 0.0, 0.0,  0.1, -0.1,  0.1, -0.1],  # 偏航力矩
])

c = airsim.MultirotorClient()
c.confirmConnection()
c.simSetVehiclePose(airsim.Pose(airsim.Vector3r(0, 0, -0.5),
                               airsim.Quaternionr(0, 0, 0, 1)), True)
c.enableApiControl(True)
c.armDisarm(True)
print("ROV 已重置。大圆轨迹：半径 %.1fm, 一圈约 %.0f 秒, 共 %.0f 秒" %
      (RADIUS, 2 * math.pi / OMEGA, RUN_SECONDS), flush=True)

t0 = time.time()
while time.time() - t0 < RUN_SECONDS:
    t = time.time() - t0
    ref_x = RADIUS * math.cos(OMEGA * t)
    ref_y = RADIUS * math.sin(OMEGA * t)

    s = c.client.call('getRovState', '')
    p = s['kinematics_estimated']['position']
    x, y, z = p['x_val'], p['y_val'], p['z_val']

    wrench = np.array([KP * (ref_x - x), KP * (ref_y - y),
                       KZ * (Z_REF - (-z)), 0.0])
    pwm = (np.linalg.lstsq(A, wrench, rcond=None)[0] / SCALE).clip(-1.5, 1.5)
    c.client.call_async('moveByMotorPWMs', list(map(float, pwm)), 0.02, '')

    if int(t) % 5 == 0 and int(t) != int(t - 0.02):
        print("t=%4ds  ref=(%+5.2f,%+5.2f)  pos=(%+6.2f,%+6.2f,%+6.2f)" %
              (int(t), ref_x, ref_y, x, y, z), flush=True)
    time.sleep(0.02)

c.client.call('moveByMotorPWMs', [0.0] * 8, 1.0, '')
print("运行结束，推进器归零。", flush=True)