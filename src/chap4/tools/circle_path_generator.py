"""
两轮差速机器人圆形轨迹参数计算器（离线工具，无ROS依赖）
输入：轮子间距d，期望运动半径R，期望线速度v
输出：左右轮目标转速
"""
import math

def calc_wheel_speed(v: float, R: float, d: float):
    """
    :param v: 机器人中心点线速度 (m/s)
    :param R: 转弯半径 (m)
    :param d: 两轮之间的距离 (m)
    :return: left_speed, right_speed (m/s)
    """
    if abs(R) < 0.0001:
        return v, v
    omega = v / R
    v_left = omega * (R - d / 2.0)
    v_right = omega * (R + d / 2.0)
    return v_left, v_right


if __name__ == "__main__":
    # ========== 在这里修改你的参数 ==========
    v = 0.3       # 中心前进速度 m/s
    R = 1.0       # 转圈半径 m
    d = 0.35      # 机器人轮距 m
    # ======================================

    vl, vr = calc_wheel_speed(v, R, d)
    print("===== 差速圆轨迹计算结果 =====")
    print(f"机器人中心速度: {v} m/s")
    print(f"转弯半径: {R} m")
    print(f"轮距: {d} m")
    print(f"左轮速度 = {vl:.4f} m/s")
    print(f"右轮速度 = {vr:.4f} m/s")
    circle_time = 2 * math.pi * R / v
    print(f"跑完一圈预估时间：{circle_time:.2f} s")
