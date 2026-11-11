# -*- coding: utf-8 -*-
"""为「ROV 六自由度手操与抗流评测」文档计算理论估算值。

只用解析式，不跑仿真：
  终端速度      v_t = sqrt(F / k)
  二次阻力系数  k = 0.5 * rho * Cd * A
  洋流阻力      F_c = k * |v_c|^2
所有输入参数都写在下面，改一个数就能重算。
"""

import math

RHO = 1025.0          # 海水密度 kg/m^3
G = 9.81
MASS = 11.0           # ROV 质量 kg（BlueROV2 Heavy 量级）

# 迎水面积 m^2 与阻力系数（按长方体估算：0.457 x 0.338 x 0.254）
A_X = 0.338 * 0.254   # 端面（纵荡）
A_Y = 0.457 * 0.254   # 侧面（横荡）
A_Z = 0.457 * 0.338   # 顶面（升沉）
CD_X, CD_Y, CD_Z = 0.30, 0.60, 0.80

KX = 0.5 * RHO * CD_X * A_X
KY = 0.5 * RHO * CD_Y * A_Y
KZ = 0.5 * RHO * CD_Z * A_Z

# 排水体积：略大于 m/rho，形成约 +0.27 kg 的正浮力
DISPLACED = 0.0110


def terminal_speed(force, k):
    return math.sqrt(max(force, 0.0) / k)


def thrust_for(speed, k):
    return k * speed * speed


def main():
    print("k_x=%.2f  k_y=%.2f  k_z=%.2f  (N·s²/m²)" % (KX, KY, KZ))
    print("各向异性比  k_z/k_x=%.1f   k_y/k_x=%.1f" % (KZ / KX, KY / KX))

    print("\n[1] 单轴定推力下的终端速度 (m/s)")
    for f in (5, 10, 20, 40):
        print("  F=%2d N : surge %.3f  sway %.3f  heave %.3f"
              % (f, terminal_speed(f, KX), terminal_speed(f, KY), terminal_speed(f, KZ)))

    print("\n[2] 达到指定速度所需推力 (N)")
    for v in (0.1, 0.3, 0.5, 0.8):
        print("  v=%.1f m/s: surge %5.1f  sway %5.1f  heave %5.1f"
              % (v, thrust_for(v, KX), thrust_for(v, KY), thrust_for(v, KZ)))

    buoy = RHO * DISPLACED * G
    weight = MASS * G
    net = buoy - weight
    print("\n[3] 静平衡: 浮力 %.2f N - 重力 %.2f N = %+.2f N (约 %+.2f kg)"
          % (buoy, weight, net, net / G))
    print("  不配平时的上浮终端速度: %.3f m/s" % terminal_speed(net, KZ))

    print("\n[4] 配平偏差 vs 漂移速度（Δm 为正 = 额外配重，使净浮力变小）")
    print("  %-8s %8s %8s %8s" % ("Δm(kg)", "净力N", "漂移m/s", "方向"))
    for dm in (-0.5, -0.27, 0.0, 0.27, 0.5, 1.0):
        f = net - dm * G
        d = "上浮" if f > 0.05 else ("下沉" if f < -0.05 else "悬停")
        print("  %-8.2f %8.2f %8.3f %8s" % (dm, f, terminal_speed(abs(f), KZ), d))

    print("\n[5] 洋流对横荡操控权限的侵蚀（可用推力 10 N）")
    for vc in (0.1, 0.2, 0.3, 0.5):
        fc = KY * vc * vc
        print("  洋流 %.1f m/s -> 阻力 %5.2f N -> 占比 %4.0f%%" % (vc, fc, 100.0 * fc / 10.0))

    print("\n[6] 顶流悬停所需推力（保持对地静止）")
    for vc in (0.2, 0.3, 0.5):
        print("  洋流 %.1f m/s -> 需 %5.2f N（横荡）, %5.2f N（纵荡）"
              % (vc, KY * vc * vc, KX * vc * vc))


if __name__ == "__main__":
    main()
