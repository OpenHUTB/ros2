"""
demo.py — 运行人形机器人逆运动学 demo，生成可视化图像
=====================================================

输出到 ./output/ :
    1. convergence_compare.png   三种方法收敛曲线对比
    2. manipulability_ellipse.png  可操作性椭圆 3D 图
    3. arm_trajectory.png        手臂构型与末端轨迹
    4. manipulability_vs_q.png   可操作性指标沿轨迹变化
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
from mpl_toolkits.mplot3d import Axes3D  # noqa

from ik import (Arm7DOF, position_jacobian,
                solve_pseudoinverse, solve_damped_ls, solve_nullspace,
                manipulability_index, manipulability_ellipsoid)

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
os.makedirs(OUT, exist_ok=True)

arm = Arm7DOF()

# ---------- 1. 三种方法收敛曲线对比 ----------
q0 = np.zeros(7)
p0 = arm.ee_position(q0)
target = p0 + np.array([0.08, 0.05, 0.04])

_, err_pi = solve_pseudoinverse(arm, q0, target, max_iter=300, tol=1e-5)
_, err_dl = solve_damped_ls(arm, q0, target, max_iter=300, tol=1e-5)
q_pref = np.array([0.1, 0.0, -0.1, 0.0, 0.1, 0.0, -0.1])
_, err_ns = solve_nullspace(arm, q0, target, q_pref, max_iter=300, tol=1e-5)

fig, ax = plt.subplots(figsize=(8, 5))
ax.semilogy(err_pi, label=f"伪逆法 (收敛于 {len(err_pi)} 步)", lw=2)
ax.semilogy(err_dl, label=f"阻尼最小二乘 (收敛于 {len(err_dl)} 步)", lw=2)
ax.semilogy(err_ns, label=f"零空间投影 (收敛于 {len(err_ns)} 步)", lw=2)
ax.set_xlabel("迭代步数")
ax.set_ylabel("末端位置误差 (m, log)")
ax.set_title("三种逆运动学方法收敛曲线对比")
ax.grid(True, which="both", ls="--", alpha=0.4)
ax.legend()
fig.tight_layout()
fig.savefig(os.path.join(OUT, "convergence_compare.png"), dpi=130)
plt.close(fig)
print("[OK] convergence_compare.png")

# ---------- 2. 可操作性椭圆 3D ----------
J = position_jacobian(arm, q0)
semi, dirs = manipulability_ellipsoid(J)
w = manipulability_index(J)

fig = plt.figure(figsize=(8, 7))
ax = fig.add_subplot(111, projection="3d")
u = np.linspace(0, 2*np.pi, 40)
v = np.linspace(0, np.pi, 20)
sx, sy, sz = semi
x = sx * np.outer(np.cos(u), np.sin(v))
y = sy * np.outer(np.sin(u), np.sin(v))
z = sz * np.outer(np.ones_like(u), np.cos(v))
R = dirs
pts = (R @ np.array([x.ravel(), y.ravel(), z.ravel()])).reshape(3, *x.shape)
ax.plot_surface(pts[0], pts[1], pts[2], alpha=0.3, color="steelblue")
for i in range(3):
    ax.quiver(0, 0, 0, dirs[0,i]*semi[i], dirs[1,i]*semi[i], dirs[2,i]*semi[i],
              color="crimson", lw=2, arrow_length_ratio=0.15)
ax.scatter([0], [0], [0], color="black", s=50, label="基座")
ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
ax.set_title(f"可操作性椭圆 (Yoshikawa w={w:.4f})
半轴长: {semi.round(4)}")
ax.legend()
fig.tight_layout()
fig.savefig(os.path.join(OUT, "manipulability_ellipse.png"), dpi=130)
plt.close(fig)
print("[OK] manipulability_ellipse.png")

# ---------- 3. 手臂构型 + 末端轨迹 ----------
traj_targets = np.linspace(p0, p0 + np.array([0.15, 0.08, 0.05]), 12)
q_current = q0.copy()
all_frames = []
manip_list = []
for t in traj_targets:
    q_hist, _ = solve_pseudoinverse(arm, q_current, t, max_iter=200, tol=1e-4, lr=0.5)
    q_current = q_hist[-1]
    all_frames.append(arm.fk_all_frames(q_current))
    J = position_jacobian(arm, q_current)
    manip_list.append(manipulability_index(J))

fig = plt.figure(figsize=(10, 5))
ax1 = fig.add_subplot(121, projection="3d")
for i, frames in enumerate(all_frames):
    xs = [T[0,3] for T in frames]
    ys = [T[1,3] for T in frames]
    zs = [T[2,3] for T in frames]
    alpha = min(0.25 + 0.06 * i, 0.95)
    ax1.plot(xs, ys, zs, "-o", alpha=alpha, color="steelblue", lw=1.5,
             markersize=3)
ee_pts = np.array([frames[-1][:3,3] for frames in all_frames])
ax1.plot(ee_pts[:,0], ee_pts[:,1], ee_pts[:,2], "r-o", lw=2, label="末端轨迹")
ax1.scatter([0],[0],[0], c="k", s=60, label="基座")
ax1.set_xlabel("X"); ax1.set_ylabel("Y"); ax1.set_zlabel("Z")
ax1.set_title("7-DOF 手臂构型与末端直线轨迹")
ax1.legend()

ax2 = fig.add_subplot(122)
ax2.plot(range(len(manip_list)), manip_list, "g-o", lw=2)
ax2.set_xlabel("轨迹采样点")
ax2.set_ylabel("Yoshikawa 可操作性指标 w")
ax2.set_title("沿末端轨迹的可操作性变化")
ax2.grid(True, ls="--", alpha=0.4)

fig.tight_layout()
fig.savefig(os.path.join(OUT, "arm_trajectory.png"), dpi=130)
plt.close(fig)
print("[OK] arm_trajectory.png")

# ---------- 4. 可操作性 vs 关节角 ----------
fig, ax = plt.subplots(figsize=(8, 5))
joint_angles = np.linspace(-1.0, 1.0, 50)
w_list = []
for a in joint_angles:
    q = np.array([a, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    J = position_jacobian(arm, q)
    w_list.append(manipulability_index(J))
ax.plot(joint_angles, w_list, "b-o", lw=2, markersize=4)
ax.set_xlabel("肩关节 1 角度 (rad)")
ax.set_ylabel("Yoshikawa 可操作性指标 w")
ax.set_title("可操作性指标 vs 肩关节 1 角度")
ax.grid(True, ls="--", alpha=0.4)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "manipulability_vs_q.png"), dpi=130)
plt.close(fig)
print("[OK] manipulability_vs_q.png")

print("\n=== 全部图像生成完毕，输出目录:", OUT, "===")
