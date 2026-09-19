# -*- coding: utf-8 -*-

import os
import csv
import math

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


RESULT_DIR = "results"

TRAJECTORIES = {
    "square": "square_trajectory.csv",
    "circle": "circle_trajectory.csv",
    "eight": "eight_trajectory.csv",
}

summary_rows = []


for name, filename in TRAJECTORIES.items():

    path = os.path.join(
        RESULT_DIR,
        filename
    )

    if not os.path.exists(path):
        print("文件不存在:", path)
        continue

    print()
    print("=" * 60)
    print("分析:", name)
    print("=" * 60)

    df = pd.read_csv(path)

    # --------------------------------------------------------
    # 重新计算位置误差
    # --------------------------------------------------------

    dx = (
        df["target_x"]
        - df["actual_x"]
    )

    dy = (
        df["target_y"]
        - df["actual_y"]
    )

    dz = (
        df["target_z"]
        - df["actual_z"]
    )

    error = np.sqrt(
        dx ** 2
        + dy ** 2
        + dz ** 2
    )

    rmse = math.sqrt(
        float(
            np.mean(
                error ** 2
            )
        )
    )

    mean_error = float(
        np.mean(error)
    )

    max_error = float(
        np.max(error)
    )

    final_error = float(
        error.iloc[-1]
    )

    duration = float(
        df["time"].iloc[-1]
        - df["time"].iloc[0]
    )

    print(
        "RMSE:",
        round(rmse, 3),
        "m"
    )

    print(
        "平均误差:",
        round(mean_error, 3),
        "m"
    )

    print(
        "最大误差:",
        round(max_error, 3),
        "m"
    )

    print(
        "最终误差:",
        round(final_error, 3),
        "m"
    )

    print(
        "运行时间:",
        round(duration, 2),
        "s"
    )


    # --------------------------------------------------------
    # 保存汇总
    # --------------------------------------------------------

    summary_rows.append({
        "trajectory": name,
        "rmse_m": rmse,
        "mean_error_m": mean_error,
        "max_error_m": max_error,
        "final_error_m": final_error,
        "duration_s": duration,
        "samples": len(df)
    })


    # --------------------------------------------------------
    # XY 轨迹图
    # --------------------------------------------------------

    plt.figure(
        figsize=(8, 7)
    )

    plt.plot(
        df["target_x"],
        df["target_y"],
        "--",
        label="Target trajectory"
    )

    plt.plot(
        df["actual_x"],
        df["actual_y"],
        label="MLP actual trajectory"
    )

    plt.xlabel("X / m")
    plt.ylabel("Y / m")

    plt.title(
        f"{name.capitalize()} Trajectory Tracking"
    )

    plt.legend()

    plt.grid(True)

    plt.axis("equal")

    plt.tight_layout()

    trajectory_png = os.path.join(
        RESULT_DIR,
        f"{name}_xy.png"
    )

    plt.savefig(
        trajectory_png,
        dpi=200
    )

    plt.close()


    # --------------------------------------------------------
    # 误差随时间变化
    # --------------------------------------------------------

    plt.figure(
        figsize=(9, 5)
    )

    plt.plot(
        df["time"],
        error
    )

    plt.xlabel("Time / s")

    plt.ylabel(
        "Position Error / m"
    )

    plt.title(
        f"{name.capitalize()} Tracking Error"
    )

    plt.grid(True)

    plt.tight_layout()

    error_png = os.path.join(
        RESULT_DIR,
        f"{name}_error.png"
    )

    plt.savefig(
        error_png,
        dpi=200
    )

    plt.close()

    print(
        "轨迹图:",
        trajectory_png
    )

    print(
        "误差图:",
        error_png
    )


# ============================================================
# 汇总 CSV
# ============================================================

summary_df = pd.DataFrame(
    summary_rows
)

summary_path = os.path.join(
    RESULT_DIR,
    "metrics_summary.csv"
)

summary_df.to_csv(
    summary_path,
    index=False,
    encoding="utf-8-sig"
)

print()
print("=" * 60)
print("全部分析完成")
print("=" * 60)

print(summary_df)

print()
print(
    "汇总文件:",
    summary_path
)