# -*- coding: utf-8 -*-

import os
import csv
import time
import math
import argparse

import airsim
import numpy as np
import torch
import torch.nn as nn


# ============================================================
# MLP
# ============================================================

class MLPController(nn.Module):

    def __init__(self):

        super().__init__()

        self.net = nn.Sequential(

            nn.Linear(6, 64),
            nn.ReLU(),

            nn.Linear(64, 64),
            nn.ReLU(),

            nn.Linear(64, 32),
            nn.ReLU(),

            nn.Linear(32, 3),

            nn.Tanh()
        )

    def forward(self, x):
        return self.net(x)


# ============================================================
# 参数
# ============================================================

MODEL_PATH = "mlp_controller.pth"

AIRSIM_PORT = 41451

CONTROL_DT = 0.15

WAYPOINT_TOLERANCE = 0.8

MAX_WAYPOINT_TIME = 15.0


# ============================================================
# 轨迹生成
# ============================================================

def make_square(
    x0,
    y0,
    z0,
    side=10.0
):

    return [

        (x0, y0, z0),

        (
            x0 + side,
            y0,
            z0
        ),

        (
            x0 + side,
            y0 + side,
            z0
        ),

        (
            x0,
            y0 + side,
            z0
        ),

        (
            x0,
            y0,
            z0
        )
    ]


def make_circle(
    x0,
    y0,
    z0,
    radius=8.0,
    points=60
):

    trajectory = []

    for i in range(
        points + 1
    ):

        t = (
            2.0
            * math.pi
            * i
            / points
        )

        x = (
            x0
            + radius
            * math.cos(t)
        )

        y = (
            y0
            + radius
            * math.sin(t)
        )

        trajectory.append(
            (
                x,
                y,
                z0
            )
        )

    return trajectory


def make_figure8(
    x0,
    y0,
    z0,
    scale=10.0,
    points=80
):

    trajectory = []

    for i in range(
        points + 1
    ):

        t = (
            2.0
            * math.pi
            * i
            / points
        )

        x = (
            x0
            + scale
            * math.sin(t)
        )

        y = (
            y0
            + scale
            * math.sin(t)
            * math.cos(t)
        )

        trajectory.append(
            (
                x,
                y,
                z0
            )
        )

    return trajectory


# ============================================================
# 参数解析
# ============================================================

parser = argparse.ArgumentParser()

parser.add_argument(
    "--trajectory",
    choices=[
        "square",
        "circle",
        "eight"
    ],
    default="square"
)

args = parser.parse_args()


# ============================================================
# 加载模型
# ============================================================

device = torch.device(
    "cuda"
    if torch.cuda.is_available()
    else "cpu"
)


checkpoint = torch.load(
    MODEL_PATH,
    map_location=device,
    weights_only=False
)


model = MLPController().to(
    device
)


model.load_state_dict(
    checkpoint[
        "model_state_dict"
    ]
)


model.eval()


INPUT_SCALE = np.array(
    checkpoint[
        "input_scale"
    ],
    dtype=np.float32
)


OUTPUT_SCALE = np.array(
    checkpoint[
        "output_scale"
    ],
    dtype=np.float32
)


print("MLP模型加载成功")

print("设备:", device)


# ============================================================
# AirSim
# ============================================================

client = airsim.MultirotorClient(
    ip="127.0.0.1",
    port=AIRSIM_PORT
)

client.confirmConnection()


client.enableApiControl(
    True
)

client.armDisarm(
    True
)


# ============================================================
# 起飞
# ============================================================

state = client.getMultirotorState()

position = (
    state
    .kinematics_estimated
    .position
)


if position.z_val > -1.0:

    print("起飞...")

    client.takeoffAsync().join()

    time.sleep(1)


# ============================================================
# 初始位置
# ============================================================

state = client.getMultirotorState()

position = (
    state
    .kinematics_estimated
    .position
)


x0 = position.x_val
y0 = position.y_val


# 固定飞行高度
z0 = min(
    position.z_val,
    -8.0
)


print(
    "实验原点:",
    x0,
    y0,
    z0
)


# 先到实验高度
client.moveToPositionAsync(
    x0,
    y0,
    z0,
    2.0
).join()


# ============================================================
# 创建轨迹
# ============================================================

if args.trajectory == "square":

    waypoints = make_square(
        x0,
        y0,
        z0
    )


elif args.trajectory == "circle":

    waypoints = make_circle(
        x0,
        y0,
        z0
    )


else:

    waypoints = make_figure8(
        x0,
        y0,
        z0
    )


print(
    "轨迹:",
    args.trajectory
)

print(
    "轨迹点:",
    len(waypoints)
)


# ============================================================
# CSV
# ============================================================

os.makedirs(
    "results",
    exist_ok=True
)


csv_path = os.path.join(
    "results",
    args.trajectory
    + "_trajectory.csv"
)


csv_file = open(
    csv_path,
    "w",
    newline="",
    encoding="utf-8"
)


writer = csv.writer(
    csv_file
)


writer.writerow([
    "time",
    "target_x",
    "target_y",
    "target_z",
    "actual_x",
    "actual_y",
    "actual_z",
    "error",
    "vx_cmd",
    "vy_cmd",
    "vz_cmd"
])


start_time = time.time()

errors = []


# ============================================================
# 逐轨迹点控制
# ============================================================

try:

    for waypoint_id, target in enumerate(
        waypoints
    ):

        tx, ty, tz = target

        print(
            f"Waypoint "
            f"{waypoint_id + 1}"
            f"/{len(waypoints)}"
        )


        waypoint_start = time.time()


        while True:

            # ----------------------------------------------
            # 当前状态
            # ----------------------------------------------

            state = (
                client
                .getMultirotorState()
            )


            kin = (
                state
                .kinematics_estimated
            )


            p = kin.position
            v = kin.linear_velocity


            # ----------------------------------------------
            # 误差
            # ----------------------------------------------

            ex = (
                tx
                - p.x_val
            )

            ey = (
                ty
                - p.y_val
            )

            ez = (
                tz
                - p.z_val
            )


            distance = math.sqrt(
                ex ** 2
                + ey ** 2
                + ez ** 2
            )


            errors.append(
                distance
            )


            # 到达目标点
            if (
                distance
                < WAYPOINT_TOLERANCE
            ):

                break


            # 防止卡死
            if (
                time.time()
                - waypoint_start
                > MAX_WAYPOINT_TIME
            ):

                print(
                    "该航点超时，"
                    "进入下一点"
                )

                break


            # ----------------------------------------------
            # MLP 输入
            # ----------------------------------------------

            input_state = np.array([
                ex,
                ey,
                ez,
                v.x_val,
                v.y_val,
                v.z_val
            ], dtype=np.float32)


            normalized = (
                input_state
                / INPUT_SCALE
            )


            x_tensor = torch.tensor(
                normalized,
                dtype=torch.float32,
                device=device
            ).unsqueeze(0)


            # ----------------------------------------------
            # 神经网络控制
            # ----------------------------------------------

            with torch.no_grad():

                output = (
                    model(
                        x_tensor
                    )
                    .cpu()
                    .numpy()[0]
                )


            command = (
                output
                * OUTPUT_SCALE
            )


            vx_cmd = float(
                command[0]
            )

            vy_cmd = float(
                command[1]
            )

            vz_cmd = float(
                command[2]
            )


            # ----------------------------------------------
            # AirSim 控制
            # ----------------------------------------------

            client.moveByVelocityAsync(
                vx_cmd,
                vy_cmd,
                vz_cmd,
                CONTROL_DT
            ).join()


            # ----------------------------------------------
            # 日志
            # ----------------------------------------------

            writer.writerow([
                time.time()
                - start_time,

                tx,
                ty,
                tz,

                p.x_val,
                p.y_val,
                p.z_val,

                distance,

                vx_cmd,
                vy_cmd,
                vz_cmd
            ])


            csv_file.flush()


finally:

    client.hoverAsync().join()

    csv_file.close()


# ============================================================
# 评价
# ============================================================

errors = np.array(
    errors,
    dtype=np.float32
)


if len(errors) > 0:

    rmse = math.sqrt(
        float(
            np.mean(
                errors ** 2
            )
        )
    )

    max_error = float(
        np.max(errors)
    )

    mean_error = float(
        np.mean(errors)
    )

else:

    rmse = 0
    max_error = 0
    mean_error = 0


print()
print("=" * 60)

print(
    "轨迹:",
    args.trajectory
)

print(
    "RMSE:",
    round(rmse, 3),
    "m"
)

print(
    "平均误差:",
    round(
        mean_error,
        3
    ),
    "m"
)

print(
    "最大误差:",
    round(
        max_error,
        3
    ),
    "m"
)

print(
    "完成时间:",
    round(
        time.time()
        - start_time,
        2
    ),
    "s"
)

print(
    "结果:",
    csv_path
)

print("=" * 60)


print(
    "悬停完成。"
)

print(
    "重新运行实验前建议"
    "重启 CarlaAir。"
)