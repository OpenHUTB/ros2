# AirLib message regression

## Prerequisites

- Linux or Windows.
- Python 3.10 or newer.
- ROS 2 launch packages only when using `main.launch.py`; the default Python
  entry point has no third-party dependencies.

## Why this module exists

AirLib sensors cross a language and transport boundary before reaching ROS.
Payload shape and range regressions can therefore compile successfully while
silently producing invalid bridge messages. This module provides a small,
deterministic contract suite for IMU quaternions, GPS ranges, LiDAR `x/y/z/
intensity` payloads and odometry vectors.

It does not replace the upstream AirLib unit tests or the AirSim runtime. It is
an independent ROS-side regression gate that can run without a GPU or simulator.

## Run

From the `llm` workspace containing `src/air`:

```bash
PYTHONPATH=src/air/airlib_message_regression \
  python3 -m airlib_message_regression.main
```

From a sourced ROS 2 environment:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select airlib_message_regression
source install/setup.bash
ros2 launch airlib_message_regression main.launch.py
```

## Expected output

```text
01 imu        PASS
02 gps        PASS
03 lidar      PASS
04 odometry   PASS
PASS: validated 4 AirLib messages
```

## Test

```bash
PYTHONPATH=src/air/airlib_message_regression python3 -m unittest discover \
  -s src/air/airlib_message_regression/tests -v
```

## AI assistance disclosure

This module, tests and documentation were prepared with large-language-model
assistance. The submitter reviewed and ran the content and is responsible for
the final submission.
