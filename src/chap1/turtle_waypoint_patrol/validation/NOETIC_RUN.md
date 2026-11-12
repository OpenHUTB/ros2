# 2026-11-12: current-source real Noetic validation

The reviewed controller SHA256 is `ca4ebe41a970f0c912ab034f089e94721ce3e3f963468de14617707286a836b2`.
The earlier 5/7 and 6/7 results remain historical failures in REAL_RUN.md; they are not counted as passes.

## Runtime and boundary

Official `osrf/ros:noetic-desktop` image, immutable manifest digest
`sha256:d5b6240d52269a86c3dcdacbca2a94e655a8fe9ea98390f2af0486b496ee6bf3`.
Image layers were SHA256-checked and extracted into an unprivileged user/PID/mount namespace and chroot.
Ubuntu 20.04.6, Python 3.8.10, ROS Noetic, actual roscore/TCPROS, and actual Qt turtlesim (`QT_QPA_PLATFORM=offscreen`) were executed.

This host forbids netlink interface enumeration. A test-host-only sitecustomize sets
`rosgraph.network._local_addrs = ['127.0.0.1']` to cache its known loopback interface.
It does not replace ROS messages, services, sockets, timers, shutdown, or turtlesim physics.
This adapter is not part of production code or the CI workflow. The original Windows VirtualBox guest was not retested.
The old ROS-internal delay's root cause has not been established; these results establish behavior in the specified runtime.

## Results

| Check | Observed result |
| --- | --- |
| Real turtlesim integration, run QwdR1gfd | 7/7, 34.157 s |
| Real turtlesim integration, run BxasJslJ | 7/7, 34.253 s |
| Real turtlesim integration, run tnSRa3gL | 7/7, 34.348 s |
| Normal controller exits, across all three runs | 18 measurements, 0.766–0.867 s, exit code 0 |
| Python 3.8 offline boundary/lifecycle suite | 22/22 |
| catkin_make and catkin_make install | PASS |
| Installed package main.launch, pause/resume/cancel and SIGINT | PASS, roslaunch exit 0, controller ros_shutdown_done |

The real verifier still enforces the original 8-second controller-exit deadline and exit code 0.
No `ros_shutdown_failed` fallback occurred in these three runs. Deliberately blocked/throwing ROS cleanup
is covered separately by offline failure injection, which requires failure exit code 2, never success.

Each run directory includes complete checks.txt, source hash, seven case CSV files, and controller logs.
`noetic_runs.json` records measured exit durations. `noetic_build.txt` includes the Python 3.8 offline tests and build/install output.
`installed_launch_result.json` and `installed_launch_roslaunch.txt` record the installed-entry smoke test.

## Reproduce on a standard Noetic desktop image/installation

```bash
source /opt/ros/noetic/setup.bash
# From the repository root:
bash src/chap1/turtle_waypoint_patrol/run_checks.sh --offline
bash src/chap1/turtle_waypoint_patrol/run_checks.sh --headless
# Repeat the latter command for independent runs; it creates a private master each time.
mkdir -p /tmp/patrol_ws/src
ln -s "$PWD/src/chap1/turtle_waypoint_patrol" /tmp/patrol_ws/src/turtle_waypoint_patrol
catkin_make -C /tmp/patrol_ws -DPYTHON_EXECUTABLE=/usr/bin/python3
catkin_make -C /tmp/patrol_ws install
source /tmp/patrol_ws/install/setup.bash
python3 src/chap1/turtle_waypoint_patrol/verify_launch.py --output /tmp/patrol_installed_launch
```

Use unused workspace/output paths. Standard hosts do not need the loopback discovery adapter described above.
A successful local run is not a guarantee of maintainer approval or of every machine's timing.
