# Panoramic camera projection

## Prerequisites

- Python 3.10 or newer.
- NumPy and OpenCV for offline stitching.
- ROS 2 Humble, `rclpy`, `sensor_msgs` and `cv_bridge` only for the optional
  six-topic online node.

## What it does

The module maps six cube faces (`front`, `right`, `back`, `left`, `up`, `down`)
to a 2:1 equirectangular panorama. It provides:

- a deterministic offline demonstration with no simulator requirement;
- reusable face-selection and stitching functions;
- an optional ROS 2 node consuming `camera/<face>/image` and publishing
  `panorama/image`;
- regression tests for face centers, output shape and missing-face handling.

The online node groups images by ROS timestamp before stitching. All six input
images must therefore use synchronized timestamps.

## Run the offline demo

From this directory:

```bash
python3 main.py --output output/panorama.png
```

Expected terminal output:

```text
faces: front, right, back, left, up, down
resolution: 1024x512
PASS: equirectangular panorama generated
```

The generated PNG contains the six labeled faces projected into a 360 by 180
degree image.

## Run with six PNG inputs

```bash
python3 main.py --faces-dir path_to_face_images --output output/panorama.png
```

## ROS 2 online node

After sourcing ROS 2 and building/installing this package:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run panoramic_camera_projection panorama_node
```

## Test

```bash
python3 -m unittest discover -s tests -v
```

## AI assistance disclosure

This module, tests and documentation were prepared with large-language-model
assistance. The submitter reviewed and ran the content and is responsible for
the final submission.
