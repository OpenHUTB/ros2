# Air-ground detection benchmark

## Prerequisites

- Python 3.10 or newer.
- No third-party runtime dependency for the offline evaluator.
- ROS 2 launch packages only when using `main.launch.py`.

## Scope

This module measures how detector quality changes with camera altitude and
pitch. It consumes offline frame annotations and predictions; it does not plan
survey routes or control a UAV. This deliberately avoids overlap with GSD-based
mission planners.

For every frame, predictions are sorted by score and greedily assigned to
unmatched ground-truth boxes using an IoU threshold. Results are grouped by
`(altitude_m, pitch_deg)` and include precision, recall, F1, small-object
recall, mean matched IoU and center-point localization error in pixels.

## Input format

See `air_ground_detection_benchmark/sample_data/sample_frames.json`. Every frame contains:

- `altitude_m`;
- `pitch_deg`;
- `ground_truth` boxes in `x, y, width, height` format;
- `predictions` with boxes and confidence scores;
- an optional `small` flag on ground-truth boxes.

Localization error is the Euclidean distance between the ground-truth and
matched prediction box centers. It is averaged over true positives.

## Run

```bash
PYTHONPATH=src/air/air_ground_detection_benchmark python3 -m air_ground_detection_benchmark.main
```

Write results to a custom CSV:

```bash
PYTHONPATH=src/air/air_ground_detection_benchmark python3 -m air_ground_detection_benchmark.main --output output/height_angle_metrics.csv
```

From a sourced ROS 2 environment:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select air_ground_detection_benchmark
source install/setup.bash
ros2 launch air_ground_detection_benchmark main.launch.py
```

## Test

```bash
PYTHONPATH=src/air/air_ground_detection_benchmark python3 -m unittest discover \
  -s src/air/air_ground_detection_benchmark/tests -v
```

## AI assistance disclosure

This module, tests and documentation were prepared with large-language-model
assistance. The submitter reviewed and ran the content and is responsible for
the final submission.
