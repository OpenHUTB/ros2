# Third-party sources

The G1 MJCF/STL assets, pretrained `motion.pt`, policy observation convention,
PD gains and action scaling originate from
[Unitree RL Gym](https://github.com/unitreerobotics/unitree_rl_gym), revision
`276801e46c5d433564f24658bac64f254b7d2d4b`, BSD-3-Clause.
The upstream copyright and license are preserved in [third_party_license.md](third_party_license.md).
Assets are fetched separately and are not duplicated into this module.

`patrol/locomotion.py` adapts the upstream deployment equations and observation
ordering. The pretrained walking policy is not an original training result.

MuJoCo supplies physical simulation, rendering and ray queries. ROS Noetic
gmapping and move_base supply SLAM and classical navigation. These components
retain their respective upstream licenses; no claim of a new SLAM algorithm is made.

FloorNet, MaskController, ImageController and CostNet, the training data generator, visual corridor
controller, indoor scene generator, ROS stdio bridge and evaluation code were
created for this project with AI assistance. Training weights under `models/`
were produced by this module's training script.
