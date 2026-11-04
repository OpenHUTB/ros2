"""语义标签映射（纯计算，不依赖 CARLA）。

CARLA 的 ``sensor.lidar.ray_cast_semantic`` 给每个点返回一个 ``object_tag``，
取值与 ``carla.CityObjectLabel`` 一致。本模块把它归并成 4 个对驾驶有意义的类别：

* ``DRIVABLE``   可行驶面：路面、车道线、人行道、地形；
* ``VEHICLE``    车辆；
* ``PEDESTRIAN`` 行人；
* ``OBSTACLE``   其余（建筑、栅栏、杆、植被、墙、标志、红绿灯、水等）。

归并有两点考虑：

1. 类别太多会让逐点分类的样本极少（路牌只占几十个点），合并后每类都有足够样本；
2. 驾驶决策只关心"能不能走 / 前面有没有东西"，不需要区分建筑和墙。

真正的标签值需要在 CARLA 里实测一次（不同版本的 tag 编号可能不同），
所以 :func:`tag_to_class` 对未知标签返回 ``-1``，由调用方决定丢弃还是报警。
"""

from typing import Dict, Iterable, Sequence

DRIVABLE = 0
VEHICLE = 1
PEDESTRIAN = 2
OBSTACLE = 3

CLASS_NAMES = ('drivable', 'vehicle', 'pedestrian', 'obstacle')
N_CLASSES = len(CLASS_NAMES)
UNKNOWN = -1

# CARLA CityObjectLabel 的枚举值 -> 类别
# 参考 carla.CityObjectLabel：0 NONE, 1 Buildings, 2 Fences, 3 Other, 4 Pedestrians,
# 5 Poles, 6 RoadLines, 7 Roads, 8 Sidewalks, 9 TrafficSigns, 10 Vegetation,
# 11 Vehicles, 12 Walls, 13 TrafficLight, 14 Water, 15 Terrain
SEMANTIC_TAG_NAMES: Dict[int, str] = {
    0: 'NONE',
    1: 'Buildings',
    2: 'Fences',
    3: 'Other',
    4: 'Pedestrians',
    5: 'Poles',
    6: 'RoadLines',
    7: 'Roads',
    8: 'Sidewalks',
    9: 'TrafficSigns',
    10: 'Vegetation',
    11: 'Vehicles',
    12: 'Walls',
    13: 'TrafficLight',
    14: 'Water',
    15: 'Terrain',
}

TAG_TO_CLASS: Dict[int, int] = {
    1: OBSTACLE,        # Buildings
    2: OBSTACLE,        # Fences
    3: OBSTACLE,        # Other
    4: PEDESTRIAN,      # Pedestrians
    5: OBSTACLE,        # Poles
    6: DRIVABLE,        # RoadLines
    7: DRIVABLE,        # Roads
    8: DRIVABLE,        # Sidewalks
    9: OBSTACLE,        # TrafficSigns
    10: OBSTACLE,       # Vegetation
    11: VEHICLE,        # Vehicles
    12: OBSTACLE,       # Walls
    13: OBSTACLE,       # TrafficLight
    14: OBSTACLE,       # Water
    15: DRIVABLE,       # Terrain
}


def tag_to_class(tag: int) -> int:
    """单个标签转类别；未知标签返回 ``UNKNOWN``（-1）。"""
    return TAG_TO_CLASS.get(int(tag), UNKNOWN)


def group_labels(tags: Sequence[int]) -> list:
    """批量转换，未知标签保留为 -1，方便调用方统计丢弃了多少点。"""
    return [tag_to_class(t) for t in tags]


def filter_known(labels: Sequence[int]):
    """返回 ``(保留索引, 有效标签)``，把 -1 丢掉。"""
    keep = [i for i, c in enumerate(labels) if c != UNKNOWN]
    return keep, [int(labels[i]) for i in keep]


def class_weights(labels: Iterable[int], n_classes: int = N_CLASSES) -> list:
    """按类别频率的倒数给训练权重，缓解"路面点远多于行人点"的不平衡。

    权重归一化到均值为 1；某类没有样本时权重记 0（不参与训练）。
    """
    counts = [0] * n_classes
    for c in labels:
        c = int(c)
        if 0 <= c < n_classes:
            counts[c] += 1
    total = sum(counts)
    if total == 0:
        return [0.0] * n_classes
    weights = [(total / (n_classes * c)) if c > 0 else 0.0 for c in counts]
    active = [w for w in weights if w > 0.0]
    mean = sum(active) / len(active) if active else 1.0
    return [round(w / mean, 6) for w in weights]


def class_distribution(labels: Iterable[int], n_classes: int = N_CLASSES) -> Dict[str, float]:
    """每一类占总样本的比例，用于数据体检。"""
    counts = [0] * n_classes
    total = 0
    for c in labels:
        c = int(c)
        if 0 <= c < n_classes:
            counts[c] += 1
            total += 1
    if total == 0:
        return {name: 0.0 for name in CLASS_NAMES[:n_classes]}
    return {CLASS_NAMES[i]: round(counts[i] / total, 4) for i in range(n_classes)}