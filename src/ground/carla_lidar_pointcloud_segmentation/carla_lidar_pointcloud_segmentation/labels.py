"""语义标签映射（纯计算，不依赖 CARLA）。

**这张表是实测出来的，不是抄文档的。**

CARLA 的语义激光雷达给每个点一个 ``object_tag``。文档说它等于
``carla.CityObjectLabel``（Roads=7、Vehicles=11、Pedestrians=4），
但实测下来（CARLA 0.9.16，PyPI 打包版，Town03）编号完全是另一套：
路面是 1、车辆是 14、行人是 12。按文档写会在采集现场把"建筑"当成"行人"。

实测方法见文档第 3.1 节：在自车前方 10 m 放一辆已知车辆、右侧 6 m 放一个已知行人，
统计落在这两个目标附近的点用什么 tag。结果：

* 自车自身半径 3 m 内只有 tag 14 -> 14 是车辆；
* 前方 BMW 半径 3 m 内是 1（路面）与 14（110 点）-> 确认 14 是车辆；
* 右侧行人半径 1.2 m 内是 1（路面）与 12（38 点）-> 12 是行人。

归并成 4 个对驾驶有意义的类别：可行驶面、车辆、行人、障碍物。
未知标签返回 ``-1``，由调用方丢弃并计数，绝不猜。
"""

from typing import Dict, Iterable, Sequence

DRIVABLE = 0
VEHICLE = 1
PEDESTRIAN = 2
OBSTACLE = 3

CLASS_NAMES = ('drivable', 'vehicle', 'pedestrian', 'obstacle')
N_CLASSES = len(CLASS_NAMES)
UNKNOWN = -1

# 实测标签表：标签值 -> (实测描述, 归属类别)
# 描述里写了判断依据，换地图或换 CARLA 版本后重新跑 --probe 核对。
MEASURED_TAG_TABLE: Dict[int, tuple] = {
    1: ('路面，高度恒定、点数最多', DRIVABLE),
    2: ('人行道/地形，贴着地面、比路面低约 0.5 m', DRIVABLE),
    3: ('建筑，远且高（高度到 9 m 以上）', OBSTACLE),
    4: ('建筑/墙，中远距离、高度到 6 m', OBSTACLE),
    6: ('杆、树，细高、距离 5~14 m', OBSTACLE),
    9: ('高处零星点（交通灯/标志）', OBSTACLE),
    12: ('行人，近距离一小簇点', PEDESTRIAN),
    14: ('车辆（含自车），贴着车身、比路面高', VEHICLE),
    20: ('远处零星物体', OBSTACLE),
    21: ('远处零星物体', OBSTACLE),
    25: ('远处零星物体', OBSTACLE),
}

TAG_TO_CLASS: Dict[int, int] = {tag: cls for tag, (_, cls) in MEASURED_TAG_TABLE.items()}

# 官方文档里的编号，仅作对照，不参与映射
OFFICIAL_CITY_OBJECT_LABEL = {
    1: 'Buildings', 2: 'Fences', 3: 'Other', 4: 'Pedestrians', 5: 'Poles',
    6: 'RoadLines', 7: 'Roads', 8: 'Sidewalks', 9: 'TrafficSigns', 10: 'Vegetation',
    11: 'Vehicles', 12: 'Walls', 13: 'TrafficLight', 14: 'Water', 15: 'Terrain',
}


def tag_to_class(tag: int) -> int:
    """单个标签转类别；未知标签返回 ``UNKNOWN``（-1）。"""
    return TAG_TO_CLASS.get(int(tag), UNKNOWN)


def first_tag_of(cls: int) -> int:
    """取某个类别在实测表里编号最小的标签，用于造合成数据或写测试。"""
    candidates = sorted(tag for tag, (_, c) in MEASURED_TAG_TABLE.items() if c == cls)
    if not candidates:
        raise ValueError('实测表里没有类别 %d' % cls)
    return candidates[0]


def tag_description(tag: int) -> str:
    """返回实测描述，用于日志和文档。"""
    item = MEASURED_TAG_TABLE.get(int(tag))
    return item[0] if item else '未知标签'


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


def tag_histogram(tags: Sequence[int]) -> Dict[str, int]:
    """原始标签直方图（按实测描述命名），体检时打印用。"""
    counts: Dict[int, int] = {}
    for tag in tags:
        counts[int(tag)] = counts.get(int(tag), 0) + 1
    return {'%d(%s)' % (tag, tag_description(tag)): n
            for tag, n in sorted(counts.items(), key=lambda item: -item[1])}