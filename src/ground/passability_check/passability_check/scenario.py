"""场景文件（JSON）的读取与校验（纯标准库）。

场景 = 一条纵剖面 + 两个门槛：

```json
{
  "name": "garage_ramp",
  "title": "地库坡道（20% 上坡，坡顶转平）",
  "note": "短坡道，坡度 11.31°；老地库常见，出口处还有一道 8 cm 的排水沟盖",
  "terrain": [[0.0, 0.0], [3.0, 0.0], [9.0, 1.2], [12.0, 1.2]],
  "min_clearance_m": 0.12,
  "safety_margin_m": 0.0
}
```

用 JSON 而不是 YAML：`json` 是标准库，不用给评分机器装 PyYAML。
"""

import io
import json
import os

from .terrain import Terrain, TerrainError

DEFAULT_MIN_CLEARANCE_M = 0.10
DEFAULT_SAFETY_MARGIN_M = 0.0


class ScenarioError(ValueError):
    """场景文件不合法。"""


def default_scenario_dir():
    """按「源码目录 → colcon 安装目录」的顺序找场景目录。"""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [os.path.join(os.path.dirname(here), "config", "scenarios")]
    try:  # 只在 ROS 2 环境里有 ament_index_python，没有就走源码目录
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(get_package_share_directory("passability_check"),
                                       "config", "scenarios"))
    except Exception:  # noqa: BLE001
        pass
    for path in candidates:
        if os.path.isdir(path):
            return path
    raise ScenarioError("找不到场景目录，已尝试：%s" % ", ".join(candidates))


def list_scenarios(directory=None):
    """列出场景目录下所有 .json（按文件名排序）。"""
    target = directory or default_scenario_dir()
    if not os.path.isdir(target):
        raise ScenarioError("场景目录不存在：%s" % target)
    names = sorted(name for name in os.listdir(target) if name.endswith(".json"))
    return [os.path.join(target, name) for name in names]


class Scenario(object):
    """一条可核算的纵剖面场景。"""

    __slots__ = ("name", "title", "note", "terrain", "min_clearance_m",
                 "safety_margin_m", "source")

    def __init__(self, name, terrain, title="", note="",
                 min_clearance_m=DEFAULT_MIN_CLEARANCE_M,
                 safety_margin_m=DEFAULT_SAFETY_MARGIN_M, source=""):
        if not name:
            raise ScenarioError("场景必须有 name")
        self.name = str(name)
        self.title = str(title or name)
        self.note = str(note or "")
        if not isinstance(terrain, Terrain):
            try:
                terrain = Terrain(terrain, name=self.name)
            except TerrainError as error:
                raise ScenarioError("场景 %s 的剖面不合法：%s" % (self.name, error))
        self.terrain = terrain
        self.min_clearance_m = _as_non_negative(min_clearance_m, "min_clearance_m")
        self.safety_margin_m = _as_non_negative(safety_margin_m, "safety_margin_m")
        self.source = source

    @classmethod
    def from_dict(cls, data, source=""):
        if not isinstance(data, dict):
            raise ScenarioError("场景 JSON 顶层必须是对象")
        for key in ("name", "terrain"):
            if key not in data:
                raise ScenarioError("场景 JSON 缺少字段 %r" % key)
        return cls(
            name=data["name"],
            terrain=data["terrain"],
            title=data.get("title", ""),
            note=data.get("note", ""),
            min_clearance_m=data.get("min_clearance_m", DEFAULT_MIN_CLEARANCE_M),
            safety_margin_m=data.get("safety_margin_m", DEFAULT_SAFETY_MARGIN_M),
            source=source,
        )

    def to_dict(self):
        return {
            "name": self.name,
            "title": self.title,
            "note": self.note,
            "terrain": [[x, z] for x, z in self.terrain.points],
            "min_clearance_m": self.min_clearance_m,
            "safety_margin_m": self.safety_margin_m,
        }

    def __repr__(self):
        return "Scenario(name=%r, points=%d, x=[%.1f, %.1f])" % (
            self.name, len(self.terrain.points), self.terrain.min_x, self.terrain.max_x)


def _as_non_negative(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ScenarioError("%s 不是数字：%r" % (label, value))
    if result < 0.0:
        raise ScenarioError("%s 不能为负：%r" % (label, result))
    return result


def load_scenario(target, directory=None):
    """按文件名/名字加载场景；也接受直接给 .json 路径。"""
    if not isinstance(target, str) or not target.strip():
        raise ScenarioError("场景名不能为空")
    path = target
    if not os.path.isfile(path):
        base = directory or default_scenario_dir()
        candidate = os.path.join(base, target if target.endswith(".json") else target + ".json")
        if not os.path.isfile(candidate):
            raise ScenarioError("场景 %r 不存在：%s" % (target, candidate))
        path = candidate
    with io.open(path, "r", encoding="utf-8") as handle:
        try:
            data = json.load(handle)
        except ValueError as error:
            raise ScenarioError("场景文件 %s 不是合法 JSON：%s" % (path, error))
    return Scenario.from_dict(data, source=path)


def load_all_scenarios(directory=None):
    """加载场景目录里的全部场景。"""
    scenarios = []
    for path in list_scenarios(directory):
        scenarios.append(load_scenario(path, directory=directory))
    return scenarios
