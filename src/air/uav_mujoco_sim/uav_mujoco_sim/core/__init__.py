"""纯 Python 核心层：不依赖 ROS，仅需 mujoco + numpy。

子模块惰性导入：ROS 侧 Python 3.8 无 mujoco，顶层导入会让 ROS 节点 import 失败；
teleop / terminal 是纯标准库，可直接导入。
"""

__all__ = [
    "Quadrotor", "QuadParams", "QuadrotorState", "GeometricController",
    "mixer", "quat_to_mat", "mat_to_quat", "quat_to_euler",
    "Scene", "SceneObject", "make_scene", "scene_to_xml", "SCENE_BUILDERS",
    "open_ground", "obstacle_field", "corridor", "gate_track", "urban_block",
    "VisionSensors", "CameraConfig", "LidarSensor", "LidarConfig",
    "ImuSensor", "ImuReading", "SEMANTIC_CLASSES", "build_semantic_map",
]

#: 名字 → 所在子模块，供 __getattr__ 惰性加载
_LAZY = {
    "Quadrotor": "quadrotor", "QuadParams": "quadrotor",
    "QuadrotorState": "quadrotor", "GeometricController": "quadrotor",
    "mixer": "quadrotor", "quat_to_mat": "quadrotor",
    "mat_to_quat": "quadrotor", "quat_to_euler": "quadrotor",
    "Scene": "scenes", "SceneObject": "scenes", "make_scene": "scenes",
    "scene_to_xml": "scenes", "SCENE_BUILDERS": "scenes",
    "open_ground": "scenes", "obstacle_field": "scenes",
    "corridor": "scenes", "gate_track": "scenes", "urban_block": "scenes",
    "VisionSensors": "sensors", "CameraConfig": "sensors",
    "LidarSensor": "sensors", "LidarConfig": "sensors",
    "ImuSensor": "sensors", "ImuReading": "sensors",
    "SEMANTIC_CLASSES": "sensors", "build_semantic_map": "sensors",
}


def __getattr__(name):
    mod = _LAZY.get(name)
    if mod is None:
        raise AttributeError("module %r has no attribute %r" % (__name__, name))
    import importlib
    m = importlib.import_module("." + mod, __name__)
    val = getattr(m, name)
    globals()[name] = val          # 缓存，后续访问不再走 __getattr__
    return val


def __dir__():
    return sorted(set(list(globals()) + list(_LAZY)))
