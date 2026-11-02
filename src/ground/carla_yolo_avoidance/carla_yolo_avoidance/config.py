# -*- coding: utf-8 -*-
"""全局配置。

本文件是**阈值 / 话题名 / 相机参数**的唯一来源：
这类数值按作业要求直接写死在代码里，节点代码中不再出现魔法数字。

唯一的例外是 **CARLA 服务地址（IP 与端口）**：
它们不写死在代码里，而是通过 ROS 参数提供，默认值在
`config/params.yaml` 中配置，由 launch 文件加载。
换机器、换端口只需要改 YAML，不需要修改任何 Python 源码。

本模块不导入 rclpy / carla / torch，因此可以被纯 Python 单元测试直接导入。
"""

import os

# ---------------------------------------------------------------------------
# 1. CARLA 服务地址（IP / 端口不写在代码里，见 config/params.yaml）
#    节点通过下面这两个 ROS 参数名读取地址；非 ROS 的小工具则直接读
#    同一份 YAML（见文件末尾的 load_carla_endpoint）。
# ---------------------------------------------------------------------------
PARAM_CARLA_HOST = 'carla_host'     # ROS 参数名：CARLA 服务端 IP / 主机名
PARAM_CARLA_PORT = 'carla_port'     # ROS 参数名：CARLA 服务端端口
PARAM_YOLO_MODEL = 'model'          # ROS 参数名：YOLOv8 权重文件

CARLA_TIMEOUT_S = 20.0              # carla.Client.set_timeout 的超时时间
PARAMS_DIR_NAME = 'config'          # 参数文件所在目录（相对包源码根目录）
PARAMS_FILE_NAME = 'params.yaml'    # 参数文件名

# ---------------------------------------------------------------------------
# 2. ROS 话题名
# ---------------------------------------------------------------------------
TOPIC_CAMERA_IMAGE = '/carla/camera/image_raw'        # sensor_msgs/Image，原图
TOPIC_DETECTIONS = '/carla/yolo/detections'           # std_msgs/String，JSON 检测框信息
TOPIC_DETECTION_MARKERS = '/carla/yolo/markers'       # visualization_msgs/MarkerArray，RViz 检测框
TOPIC_ANNOTATED_IMAGE = '/carla/yolo/image_annotated'  # sensor_msgs/Image，画了框的图
TOPIC_CONTROL_CMD = '/carla/vehicle/control_cmd'      # std_msgs/Float32MultiArray [油门, 刹车, 转向]
TOPIC_DECISION_STATUS = '/carla/decision/status'      # std_msgs/String，决策文本，便于调试

# RViz / TF 使用的坐标系名
MAP_FRAME = 'map'
CAMERA_FRAME = 'carla_camera_link'

# ---------------------------------------------------------------------------
# 3. 相机参数（CARLA RGB 相机）
# ---------------------------------------------------------------------------
CAMERA_IMAGE_WIDTH = 800
CAMERA_IMAGE_HEIGHT = 600
CAMERA_FOV_DEG = 90.0                 # CARLA 相机水平视场角
CAMERA_POSITION = (2.0, 0.0, 1.60)    # 车体坐标系：x 前, y 右, z 上
CAMERA_ROTATION = (0.0, 0.0, 0.0)     # roll, pitch, yaw
CAMERA_PUBLISH_FPS = 20.0             # 相机回调里最多按这个频率发布

# ---------------------------------------------------------------------------
# 4. 车辆生成
# ---------------------------------------------------------------------------
EGO_VEHICLE_BLUEPRINT = 'vehicle.tesla.model3'   # 找不到时自动退化为第一个可用车型
SPAWN_LEAD_VEHICLE = True                        # 是否在前方生成一辆静止障碍车
LEAD_VEHICLE_BLUEPRINT = 'vehicle.audi.a2'
LEAD_VEHICLE_AHEAD_M = 35.0                      # 障碍车放在自车前方 35 m（同一车道）
LEAD_VEHICLE_LATERAL_M = 0.0                     # 横向偏移，0 表示正前方

# ---------------------------------------------------------------------------
# 5. 控制指令：控制量顺序固定为 [throttle, brake, steer]
#    作业要求只控制油门与刹车，不做转向控制，因此 steer 恒为 0.0
# ---------------------------------------------------------------------------
CONTROL_CMD_SIZE = 3
STEER_ALWAYS_ZERO = True
INITIAL_BRAKE = 1.0        # 启动后、收到第一条指令前保持刹停（安全兜底）
CONTROL_APPLY_HZ = 20.0    # 把控制指令写入 CARLA 车辆的频率

# ---------------------------------------------------------------------------
# 6. YOLOv8 检测参数（只做推理，使用官方预训练权重）
# ---------------------------------------------------------------------------
YOLO_WEIGHTS = 'yolov8n.pt'        # 官方预训练权重，首次运行自动下载
YOLO_CONF_THRESHOLD = 0.35         # 置信度阈值
YOLO_IOU_THRESHOLD = 0.45          # NMS IoU 阈值
YOLO_IMG_SIZE = 640                # 推理输入尺寸
YOLO_DEVICE = ''                   # '' = 自动选择（有 CUDA 用 GPU，否则 CPU）
YOLO_MAX_DETECTIONS = 20           # 单帧最多输出的目标数

# COCO 数据集中属于"车辆"的类别 id：2=car, 3=motorcycle, 5=bus, 7=truck
VEHICLE_CLASS_IDS = (2, 3, 5, 7)

# 各类车辆的真实宽度（米），用于单目测距。改成自己场景的近似值即可。
CLASS_REAL_WIDTH_M = {
    2: 1.80,   # car
    3: 0.80,   # motorcycle
    5: 2.50,   # bus
    7: 2.50,   # truck
}
DEFAULT_REAL_WIDTH_M = 1.80

# 类别名字中只要含有这些关键词，也当作车辆处理（防止权重类别名不同）
VEHICLE_CLASS_KEYWORDS = ('car', 'truck', 'bus', 'motor', 'vehicle')

# ---------------------------------------------------------------------------
# 7. 距离估计（单目针孔模型）
#    focal_px = (image_width / 2) / tan(fov / 2)
#    distance = focal_px * real_width_m / bbox_width_px
# ---------------------------------------------------------------------------
MIN_VALID_DISTANCE_M = 1.0     # 小于该距离视为无效（贴脸误检）
MAX_VALID_DISTANCE_M = 120.0   # 大于该距离视为无效（框太小，误差过大）
MIN_VALID_BBOX_WIDTH_PX = 4.0  # 检测框宽于该像素数才认为距离可信

# ---------------------------------------------------------------------------
# 8. 避障决策阈值（本作业核心"安全阈值"）
#    d < FULL_BRAKE_DISTANCE_M  -> 急刹
#    d < SAFE_DISTANCE_M        -> 点刹减速
#    否则                        -> 正常前进
# ---------------------------------------------------------------------------
SAFE_DISTANCE_M = 20.0         # 安全距离阈值：小于它就必须刹车
FULL_BRAKE_DISTANCE_M = 8.0    # 小于它则全力刹车
CRUISE_THROTTLE = 0.45         # 正常前进时的油门
SLOW_DOWN_BRAKE = 0.35         # 减速时的刹车力度
FULL_BRAKE = 1.0               # 急刹力度
SLOW_DOWN_THROTTLE = 0.0       # 减速时不踩油门

# 只关心"本车道"目标：横向偏移超过该值的检测忽略
ROI_LATERAL_LIMIT_M = 3.5

# 检测结果超时（秒）：YOLO 节点挂掉时按失效处理
DETECTION_STALE_TIMEOUT_S = 5.0
# 检测失效时是否刹停（True = 失效安全，宁可停车）
BRAKE_WHEN_DETECTION_STALE = True

# 决策节点发布控制指令的频率
DECISION_PUBLISH_HZ = 20.0

# 日志刷屏控制：每 N 次循环打印一次
LOG_EVERY_N = 20


# ---------------------------------------------------------------------------
# 9. 参数文件读取（只给"不依赖 ROS"的小工具用）
#    ROS 节点自己不需要读 YAML：参数由 launch 文件加载后通过 ROS 参数下发。
#    这里提供同一份 YAML 的读取入口，保证脚本、自检工具与节点用的是
#    同一个 CARLA 地址，避免"两处配置不一致"。
#    所有路径都由 __file__ 推导，不使用任何写死的绝对路径。
# ---------------------------------------------------------------------------
def package_source_root():
    """返回包源码根目录（.../src/carla_yolo_avoidance），由 __file__ 推导。"""
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def default_params_path():
    """返回 config/params.yaml 的路径（仓库相对结构，运行时按 __file__ 推导）。"""
    return os.path.join(package_source_root(), PARAMS_DIR_NAME, PARAMS_FILE_NAME)


def load_carla_endpoint(params_path=None):
    """从 params.yaml 读取 CARLA 地址，返回 (host, port)。

    参数:
        params_path: YAML 路径；None 表示用包内默认的 config/params.yaml
    返回:
        (host: str, port: int)；读不到时返回 ('', 0)，由调用方给出提示
    异常:
        RuntimeError —— 文件不存在或缺少 PyYAML
    """
    path = params_path or default_params_path()
    if not os.path.isfile(path):
        raise RuntimeError('找不到参数文件：{}'.format(path))

    try:
        import yaml
    except ImportError as exc:      # ROS 2 自带 python3-yaml，正常不会走到这里
        raise RuntimeError('读取参数文件需要 PyYAML（python3 -m pip install PyYAML）：{}'.format(exc))

    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}

    # 支持两种写法：按节点名分组（launch 加载的正式写法）或直接平铺
    section = data.get('/carla_camera_node')
    params = None
    if isinstance(section, dict):
        params = section.get('ros__parameters')
    if not isinstance(params, dict):
        params = data.get('ros__parameters') if isinstance(data.get('ros__parameters'), dict) else data

    host = str(params.get(PARAM_CARLA_HOST) or '').strip()
    try:
        port = int(params.get(PARAM_CARLA_PORT) or 0)
    except (TypeError, ValueError):
        port = 0
    return host, port

