# -*- coding: utf-8 -*-
"""赛道与标志场景定义。

MuJoCo 世界为带地面纹理的开阔场地：小车沿半径 2.5 m 的圆形赛道巡航，
限速牌按圆周角度布置，作用域为「进入相机视野至驶离」。
"""
import math

# 世界尺度
PX_PER_M = 100.0
TRACK_CENTER = (400, 400)         # 赛道圆心（px）
TRACK_RADIUS = 250                # 赛道半径（px）= 2.5 m

# 默认巡航与限速。本平台为 1:10 缩比模型（赛道半径 2.5 m），
# 速度按几何相似缩放：v_sim = v_kmh / 3.6 / 10 = v_kmh / 36（m/s），
# 牌面数值沿用真实交通 km/h 标注。
CRUISE_SPEED = 45.0 / 36.0        # 巡航 45 km/h（缩比）= 1.25 m/s
KMH2MS = 1.0 / 36.0

# 车载相机参数
CAM_FOV_DEG = 90.0                # 水平视场角
CAM_RANGE_M = 4.5                 # 最大视距（真值区间计算用）
CAM_W, CAM_H = 320, 240           # 图像分辨率
SIGN_PHYS_M = 0.6                 # 标志牌物理边长（m）
SIGN_LATERAL_M = 0.55             # 牌面相对赛道边缘的外侧横向偏移（m）
CAM_HEIGHT_M = 0.2                # 相机安装高度（m）
SIGN_CENTER_H_M = 0.35            # 牌面中心离地高度（m）


def sign_xy(angle_deg):
    """给定圆周角度，返回标志牌世界坐标（m），牌面位于赛道外侧。"""
    a = math.radians(angle_deg)
    r = TRACK_RADIUS / PX_PER_M + SIGN_LATERAL_M
    return (TRACK_CENTER[0] / PX_PER_M + r * math.cos(a),
            TRACK_CENTER[1] / PX_PER_M + r * math.sin(a))


class Sign:
    """一块限速牌：value_kmh 为牌面数值，deg 为圆周角度。"""

    def __init__(self, angle_deg, value_kmh, degrade=None):
        self.angle_deg = float(angle_deg)
        self.value_kmh = int(value_kmh)
        # 该牌的相机退化强度：dict(rotate, blur, brightness, contrast, noise)
        self.degrade = degrade or {}


# 预置场景：用于演示与多场景评测（起始角 180°，沿角度增大方向行驶）
SCENES = {
    # 基准场景：无退化，两块牌
    "default": {
        "signs": [
            {"angle": 330, "value": 30},
            {"angle": 150, "value": 15},
        ],
        "degrade": {},
    },
    # 退化场景：同一布局，施加较强模糊/暗光/旋转/噪声
    "degraded": {
        "signs": [
            {"angle": 330, "value": 30},
            {"angle": 150, "value": 15},
        ],
        "degrade": {"rotate": 12, "blur": 1.4, "brightness": 0.35,
                    "contrast": 0.4, "noise": 0.06},
    },
    # 密集场景：三块牌短间距连续变速
    "dense": {
        "signs": [
            {"angle": 260, "value": 30},
            {"angle": 10, "value": 15},
            {"angle": 140, "value": 20},
        ],
        "degrade": {"rotate": 6, "blur": 0.8, "brightness": 0.15,
                    "contrast": 0.2, "noise": 0.03},
    },
}


def scene_from_json(data):
    """从 dict 构造场景（供 run_scenes / 参数注入使用）。"""
    signs = [Sign(s["angle"], s["value"],
                  s.get("degrade", data.get("degrade"))) for s in data["signs"]]
    return signs
