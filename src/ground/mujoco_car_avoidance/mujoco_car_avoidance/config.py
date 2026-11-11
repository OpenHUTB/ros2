# -*- coding: utf-8 -*-
"""模块级配置：路径解析 + 全部可调参数。

约定：所有阈值只在这里定义一次，业务代码不得写魔数。
运行时可通过命令行参数覆盖其中的一部分（见 main.py），
ROS 2 启动时可通过 config/params.yaml 覆盖。
"""

from pathlib import Path

# ----------------------------------------------------------------------------
# 路径
# ----------------------------------------------------------------------------
#: Python 包目录：.../src/ground/mujoco_car_avoidance/mujoco_car_avoidance
MODULE_DIR = Path(__file__).resolve().parent
#: 功能包根目录：.../src/ground/mujoco_car_avoidance
PACKAGE_ROOT = MODULE_DIR.parent

SCENE_FILE_NAME = "car_road_scene.xml"
#: 默认输出目录（曲线、CSV）；仓库不提交该目录内容
DEFAULT_OUTPUT_DIR = PACKAGE_ROOT / "results"


def resolve_scene_path(explicit=None):
    """定位场景 XML。

    查找顺序：命令行显式指定 -> 源码树 models/ -> colcon 安装后的 share/mujoco_car_avoidance/models/。
    两者都不存在时返回源码树候选路径，由调用方打印可读的错误信息。
    """
    if explicit:
        return Path(explicit).expanduser().resolve()

    candidates = [PACKAGE_ROOT / "models" / SCENE_FILE_NAME]
    try:  # 仅在通过 colcon 安装后可用；源码树直接运行时忽略
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            Path(get_package_share_directory("mujoco_car_avoidance")) / "models" / SCENE_FILE_NAME
        )
    except Exception:  # pragma: no cover - 未安装 ROS 包时的正常分支
        pass

    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return candidates[0]


# ----------------------------------------------------------------------------
# 仿真
# ----------------------------------------------------------------------------
TIMESTEP = 0.002          # 物理步长 (s)，与 XML 中 <option timestep> 一致
CONTROL_HZ = 50.0         # 决策/控制频率 (Hz)
SIM_DURATION = 30.0       # 单次运行的仿真时长上限 (s)
REALTIME_FACTOR_WARN = 1.0  # 实时率低于该值只做提示，不视为失败

#: 起步静置时间 (s)：先零轮速推进一段时间，让接触建立、车身稳定后再进入控制闭环。
#: 实测教训：不做静置时，轮胎恰好与地面相切处接触会断续，车轮空转并引起偏航。
SETTLE_SECONDS = 0.6

#: 直线行驶航向保持：角速度修正 = -gain * 航向偏差(rad)，限幅 ±limit。
#: gain = 0 表示关闭。该修正只在自主模式下生效，手动模式不干预驾驶者转向。
HEADING_HOLD_GAIN = 0.8
HEADING_HOLD_LIMIT = 0.6

# ----------------------------------------------------------------------------
# 底盘与传动（几何量必须与 XML 保持一致）
# ----------------------------------------------------------------------------
WHEEL_RADIUS_M = 0.05     # 车轮半径 (m)
WHEEL_SEPARATION_M = 0.25  # 左右轮中心距 (m)
MAX_WHEEL_OMEGA = 60.0    # 轮关节角速度上限 (rad/s)，对应 ctrlrange
#: 单个驱动轮的出力上限 (N)，对应 XML 中的 forcerange。
#: 必须限幅：不限幅时 40 rad/s 的阶跃指令会让伺服瞬间输出几百牛，
#: 实测 0.05 s 内把车加速到 2.77 m/s 并掀翻（upright=-1，车轮悬空 100% 滑移）。
#: 取 8 N 时整车最大加速度 ≈ 2*8/4.6 ≈ 3.5 m/s²，足以跟上 2 m/s² 的速度规划。
MAX_WHEEL_FORCE_N = 8.0

# ----------------------------------------------------------------------------
# 射线（激光雷达式扇面）
# ----------------------------------------------------------------------------
LIDAR_SITE_NAME = "lidar"
#: 扇面角度（度），0 为正前方。
#: 取 5° 间隔：1.8 m 宽的车道在 7 m 处的角宽只有 ±7.3°，
#: 若用 15° 间隔（9 束），中远距离上只有正前方一束能扫到车道内，会漏检；
#: 25 束在 50 Hz 下共 1250 次/秒 mj_ray 调用，开销可忽略。
LIDAR_ANGLES_DEG = tuple(float(v) for v in range(-60, 61, 5))
LIDAR_MAX_RANGE_M = 20.0
#: 参与射线检测的 geom group 掩码（1 = 参与）。
#:   group 0 = 障碍物 / 路侧干扰物（可见）
#:   group 1 = 地面与路面（必须屏蔽：车身俯仰 2~3° 时射线会提前打到路面，
#:             测出 2~3 m 的假障碍物，实测 0.11/tan(2.33°) = 2.70 m）
#:   group 3 = 自车（必须屏蔽，见 XML 顶部说明）
LIDAR_GEOM_GROUPS = (1, 0, 1, 0, 1, 1)

#: 本车道半宽 (m)：命中点的横向偏移超过该值即视为邻车道/路侧目标，不参与避障决策。
#: 取值说明：本车道障碍物半宽 0.35 m，0.8 m 有足够余量；
#: 路侧干扰物在 y=1.8 m 处，即使车头偏航 10° 其横向偏移仍 >1.3 m。
#: （早期取 0.9 且路侧物在 y=1.4 时，擦着开过去出现过横向偏移 0.898 m 的擦线命中，
#:   与门限只差 0.002，导致被误判进本车道并触发刹停。）
LANE_HALF_WIDTH_M = 0.8

# ----------------------------------------------------------------------------
# 输入
# ----------------------------------------------------------------------------
#: 手动模式（--interactive）下键盘指令的保持时间 (s)
TELEOP_HOLD_S = 0.4
#: 外部 cmd_vel 生效后接管自主避障的时间 (s)，用于 ROS 2 手动接管
EXTERNAL_CMD_TIMEOUT_S = 0.5
