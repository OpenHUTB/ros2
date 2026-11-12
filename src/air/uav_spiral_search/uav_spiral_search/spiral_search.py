"""Archimedean-spiral area search with proportional tracking and target lock.

All positions and velocities are in the AirSim world NED frame:
x north, y east, z down, so altitude is negative z. Yaw is the drone
heading in radians (0 = facing north, positive towards east).

The drone expands an Archimedean spiral

    r(theta) = start_radius + spacing * theta / (2 * pi)

around the point where the search started, advancing the reference
point along the curve at cruise speed while a proportional controller
tracks it. When the HSV detector reports the target for
``confirm_frames`` consecutive frames the mission locks: the target
ground position is estimated with a fixed assumed distance along the
current heading, and the drone hovers above that estimate.
"""
from dataclasses import dataclass
import math

TWO_PI = 2.0 * math.pi


@dataclass(frozen=True)
class Settings:
    search_height: float = -3.0        # NED 搜索高度（负值为空中）
    hover_height: float = -5.0         # 锁定目标后的悬停高度
    climb_tolerance: float = 0.3       # 进入搜索状态的高度误差阈值，单位 m
    spiral_spacing: float = 4.0        # 相邻两圈螺旋的径向间距，单位 m
    start_radius: float = 2.0          # 螺旋起始半径，单位 m
    max_radius: float = 30.0           # 搜索区域半径上限，单位 m
    cruise_speed: float = 1.6          # 参考点沿螺旋推进的速度，单位 m/s
    max_tracking_error: float = 2.5    # 参考点超前无人机的距离上限，单位 m
    position_gain: float = 1.2         # 位置误差 -> 速度 的比例增益
    max_speed: float = 2.0             # 水平速度上限，单位 m/s
    altitude_gain: float = 1.0         # 高度误差 -> 垂直速度 的增益
    max_vertical_speed: float = 1.0    # 垂直速度上限，单位 m/s
    confirm_frames: int = 3            # 连续检测到目标的帧数阈值
    detection_timeout: float = 1.0     # 检测结果过期时间，单位 s
    odom_timeout: float = 0.5          # 里程计过期时间，单位 s
    assumed_target_distance: float = 8.0  # 前视相机无深度信息，按该距离估计目标位置，单位 m
    camera_half_fov: float = 0.5      # 相机水平半视场角，单位 rad（默认约 28.6°）

    def __post_init__(self):
        for name, value in vars(self).items():
            if name == 'confirm_frames':
                if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                    raise ValueError('confirm_frames must be a positive integer')
                continue
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        for name in ('climb_tolerance', 'spiral_spacing', 'max_radius',
                     'cruise_speed', 'max_tracking_error', 'position_gain',
                     'max_speed', 'altitude_gain', 'max_vertical_speed',
                     'detection_timeout', 'odom_timeout',
                     'assumed_target_distance', 'camera_half_fov'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive')
        if self.start_radius < 0:
            raise ValueError('start_radius must be nonnegative')
        if self.max_radius <= self.start_radius:
            raise ValueError('max_radius must exceed start_radius')
        if self.cruise_speed > self.max_speed:
            raise ValueError('cruise_speed must not exceed max_speed')
        if self.camera_half_fov >= math.pi:
            raise ValueError('camera_half_fov must be below pi radians')
        if self.search_height >= 0 or self.hover_height >= 0:
            raise ValueError('NED heights must be negative (above ground)')


class SpiralSearchController:
    """Mission state machine.

    waiting_odom -> climbing -> searching -> locked | exhausted
    """

    def __init__(self, settings=None):
        self.settings = settings or Settings()
        self.pose = None              # (x, y, z, yaw)
        self.pose_time = None
        self.last_step_time = None
        self.last_path_pose = None
        self.state = 'waiting_odom'
        self.origin = None            # (x, y) 螺旋中心
        self.theta = 0.0              # 螺旋参数（极角，rad）
        self.reference = None         # 最近一次螺旋参考点 (x, y)
        self.confirm_count = 0
        self.last_detection_time = None
        self.last_cx = 0.0             # 最近一次检测框归一化横坐标
        self.search_start = None
        self.path_length = 0.0
        self.target_estimate = None   # (x, y) 目标地面位置估计
        self.lock_time = None
        self.hover_point = None       # (x, y, z) 终态悬停点

    # ------------------------------------------------------------------
    # 输入
    # ------------------------------------------------------------------
    def update_pose(self, x, y, z, yaw, now):
        if not all(math.isfinite(v) for v in (x, y, z, yaw, now)):
            self.pose = None
            return
        self.pose = (x, y, z, yaw)
        self.pose_time = now

    def update_detection(self, visible, now, cx=0.0):
        """按帧累计连续检测；一帧未见目标即清零。

        cx 为检测框中心的归一化横坐标 [-1, 1]（画面中心为 0），
        用于锁定时刻的目标方位估计。
        """
        if visible:
            self.confirm_count += 1
            self.last_detection_time = now
            self.last_cx = max(-1.0, min(1.0, cx))
        else:
            self.confirm_count = 0

    # ------------------------------------------------------------------
    # 螺旋几何
    # ------------------------------------------------------------------
    def radius(self, theta):
        s = self.settings
        return s.start_radius + s.spiral_spacing * theta / TWO_PI

    def spiral_point(self, theta):
        r = self.radius(theta)
        ox, oy = self.origin
        return ox + r * math.cos(theta), oy + r * math.sin(theta)

    def advance_theta(self, dt):
        """沿弧长以 cruise_speed 推进参考点。

        阿基米德螺旋 ds/dtheta = sqrt(r^2 + b^2)，b = spacing/(2*pi)。
        """
        s = self.settings
        r = self.radius(self.theta)
        b = s.spiral_spacing / TWO_PI
        self.theta += s.cruise_speed * dt / math.sqrt(r * r + b * b)

    # ------------------------------------------------------------------
    # 控制
    # ------------------------------------------------------------------
    def step(self, now):
        """Return (world_vx, world_vy, world_vz), state, report dict."""
        s = self.settings
        dt = (0.0 if self.last_step_time is None
              else max(0.0, min(0.1, now - self.last_step_time)))
        self.last_step_time = now

        if self.pose is None or self.pose_time is None \
                or not 0 <= now - self.pose_time <= s.odom_timeout:
            return (0.0, 0.0, 0.0), 'odom_stale', self._report(now, (0.0, 0.0, 0.0))

        # 图像流中断时，冻结的确认计数不能继续算作"连续检测"。
        if (self.confirm_count and self.last_detection_time is not None
                and now - self.last_detection_time > s.detection_timeout):
            self.confirm_count = 0

        x, y, z, yaw = self.pose
        if self.last_path_pose is not None:
            self.path_length += math.hypot(x - self.last_path_pose[0],
                                           y - self.last_path_pose[1])
        self.last_path_pose = (x, y)

        if self.state == 'waiting_odom':
            self.state = 'climbing'

        if self.state == 'climbing':
            z_error = s.search_height - z
            if abs(z_error) <= s.climb_tolerance:
                self.state = 'searching'
                self.origin = (x, y)
                self.theta = 0.0
                self.search_start = now
            else:
                vz = max(-s.max_vertical_speed, min(s.max_vertical_speed,
                                                    s.altitude_gain * z_error))
                return (0.0, 0.0, vz), self.state, self._report(now, (0.0, 0.0, vz))

        if self.state == 'searching':
            if self.confirm_count >= s.confirm_frames:
                # 连续 confirm_frames 帧看到目标：锁定。
                # 前视相机无深度：方位由检测框水平偏差校正（cx 映射到
                # ±半视场角），距离取假设值 assumed_target_distance。
                self.state = 'locked'
                self.lock_time = now
                bearing = yaw + self.last_cx * s.camera_half_fov
                self.target_estimate = (
                    x + s.assumed_target_distance * math.cos(bearing),
                    y + s.assumed_target_distance * math.sin(bearing))
                self.hover_point = (self.target_estimate[0], self.target_estimate[1],
                                    s.hover_height)
            elif self.radius(self.theta) >= s.max_radius:
                self.state = 'exhausted'
                self.lock_time = now
                self.hover_point = (x, y, s.search_height)
            else:
                # 无人机跟上参考点后才继续推进，避免参考点越跑越远。
                ref_x, ref_y = self.spiral_point(self.theta)
                if math.hypot(x - ref_x, y - ref_y) <= s.max_tracking_error:
                    self.advance_theta(dt)
                    ref_x, ref_y = self.spiral_point(self.theta)
                self.reference = (ref_x, ref_y)
                command = self._track(ref_x, ref_y, s.search_height, x, y, z)
                return command, self.state, self._report(now, command)

        # locked / exhausted：在悬停点上方保持位置。
        hx, hy, hz = self.hover_point
        command = self._track(hx, hy, hz, x, y, z)
        return command, self.state, self._report(now, command)

    def _track(self, ref_x, ref_y, ref_z, x, y, z):
        s = self.settings
        vx = s.position_gain * (ref_x - x)
        vy = s.position_gain * (ref_y - y)
        horizontal = math.hypot(vx, vy)
        if horizontal > s.max_speed:
            scale = s.max_speed / horizontal
            vx *= scale
            vy *= scale
        vz = max(-s.max_vertical_speed, min(s.max_vertical_speed,
                                            s.altitude_gain * (ref_z - z)))
        return (vx, vy, vz)

    # ------------------------------------------------------------------
    # 指标
    # ------------------------------------------------------------------
    def _report(self, now, command):
        search_time = None
        if self.search_start is not None:
            end = self.lock_time if self.lock_time is not None else now
            search_time = max(0.0, end - self.search_start)
        return {
            'state': self.state,
            'search_time_s': round(search_time, 2) if search_time is not None else None,
            'radius_m': round(self.radius(self.theta), 2)
                        if self.origin is not None else None,
            'path_len_m': round(self.path_length, 2),
            'confirm_frames': self.confirm_count,
            'target_estimate_ned': ([round(v, 2) for v in self.target_estimate]
                                    if self.target_estimate is not None else None),
            'velocity_ned': [round(v, 3) for v in command],
        }
