"""ROV 浮力-压载配平与初稳性核算（只依赖标准库）.

下水前要先算清楚三件事，本模块把它们写成可以手算核对的公式：

1. **中性浮力**：排水质量减去干重，就是必须挂上去的压载质量；
2. **配平**：压载挂在哪个位置，才能把重心拉回浮心正下方，使横倾和纵倾归零；
3. **初稳性**：初稳心高 GM、横摇周期与可承受的扰动力矩，判断这套配重能不能作业。

全潜 ROV 与水线面船只的区别（容易算错的地方）：

    水面船   GM = BM - (z_G - z_B),  BM = I_waterplane / V
    全潜 ROV GM = z_B - z_G                     没有 BM 项

ROV 整体没入水中，横倾时排水体积形状不变，浮心相对船体不动，恢复力矩只来自
"重心低于浮心"这一条，即 GZ = (z_B - z_G) * sin(phi)，对任意倾角都成立。
把水面船那套 BM = I/V 直接套到 ROV 上会算出虚高的稳心高。

符号与公式：

    W = m*g,            B = rho*V*g,        F_net = B - W
    GM = z_B - z_G                          (m)
    tan(phi)   = (y_G - y_B) / GM           (rad 转 deg 输出)
    tan(theta) = (x_G - x_B) / GM
    T_roll = 2*pi*sqrt(I_xx / (Delta*GM)),  I_xx = m*(beam^2 + height^2)/12
    M_allow = Delta*GM*sin(phi_limit)       (允许扰动力矩, N*m)

约定：x 轴指向船艏，y 轴指向右舷，z 轴向上，原点在船体外壳几何中心。
"""

import math
from dataclasses import dataclass

G = 9.80665                     # 重力加速度 m/s^2
RHO_NOMINAL = 1027.0            # 10 C、35 psu 表层海水密度 kg/m^3
TEMP_NOMINAL = 10.0             # 参考温度 C
SALINITY_NOMINAL = 35.0         # 参考盐度 psu

GM_MIN = 0.02                   # 初稳心高告警门限 m
GM_RECOMMENDED = 0.05           # 初稳心高建议值 m
TRIM_LIMIT_DEG = 2.0            # 静止作业允许的初始倾角 deg
DISTURB_LIMIT_DEG = 5.0         # 作业中被外力扰动允许的倾角 deg
NET_BUOYANCY_TOL = 0.01         # 净浮力允许偏差（相对重量）
ADDED_MASS_FACTOR = 1.3         # 横摇附加惯量系数（水下附加质量典型 1.2~1.5）

DEFAULT_COMPONENTS = (
    ('frame', 85.0, 0.00, 0.00, 0.02),
    ('battery', 60.0, -0.15, 0.00, -0.08),
    ('thruster_set', 45.0, 0.05, 0.00, 0.00),
    ('electronics', 25.0, 0.10, 0.00, 0.10),
    ('manipulator', 50.0, 0.25, 0.02, -0.05),
    ('tool_skid', 20.0, 0.05, 0.00, -0.18),
)


def _require(condition, message):
    """参数检查：不满足就抛 ValueError，避免把错误参数算出一个"好看"的数."""
    if not condition:
        raise ValueError(message)


def _is_finite(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


@dataclass(frozen=True)
class Component:
    """一个重量部件：质量与它在船体坐标下的重心位置."""

    name: str
    mass_kg: float
    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 0.0

    def __post_init__(self):
        _require(isinstance(self.name, str) and self.name != '', '部件名不能为空')
        _require(_is_finite(self.mass_kg) and self.mass_kg > 0.0,
                 '部件 %s 的质量必须为正数' % self.name)
        for axis, value in (('x', self.x_m), ('y', self.y_m), ('z', self.z_m)):
            _require(_is_finite(value), '部件 %s 的 %s 坐标必须是有限数' % (self.name, axis))


def water_density(temp_c=TEMP_NOMINAL, salinity_psu=SALINITY_NOMINAL, depth_m=0.0):
    """海水密度的一阶近似 kg/m^3.

    rho = 1027.0 - 0.20*(T-10) + 0.80*(S-35) + 0.0046*depth

    依据：温度每升高 1 C 密度降约 0.2 kg/m^3；盐度每升高 1 psu 升约 0.8 kg/m^3；
    海水压缩性约 4.6e-5 /bar，1 bar 约 10 m 水深，故每米约 +0.0046 kg/m^3。
    在 0~30 C、30~38 psu、0~6000 m 范围内与 UNESCO 状态方程偏差小于 0.3%。
    """
    _require(_is_finite(temp_c) and -2.0 <= temp_c <= 40.0,
             '温度需在 -2~40 C 之间')
    _require(_is_finite(salinity_psu) and 0.0 <= salinity_psu <= 45.0,
             '盐度需在 0~45 psu 之间')
    _require(_is_finite(depth_m) and 0.0 <= depth_m <= 11000.0,
             '深度需在 0~11000 m 之间')
    return (RHO_NOMINAL
            - 0.20 * (temp_c - TEMP_NOMINAL)
            + 0.80 * (salinity_psu - SALINITY_NOMINAL)
            + 0.0046 * depth_m)


def build_components(rows):
    """把 (name, mass, x, y, z) 元组表转成 Component 列表."""
    _require(rows, '至少需要一个部件')
    return [Component(*row) for row in rows]


def cog(components):
    """返回整体干重重心 (x, y, z) 与总质量 kg."""
    _require(components, '至少需要一个部件')
    total = sum(item.mass_kg for item in components)
    _require(total > 0.0, '总质量必须为正数')
    x = sum(item.mass_kg * item.x_m for item in components) / total
    y = sum(item.mass_kg * item.y_m for item in components) / total
    z = sum(item.mass_kg * item.z_m for item in components) / total
    return x, y, z, total


def trim_angle_deg(offset_m, gm_m):
    """由重心-浮心偏心量与初稳心高求平衡倾角 deg.

    tan(angle) = offset / GM，GM 不为正时视为失稳，返回 90 deg。
    """
    _require(_is_finite(offset_m), '偏心量必须是有限数')
    _require(_is_finite(gm_m), '初稳心高必须是有限数')
    if gm_m <= 1e-9:
        return 90.0 if offset_m >= 0.0 else -90.0
    return math.degrees(math.atan2(offset_m, gm_m))


def gm_level(gm_m):
    """初稳心高的三档判定：unstable / marginal / ok."""
    _require(_is_finite(gm_m), '初稳心高必须是有限数')
    if gm_m <= 0.0:
        return 'unstable'
    if gm_m < GM_MIN:
        return 'unstable'
    if gm_m < GM_RECOMMENDED:
        return 'marginal'
    return 'ok'


def roll_period_s(mass_kg, gm_m, beam_m, height_m, added_mass_factor=ADDED_MASS_FACTOR):
    """横摇固有周期 s：T = 2*pi*sqrt(I_xx/(m*g*GM)).

    I_xx 按均匀长方体取 m*(beam^2+height^2)/12，再乘附加惯量系数。
    """
    _require(_is_finite(mass_kg) and mass_kg > 0.0, '总质量必须为正数')
    _require(_is_finite(beam_m) and beam_m > 0.0, '船宽必须为正数')
    _require(_is_finite(height_m) and height_m > 0.0, '船高必须为正数')
    _require(_is_finite(added_mass_factor) and added_mass_factor >= 1.0,
             '附加惯量系数需不小于 1.0')
    if gm_m <= 1e-9:
        raise ValueError('初稳心高不为正，无法定义横摇周期')
    inertia = mass_kg * (beam_m ** 2 + height_m ** 2) / 12.0 * added_mass_factor
    omega_sq = mass_kg * G * gm_m / inertia
    _require(omega_sq > 0.0, '横摇频率必须为正数')
    return 2.0 * math.pi / math.sqrt(omega_sq)


class RovTrim:
    """一台全潜 ROV 的浮力、配平与初稳性核算.

    默认参数：0.90 x 0.70 x 0.50 m 的长方体外壳（排水 0.315 m^3 约 323 kg），
    六个部件合计 285 kg，压载托盘在船底 z=-0.18 m、限位 x=+-0.40 m、y=+-0.30 m。
    """

    def __init__(self, length_m=0.90, beam_m=0.70, height_m=0.50,
                 components=None, density=None, gravity=G,
                 center_of_buoyancy=(0.0, 0.0, 0.0),
                 tray_limit_x_m=0.40, tray_limit_y_m=0.30, tray_z_m=-0.18,
                 added_mass_factor=ADDED_MASS_FACTOR,
                 trim_limit_deg=TRIM_LIMIT_DEG):
        for name, value in (('length_m', length_m), ('beam_m', beam_m),
                            ('height_m', height_m), ('gravity', gravity)):
            _require(_is_finite(value) and value > 0.0, '%s 必须为正数' % name)
        self.length_m = float(length_m)
        self.beam_m = float(beam_m)
        self.height_m = float(height_m)
        self.gravity = float(gravity)
        self.density = float(water_density() if density is None else density)
        _require(self.density > 0.0, '密度必须为正数')

        self.components = build_components(components or DEFAULT_COMPONENTS)
        x_b, y_b, z_b = center_of_buoyancy
        for name, value in (('x', x_b), ('y', y_b), ('z', z_b)):
            _require(_is_finite(value), '浮心 %s 坐标必须是有限数' % name)
        self.center_of_buoyancy = (float(x_b), float(y_b), float(z_b))

        _require(_is_finite(tray_limit_x_m) and tray_limit_x_m >= 0.0,
                 '托盘纵向限位需为非负数')
        _require(_is_finite(tray_limit_y_m) and tray_limit_y_m >= 0.0,
                 '托盘横向限位需为非负数')
        _require(_is_finite(tray_z_m), '托盘高度必须是有限数')
        _require(abs(tray_z_m) <= self.height_m / 2.0 + 1e-9,
                 '托盘高度必须落在外壳高度范围内')
        _require(_is_finite(added_mass_factor) and added_mass_factor >= 1.0,
                 '附加惯量系数需不小于 1.0')
        _require(_is_finite(trim_limit_deg) and trim_limit_deg > 0.0,
                 '倾角门限必须为正数')

        self.tray_limit_x_m = float(tray_limit_x_m)
        self.tray_limit_y_m = float(tray_limit_y_m)
        self.tray_z_m = float(tray_z_m)
        self.added_mass_factor = float(added_mass_factor)
        self.trim_limit_deg = float(trim_limit_deg)

        # 干重（不含压载）的静力学量
        x_g, y_g, z_g, mass = cog(self.components)
        self.dry_mass_kg = mass
        self.cog_dry = (x_g, y_g, z_g)
        self.volume_m3 = self.length_m * self.beam_m * self.height_m
        self.displacement_mass_kg = self.density * self.volume_m3
        self.weight_dry_n = self.dry_mass_kg * self.gravity
        self.buoyancy_n = self.displacement_mass_kg * self.gravity
        self.net_buoyancy_n = self.buoyancy_n - self.weight_dry_n
        self.required_ballast_kg = self.displacement_mass_kg - self.dry_mass_kg

    # ------------------------------------------------------------------ 静力学
    def gm_m(self, cog_z=None):
        """初稳心高 GM = z_B - z_G；全潜状态没有 BM 项."""
        z_g = self.cog_dry[2] if cog_z is None else cog_z
        return self.center_of_buoyancy[2] - z_g

    def trim(self, cog_xyz):
        """给定重心位置，返回横倾/纵倾角 (deg) 与 GM (m)."""
        gm = self.gm_m(cog_z=cog_xyz[2])
        roll = trim_angle_deg(cog_xyz[1] - self.center_of_buoyancy[1], gm)
        pitch = trim_angle_deg(cog_xyz[0] - self.center_of_buoyancy[0], gm)
        return roll, pitch, gm

    def combined_cog(self, ballast_kg=0.0, ballast_x=0.0, ballast_y=0.0, ballast_z=None):
        """干重 + 压载的合成重心."""
        _require(_is_finite(ballast_kg) and ballast_kg >= 0.0, '压载质量不能为负')
        if ballast_kg == 0.0:
            return self.cog_dry
        z_b = self.tray_z_m if ballast_z is None else ballast_z
        total = self.dry_mass_kg + ballast_kg
        x_g, y_g, z_g = self.cog_dry
        return ((self.dry_mass_kg * x_g + ballast_kg * ballast_x) / total,
                (self.dry_mass_kg * y_g + ballast_kg * ballast_y) / total,
                (self.dry_mass_kg * z_g + ballast_kg * z_b) / total)

    def state(self, ballast_kg=0.0, ballast_x=0.0, ballast_y=0.0, ballast_z=None):
        """某一压载方案下的完整状态（质量、重心、倾角、GM、横摇周期）."""
        cog_xyz = self.combined_cog(ballast_kg, ballast_x, ballast_y, ballast_z)
        mass = self.dry_mass_kg + ballast_kg
        roll, pitch, gm = self.trim(cog_xyz)
        if gm > 1e-9:
            period = roll_period_s(mass, gm, self.beam_m, self.height_m,
                                   self.added_mass_factor)
        else:
            period = None
        weight = mass * self.gravity
        net = self.buoyancy_n - weight
        return {
            'mass_kg': mass,
            'cog': {'x_m': cog_xyz[0], 'y_m': cog_xyz[1], 'z_m': cog_xyz[2]},
            'weight_n': weight,
            'net_buoyancy_n': net,
            'net_buoyancy_pct': net / weight * 100.0 if weight > 0.0 else 0.0,
            'gm_m': gm,
            'gm_level': gm_level(gm),
            'roll_deg': roll,
            'pitch_deg': pitch,
            'roll_period_s': period,
            'max_trim_deg': max(abs(roll), abs(pitch)),
        }

    # ------------------------------------------------------------------ 反解
    def ballast_plan(self, target_gm_m=None):
        """反解压载质量与安装位置：中性浮力 + 倾角归零 + （可选）目标 GM.

        质量由中性浮力直接给出；x/y 由力矩平衡给出；z 由目标 GM 反解。
        """
        mass = self.required_ballast_kg
        plan = {
            'mass_kg': mass,
            'feasible': False,
            'reason': '',
            'x_required_m': None,
            'y_required_m': None,
            'z_required_m': None,
            'x_m': None,
            'y_m': None,
            'z_m': None,
            'target_gm_m': target_gm_m,
        }
        if target_gm_m is not None:
            _require(_is_finite(target_gm_m) and target_gm_m > 0.0,
                     '目标 GM 必须为正数')
        if mass <= 0.0:
            plan['mass_kg'] = 0.0
            plan['reason'] = ('干重已超过排水量 %.1f kg，需要 %.1f kg 浮力块而不是压载'
                              % (self.displacement_mass_kg, -mass))
            return plan

        x_g, y_g, z_g = self.cog_dry
        x_req = -self.dry_mass_kg * x_g / mass
        y_req = -self.dry_mass_kg * y_g / mass
        if target_gm_m is None:
            z_req = self.tray_z_m
        else:
            z_g_needed = self.center_of_buoyancy[2] - target_gm_m
            z_req = (mass + self.dry_mass_kg) * z_g_needed - self.dry_mass_kg * z_g
            z_req /= mass
        plan.update({'x_required_m': x_req, 'y_required_m': y_req,
                     'z_required_m': z_req})

        x_ok = abs(x_req) <= self.tray_limit_x_m + 1e-9
        y_ok = abs(y_req) <= self.tray_limit_y_m + 1e-9
        z_min, z_max = -self.height_m / 2.0, self.height_m / 2.0
        z_ok = z_min - 1e-9 <= z_req <= z_max + 1e-9
        plan['feasible'] = bool(x_ok and y_ok and z_ok)

        x_use = min(max(x_req, -self.tray_limit_x_m), self.tray_limit_x_m)
        y_use = min(max(y_req, -self.tray_limit_y_m), self.tray_limit_y_m)
        z_use = min(max(z_req, z_min), z_max)
        plan.update({'x_m': x_use, 'y_m': y_use, 'z_m': z_use})

        if not plan['feasible']:
            reasons = []
            if not x_ok:
                reasons.append('纵向需要 %.3f m，超出托盘限位 +-%.3f m'
                               % (x_req, self.tray_limit_x_m))
            if not y_ok:
                reasons.append('横向需要 %.3f m，超出托盘限位 +-%.3f m'
                               % (y_req, self.tray_limit_y_m))
            if not z_ok:
                reasons.append('高度需要 %.3f m，超出外壳范围 +-%.3f m'
                               % (z_req, self.height_m / 2.0))
            plan['reason'] = '；'.join(reasons)

        after = self.state(mass, x_use, y_use, z_use)
        plan['state'] = after
        plan['residual_trim_deg'] = after['max_trim_deg']
        plan['gm_after_m'] = after['gm_m']
        if plan['feasible'] and abs(after['max_trim_deg']) > 1e-6:
            plan['feasible'] = False
            plan['reason'] = '限位裁剪后仍有 %.3f deg 残余倾角' % after['max_trim_deg']
        return plan

    def offset_for_trim(self, gm_m, angle_deg=None):
        """给定 GM，反解产生 angle_deg 倾角所需的横向偏心量 m."""
        limit = self.trim_limit_deg if angle_deg is None else angle_deg
        _require(_is_finite(limit) and limit > 0.0, '倾角必须为正数')
        _require(gm_m > 1e-9, '初稳心高不为正，无法反解偏心量')
        return gm_m * math.tan(math.radians(min(limit, 89.9)))

    def max_disturbance_moment(self, state, angle_deg=None):
        """在倾角门限内可承受的最大扰动力矩 N*m: M = Delta*GM*sin(phi_limit)."""
        limit = DISTURB_LIMIT_DEG if angle_deg is None else angle_deg
        _require(_is_finite(limit) and limit > 0.0, '倾角必须为正数')
        _require(state['gm_m'] > 1e-9, '初稳心高不为正，无法给出允许扰动力矩')
        delta = state['mass_kg'] * self.gravity
        return delta * state['gm_m'] * math.sin(math.radians(limit))

    def depth_sweep(self, depths_m=(0.0, 100.0, 1000.0, 3000.0, 6000.0)):
        """不同深度的海水密度与浮力漂移：深水变"轻"多少，需要补多少压载."""
        rows = []
        for depth in depths_m:
            _require(_is_finite(depth) and depth >= 0.0, '深度必须为非负数')
            rho = water_density(depth_m=depth)
            buoyancy = rho * self.volume_m3 * self.gravity
            net = buoyancy - self.state(self.required_ballast_kg, 0.0, 0.0,
                                        self.tray_z_m)['weight_n']
            rows.append({
                'depth_m': depth,
                'density_kg_m3': rho,
                'buoyancy_n': buoyancy,
                'net_buoyancy_n': net,
                'extra_ballast_kg': net / self.gravity,
            })
        return rows

    # ------------------------------------------------------------------ 报告
    def report(self, target_gm_m=None):
        """汇总成可直接写 JSON 的报告."""
        x_g, y_g, z_g = self.cog_dry
        x_b, y_b, z_b = self.center_of_buoyancy
        dry_state = self.state(0.0)
        plan = self.ballast_plan(target_gm_m=target_gm_m)
        warnings = []

        if self.required_ballast_kg <= 0.0:
            warnings.append('干重超过排水量，需要加浮力块而不是压载')
        elif abs(dry_state['net_buoyancy_pct']) > NET_BUOYANCY_TOL * 100.0:
            warnings.append('未加压载时净浮力 %.1f N（%.1f%%），下水后会上浮 %.1f kg'
                            % (dry_state['net_buoyancy_n'],
                               dry_state['net_buoyancy_pct'],
                               dry_state['net_buoyancy_n'] / self.gravity))
        if abs(dry_state['roll_deg']) > self.trim_limit_deg \
                or abs(dry_state['pitch_deg']) > self.trim_limit_deg:
            warnings.append('空载横倾 %.1f deg / 纵倾 %.1f deg，超过 %.1f deg 门限'
                            % (dry_state['roll_deg'], dry_state['pitch_deg'],
                               self.trim_limit_deg))

        after = plan.get('state')
        if after:
            if after['gm_level'] == 'unstable':
                warnings.append('配平后初稳心高 %.3f m 低于 %.3f m，横倾会自行发散'
                                % (after['gm_m'], GM_MIN))
            elif after['gm_level'] == 'marginal':
                warnings.append('配平后初稳心高 %.3f m 低于建议值 %.3f m，建议压载下移'
                                % (after['gm_m'], GM_RECOMMENDED))
            if after['max_trim_deg'] > self.trim_limit_deg:
                warnings.append('配平后残余倾角 %.2f deg，超过 %.1f deg 门限'
                                % (after['max_trim_deg'], self.trim_limit_deg))
            if not plan['feasible']:
                warnings.append('压载方案不可行：%s' % plan['reason'])
            else:
                warnings.append('压载 %.2f kg 装在 (x=%.3f, y=%.3f, z=%.3f) m，'
                                '倾角归零，GM=%.3f m'
                                % (plan['mass_kg'], plan['x_m'], plan['y_m'],
                                   plan['z_m'], after['gm_m']))

        tolerance = None
        if after and after['gm_m'] > 1e-9:
            tolerance = {
                'trim_limit_deg': self.trim_limit_deg,
                'offset_m_for_trim': self.offset_for_trim(after['gm_m']),
                'disturb_limit_deg': DISTURB_LIMIT_DEG,
                'max_disturbance_moment_n_m': self.max_disturbance_moment(after),
            }

        return {
            'parameters': {
                'length_m': self.length_m,
                'beam_m': self.beam_m,
                'height_m': self.height_m,
                'volume_m3': self.volume_m3,
                'density_kg_m3': self.density,
                'gravity_m_s2': self.gravity,
                'tray_limit_x_m': self.tray_limit_x_m,
                'tray_limit_y_m': self.tray_limit_y_m,
                'tray_z_m': self.tray_z_m,
                'added_mass_factor': self.added_mass_factor,
            },
            'mass': {
                'dry_kg': self.dry_mass_kg,
                'displacement_kg': self.displacement_mass_kg,
                'required_ballast_kg': self.required_ballast_kg,
            },
            'buoyancy': {
                'buoyancy_n': self.buoyancy_n,
                'weight_dry_n': self.weight_dry_n,
                'net_buoyancy_n': dry_state['net_buoyancy_n'],
                'net_buoyancy_pct': dry_state['net_buoyancy_pct'],
            },
            'cog': {'x_m': x_g, 'y_m': y_g, 'z_m': z_g},
            'cob': {'x_m': x_b, 'y_m': y_b, 'z_m': z_b},
            'stability_dry': {
                'gm_m': dry_state['gm_m'],
                'gm_level': dry_state['gm_level'],
                'roll_period_s': dry_state['roll_period_s'],
            },
            'trim_dry': {
                'roll_deg': dry_state['roll_deg'],
                'pitch_deg': dry_state['pitch_deg'],
            },
            'ballast_plan': {key: value for key, value in plan.items() if key != 'state'},
            'state_after_ballast': after,
            'tolerance': tolerance,
            'depth_sweep': self.depth_sweep(),
            'warnings': warnings,
            'notes': [
                '全潜 ROV 没有水线面，GM = z_B - z_G，不含 BM = I/V 项',
                '倾角公式按净浮力为零（已配平）推导；未配平时垂向合力会另加分量，'
                '空载倾角只作参考',
                '横摇周期按均匀长方体惯量乘附加惯量系数 %.2f 估算' % self.added_mass_factor,
                '静水小角度静力学模型，不含自由液面、缆绳张力与推进器推力',
            ],
        }
