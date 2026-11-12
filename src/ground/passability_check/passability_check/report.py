"""报告文本、字符剖面图、断言语义与批量对比表（纯标准库）。"""

import math

from .clearance import scan_profile, lower_contour_world

ASSERT_NAMES = ("接近角", "离去角", "纵向通过角", "离地间隙", "全程不托底")

# 结论的三档取值。数值判据（离地间隙、全程不托底）是沿真实剖面逐位姿扫描，
# 角度判据是「无限长斜面」的保守近似 —— 两者冲突时以数值判据为准，
# 角度判据不通过只降级成备注，不改写结论。
VERDICT_PASSED = "PASSED"
VERDICT_PASS_WITH_NOTE = "PASS_WITH_NOTE"
VERDICT_FAILED = "FAILED"

TOL = 1e-6


def _fmt_deg(value):
    return "%.2f°" % value


def build_assertions(vehicle, scenario, report):
    """五条断言。前三条是角度判据（由剖面折角自动得出），后两条是数值判据。"""
    terrain = scenario.terrain
    need_concave = terrain.max_concave_deg()
    need_convex = terrain.max_convex_deg()
    margin = scenario.safety_margin_m
    results = []

    passed = vehicle.approach_deg + TOL >= need_concave
    results.append({
        "type": ASSERT_NAMES[0],
        "passed": passed,
        "value": vehicle.approach_deg,
        "need": need_concave,
        "detail": "实测 %s，剖面最大下凹折角 %s（差 %s）"
                  % (_fmt_deg(vehicle.approach_deg), _fmt_deg(need_concave),
                     _fmt_deg(vehicle.approach_deg - need_concave)),
    })

    passed = vehicle.departure_deg + TOL >= need_concave
    results.append({
        "type": ASSERT_NAMES[1],
        "passed": passed,
        "value": vehicle.departure_deg,
        "need": need_concave,
        "detail": "实测 %s，反向（倒车）遇到同一条下凹折角 %s（差 %s）"
                  % (_fmt_deg(vehicle.departure_deg), _fmt_deg(need_concave),
                     _fmt_deg(vehicle.departure_deg - need_concave)),
    })

    passed = vehicle.breakover_deg + TOL >= need_convex
    results.append({
        "type": ASSERT_NAMES[2],
        "passed": passed,
        "value": vehicle.breakover_deg,
        "need": need_convex,
        "detail": "实测 %s，剖面最大上凸折角 %s（差 %s）"
                  % (_fmt_deg(vehicle.breakover_deg), _fmt_deg(need_convex),
                     _fmt_deg(vehicle.breakover_deg - need_convex)),
    })

    need_clearance = scenario.min_clearance_m + margin
    passed = vehicle.min_clearance + TOL >= need_clearance
    results.append({
        "type": ASSERT_NAMES[3],
        "passed": passed,
        "value": vehicle.min_clearance,
        "need": need_clearance,
        "detail": "最低离地 %.3f m，要求 >= %.3f m（差 %+.3f m）"
                  % (vehicle.min_clearance, need_clearance,
                     vehicle.min_clearance - need_clearance),
    })

    worst = report["worst"]
    need_margin = margin
    passed = report["min_margin"] + TOL >= need_margin
    results.append({
        "type": ASSERT_NAMES[4],
        "passed": passed,
        "value": report["min_margin"],
        "need": need_margin,
        "detail": "最小余量 %+.3f m（要求 >= %+.3f m），最大侵入 %.3f m 位于%s x=%.2f m"
                  % (report["min_margin"], need_margin, report["max_depth"],
                     worst["section"], worst["x"]),
    })
    return results


def angle_passed(assertions):
    """三条角度判据是否全通过（保守近似，可能对短坡面偏严）。"""

    return all(item["passed"] for item in assertions[:3])


def numeric_passed(assertions):
    """离地间隙与全程不托底两条数值判据是否全通过（结论的唯一依据）。"""

    return all(item["passed"] for item in assertions[3:])


def all_passed(assertions):
    """五条断言是否全部通过（= 结论为 PASSED）。"""

    return all(item["passed"] for item in assertions)


def verdict(assertions):
    """由断言给出结论。

    * 数值判据不通过 → ``FAILED``：沿这条剖面逐位姿扫描确实托底/余量不足；
    * 数值通过、角度判据不通过 → ``PASS_WITH_NOTE``：能过，但角度参数在
      这条剖面上偏保守（通常是短坡面：车头还没完全爬上坡面就已经过去了），
      需要人看一眼备注；
    * 全部通过 → ``PASSED``。

    以前 ``result`` 用「五条全过」，于是出现「结论 FAILED、备注说能过」的
    自相矛盾（评审 @KKK555-MAX 指出）。现在结论字段与备注口径一致。
    """

    if not numeric_passed(assertions):
        return VERDICT_FAILED
    return VERDICT_PASSED if angle_passed(assertions) else VERDICT_PASS_WITH_NOTE


def _breaks_text(terrain):
    breaks = terrain.breaks()
    if not breaks:
        return "折点：无（剖面是单一坡面或平地）"
    parts = []
    for item in breaks:
        kind = "下凹" if item["kind"] == "concave" else "上凸"
        parts.append("x=%.2f m %s %s" % (item["x"], kind, _fmt_deg(abs(item["delta_deg"]))))
    return "折点：" + "；".join(parts)


def format_report(vehicle, scenario, report, assertions, header=""):
    """把一次核算结果排版成终端报告（纯文本，可直接贴进 issue / PR）。"""
    worst = report["worst"]
    lines = []
    if header:
        lines.append(header)
    lines.append("车辆：%s，轴距 %.2f m，前悬 %.2f m，后悬 %.2f m，车长 %.2f m，车宽 %.2f m"
                 % (vehicle.name, vehicle.wheelbase, vehicle.front_overhang,
                    vehicle.rear_overhang, vehicle.length, vehicle.width))
    lines.append("离地：车头 %.3f m / 车底 %.3f m（轴距 %.0f%% 处）/ 车尾 %.3f m，最低 %.3f m"
                 % (vehicle.clearance_front, vehicle.clearance_mid,
                    vehicle.mid_ratio * 100.0, vehicle.clearance_rear,
                    vehicle.min_clearance))
    lines.append("场景：%s（%s）" % (scenario.title, scenario.name))
    up_deg, down_deg = _slope_extreme(scenario.terrain)
    lines.append("剖面：%.2f m ~ %.2f m，%d 个点，最大上坡 %s / 最大下坡 %s"
                 % (scenario.terrain.min_x, scenario.terrain.max_x,
                    len(scenario.terrain.points), _fmt_deg(up_deg), _fmt_deg(abs(down_deg))))
    lines.append(_breaks_text(scenario.terrain))
    lines.append("角度：接近角 %s，离去角 %s，纵向通过角 %s"
                 % (_fmt_deg(vehicle.approach_deg), _fmt_deg(vehicle.departure_deg),
                    _fmt_deg(vehicle.breakover_deg)))
    lines.append("采样：下缘 %d 点 × %d 档位姿（步长 %.3f m），俯仰 %+.2f° ~ %+.2f°"
                 % (report["samples"], report["steps"], report["step"],
                    report["pitch_min_deg"], report["pitch_max_deg"]))
    lines.append("侵入：最大 %.3f m（%s x=%.2f m），最小余量 %+.3f m"
                 % (report["max_depth"], worst["section"], worst["x"], report["min_margin"]))
    passed = sum(1 for item in assertions if item["passed"])
    lines.append("断言：%d 通过 / %d 失败（共 %d 条）"
                 % (passed, len(assertions) - passed, len(assertions)))
    for item in assertions:
        lines.append("  [%s] %-6s %s" % ("PASS" if item["passed"] else "FAIL",
                                         item["type"], item["detail"]))
    lines.append("结论：%s%s" % (verdict(assertions), _verdict_note(assertions)))
    note = _conservative_note(vehicle, scenario, assertions)
    if note:
        lines.append("备注：%s" % note)
    return "\n".join(lines)


def _verdict_note(assertions):
    """结论后缀：PASS_WITH_NOTE 时一句话说明它为什么不是 FAILED。"""

    if verdict(assertions) != VERDICT_PASS_WITH_NOTE:
        return ""
    failed = [item["type"] for item in assertions[:3] if not item["passed"]]
    return "（数值复核能过，角度判据偏保守：%s；详见备注）" % "、".join(failed)


def _shortest_slope_m(terrain):
    """剖面里最短的一段坡面长度（米）。减速带的坡面只有 0.35 m。"""
    lengths = []
    points = terrain.points
    for index in range(len(points) - 1):
        x0, z0 = points[index]
        x1, z1 = points[index + 1]
        if abs(z1 - z0) > 1e-9:
            lengths.append(math.hypot(x1 - x0, z1 - z0))
    return min(lengths) if lengths else 0.0


def _conservative_note(vehicle, scenario, assertions):
    """角度判据不通过、数值判据通过时，说明为什么（短坡面会让角度判据偏保守）。"""
    angle_failed = [item["type"] for item in assertions[:3] if not item["passed"]]
    numeric_ok = all(item["passed"] for item in assertions[3:])
    if not angle_failed or not numeric_ok:
        return ""
    shortest = _shortest_slope_m(scenario.terrain)
    return ("角度判据（%s）不通过，但沿这条剖面数值复核显示能过：剖面里最短的坡面只有 %.2f m，"
            "比前悬 %.2f m 还短，车头还没完全爬上坡面就已经过去了，无限长斜面判据在这种短坡面上"
            "偏保守，结论以数值复核为准。"
            % ("、".join(angle_failed), shortest, vehicle.front_overhang))


def _slope_extreme(terrain):
    """剖面里最陡的正/负坡度（度）。"""
    slopes = [math.degrees(terrain.slope(x)) for x, _ in terrain.points]
    return max(slopes), min(slopes)


# ------------------------------------------------------------------ 字符剖面图


def _interp_x(points, x):
    """在按 x 单调排列的点列里按 x 线性插值取 z。"""
    if not points:
        return None
    if x < points[0][0] or x > points[-1][0]:
        return None
    for index in range(len(points) - 1):
        x0, z0 = points[index]
        x1, z1 = points[index + 1]
        if x0 - 1e-9 <= x <= x1 + 1e-9:
            if abs(x1 - x0) < 1e-12:
                return z0
            ratio = (x - x0) / (x1 - x0)
            return z0 + (z1 - z0) * ratio
    return points[-1][1]


def render_profile(terrain, vehicle, pose, worst=None, width=92, height=20, pad=0.8):
    """把某一位姿下的车与剖面画成字符图。

    图例：`#` 地面，`o` 车身，`@` 车轮，`X` 侵入点（托底处），
    `^` 下凹折点（车头/车尾的风险点），`v` 上凸折点（车底中部的风险点）。
    """
    width = max(40, int(width))
    height = max(8, int(height))

    lower_pts = [(item["x"], item["z"]) for item in lower_contour_world(vehicle, pose, 41)]
    upper_pts = [_rotate(pose, x_v, z_v) for x_v, z_v in vehicle.body_outline()]

    # 窗口：以车身为中心，左右各留出一点上下文（坡底/坡顶要能看见）
    # 字符单元格宽高比约 1:2，纵向一格 ≈ 横向两格，才看得出真实倾角
    center = 0.5 * ((pose["rear_x"] - vehicle.rear_overhang)
                    + (pose["front_x"] + vehicle.front_overhang))
    half = max(3.0, 0.4 * vehicle.length + 1.0)
    # 剖面两端按水平延长，所以窗口越界也不会取到空值
    x0 = center - half
    x1 = center + half
    dx = (x1 - x0) / width
    natural_dz = dx * 2.0   # 字符单元格约 1:2，纵向一格 ≈ 横向两格
    terrain_low = min(terrain.height(x0 + (x1 - x0) * index / 32.0) for index in range(33))
    terrain_high = max(terrain.height(x0 + (x1 - x0) * index / 32.0) for index in range(33))
    content = max(terrain_high, pose["rear_z"] + vehicle.height) - min(
        terrain_low, pose["rear_z"], pose["front_z"])
    # 内容太矮时适度纵向放大，最多 1.6 倍（减速带只有 10 cm，不放大看不出来）
    dz = max(natural_dz / 1.6, min(natural_dz, content / float(height - 2)))
    z_bottom = min(terrain_low, pose["rear_z"], pose["front_z"]) - 0.12
    z_top = z_bottom + dz * height

    grid = [[" "] * width for _ in range(height)]

    def row_of(z):
        return int(round((z_top - z) / dz))

    def col_of(x):
        return int(round((x - x0) / dx))

    def put(col, row, char, only_empty=False):
        if 0 <= col < width and 0 <= row < height:
            if only_empty and grid[row][col] != " ":
                return
            grid[row][col] = char

    # 地面
    for col in range(width):
        surface = terrain.height(x0 + col * dx)
        for row in range(max(0, row_of(surface)), height):
            grid[row][col] = "#"

    # 车身：上轮廓到车身下缘之间的实心剖面
    for col in range(width):
        x = x0 + col * dx
        low = _interp_x(lower_pts, x)
        high = _interp_x(upper_pts, x)
        if low is None or high is None:
            continue
        for row in range(max(0, row_of(high)), min(height, row_of(low) + 1)):
            grid[row][col] = "o"

    # 车轮：0.62 m 直径的近似圆块
    for axle_x, axle_z in ((pose["rear_x"], pose["rear_z"]),
                           (pose["front_x"], pose["front_z"])):
        center_z = axle_z + 0.31
        for d_row in (-1, 0, 1):
            for d_col in (-2, -1, 0, 1, 2):
                if abs(d_row) == 1 and abs(d_col) == 2:
                    continue
                put(col_of(axle_x + d_col * dx), row_of(center_z + d_row * dz * 0.5), "@")

    # 折点标记：只画在空白处，别盖住车身
    markers = {"concave": "^", "convex": "v"}
    for item in terrain.breaks():
        if x0 <= item["x"] <= x1:
            put(col_of(item["x"]), row_of(terrain.height(item["x"])) - 1,
                markers[item["kind"]], only_empty=True)

    # 侵入点
    if worst is not None and worst.get("depth", 0.0) > 0.002:
        col, row = col_of(worst["x"]), row_of(worst["z"])
        put(col, row, "X")
        put(col, row + 1, "X")

    rows = ["".join(row).rstrip() for row in grid]
    while rows and not rows[0].strip():
        rows.pop(0)
    while rows and not rows[-1].strip():
        rows.pop()
    rows.append("-" * width)
    rows.append("窗口 x ∈ [%.2f, %.2f] m，横向一格 %.3f m，纵向一格 %.3f m（纵向放大 %.1f×）；"
                "后轴在 x=%.2f m，俯仰 %+.2f°"
                % (x0, x1, dx, dz, natural_dz / dz, pose["rear_x"], pose["pitch_deg"]))
    rows.append("图例：# 地面  o 车身  @ 车轮  X 侵入点  ^ 下凹折点  v 上凸折点")
    return "\n".join(rows)


def _rotate(pose, x_v, z_v):
    cos_p = math.cos(pose["pitch"])
    sin_p = math.sin(pose["pitch"])
    return (pose["rear_x"] + x_v * cos_p - z_v * sin_p,
            pose["rear_z"] + x_v * sin_p + z_v * cos_p)


# ------------------------------------------------------------------ 批量输出


def scenario_matrix_csv(rows):
    """批量对比表（CSV 文本）。`rows` 是 dict 列表。"""
    fields = ("scenario", "vehicle", "approach_deg", "departure_deg", "breakover_deg",
              "min_clearance_m", "max_depth_m", "min_margin_m", "result")
    out = [",".join(fields)]
    for row in rows:
        out.append(",".join(str(row.get(key, "")) for key in fields))
    return "\n".join(out)


def to_json_payload(vehicle, scenario, report, assertions):
    """给节点/外部工具用的 JSON 结构。"""
    worst = report["worst"]
    return {
        "vehicle": {
            "name": vehicle.name,
            "wheelbase": vehicle.wheelbase,
            "front_overhang": vehicle.front_overhang,
            "rear_overhang": vehicle.rear_overhang,
            "clearance_front": vehicle.clearance_front,
            "clearance_mid": vehicle.clearance_mid,
            "clearance_rear": vehicle.clearance_rear,
            "min_clearance": vehicle.min_clearance,
            "approach_deg": vehicle.approach_deg,
            "departure_deg": vehicle.departure_deg,
            "breakover_deg": vehicle.breakover_deg,
        },
        "scenario": {
            "name": scenario.name,
            "title": scenario.title,
            "min_clearance_m": scenario.min_clearance_m,
            "safety_margin_m": scenario.safety_margin_m,
            "max_concave_deg": scenario.terrain.max_concave_deg(),
            "max_convex_deg": scenario.terrain.max_convex_deg(),
        },
        "scan": {
            "steps": report["steps"],
            "step_m": report["step"],
            "max_depth_m": report["max_depth"],
            "min_margin_m": report["min_margin"],
            "worst_section": worst["section"],
            "worst_x": worst["x"],
            "pitch_min_deg": report["pitch_min_deg"],
            "pitch_max_deg": report["pitch_max_deg"],
        },
        "assertions": [
            {"type": item["type"], "passed": item["passed"], "detail": item["detail"]}
            for item in assertions
        ],
        "result": verdict(assertions),
        "numeric_ok": numeric_passed(assertions),
        "angle_ok": angle_passed(assertions),
    }


def evaluate(vehicle, scenario, samples=41, steps=480):
    """一次完整核算：扫描 → 断言。返回 (report, assertions)。"""
    report = scan_profile(scenario.terrain, vehicle, samples=samples, steps=steps)
    assertions = build_assertions(vehicle, scenario, report)
    return report, assertions
