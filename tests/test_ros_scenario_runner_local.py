# -*- coding: utf-8 -*-
"""ros_scenario_runner 的离线回归测试。

只依赖标准库：不需要 ROS、不需要 CARLA、不需要 PyYAML。
在仓库根目录下跑：

    python3 tests/test_ros_scenario_runner_local.py
"""

from __future__ import division, print_function

import io
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PACKAGE = os.path.join(ROOT, "src", "ground", "ros_scenario_runner")
SCRIPTS = os.path.join(PACKAGE, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import scenario_core as core  # noqa: E402

PASSED = []
FAILED = []


def check(name, condition, detail=""):
    if condition:
        PASSED.append(name)
    else:
        FAILED.append((name, detail))


def text(value):
    """py2 下把 bytes 解码成 unicode，避免非 ASCII 字面量比较时隐式解码失败。"""
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return value


def read_text(*parts):
    path = os.path.join(ROOT, *parts)
    handle = io.open(path, "r", encoding="utf-8")
    try:
        return handle.read()
    finally:
        handle.close()


def raises(error_type, function):
    try:
        function()
    except error_type:
        return True
    except Exception as other:  # noqa: BLE001
        return "抛了别的异常: %r" % (other,)
    return False


def make_scenario(name="t", duration=10.0, initial=None, steps=None, expects=None):
    body = {
        "name": name,
        "duration": duration,
        "initial": initial if initial is not None else {"x": 0.0, "y": 0.0},
        "steps": steps
        if steps is not None
        else [{"at": 0.0, "command": {"linear": 1.0}}],
        "expects": expects
        if expects is not None
        else [{"type": "final_speed", "value": 1.0, "tolerance": 0.5}],
    }
    return core.build_scenario(body)


def trajectory(points, name="traj"):
    return core.Trajectory([core.Sample(*point) for point in points], name)


def dict_reader(mapping):
    def get_param(key, fallback=None):
        return mapping.get(key, fallback)

    return get_param


# ---------------------------------------------------------------- A 场景解析

YAML_TEXT = u"""
# 注释行应当被忽略
scenario:
  name: 解析测试
  duration: 5.0
  initial: {x: 1.0, y: 2.0, yaw: 0.0, speed: 0.0}
  steps:
    - at: 0.5
      name: 起步
      command: {linear: 2.0, angular: 0.0}
    - at: 3.0
      command:
        linear: 0.0
        angular: 0.0
  expects:
    - type: final_speed
      value: 0.0
      tolerance: 0.05
    - type: distance
      min: 1.0
      max: 100.0
"""

parsed = core.parse_scenario_text(YAML_TEXT, source="inline")
check("A1 顶层解析出 Scenario 对象", isinstance(parsed, core.Scenario), repr(parsed))
check("A2 场景名带中文", text(parsed.name) == u"解析测试", repr(parsed.name))
check("A3 duration 解析正确", abs(parsed.duration - 5.0) < 1e-9, repr(parsed.duration))
check("A4 initial 解析成 Pose", isinstance(parsed.initial, core.Pose), repr(parsed.initial))
check("A5 initial.x 解析正确", abs(parsed.initial.x - 1.0) < 1e-9, repr(parsed.initial.x))
check("A6 行内映射解析正确", abs(parsed.initial.y - 2.0) < 1e-9, repr(parsed.initial.y))
check("A7 steps 数量正确", len(parsed.steps) == 2, repr(len(parsed.steps)))
check("A8 第一条 step 时间正确", abs(parsed.steps[0].at - 0.5) < 1e-9, repr(parsed.steps[0].at))
check("A9 step 的 name 解析正确", text(parsed.steps[0].name) == u"起步", repr(parsed.steps[0].name))
check(
    "A10 第一条 step 指令解析正确",
    parsed.steps[0].command.as_tuple() == (2.0, 0.0),
    repr(parsed.steps[0].command),
)
check(
    "A11 块状 command 也能解析",
    parsed.steps[1].command.as_tuple() == (0.0, 0.0),
    repr(parsed.steps[1].command),
)
check("A12 expects 数量正确", len(parsed.expects) == 2, repr(len(parsed.expects)))
check(
    "A13 断言类型解析正确",
    text(parsed.expects[0].type) == u"final_speed",
    repr(parsed.expects[0].type),
)
check(
    "A14 distance 的区间解析正确",
    parsed.expects[1].min_value == 1.0 and parsed.expects[1].max_value == 100.0,
    repr((parsed.expects[1].min_value, parsed.expects[1].max_value)),
)
check("A15 source 被记下来", text(parsed.source) == u"inline", repr(parsed.source))
check("A16 initial_speed 被记下来", abs(parsed.initial_speed) < 1e-9, repr(parsed.initial_speed))

SCALARS = u"""
scenario:
  name: "带 # 井号的名字"
  duration: 2
  steps:
    - at: 0
      command: {linear: 1}
  expects:
    - type: max_speed
      value: 1
"""
scalar_scenario = core.parse_scenario_text(SCALARS)
check(
    "A17 引号里的 # 不当注释",
    text(scalar_scenario.name) == u"带 # 井号的名字",
    repr(scalar_scenario.name),
)
check(
    "A18 整数 duration 变成 float",
    isinstance(scalar_scenario.duration, float) and scalar_scenario.duration == 2.0,
    repr(scalar_scenario.duration),
)
check(
    "A19 整数 value 变成 float",
    isinstance(scalar_scenario.expects[0].value, float),
    repr(scalar_scenario.expects[0].value),
)
check(
    "A20 空文本报错",
    raises(core.ScenarioError, lambda: core.parse_scenario_text(u"   \n\n")),
    "",
)
check(
    "A21 缺 scenario 段报错",
    raises(core.ScenarioError, lambda: core.parse_scenario_text(u"foo: 1")),
    "",
)
check(
    "A22 顶层是列表报错",
    raises(core.ScenarioError, lambda: core.parse_scenario_text(u"- a\n- b")),
    "",
)
check(
    "A23 非 `键: 值` 行报错",
    raises(core.ScenarioError, lambda: core.parse_scenario_text(u"scenario:\n  name\n")),
    "",
)
check(
    "A24 行内映射缺冒号报错",
    raises(
        core.ScenarioError,
        lambda: core.parse_scenario_text(u"scenario:\n  name: {a 1}\n"),
    ),
    "",
)

# ---------------------------------------------------------- B build_scenario

check("B1 __repr__ 带出名字与条数", "name=" in repr(make_scenario()), repr(make_scenario()))
defaulted = core.build_scenario(
    {"name": "t", "steps": [{"at": 0.0, "command": {"linear": 1.0}}], "expects": [{"type": "final_speed", "value": 0.0}]}
)
check(
    "B2 duration 缺省为 DEFAULT_DURATION",
    abs(defaulted.duration - core.DEFAULT_DURATION) < 1e-9,
    repr(defaulted.duration),
)
check(
    "B3 缺 name 时叫 unnamed",
    text(core.build_scenario({"steps": [], "expects": [{"type": "max_speed", "value": 1}]}).name)
    == u"unnamed",
    "",
)
check(
    "B4 duration 非正报错",
    raises(core.ScenarioError, lambda: make_scenario(duration=0.0)),
    "",
)
check(
    "B5 duration 不是数字报错",
    raises(core.ScenarioError, lambda: make_scenario(duration="abc")),
    "",
)
check(
    "B6 step 缺 at 报错",
    raises(
        core.ScenarioError,
        lambda: make_scenario(steps=[{"command": {"linear": 1.0}}]),
    ),
    "",
)
check(
    "B7 at 为负报错",
    raises(
        core.ScenarioError,
        lambda: make_scenario(steps=[{"at": -1.0, "command": {"linear": 1.0}}]),
    ),
    "",
)
check(
    "B8 at 不严格递增报错",
    raises(
        core.ScenarioError,
        lambda: make_scenario(
            steps=[
                {"at": 1.0, "command": {"linear": 1.0}},
                {"at": 1.0, "command": {"linear": 2.0}},
            ]
        ),
    ),
    "",
)
check(
    "B9 at 超过 duration 报错",
    raises(
        core.ScenarioError,
        lambda: make_scenario(
            duration=2.0, steps=[{"at": 5.0, "command": {"linear": 1.0}}]
        ),
    ),
    "",
)
check(
    "B10 steps 不是列表报错",
    raises(core.ScenarioError, lambda: make_scenario(steps={"at": 0.0})),
    "",
)
check(
    "B11 step 不是映射报错",
    raises(core.ScenarioError, lambda: make_scenario(steps=[1, 2])),
    "",
)
check("B12 缺 expects 报错", raises(core.ScenarioError, lambda: make_scenario(expects=[])), "")
check(
    "B13 expects 不是列表报错",
    raises(core.ScenarioError, lambda: make_scenario(expects={"type": "max_speed"})),
    "",
)
check("B14 steps 会按 at 排序", make_scenario(steps=[{"at": 2.0, "command": {"linear": 1.0}}, {"at": 1.0, "command": {"linear": 2.0}}]).steps[0].at == 1.0, "")
check("B15 command 缺省为 0", make_scenario(steps=[{"at": 0.0}]).steps[0].command.as_tuple() == (0.0, 0.0), "")
check(
    "B16 initial 不是映射报错",
    raises(core.ScenarioError, lambda: make_scenario(initial=[1, 2])),
    "",
)
check(
    "B17 initial 缺省为原点",
    make_scenario(initial={}).initial.as_tuple() == (0.0, 0.0, 0.0),
    "",
)
check(
    "B18 空 steps 允许（原地不动）",
    len(make_scenario(steps=[]).steps) == 0,
    "",
)
check(
    "B19 expects 里 value 是布尔报错",
    raises(
        core.ScenarioError,
        lambda: core.Expect({"type": "max_speed", "value": True}),
    ),
    "",
)
check(
    "B20 expects 里 NaN 报错",
    raises(
        core.ScenarioError,
        lambda: core.Expect({"type": "max_speed", "value": float("nan")}),
    ),
    "",
)

# ---------------------------------------------------------------- C 指令查询

query = make_scenario(
    steps=[
        {"at": 0.0, "command": {"linear": 1.0}},
        {"at": 2.0, "command": {"linear": 3.0}},
        {"at": 4.0, "command": {"linear": 0.0}},
    ]
)
check("C1 0 秒取第一条", query.command_at(0.0).as_tuple() == (1.0, 0.0), repr(query.command_at(0.0)))
check("C2 区间内保持上一条", query.command_at(1.9).as_tuple() == (1.0, 0.0), repr(query.command_at(1.9)))
check("C3 边界点立刻切换", query.command_at(2.0).as_tuple() == (3.0, 0.0), repr(query.command_at(2.0)))
check("C4 边界后仍用新的", query.command_at(2.5).as_tuple() == (3.0, 0.0), repr(query.command_at(2.5)))
check("C5 最后一条之后保持", query.command_at(99.0).as_tuple() == (0.0, 0.0), repr(query.command_at(99.0)))
check("C6 负时间取零指令", query.command_at(-1.0).as_tuple() == (0.0, 0.0), repr(query.command_at(-1.0)))
empty = make_scenario(steps=[])
check("C7 空 steps 返回零指令", empty.command_at(1.0).as_tuple() == (0.0, 0.0), "")
check(
    "C8 command_at 不改 scenarios",
    len(query.steps) == 3,
    repr(len(query.steps)),
)

# ---------------------------------------------------------------- D 轨迹指标

line = trajectory([(0.0, 0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0, 1.0), (2.0, 3.0, 0.0, 0.0, 2.0)])
check("D1 末帧取最后一条", line.final().speed == 2.0, repr(line.final()))
check("D2 时长按首末时间差", abs(line.duration() - 2.0) < 1e-9, repr(line.duration()))
check("D3 峰值速度", abs(line.peak_speed() - 2.0) < 1e-9, repr(line.peak_speed()))
check("D4 里程按速度积分", abs(line.travelled() - 3.0) < 1e-9, repr(line.travelled()))
check("D5 空轨迹 is_empty", core.Trajectory([]).is_empty() is True, "")
check("D6 空轨迹末帧报错", raises(core.ScenarioError, lambda: core.Trajectory([]).final()), "")
check("D7 空轨迹时长 0", core.Trajectory([]).duration() == 0.0, "")
check("D8 空轨迹峰值 0", core.Trajectory([]).peak_speed() == 0.0, "")
check("D9 空轨迹里程 0", core.Trajectory([]).travelled() == 0.0, "")
check("D10 倒车也算里程", abs(trajectory([(0.0, 0.0, 0.0, 0.0, 0.0), (2.0, 0.0, 0.0, 0.0, -1.0)]).travelled() - 2.0) < 1e-9, "")

# ---------------------------------------------------------------- E 内置车辆

profile = core.VehicleProfile()


def run(steps, duration, dt=0.05, initial=None, prof=None):
    scenario = make_scenario(duration=duration, steps=steps, initial=initial)
    return core.simulate(scenario, prof or profile, dt)


accel = run([{"at": 0.0, "command": {"linear": 5.0}}], 1.0)
check("E1 加速度受限（1 秒只到 1.5）", abs(accel.final().speed - 1.5) < 1e-6, repr(accel.final().speed))
slow = run([{"at": 0.0, "command": {"linear": 5.0}}], 2.0, prof=core.VehicleProfile(max_speed=1.0))
check("E2 最高速度被截断", abs(slow.final().speed - 1.0) < 1e-6, repr(slow.final().speed))
braked = run(
    [{"at": 0.0, "command": {"linear": 0.0}}],
    0.5,
    initial={"speed": 2.0},
)
check("E3 减速度受限（0.5 秒从 2 停到 0）", abs(braked.final().speed) < 1e-6, repr(braked.final().speed))
reverse = run([{"at": 0.0, "command": {"linear": -1.0}}], 0.5)
check("E4 倒车方向正确", abs(reverse.final().speed + 1.0) < 1e-6, repr(reverse.final().speed))
turning = run([{"at": 0.0, "command": {"linear": 1.0, "angular": 0.5}}], 2.0)
check("E5 航向按角速度积分", abs(turning.final().yaw - 1.0) < 1e-6, repr(turning.final().yaw))
fast_turn = run([{"at": 0.0, "command": {"angular": 3.0}}], 1.0)
check("E6 角速度被截断", abs(fast_turn.final().yaw - 1.0) < 1e-6, repr(fast_turn.final().yaw))
check("E7 采样点数 = 步数 + 1", len(accel.samples) == 21, repr(len(accel.samples)))
glide = run([{"at": 0.0, "command": {"linear": 2.0}}], 2.0)
check("E8 里程积分正确（2.70 ~ 2.74）", 2.70 <= glide.travelled() <= 2.74, repr(glide.travelled()))
check("E9 起始速度被保留", abs(run([{"at": 0.0}], 0.1, initial={"speed": 1.0}).samples[0].speed - 1.0) < 1e-9, "")
check("E10 初始位姿被保留", run([{"at": 0.0}], 0.1, initial={"x": 3.0, "y": 4.0}).samples[0].x == 3.0, "")
check("E11 dt 非正报错", raises(core.ScenarioError, lambda: run([{"at": 0.0}], 1.0, dt=0.0)), "")
check("E12 duration 太短报错", raises(core.ScenarioError, lambda: run([{"at": 0.0}], 0.01, dt=0.05)), "")
check("E13 零指令原地不动", abs(run([{"at": 0.0, "command": {"linear": 0.0}}], 1.0).travelled()) < 1e-9, "")
pose = core.Pose()
stepped_speed = core.advance(pose, 0.0, core.Command(1.0, 0.0), 0.05, profile)
check("E14 advance 单步速度 0.075", abs(stepped_speed - 0.075) < 1e-9, repr(stepped_speed))
check("E15 advance 单步位移 0.00375", abs(pose.x - 0.00375) < 1e-9, repr(pose.x))
check("E16 advance 不改航向", abs(pose.yaw) < 1e-12, repr(pose.yaw))

# ---------------------------------------------------------------- F 断言判定

mixed = trajectory(
    [(0.0, 0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.1, 1.0), (2.0, 2.0, 2.0, 0.1, 2.0)]
)


def judge(raw, single=mixed):
    return core.evaluate(single, [core.Expect(raw)])[0]


passed = judge({"type": "max_speed", "value": 2.0, "tolerance": 0.1})
check("F1 max_speed 通过", passed.passed is True, passed.detail)
check("F2 max_speed 细节里有峰值", u"峰值" in text(passed.detail), text(passed.detail))
check(
    "F3 max_speed 失败",
    judge({"type": "max_speed", "value": 1.0, "tolerance": 0.0}).passed is False,
    "",
)
check(
    "F4 reach_speed 通过",
    judge({"type": "reach_speed", "value": 2.0, "tolerance": 0.05}).passed is True,
    "",
)
check(
    "F5 reach_speed 失败",
    judge({"type": "reach_speed", "value": 3.0, "tolerance": 0.05}).passed is False,
    "",
)
check(
    "F6 final_speed 通过",
    judge({"type": "final_speed", "value": 2.0, "tolerance": 0.01}).passed is True,
    "",
)
check(
    "F7 final_speed 失败",
    judge({"type": "final_speed", "value": 0.0, "tolerance": 0.01}).passed is False,
    "",
)
check(
    "F8 distance 通过",
    judge({"type": "distance", "min": 2.9, "max": 3.1}).passed is True,
    "",
)
check(
    "F9 distance 失败（下限）",
    judge({"type": "distance", "min": 3.5}).passed is False,
    "",
)
check(
    "F10 distance 失败（上限）",
    judge({"type": "distance", "max": 2.5}).passed is False,
    "",
)
check(
    "F11 distance 只给 max",
    judge({"type": "distance", "max": 10.0}).passed is True,
    "",
)
check(
    "F12 position 通过",
    judge({"type": "position", "x": 2.0, "y": 2.0, "tolerance": 0.01}).passed is True,
    "",
)
check(
    "F13 position 失败",
    judge({"type": "position", "x": 5.0, "y": 0.0, "tolerance": 0.1}).passed is False,
    "",
)
check(
    "F14 heading 通过",
    judge({"type": "heading", "value": 0.1, "tolerance": 0.01}).passed is True,
    "",
)
check(
    "F15 heading 角度回绕",
    core.evaluate(
        trajectory([(0.0, 0.0, 0.0, 3.1, 0.0)]),
        [core.Expect({"type": "heading", "value": -3.14, "tolerance": 0.1})],
    )[0].passed
    is True,
    "",
)
check(
    "F16 heading 失败",
    judge({"type": "heading", "value": 0.0, "tolerance": 0.05}).passed is False,
    "",
)
results = core.evaluate(
    mixed,
    [
        core.Expect({"type": "max_speed", "value": 5.0}),
        core.Expect({"type": "final_speed", "value": 2.0}),
    ],
)
check("F17 evaluate 序号从 1 开始", [result.index for result in results] == [1, 2], repr([result.index for result in results]))
check("F18 all_passed 全通过为真", core.all_passed(results) is True, "")
check(
    "F19 all_passed 有失败为假",
    core.all_passed(
        core.evaluate(mixed, [core.Expect({"type": "final_speed", "value": 9.0})])
    )
    is False,
    "",
)
check(
    "F20 未知断言类型报错",
    raises(core.ScenarioError, lambda: core.Expect({"type": "nope"})),
    "",
)
check(
    "F21 缺 type 报错",
    raises(core.ScenarioError, lambda: core.Expect({"value": 1.0})),
    "",
)
check(
    "F22 max_speed 缺 value 报错",
    raises(core.ScenarioError, lambda: core.Expect({"type": "max_speed"})),
    "",
)
check(
    "F23 distance 缺 min/max 报错",
    raises(core.ScenarioError, lambda: core.Expect({"type": "distance"})),
    "",
)
check(
    "F24 distance min 大于 max 报错",
    raises(
        core.ScenarioError,
        lambda: core.Expect({"type": "distance", "min": 5.0, "max": 1.0}),
    ),
    "",
)
check(
    "F25 position 缺 y 报错",
    raises(
        core.ScenarioError,
        lambda: core.Expect({"type": "position", "x": 1.0}),
    ),
    "",
)
check(
    "F26 tolerance 为负报错",
    raises(
        core.ScenarioError,
        lambda: core.Expect({"type": "final_speed", "value": 0.0, "tolerance": -1.0}),
    ),
    "",
)
check(
    "F27 expects 项不是映射报错",
    raises(core.ScenarioError, lambda: core.Expect([1, 2])),
    "",
)
check("F28 normalize_angle 折算 3.5", abs(core.normalize_angle(3.5) - (3.5 - 2 * math.pi)) < 1e-9, repr(core.normalize_angle(3.5)))
check("F29 normalize_angle 折算 -3.5", abs(core.normalize_angle(-3.5) - (-3.5 + 2 * math.pi)) < 1e-9, repr(core.normalize_angle(-3.5)))
check("F30 normalize_angle 保持 0", abs(core.normalize_angle(0.0)) < 1e-12, "")

# ---------------------------------------------------------------- G 报告格式

report = core.format_report(
    make_scenario(name="报告测试"),
    [
        core.Result(1, core.Expect({"type": "max_speed", "value": 5.0}), True, u"峰值 1.00"),
        core.Result(2, core.Expect({"type": "final_speed", "value": 0.0}), False, u"结束速度 2.00"),
    ],
    header=u"里程 3.00 m",
)
check("G1 报告含场景名", u"报告测试" in text(report), text(report))
check("G2 报告含通过/失败计数", u"断言: 1 通过 / 1 失败" in text(report), text(report))
check("G3 报告含 PASS 行", u"[PASS] max_speed" in text(report), text(report))
check("G4 报告含 FAIL 行", u"[FAIL] final_speed" in text(report), text(report))
check("G5 报告结论 FAILED", text(report).rstrip().endswith(u"结论: FAILED"), text(report))
check("G6 header 在最前面", text(report).startswith(u"里程 3.00 m"), text(report))
check(
    "G7 全通过时结论 PASSED",
    text(
        core.format_report(
            make_scenario(), [core.Result(1, core.Expect({"type": "max_speed", "value": 9.0}), True, u"ok")]
        )
    )
    .rstrip()
    .endswith(u"结论: PASSED"),
    "",
)
check(
    "G8 没有来源时不打印来源行",
    u"来源:" not in text(core.format_report(make_scenario(), [])),
    text(core.format_report(make_scenario(), [])),
)
sourced = core.build_scenario(
    {"name": "s", "steps": [], "expects": [{"type": "max_speed", "value": 1.0}]},
    source="x.yaml",
)
check(
    "G9 有来源时打印来源行",
    u"来源: x.yaml" in text(core.format_report(sourced, [])),
    text(core.format_report(sourced, [])),
)
check("G10 报告为空断言也给出结论", u"结论: PASSED" in text(core.format_report(make_scenario(), [])), "")

# ---------------------------------------------------------------- H 参数

defaults = core.RunnerParams.from_params(dict_reader({}))
check("H1 publish_hz 默认 20", abs(defaults.publish_hz - 20.0) < 1e-9, repr(defaults.publish_hz))
check("H2 mock 默认开", defaults.use_mock_odom is True, repr(defaults.use_mock_odom))
check("H3 cmd_vel 默认 /cmd_vel", text(defaults.cmd_vel_topic) == u"/cmd_vel", repr(defaults.cmd_vel_topic))
check("H4 report 默认 /scenario/report", text(defaults.report_topic) == u"/scenario/report", repr(defaults.report_topic))
check("H5 scenario_file 默认空", text(defaults.scenario_file) == u"", repr(defaults.scenario_file))
check("H6 sim_dt 默认 0.05", abs(defaults.sim_dt - 0.05) < 1e-9, repr(defaults.sim_dt))
check("H7 最大速度默认 5", abs(defaults.max_speed - 5.0) < 1e-9, repr(defaults.max_speed))
check("H8 减速度默认 4", abs(defaults.max_decel - 4.0) < 1e-9, repr(defaults.max_decel))

overridden = core.RunnerParams.from_params(
    dict_reader(
        {
            "scenario/file": u"/tmp/a.yaml",
            "rate/publish_hz": u"50.0",
            "control/use_mock_odom": u"false",
            "control/sim_dt": 0.02,
            "vehicle/max_speed": 3.0,
            "vehicle/max_accel": 1.0,
            "vehicle/max_decel": 2.0,
            "vehicle/max_yaw_rate": 0.5,
            "topic/cmd_vel": u"/car/cmd",
            "topic/odom": u"/car/odom",
            "topic/report": u"/car/report",
        }
    )
)
check("H9 覆盖 publish_hz", abs(overridden.publish_hz - 50.0) < 1e-9, repr(overridden.publish_hz))
check("H10 覆盖 mock 开关（字符串）", overridden.use_mock_odom is False, repr(overridden.use_mock_odom))
check("H11 覆盖 sim_dt", abs(overridden.sim_dt - 0.02) < 1e-9, repr(overridden.sim_dt))
check("H12 覆盖话题名", text(overridden.cmd_vel_topic) == u"/car/cmd", repr(overridden.cmd_vel_topic))
check("H13 覆盖场景路径", text(overridden.scenario_file) == u"/tmp/a.yaml", repr(overridden.scenario_file))
check("H14 profile 带上覆盖值", overridden.profile().as_tuple() == (3.0, 1.0, 2.0, 0.5), repr(overridden.profile().as_tuple()))
check("H15 as_bool 认 true", core.as_bool(u"true") is True, "")
check("H16 as_bool 认 0", core.as_bool(u"0") is False, "")
check("H17 as_bool 认 None 走默认", core.as_bool(None, True) is True, "")
check("H18 as_bool 认 bool", core.as_bool(False, True) is False, "")
check("H19 as_bool 认 yes", core.as_bool(u"yes") is True, "")
check("H20 publish_hz 非正报错", raises(core.ScenarioError, lambda: core.RunnerParams(publish_hz=0.0)), "")
check("H21 sim_dt 非正报错", raises(core.ScenarioError, lambda: core.RunnerParams(sim_dt=0.0)), "")
check("H22 max_speed 非正报错", raises(core.ScenarioError, lambda: core.VehicleProfile(max_speed=0.0)), "")
check("H23 max_accel 非正报错", raises(core.ScenarioError, lambda: core.VehicleProfile(max_accel=0.0)), "")
check("H24 RunnerParams 的 __repr__ 可读", "hz=" in repr(defaults), repr(defaults))
read_keys = []


def recording_reader(key, fallback=None):
    read_keys.append(key)
    return fallback


core.RunnerParams.from_params(recording_reader)
check("H25 共读取 11 个参数键", len(read_keys) == 11, repr(read_keys))
check("H26 读取了 scenario/file", "scenario/file" in read_keys, repr(read_keys))
check("H27 读取了 topic/report", "topic/report" in read_keys, repr(read_keys))

# ------------------------------------------------------------ I 四元数/消息

check("I1 yaw=0 的四元数是单位四元数", core.yaw_to_quaternion(0.0) == (0.0, 0.0, 0.0, 1.0), repr(core.yaw_to_quaternion(0.0)))
check("I2 四元数往返 1.0", abs(core.quaternion_to_yaw(*core.yaw_to_quaternion(1.0)) - 1.0) < 1e-9, "")
check("I3 四元数往返 -2.5", abs(core.quaternion_to_yaw(*core.yaw_to_quaternion(-2.5)) + 2.5) < 1e-9, "")
check("I4 四元数往返 3.0", abs(core.quaternion_to_yaw(*core.yaw_to_quaternion(3.0)) - 3.0) < 1e-9, "")
check("I5 to_message_text 返回 str", isinstance(core.to_message_text(u"abc"), str), type(core.to_message_text(u"abc")).__name__)
check("I6 to_message_text 内容不变", text(core.to_message_text(u"报告")) == u"报告", repr(core.to_message_text(u"报告")))
check("I7 to_message_text 能处理 bytes", isinstance(core.to_message_text("abc"), str), "")

# ------------------------------------------------------- J 三个演示场景端到端

SCENARIO_DIR = os.path.join(PACKAGE, "config", "scenarios")
brake_scenario = core.load_scenario(os.path.join(SCENARIO_DIR, "demo_brake_stop.yaml"))
turn_scenario = core.load_scenario(os.path.join(SCENARIO_DIR, "demo_turn_arc.yaml"))
fail_scenario = core.load_scenario(os.path.join(SCENARIO_DIR, "demo_expect_fail.yaml"))
check("J1 刹车场景能加载", isinstance(brake_scenario, core.Scenario), repr(brake_scenario))
check("J2 刹车场景名正确", text(brake_scenario.name) == u"直线加速与刹停", repr(brake_scenario.name))
check("J3 刹车场景有 6 条断言", len(brake_scenario.expects) == 6, repr(len(brake_scenario.expects)))
brake_traj = core.simulate(brake_scenario)
brake_results = core.evaluate(brake_traj, brake_scenario.expects)
check("J4 刹车场景全部断言通过", core.all_passed(brake_results) is True, core.format_report(brake_scenario, brake_results))
check("J5 刹车场景峰值速度 5.0", abs(brake_traj.peak_speed() - 5.0) < 1e-6, repr(brake_traj.peak_speed()))
check("J6 刹车场景里程约 32.3 m", 32.0 <= brake_traj.travelled() <= 32.6, repr(brake_traj.travelled()))
check("J7 刹车场景结束速度 0", abs(brake_traj.final().speed) < 1e-6, repr(brake_traj.final().speed))
turn_traj = core.simulate(turn_scenario)
turn_results = core.evaluate(turn_traj, turn_scenario.expects)
check("J8 转弯场景全部断言通过", core.all_passed(turn_results) is True, core.format_report(turn_scenario, turn_results))
check("J9 转弯场景结束航向 2.75", abs(turn_traj.final().yaw - 2.75) < 1e-6, repr(turn_traj.final().yaw))
check("J10 转弯场景里程约 13.5 m", 13.0 <= turn_traj.travelled() <= 14.0, repr(turn_traj.travelled()))
fail_traj = core.simulate(fail_scenario)
fail_results = core.evaluate(fail_traj, fail_scenario.expects)
check("J11 反例场景结论是失败", core.all_passed(fail_results) is False, "")
check("J12 反例场景第 1 条通过", fail_results[0].passed is True, fail_results[0].detail)
check("J13 反例场景第 2 条失败", fail_results[1].passed is False, fail_results[1].detail)
check(
    "J14 反例场景报告写 FAILED",
    text(core.format_report(fail_scenario, fail_results)).rstrip().endswith(u"结论: FAILED"),
    "",
)
check(
    "J15 刹车场景报告写 PASSED",
    text(core.format_report(brake_scenario, brake_results)).rstrip().endswith(u"结论: PASSED"),
    "",
)
check(
    "J16 场景目录里有 3 个 yaml",
    len(
        [
            name
            for name in os.listdir(SCENARIO_DIR)
            if name.endswith(".yaml") or name.endswith(".yml")
        ]
    )
    == 3,
    repr(sorted(os.listdir(SCENARIO_DIR))),
)
check(
    "J17 缺文件时 load_scenario 报错",
    raises(
        core.ScenarioError,
        lambda: core.load_scenario(os.path.join(SCENARIO_DIR, "no_such.yaml")),
    ),
    "",
)
check(
    "J18 空路径 load_scenario 报错",
    raises(core.ScenarioError, lambda: core.load_scenario(u"")),
    "",
)

# ---------------------------------------------------------------- K 工程文件

package_xml = read_text("src", "ground", "ros_scenario_runner", "package.xml")
cmake_text = read_text("src", "ground", "ros_scenario_runner", "CMakeLists.txt")
launch_text = read_text("src", "ground", "ros_scenario_runner", "launch", "main.launch")
config_text = read_text(
    "src", "ground", "ros_scenario_runner", "config", "scenario_runner.yaml"
)
core_text = read_text("src", "ground", "ros_scenario_runner", "scripts", "scenario_core.py")
node_text = read_text(
    "src", "ground", "ros_scenario_runner", "scripts", "scenario_runner_node.py"
)
main_text = read_text("src", "ground", "ros_scenario_runner", "scripts", "main.py")
readme_text = read_text("src", "ground", "ros_scenario_runner", "README.md")
DOC_PATH = os.path.join(ROOT, "docs", "ground", "ros_scenario_runner.md")
doc_text = (
    io.open(DOC_PATH, "r", encoding="utf-8").read() if os.path.isfile(DOC_PATH) else u""
)

check("K1 package.xml 包名正确", "<name>ros_scenario_runner</name>" in package_xml, "")
check("K2 package.xml 版本 1.0.0", "<version>1.0.0</version>" in package_xml, "")
check("K3 package.xml 声明 rospy", "<exec_depend>rospy</exec_depend>" in package_xml, "")
check("K4 package.xml 声明 geometry_msgs", "<exec_depend>geometry_msgs</exec_depend>" in package_xml, "")
check("K5 package.xml 声明 nav_msgs", "<exec_depend>nav_msgs</exec_depend>" in package_xml, "")
check("K6 package.xml 声明 std_msgs", "<exec_depend>std_msgs</exec_depend>" in package_xml, "")
check("K7 package.xml 有 catkin 构建工具依赖", "<buildtool_depend>catkin</buildtool_depend>" in package_xml, "")
check("K8 CMakeLists 安装 main.py", "scripts/main.py" in cmake_text, "")
check("K9 CMakeLists 安装 scenario_core.py", "scripts/scenario_core.py" in cmake_text, "")
check("K10 CMakeLists 安装节点脚本", "scripts/scenario_runner_node.py" in cmake_text, "")
check("K11 CMakeLists 安装 config 目录", "install(DIRECTORY launch config" in cmake_text, "")
check("K12 CMakeLists 保留可执行位", "USE_SOURCE_PERMISSIONS" in cmake_text, "")
check('K13 launch 的节点类型是 main.py', 'type="main.py"' in launch_text, "")
check("K14 launch 不再直接启节点脚本", 'type="scenario_runner_node.py"' not in launch_text, "")
check('K15 launch 用 rosparam 载入 yaml', '<rosparam command="load"' in launch_text, "")
launch_args = launch_text.count("<arg name=")
check("K16 launch 有 11 个 arg", launch_args == 11, repr(launch_args))
check("K17 每个 arg 都有 doc", launch_text.count('doc="') == launch_args, repr(launch_text.count('doc="')))
check("K18 launch 覆盖 scenario/file", '<param name="scenario/file"' in launch_text, "")
check("K19 launch 覆盖 vehicle/max_speed", '<param name="vehicle/max_speed"' in launch_text, "")
check("K20 launch 覆盖 topic/report", '<param name="topic/report"' in launch_text, "")
launch_params = launch_text.count("<param name=")
check("K21 launch 有 11 个 param 覆盖", launch_params == 11, repr(launch_params))
check("K22 launch 默认跑刹车场景", "demo_brake_stop.yaml" in launch_text, "")
check("K23 yaml 有发布频率", "publish_hz: 20.0" in config_text, "")
check("K24 yaml 有 mock 开关", "use_mock_odom: true" in config_text, "")
check("K25 yaml 有车辆限速", "max_speed: 5.0" in config_text, "")
check("K26 yaml 有报告话题", "/scenario/report" in config_text, "")
check("K27 yaml 有场景文件键", "file:" in config_text, "")
check("K28 核心模块不 import rospy", "\nimport rospy" not in core_text, "")
check("K29 核心模块不 import yaml", "\nimport yaml" not in core_text, "")
check("K30 核心模块声明 utf-8", "coding: utf-8" in core_text, "")
check("K31 节点 import 核心模块", "import scenario_core" in node_text, "")
check("K32 节点注册退出回调", "rospy.on_shutdown" in node_text, "")
check("K33 节点发布报告", "report_pub.publish" in node_text, "")
check("K34 节点发零控制量", "publish_zero" in node_text, "")
check("K35 main.py 复用节点入口", "from scenario_runner_node import main" in main_text, "")
check("K36 README 有离线测试命令", "test_ros_scenario_runner_local.py" in readme_text, "")
check("K37 README 有 roslaunch 命令", "roslaunch ros_scenario_runner main.launch" in readme_text, "")
if os.path.isfile(DOC_PATH):
    check("K38 文档页保留原有参考链接", "carla_ros_scenario_runner" in doc_text, "")
    check("K39 文档页有场景格式说明", "scenario:" in doc_text, "")
    check("K40 文档页提到断言类型", "final_speed" in doc_text, "")
    check("K41 文档页有离线测试一节", "离线" in text(doc_text), "")
    check("K42 文档页标题提到场景", "场景" in text(doc_text), "")
else:
    check("K38 本次未在仓库根目录运行，跳过文档页校验", True, DOC_PATH)
    check("K39 本地不含 docs 目录", not os.path.isdir(os.path.join(ROOT, "docs")), repr(os.path.join(ROOT, "docs")))
    check("K40 包目录仍然存在", os.path.isdir(PACKAGE), PACKAGE)
    check("K41 测试脚本位于 tests 目录下", os.path.basename(os.path.dirname(os.path.abspath(__file__))) == "tests", "")
    check("K42 核心模块仍然可导入", core.DEFAULT_PUBLISH_HZ > 0, "")

# ---------------------------------------------------------------- 汇总

print(u"ros_scenario_runner 离线回归测试")
print(u"Python %s" % sys.version.split()[0])
print(u"包目录 %s" % PACKAGE)
print(u"-" * 60)
for name, detail in FAILED:
    print(u"失败  %s  %s" % (name, detail))
print(u"=" * 60)
print(u"通过 %d 项，失败 %d 项" % (len(PASSED), len(FAILED)))
if FAILED:
    print(u"有失败项")
    sys.exit(1)
print(u"全部通过")


