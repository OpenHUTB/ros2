"""场景与两个补充实验的定义。

五个主场景共用同一条线路（2400 m 站间距、160 km/h 定速），只在"编组怎么建模"
和"制动指令怎么传播"上不同，结果差异因此只能来自车钩与传播延迟。

两个补充实验：
- `sweep`：编组数 × 每节延迟，用来看车钩力随这两个量怎么长；
- `grade_cases`：平整 / 上坡变坡点 / 下坡变坡点，用来看坡度本身进不进车钩。
"""

from .params import DEFAULT_CAR, SINGLE_BODY

DOWNHILL_BREAK = ((1200.0, 2400.0, -0.012),)
UPHILL_BREAK = ((1200.0, 2400.0, 0.012),)

SCENARIOS = {
    "single_body": {
        "name": "single_body",
        "note": "单质点参照：4 节车合并成一节 300 t，没有车钩",
        "n_cars": 1,
        "car": SINGLE_BODY,
        "delay": 0.0,
        "direction": "sync",
    },
    "consist_sync": {
        "name": "consist_sync",
        "note": "4 节编组，全列同时施加牵引与制动（理想通信）",
        "n_cars": 4,
        "car": DEFAULT_CAR,
        "delay": 0.0,
        "direction": "sync",
    },
    "consist_head_first": {
        "name": "consist_head_first",
        "note": "4 节编组，指令车头→车尾，每节延迟 60 ms",
        "n_cars": 4,
        "car": DEFAULT_CAR,
        "delay": 0.06,
        "direction": "head",
    },
    "consist_tail_first": {
        "name": "consist_tail_first",
        "note": "4 节编组，指令车尾→车头，每节延迟 60 ms",
        "n_cars": 4,
        "car": DEFAULT_CAR,
        "delay": 0.06,
        "direction": "tail",
    },
    "consist_long_tail": {
        "name": "consist_long_tail",
        "note": "8 节编组，指令车尾→车头，每节延迟 100 ms",
        "n_cars": 8,
        "car": DEFAULT_CAR,
        "delay": 0.10,
        "direction": "tail",
    },
}

ORDER = ["single_body", "consist_sync", "consist_head_first",
         "consist_tail_first", "consist_long_tail"]

SHORT_LABELS = {
    "single_body": "1 BODY",
    "consist_sync": "4 SYNC",
    "consist_head_first": "4 HEAD",
    "consist_tail_first": "4 TAIL",
    "consist_long_tail": "8 TAIL",
}

SWEEP_N = (4, 6, 8)
SWEEP_DELAY = (0.0, 0.06, 0.12, 0.20)

GRADE_CASES = (
    ("flat", ()),
    ("up 12", UPHILL_BREAK),
    ("down 12", DOWNHILL_BREAK),
)
GRADE_CARS = (4, 8)
GRADE_DELAY = 0.06
