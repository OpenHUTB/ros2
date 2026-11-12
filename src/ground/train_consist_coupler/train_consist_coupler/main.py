"""命令行入口：离线跑全部场景、扫描与变坡点实验，输出图和 CSV。

    python3 -m train_consist_coupler.main --offline --output results
"""

import argparse
import csv
import os

from . import plots
from .params import DEFAULT_CAR
from .scenarios import (GRADE_CARS, GRADE_CASES, GRADE_DELAY, ORDER,
                        SCENARIOS, SHORT_LABELS, SWEEP_DELAY, SWEEP_N)
from .sim import run_scenario

HEADER = ("SCENARIO", "CARS", "DELAY", "TIME", "VMAX", "STOP_ERR",
          "BUFF", "DRAFT", "JERK")


def run_main_scenarios():
    return {name: run_scenario(dict(SCENARIOS[name])) for name in ORDER}


def run_sweep():
    table = {}
    for n_cars in SWEEP_N:
        for delay in SWEEP_DELAY:
            spec = {"name": "sweep", "n_cars": n_cars, "car": DEFAULT_CAR,
                    "delay": delay}
            buff = run_scenario(dict(spec, direction="tail"))["peak_buff"]
            draft = run_scenario(dict(spec, direction="head"))["peak_draft"]
            table[(n_cars, delay)] = {"buff": buff, "draft": draft}
    return table


def run_grade_cases():
    table = {}
    for name, grades in GRADE_CASES:
        for n_cars in GRADE_CARS:
            result = run_scenario({"name": name, "n_cars": n_cars,
                                   "car": DEFAULT_CAR, "delay": GRADE_DELAY,
                                   "direction": "head", "grades": grades})
            table[(name, n_cars)] = result
    return table


def print_table(results):
    line = ("%-20s %4s %6s %8s %8s %9s %9s %9s %7s"
            % ("SCENARIO", "CARS", "DELAY", "TIME", "VMAX", "STOP_ERR",
               "BUFF", "DRAFT", "JERK"))
    print(line)
    print("-" * len(line))
    for name in ORDER:
        row = results[name]
        print("%-20s %4d %5.0fms %7.1fs %6.1fkm/h %+8.3fm %8.1fkN %8.1fkN %6.2fm/s3"
              % (name, row["n_cars"], row["delay"] * 1e3, row["run_time"],
                 row["v_max"] * 3.6, row["stop_error"], row["peak_buff"] / 1e3,
                 row["peak_draft"] / 1e3, row["max_jerk"]))


def write_main_csv(results, path):
    with open(path, "w", newline="", encoding="utf-8") as fp:
        writer = csv.writer(fp)
        writer.writerow(["scenario", "cars", "delay_s", "direction", "run_time_s",
                         "v_max_kmh", "stop_error_m", "peak_buff_kN",
                         "peak_draft_kN", "max_jerk_m_s3", "brake_start_m"])
        for name in ORDER:
            row = results[name]
            writer.writerow([name, row["n_cars"], row["delay"], row["direction"],
                             "%.3f" % row["run_time"], "%.2f" % (row["v_max"] * 3.6),
                             "%.4f" % row["stop_error"], "%.2f" % (row["peak_buff"] / 1e3),
                             "%.2f" % (row["peak_draft"] / 1e3), "%.3f" % row["max_jerk"],
                             "%.1f" % row["brake_start"]])


def write_sweep_csv(sweep, path):
    with open(path, "w", newline="", encoding="utf-8") as fp:
        writer = csv.writer(fp)
        writer.writerow(["cars", "delay_ms", "peak_buff_kN", "peak_draft_kN"])
        for (n_cars, delay), row in sorted(sweep.items()):
            writer.writerow([n_cars, "%.0f" % (delay * 1e3),
                             "%.2f" % (row["buff"] / 1e3),
                             "%.2f" % (abs(row["draft"]) / 1e3)])


def write_grade_csv(grade, path):
    with open(path, "w", newline="", encoding="utf-8") as fp:
        writer = csv.writer(fp)
        writer.writerow(["case", "cars", "peak_buff_kN", "peak_draft_kN",
                         "stop_error_m"])
        for (name, n_cars), row in sorted(grade.items()):
            writer.writerow([name, n_cars, "%.2f" % (row["peak_buff"] / 1e3),
                             "%.2f" % (row["peak_draft"] / 1e3),
                             "%.4f" % row["stop_error"]])


def main(argv=None):
    parser = argparse.ArgumentParser(description="多编组列车纵向动力学与车钩力仿真")
    parser.add_argument("--offline", action="store_true",
                        help="不需要 ROS，直接跑离线仿真（与本模块行为一致）")
    parser.add_argument("--output", default="results", help="输出目录")
    parser.add_argument("--quick", action="store_true", help="只跑五个主场景，跳过扫描")
    args = parser.parse_args(argv)

    os.makedirs(args.output, exist_ok=True)

    results = run_main_scenarios()
    print_table(results)
    write_main_csv(results, os.path.join(args.output, "scenarios.csv"))

    plots.figure_force_route(results, os.path.join(args.output, "force_route.png"))
    plots.figure_coupler_history(results, os.path.join(args.output, "coupler_time.png"))
    plots.figure_peak_bars(results, ORDER, SHORT_LABELS,
                           os.path.join(args.output, "peak_force.png"))
    plots.figure_stop_error(results, ORDER, SHORT_LABELS,
                            os.path.join(args.output, "stop_error.png"))

    if not args.quick:
        sweep = run_sweep()
        write_sweep_csv(sweep, os.path.join(args.output, "sweep.csv"))
        plots.figure_sweep(sweep, SWEEP_N, SWEEP_DELAY,
                           os.path.join(args.output, "sweep.png"))
        grade = run_grade_cases()
        write_grade_csv(grade, os.path.join(args.output, "grade.csv"))
        print()
        print("变坡点实验（每节延迟 %.0f ms，车头先动）" % (GRADE_DELAY * 1e3))
        for (name, n_cars), row in sorted(grade.items()):
            print("  %-8s %d 节  压钩 %6.1f kN  拉钩 %7.1f kN  停车误差 %+0.3f m"
                  % (name, n_cars, row["peak_buff"] / 1e3,
                     row["peak_draft"] / 1e3, row["stop_error"]))

    print()
    print("图与 CSV 已写入 %s" % os.path.abspath(args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
