"""由已提交的 depth_log.csv 重新生成 summary.md（证据与数据保持同源）。

用途：当运行环境无法重跑仿真（例如本机应用控制策略拦截了 MuJoCo 的 DLL）时，
仍然可以从已记录的逐点数据重算指标，保证 summary 与 csv 不打架。

用法：
    python tools/summarize_csv.py [--csv docs/depth_log.csv] [--params config/params.yaml]
"""
import argparse
import csv
import os

import numpy as np
import yaml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--csv", default=os.path.join(HERE, "docs", "depth_log.csv"))
    p.add_argument("--params", default=os.path.join(HERE, "config", "params.yaml"))
    p.add_argument("--out", default=os.path.join(HERE, "docs", "summary.md"))
    args = p.parse_args()

    with open(args.params, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    with open(args.csv, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        for k in ("t", "setpoint", "depth", "error", "thrust", "vz"):
            r[k] = float(r[k])
        r["segment"] = int(r["segment"])

    settle = float(cfg["mission"]["settle_time"])
    # 按「连续相同的目标深度」分组：旧数据的 setpoints 是重复两次凑时长，
    # 按段号分会把过渡段当成独立目标，指标会失真。
    groups = []
    for r in rows:
        if not groups or groups[-1][0] != r["setpoint"]:
            groups.append((r["setpoint"], []))
        groups[-1][1].append(r)

    lines = []
    stats = []
    for gi, (setpoint, g_rows) in enumerate(groups):
        tail = [r for r in g_rows if r["t"] >= g_rows[-1]["t"] - settle]
        errs = [abs(r["error"]) for r in tail]
        stats.append((setpoint, float(np.mean(errs)), float(np.max(errs))))
        lines.append("定深 %d（目标 %.2f m，共 %.0f s）-> 稳态平均误差 %.3f m，最大误差 %.3f m"
                     % (gi, setpoint, g_rows[-1]["t"] - g_rows[0]["t"] + 0.05,
                        stats[-1][1], stats[-1][2]))

    dt = float(rows[1]["t"] - rows[0]["t"])
    worst = max(s[1] for s in stats)
    threshold = float(cfg["mission"]["pass_threshold"])

    with open(args.out, "w", encoding="utf-8") as fh:
        fh.write("水下 ROV 定深控制实验摘要（由 depth_log.csv 重新统计）\n")
        fh.write("数据来源: %s（%d 个控制点，采样间隔 %.3f s -> %.0f Hz）\n"
                 % (os.path.relpath(args.csv, HERE), len(rows), dt, 1.0 / dt))
        fh.write("任务: %s（每段 = (目标深度 m, 保持 s)）\n" % (cfg["mission"]["segments"],))
        fh.write("控制律: 串级 u = m*(kv*(v_des - vz)) - 净浮力, v_des = clip(kp_pos*e, ±%.1f m/s)\n"
                 % float(cfg["control"]["max_vertical_speed"]))
        fh.write("增益: kp_pos=%.2f 1/s, kv=%.1f 1/s；推力限幅 ±%.0f N\n"
                 % (float(cfg["control"]["position_gain"]),
                    float(cfg["control"]["velocity_gain"]),
                    float(cfg["model"]["max_thrust"])))
        for ln in lines:
            fh.write(ln + "\n")
        fh.write("最差稳态误差 %.3f m（PASS 阈值 %.3f m）\n" % (worst, threshold))
        fh.write("结论: %s\n" % ("PASS" if worst <= threshold else "FAIL"))
    print("written:", args.out)


if __name__ == "__main__":
    main()
