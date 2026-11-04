"""把闭环日志画成收敛曲线（检测值 vs 地图真值 + 转向/车速）。

只用 CSV 里已有的列，方便对任何一次实跑复现同一张图。
本仓库提交的结果图就是这条命令生成的：

    python3 tools/plot_trace.py out_final/lateral_trace.csv closed_loop.png

依赖 Pillow（`python3 -m pip install pillow` 或 `apt install python3-pil`）。
"""
import csv
import os
import sys

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:  # pragma: no cover - 取决于环境
    sys.exit("缺少 Pillow，无法画图。安装：python3 -m pip install pillow")

W, H = 1280, 760
MARGIN_L, MARGIN_R, MARGIN_T, MARGIN_B = 104, 44, 78, 74
GAP = 34

FONT_CANDIDATES = [
    r"C:\Windows\Fonts\msyh.ttc",
    r"C:\Windows\Fonts\msyhbd.ttc",
    r"C:\Windows\Fonts\simhei.ttf",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
]


def load_font(size):
    for p in FONT_CANDIDATES:
        if os.path.isfile(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:  # noqa: BLE001
                pass
    return ImageFont.load_default()


def nice_ticks(lo, hi, n=5):
    span = hi - lo
    if span <= 0:
        return [lo]
    raw = span / (n - 1)
    mag = 10 ** (len(str(int(abs(raw)))) - 1) if abs(raw) >= 1 else 1.0
    for m in (1, 2, 2.5, 5, 10):
        step = m * mag
        if step >= raw:
            break
    start = (int(lo / step) - 1) * step
    ticks = []
    v = start
    while v <= hi + step * 0.5:
        if v >= lo - step * 0.5:
            ticks.append(round(v, 6))
        v += step
    return ticks


def draw_panel(dr, box, series, y_range, y_label, title, font, font_s, unit=""):
    x0, y0, x1, y1 = box
    lo, hi = y_range
    t_min, t_max = series["t"][0], series["t"][-1]

    def px(t):
        return x0 + (t - t_min) / max(1e-9, t_max - t_min) * (x1 - x0)

    def py(v):
        v = max(lo, min(hi, v))
        return y1 - (v - lo) / max(1e-9, hi - lo) * (y1 - y0)

    # 背景与网格
    dr.rectangle([x0, y0, x1, y1], fill=(252, 252, 250), outline=(120, 120, 120))
    for v in nice_ticks(lo, hi, 5):
        y = py(v)
        dr.line([(x0, y), (x1, y)], fill=(228, 228, 224))
        dr.text((x0 - 10, y - 8), f"{v:g}", fill=(70, 70, 70), font=font_s, anchor="ra")
    for t in nice_ticks(t_min, t_max, 6):
        x = px(t)
        dr.line([(x, y0), (x, y1)], fill=(238, 238, 234))
        dr.text((x, y1 + 8), f"{t:g}", fill=(70, 70, 70), font=font_s, anchor="ma")

    dr.text((x0, y0 - 28), title, fill=(30, 30, 30), font=font)

    for name, color, width, dashed in series["lines"]:
        ys = series[name]
        pts = [(px(t), py(v)) for t, v in zip(series["t"], ys) if v is not None]
        if len(pts) < 2:
            continue
        if dashed:
            for i in range(0, len(pts) - 1, 6):
                dr.line([pts[i], pts[min(i + 3, len(pts) - 1)]], fill=color, width=width)
        else:
            dr.line(pts, fill=color, width=width, joint="curve")

    # 图例
    lx, ly = x0 + 14, y0 + 12
    for name, color, width, dashed in series["lines"]:
        dr.line([(lx, ly + 7), (lx + 26, ly + 7)], fill=color, width=width)
        dr.text((lx + 34, ly), series["labels"][name], fill=(40, 40, 40), font=font_s)
        ly += 22
    return box


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "out_closed_3/lateral_trace.csv"
    out_path = sys.argv[2] if len(sys.argv) > 2 else "closed_loop.png"
    rows = list(csv.DictReader(open(csv_path, encoding="utf-8")))

    def col(name):
        out = []
        for r in rows:
            v = r.get(name, "")
            out.append(float(v) if v not in ("", None) else None)
        return out

    t = [float(r["t_s"]) for r in rows]
    e_y = col("e_y_m")
    gt = col("gt_e_y_m")
    steer = col("steer")
    speed = col("speed_mps")

    font = load_font(21)
    font_s = load_font(15)

    img = Image.new("RGB", (W, H), "white")
    dr = ImageDraw.Draw(img)
    dr.text((MARGIN_L, 18), "车道保持闭环实跑：检测值 vs CARLA 地图真值",
            fill=(20, 20, 20), font=load_font(24))

    top_h = (H - MARGIN_T - MARGIN_B - GAP) * 0.62
    top = (MARGIN_L, MARGIN_T, W - MARGIN_R, MARGIN_T + top_h)
    bottom = (MARGIN_L, MARGIN_T + top_h + GAP + 20, W - MARGIN_R, H - MARGIN_B)

    ymax = max(
        0.3,
        max(abs(v) for v in e_y + gt if v is not None) * 1.15,
    )
    s1 = {
        "t": t, "e_y": e_y, "gt": gt,
        "lines": [("gt", (40, 40, 40), 2, True), ("e_y", (200, 40, 40), 2, False)],
        "labels": {"e_y": "检测值 e_y", "gt": "地图真值"},
    }
    draw_panel(dr, top, s1, (-ymax, ymax), "横向偏差", "横向偏差 e_y（米）", font, font_s, "m")
    dr.line([(top[0], (top[1] + top[3]) / 2), (top[2], (top[1] + top[3]) / 2)],
            fill=(150, 150, 150))

    smax = max(0.05, max(abs(v) for v in steer if v is not None)) * 1.15
    s2 = {
        "t": t, "steer": steer,
        "lines": [("steer", (30, 90, 190), 2, False)],
        "labels": {"steer": "方向盘 steer"},
    }
    draw_panel(dr, bottom, s2, (-smax, smax), "转向", "方向盘指令 steer（-1~1）", font, font_s)

    dr.text((W / 2.0, H - 22), "时间 / s", fill=(70, 70, 70),
            font=font_s, anchor="ma")
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    img.save(out_path)
    print("已保存", out_path, img.size)


if __name__ == "__main__":
    main()
