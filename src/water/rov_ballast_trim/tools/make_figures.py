#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成文档配图：配平灵敏度与恢复力矩曲线（Pillow，纯数据、无 matplotlib）.

    python tools/make_figures.py --output ../../docs/img/water/rov_ballast_trim

左图：压载纵向位置 -> 纵倾角，标出托盘限位与当前解；
右图：横倾角 -> 恢复力矩，对比现状 GM 与目标 GM，标出 5 deg 作业门限。
坐标轴刻度用英文，避免依赖本机中文字体。
"""
import argparse
import math
from pathlib import Path
import sys

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rov_ballast_trim.core import G, RovTrim  # noqa: E402

WIDTH, HEIGHT = 1280, 520
MARGIN_LEFT = 70
MARGIN_RIGHT = 40
MARGIN_TOP = 80
MARGIN_BOTTOM = 70
BG = (255, 255, 255)
FG = (30, 30, 30)
GRID = (228, 228, 228)
CURVE = (31, 90, 170)
CURVE2 = (200, 90, 30)
LIMIT = (150, 150, 150)
BAND = (232, 241, 252)


def _font(size):
    for name in ('segoeui.ttf', 'arial.ttf', 'DejaVuSans.ttf'):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def _panel(draw, box, x_range, y_range, title, band=None):
    """画坐标框与网格，返回 数据坐标 -> 像素坐标 的两个映射."""
    left, top, right, bottom = box
    (x0, x1), (y0, y1) = x_range, y_range

    def sx(value):
        return left + (value - x0) / (x1 - x0) * (right - left)

    def sy(value):
        return bottom - (value - y0) / (y1 - y0) * (bottom - top)

    if band:
        draw.rectangle([sx(band[0]), sy(y1), sx(band[1]), sy(y0)], fill=BAND)
    draw.rectangle(box, outline=FG, width=2)
    font_tick = _font(14)
    for i in range(6):
        value = x0 + (x1 - x0) * i / 5.0
        draw.line([(sx(value), top), (sx(value), bottom)], fill=GRID)
        text = '%.2f' % value
        width = draw.textlength(text, font=font_tick)
        draw.text((sx(value) - width / 2, bottom + 8), text, fill=FG, font=font_tick)
    for i in range(6):
        value = y0 + (y1 - y0) * i / 5.0
        draw.line([(left, sy(value)), (right, sy(value))], fill=GRID)
        text = '%.0f' % value
        width = draw.textlength(text, font=font_tick)
        draw.text((left - 10 - width, sy(value) - 7), text, fill=FG, font=font_tick)
    draw.text((left, top - 40), title, fill=FG, font=_font(18))
    return sx, sy


def _label(draw, xy, text, colour, font):
    """带白底的小标注，避免压在曲线上看不清."""
    left, top, right, bottom = draw.textbbox(xy, text, font=font)
    draw.rectangle([left - 3, top - 2, right + 3, bottom + 2], fill=BG)
    draw.text(xy, text, fill=colour, font=font)


def draw_figure(trim, ballast_kg, ballast_z, target_gm):
    image = Image.new('RGB', (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(image)
    small = _font(14)

    plan = trim.ballast_plan()
    gm_now = plan['state']['gm_m']
    x_lim = trim.tray_limit_x_m

    # 左图：压载纵向位置 -> 纵倾角
    box_a = (MARGIN_LEFT, MARGIN_TOP + 20, 580, HEIGHT - MARGIN_BOTTOM)
    sx, sy = _panel(draw, box_a, (-0.50, 0.50), (-40.0, 70.0),
                    'A. pitch (deg) vs ballast position x (m)',
                    band=(-x_lim, x_lim))
    draw.line([(box_a[0], sy(0.0)), (box_a[2], sy(0.0))], fill=(80, 80, 80), width=1)
    points = []
    for i in range(241):
        x = -x_lim + 2.0 * x_lim * i / 240.0
        state = trim.state(ballast_kg, x, 0.0, ballast_z)
        points.append((sx(x), sy(max(min(state['pitch_deg'], 69.5), -39.5))))
    draw.line(points, fill=CURVE, width=3)
    for value in (-x_lim, x_lim):
        draw.line([(sx(value), box_a[1]), (sx(value), box_a[3])], fill=LIMIT, width=2)
    draw.ellipse([sx(plan['x_m']) - 7, sy(0.0) - 7, sx(plan['x_m']) + 7, sy(0.0) + 7],
                 fill=CURVE2)
    _label(draw, (sx(plan['x_m']) + 12, sy(0.0) + 8),
           'solution x=%.3f m, pitch=0' % plan['x_m'], CURVE2, small)
    draw.text((sx(-x_lim) + 8, box_a[3] - 26), 'tray limit +-%.2f m' % x_lim,
              fill=(110, 110, 110), font=small)

    # 右图：横倾角 -> 恢复力矩
    box_b = (760, MARGIN_TOP + 20, WIDTH - MARGIN_RIGHT, HEIGHT - MARGIN_BOTTOM)
    delta = plan['state']['mass_kg'] * G
    top_moment = delta * 0.05 * math.sin(math.radians(30.0))
    sx2, sy2 = _panel(draw, box_b, (0.0, 30.0), (0.0, top_moment * 1.08),
                      'B. righting moment (N*m) vs heel angle (deg)')
    for gm, colour in ((gm_now, CURVE), (target_gm, CURVE2)):
        curve = [(sx2(phi), sy2(delta * gm * math.sin(math.radians(phi))))
                 for phi in [i * 0.25 for i in range(121)]]
        draw.line(curve, fill=colour, width=3)

    limit_deg = 5.0
    draw.line([(sx2(limit_deg), box_b[1]), (sx2(limit_deg), box_b[3])],
              fill=(120, 120, 120), width=2)
    draw.text((sx2(limit_deg) + 8, box_b[1] + 8), 'operating limit 5 deg',
              fill=(90, 90, 90), font=small)
    moment_now = delta * gm_now * math.sin(math.radians(limit_deg))
    moment_target = delta * target_gm * math.sin(math.radians(limit_deg))
    draw.ellipse([sx2(limit_deg) - 5, sy2(moment_now) - 5,
                  sx2(limit_deg) + 5, sy2(moment_now) + 5], fill=CURVE)
    draw.text((sx2(limit_deg) + 14, sy2(moment_now) - 8),
              '%.2f N*m now, %.2f N*m target' % (moment_now, moment_target),
              fill=(60, 60, 60), font=small)

    # 曲线端点直接标注，省掉图例框
    label_deg = 15.0
    for gm, colour, label in ((gm_now, CURVE, 'GM=%.3f m (as built)' % gm_now),
                              (target_gm, CURVE2, 'GM=%.3f m (target)' % target_gm)):
        value = delta * gm * math.sin(math.radians(label_deg))
        _label(draw, (sx2(label_deg) + 8, sy2(value) - 20), label, colour, small)

    draw.text((MARGIN_LEFT, 20),
              'ROV ballast trim: hull 0.90 x 0.70 x 0.50 m, dry 285 kg, '
              'ballast %.2f kg at z=%.2f m' % (ballast_kg, ballast_z),
              fill=(70, 70, 70), font=_font(15))
    return image


def main(argv=None):
    parser = argparse.ArgumentParser(description='生成配平灵敏度与恢复力矩配图')
    parser.add_argument('--output', type=Path, default=Path('outputs'))
    parser.add_argument('--target-gm', type=float, default=0.05)
    args = parser.parse_args(argv)

    trim = RovTrim()
    plan = trim.ballast_plan()
    image = draw_figure(trim, plan['mass_kg'], plan['z_m'], args.target_gm)
    args.output.mkdir(parents=True, exist_ok=True)
    path = args.output / 'trim_sensitivity.png'
    image.save(path, optimize=True)
    print('已写入: %s (%.1f KB)' % (path, path.stat().st_size / 1024.0))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

