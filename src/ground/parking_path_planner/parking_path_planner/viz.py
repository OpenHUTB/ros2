"""把规划结果渲染成 SVG，便于在没有 RViz2 的环境下直接查看轨迹。"""

import math

from .planner import count_gear_shifts, goal_pose_from_slot, slot_corners_world

WIDTH = 680
HEIGHT = 520
PAD = 56


def _fmt(value):
    return f'{value:.1f}'


def render_svg(result, slot, vehicle, start, outfile):
    """渲染泊车轨迹并写入 ``outfile``。"""
    if not result.success:
        raise ValueError('规划失败，无可渲染的轨迹')

    points = [(p[0], p[1]) for p in result.poses]
    corners = slot_corners_world(slot)
    goal = goal_pose_from_slot(slot, vehicle)

    xs = [p[0] for p in points] + [c[0] for c in corners] + [start[0]]
    ys = [p[1] for p in points] + [c[1] for c in corners] + [start[1]]
    span_x = max(max(xs) - min(xs), 1.0)
    span_y = max(max(ys) - min(ys), 1.0)
    scale = min((WIDTH - 2 * PAD) / span_x, (HEIGHT - 2 * PAD) / span_y)
    offset_x = (WIDTH - span_x * scale) / 2.0 - min(xs) * scale
    offset_y = (HEIGHT - span_y * scale) / 2.0 - min(ys) * scale

    def px(x, y):
        return x * scale + offset_x, HEIGHT - (y * scale + offset_y)

    def polyline(seq, color, width, dash=None):
        path = ' '.join(f'{px(x, y)[0]:.1f},{px(x, y)[1]:.1f}' for x, y in seq)
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ''
        return (f'<polyline points="{path}" fill="none" stroke="{color}" '
                f'stroke-width="{width}"{dash_attr} stroke-linecap="round" '
                f'stroke-linejoin="round"/>')

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {WIDTH} {HEIGHT}" '
        f'width="100%" font-family="system-ui, -apple-system, sans-serif">',
        f'<rect width="{WIDTH}" height="{HEIGHT}" fill="#ffffff"/>',
    ]

    # 车位
    slot_pts = ' '.join(f'{px(x, y)[0]:.1f},{px(x, y)[1]:.1f}' for x, y in corners)
    parts.append(f'<polygon points="{slot_pts}" fill="#E6F1FB" stroke="#185FA5" '
                 f'stroke-width="1.4"/>')
    # 入口边线
    parts.append(polyline([corners[0], corners[1]], '#5F5E5A', 2.0, dash='7 5'))

    # 车体轮廓
    step = max(1, len(points) // 14)
    for i in range(0, len(points), step):
        x, y, yaw = result.poses[min(i, len(result.poses) - 1)][:3]
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        half_l, half_w = vehicle.length / 2.0, vehicle.width / 2.0
        body = []
        for dx, dy in ((-half_l, -half_w), (half_l, -half_w),
                       (half_l, half_w), (-half_l, half_w)):
            wx = x + dx * cos_y - dy * sin_y
            wy = y + dx * sin_y + dy * cos_y
            body.append(f'{px(wx, wy)[0]:.1f},{px(wx, wy)[1]:.1f}')
        parts.append(f'<polygon points="{" ".join(body)}" fill="none" '
                     f'stroke="#8A8A85" stroke-width="0.8" opacity="0.55"/>')

    # 轨迹：前进蓝、倒车橙
    for gear, color in ((1, '#185FA5'), (-1, '#D85A30')):
        segment = [(p[0], p[1]) for p in result.poses if p[3] == gear]
        if len(segment) >= 2:
            parts.append(polyline(segment, color, 3.0))

    # 起点与目标
    sx, sy = px(start[0], start[1])
    gx, gy = px(goal[0], goal[1])
    parts.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="7" fill="#639922"/>')
    parts.append(f'<text x="{sx + 13:.1f}" y="{sy + 5:.1f}" font-size="13" '
                 f'fill="#3B6D11">起点</text>')
    parts.append(f'<circle cx="{gx:.1f}" cy="{gy:.1f}" r="7" fill="#E24B4A"/>')
    parts.append(f'<text x="{gx + 13:.1f}" y="{gy + 5:.1f}" font-size="13" '
                 f'fill="#A32D2D">目标位姿</text>')

    parts.append(f'<text x="{PAD}" y="{HEIGHT - 18}" font-size="13" fill="#5F5E5A">'
                 f'蓝：前进　橙：倒车　灰框：车辆轮廓　　里程 '
                 f'{result.cost:.2f} m　换挡 {count_gear_shifts(result.poses)} 次'
                 f'</text>')
    parts.append('</svg>')

    with open(outfile, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(parts))
    return outfile
