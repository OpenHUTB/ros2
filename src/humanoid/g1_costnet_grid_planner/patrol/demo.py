import json

import numpy as np
from PIL import Image, ImageDraw

from .planning import astar, predict_costs


def run(weights, output):
    grid = np.zeros((32, 32), dtype=bool)
    grid[5:26, 10:13] = True
    grid[8:29, 21:24] = True
    start, goal = (27, 3), (4, 28)
    costs = predict_costs(grid, goal, weights)
    baseline, baseline_expanded = astar(grid, start, goal)
    assisted, assisted_expanded = astar(grid, start, goal, costs)
    if len(assisted) != len(baseline):
        raise RuntimeError('Search methods disagree on optimal path cost')
    output.mkdir(parents=True, exist_ok=True)
    report = {
        'path_cost': len(assisted) - 1,
        'baseline_expanded': baseline_expanded,
        'assisted_expanded': assisted_expanded,
        'baseline_route': baseline,
        'assisted_route': assisted,
    }
    (output / 'planner.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    image = Image.new('RGB', (640, 355), 'white')
    draw = ImageDraw.Draw(image)
    for panel, (route, label) in enumerate((
        (baseline, f'Manhattan A* ({baseline_expanded} expanded)'),
        (assisted, f'CostNet A* ({assisted_expanded} expanded)'),
    )):
        left = 16 + 320 * panel
        draw.text((left, 9), label, fill='#13202a')
        for y, x in np.argwhere(grid):
            draw.rectangle((left+x*9, 30+y*9, left+x*9+8, 30+y*9+8), fill='#333b42')
        draw.line([(left+x*9+4, 34+y*9) for y,x in route], fill='#087a61', width=3)
        for y, x in (start, goal):
            draw.ellipse((left+x*9, 30+y*9, left+x*9+8, 30+y*9+8), fill='#b04827')
    draw.text((16, 330), f'Path cost: {report["path_cost"]} cells for both methods', fill='#13202a')
    image.save(output / 'planner.png')
    return report
