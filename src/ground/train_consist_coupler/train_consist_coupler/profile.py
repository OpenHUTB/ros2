"""参考速度曲线：反向推制动包络 + 正向积牵引能力。

公式与 docs/ground/train_ato_stop.md 一致，只是把总质量、总牵引力和总阻力换成
编组聚合值，因此单节车和多节编组用的是同一条曲线（ATO 是列车级控制）。
"""

import math

from .params import G


class Reference:
    def __init__(self, values, dx, length):
        self.values = values
        self.dx = dx
        self.length = length

    def at(self, x):
        if x <= 0.0:
            return self.values[0]
        position = x / self.dx
        index = int(position)
        if index >= len(self.values) - 1:
            return self.values[-1]
        frac = position - index
        return self.values[index] * (1.0 - frac) + self.values[index + 1] * frac

    def min_over(self, x0, x1):
        lo = max(0, int(x0 / self.dx))
        hi = min(len(self.values) - 1, int(x1 / self.dx))
        return min(self.values[lo:hi + 1])


def build_reference(route, consist, control, dx=0.5):
    n = int(round(route.length / dx)) + 1
    mass = consist.car.mass
    gamma = consist.car.gamma

    # 第一步：从停车点反向推包络
    envelope = [0.0] * n
    for i in range(n - 2, -1, -1):
        v_next = envelope[i + 1]
        resistance_acc = consist.car.resistance(v_next) / mass
        grade_acc = G * route.grade(i * dx)
        a_eff = control.a_plan + (resistance_acc + grade_acc) / gamma
        envelope[i] = min(route.limit_at(i * dx),
                          math.sqrt(max(0.0, v_next * v_next + 2.0 * a_eff * dx)))

    # 第二步：从起点正向积牵引能力，与包络取小
    reference = [0.0] * n
    for i in range(n - 1):
        v = reference[i]
        traction_acc = consist.car.traction(v) / mass
        resistance_acc = consist.car.resistance(v) / mass
        grade_acc = G * route.grade((i + 1) * dx)
        a_cap = max(0.0, (traction_acc - resistance_acc - grade_acc) / gamma)
        reference[i + 1] = min(envelope[i + 1],
                               math.sqrt(v * v + 2.0 * a_cap * dx))

    return Reference(reference, dx, route.length), Reference(envelope, dx, route.length)
