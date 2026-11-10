"""三级滞回状态机：把横向偏移变成预警事件。

阈值和状态定义照搬仓库里基于地图的 LDW 模块，这样两边的输出可以直接对照，
差别只在偏移是怎么算出来的：这边来自图像，那边来自地图路点。
"""
NORMAL = 'normal'
DEVIATION = 'deviation'
DEPARTURE = 'departure'

# 默认阈值（米），与 carla_lane_departure_warning 保持一致
WARN_ENTER = 0.5
WARN_EXIT = 0.3
DEPART_ENTER = 0.9
DEPART_EXIT = 0.65


class LaneDepartureStateMachine(object):
    """带滞回的三级状态机，只发出真实跃迁事件。"""

    def __init__(self, warn_enter=WARN_ENTER, warn_exit=WARN_EXIT,
                 depart_enter=DEPART_ENTER, depart_exit=DEPART_EXIT):
        if not (warn_exit < warn_enter < depart_exit < depart_enter):
            raise ValueError('阈值必须满足 warn_exit < warn_enter < depart_exit < depart_enter')
        self.warn_enter = warn_enter
        self.warn_exit = warn_exit
        self.depart_enter = depart_enter
        self.depart_exit = depart_exit
        self.state = NORMAL
        self.events = []

    def update(self, offset):
        """喂一拍的横向偏移（左正右负），返回 (当前状态, 本拍事件或 None)。"""
        magnitude = abs(offset)
        direction = 'left' if offset > 0 else 'right'
        previous = self.state

        if self.state == NORMAL:
            if magnitude >= self.depart_enter:
                self.state = DEPARTURE
            elif magnitude >= self.warn_enter:
                self.state = DEVIATION
        elif self.state == DEVIATION:
            if magnitude >= self.depart_enter:
                self.state = DEPARTURE
            elif magnitude < self.warn_exit:
                self.state = NORMAL
        else:                                   # DEPARTURE
            if magnitude < self.depart_exit:
                self.state = DEVIATION

        event = self._event(previous, direction)
        if event:
            self.events.append((event, offset))
        return self.state, event

    def _event(self, previous, direction):
        if previous == self.state:
            return None
        if previous == DEPARTURE and self.state == DEVIATION:
            return 'departure_end'              # 从脱离退回偏离
        if self.state == DEVIATION:
            return 'deviation_start_%s' % direction
        if self.state == DEPARTURE:
            return 'departure_start_%s' % direction
        return 'deviation_clear'                # deviation -> normal

    def reset(self):
        self.state = NORMAL
        self.events = []


def compare_events(visual_events, map_events, tolerance=0.1):
    """比较两路事件序列：按秒对齐，统计匹配与偏差。

    两边的事件都是 (秒, 名称) 的列表；同一类事件在 tolerance 秒内
    视为匹配，多出来的分别计为视觉误报和视觉漏报。
    """
    matched, false_alarm, missed, delays = 0, 0, 0, []
    used = set()
    for moment, name in visual_events:
        hit = None
        for index, (other_moment, other_name) in enumerate(map_events):
            if index in used or other_name != name:
                continue
            if abs(other_moment - moment) <= tolerance:
                hit = index
                break
        if hit is None:
            false_alarm += 1
        else:
            used.add(hit)
            matched += 1
            delays.append(moment - map_events[hit][0])
    missed = len(map_events) - len(used)
    return {
        'matched': matched,
        'false_alarm': false_alarm,
        'missed': missed,
        'mean_delay': float(sum(delays) / len(delays)) if delays else 0.0,
        'max_delay': float(max(abs(value) for value in delays)) if delays else 0.0,
    }