"""读取 joint_qos_sub 产出的 CSV，打印各场景对比表。

默认每个 CSV 占一行；加 ``--median`` 时按文件名前缀分组
（``<profile>.r<N>.csv`` 归到 ``<profile>``），对每个指标取各次运行的中位数。
重复测量取中位数是本模块约定的口径：单次结果对系统负载很敏感。

``--expected N`` 指定发布端应发总条数（RATE × DURATION），missing/drop%
以它为分母；不给时退化为按 CSV 里的序号区间（max(seq)+1）统计。
"""

import csv
import os
import statistics
import sys
from collections import OrderedDict

from .stats import summarize

FIELDS = ('seq', 'stamp_ns', 'recv_ns', 'latency_ms', 'gap')
METRICS = (
    ('count', 'count'),
    ('missing', 'missing'),
    ('drop_rate', 'drop%'),
    ('latency_p50_ms', 'p50'),
    ('latency_p95_ms', 'p95'),
    ('latency_p99_ms', 'p99'),
    ('latency_max_ms', 'max'),
    ('jitter_ms', 'jitter'),
    ('out_of_order', 'ooo'),
)
PER_FILE_COLUMNS = (('file', 'file'),) + METRICS
MEDIAN_COLUMNS = (('profile', 'profile'), ('runs', 'runs')) + METRICS


def load_rows(path):
    rows = []
    with open(path, newline='') as handle:
        for record in csv.DictReader(handle):
            missing = set(FIELDS) - set(record)
            if missing:
                raise ValueError('%s: missing columns %s' % (path, sorted(missing)))
            rows.append({
                'seq': int(record['seq']),
                'stamp_ns': int(record['stamp_ns']),
                'recv_ns': int(record['recv_ns']),
                'latency_ms': float(record['latency_ms']),
                'gap': int(record['gap']),
            })
    return rows


def profile_of(path):
    """``results/<profile>.r<N>.csv`` → ``<profile>``；不符合命名时退化为文件名。"""
    base = os.path.basename(path)
    if base.endswith('.csv'):
        base = base[:-4]
    return base.split('.r')[0]


def _format(key, value):
    if key == 'drop_rate':
        return '%.1f%%' % (value * 100.0)
    if key in ('count', 'missing', 'out_of_order'):
        return '%d' % round(value)
    return '%.3f' % value


def _render(header, rows):
    lines = ['| ' + ' | '.join(header) + ' |',
             '|' + '|'.join(['---'] * len(header)) + '|']
    for values in rows:
        lines.append('| ' + ' | '.join(values) + ' |')
    return '\n'.join(lines)


def format_table(paths, expected_msgs=0):
    rows = []
    for path in paths:
        summary = summarize(load_rows(path), expected_msgs)
        rows.append([path if key == 'file' else _format(key, summary[key])
                     for key, _title in PER_FILE_COLUMNS])
    return _render([title for _, title in PER_FILE_COLUMNS], rows)


def format_median_table(paths, expected_msgs=0):
    groups = OrderedDict()
    for path in paths:
        groups.setdefault(profile_of(path), []).append(path)

    rows = []
    for profile, group in groups.items():
        summaries = [summarize(load_rows(path), expected_msgs)
                     for path in group]
        values = []
        for key, _title in MEDIAN_COLUMNS:
            if key == 'profile':
                values.append(profile)
            elif key == 'runs':
                values.append('%d' % len(group))
            else:
                values.append(_format(
                    key, statistics.median([s[key] for s in summaries])))
        rows.append(values)
    return _render([title for _, title in MEDIAN_COLUMNS], rows)


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    median = '--median' in args
    expected = 0
    if '--expected' in args:
        index = args.index('--expected')
        expected = int(args[index + 1])
        del args[index:index + 2]
    paths = [arg for arg in args if arg != '--median']
    if not paths:
        print('usage: qos_summarize [--median] [--expected N] <csv> [csv ...]',
              file=sys.stderr)
        return 1
    print(format_median_table(paths, expected) if median
          else format_table(paths, expected))
    return 0


if __name__ == '__main__':
    sys.exit(main())
