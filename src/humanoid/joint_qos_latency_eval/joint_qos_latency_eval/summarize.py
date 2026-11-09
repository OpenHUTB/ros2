"""读取 joint_qos_sub 产出的 CSV，打印各场景对比表。"""

import csv
import sys

from .stats import summarize

FIELDS = ('seq', 'stamp_ns', 'recv_ns', 'latency_ms', 'gap')
COLUMNS = (
    ('file', 'file'),
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


def format_table(paths):
    header = [title for _, title in COLUMNS]
    lines = ['| ' + ' | '.join(header) + ' |',
             '|' + '|'.join(['---'] * len(header)) + '|']
    for path in paths:
        summary = summarize(load_rows(path))
        values = []
        for key, _title in COLUMNS:
            if key == 'file':
                values.append('%s' % path)
            elif key == 'drop_rate':
                values.append('%.1f%%' % (summary[key] * 100.0))
            elif key == 'count' or key == 'missing' or key == 'out_of_order':
                values.append('%d' % summary[key])
            else:
                values.append('%.3f' % summary[key])
        lines.append('| ' + ' | '.join(values) + ' |')
    return '\n'.join(lines)


def main(argv=None):
    paths = list(sys.argv[1:] if argv is None else argv)
    if not paths:
        print('usage: qos_summarize <csv> [csv ...]', file=sys.stderr)
        return 1
    print(format_table(paths))
    return 0


if __name__ == '__main__':
    sys.exit(main())
