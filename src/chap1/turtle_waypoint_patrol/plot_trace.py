#!/usr/bin/env python3
"""Plot recorded turtlesim poses; requires matplotlib, never simulates poses."""
import argparse
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('trace_csv')
parser.add_argument('output_png')
args = parser.parse_args()
with open(args.trace_csv, newline='') as stream:
    rows = list(csv.DictReader(stream))
if not rows:
    parser.error('trace has no recorded poses')
x = [float(r['x']) for r in rows]
y = [float(r['y']) for r in rows]
fig, ax = plt.subplots(figsize=(6, 6))
ax.plot(x, y, label='Recorded turtlesim pose')
ax.scatter([x[0], x[-1]], [y[0], y[-1]], label='Start / end')
ax.set(xlabel='x (turtlesim units)', ylabel='y (turtlesim units)', xlim=(0,11), ylim=(0,11))
ax.set_aspect('equal')
ax.grid(True)
ax.legend()
fig.tight_layout()
fig.savefig(args.output_png, dpi=150)
