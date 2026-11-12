"""把 bench_compliance 的刚度扫描结果画成文档用的指标图。

左图：手腕稳态让位量随刚度变化（含峰值/残留两条辅助线）
右图：恢复时间与等效刚度（双纵轴），说明高刚度区饱和

用法::

    python scripts/bench_compliance.py --json bench.json
    python scripts/plot_compliance.py bench.json
"""

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, PKG_ROOT)

import matplotlib                                            # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt                              # noqa: E402
from matplotlib.ticker import NullLocator                    # noqa: E402

FONTS = ['C:/Windows/Fonts/msyh.ttc', 'C:/Windows/Fonts/simhei.ttf',
         '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
         '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc']

INK = '#22262e'
GRID = '#d8dde6'
BLUE = '#2f6fd0'
ORANGE = '#e0821a'
RED = '#c0392b'


def setup_font():
    from matplotlib import font_manager
    for path in FONTS:
        if os.path.exists(path):
            font_manager.fontManager.addfont(path)
            name = font_manager.FontProperties(fname=path).get_name()
            plt.rcParams['font.family'] = name
            break
    plt.rcParams['axes.unicode_minus'] = False


def main():
    ap = argparse.ArgumentParser(description='绘制 LAC 柔顺性指标图')
    ap.add_argument('json', help='bench_compliance.py 输出的 JSON')
    ap.add_argument('--out', default=os.path.join(
        PKG_ROOT, '..', '..', '..', 'docs', 'img', 'humanoid',
        'humanoid_g1_compliance_metrics.png'))
    args = ap.parse_args()

    setup_font()
    with open(args.json, encoding='utf-8') as fh:
        data = json.load(fh)
    rows = sorted(data['rows'], key=lambda r: r['K_lin_la'])
    ks = [r['K_lin_la'] for r in rows]
    peak = [r['peak_mm'] for r in rows]
    steady = [r['steady_mm'] for r in rows]
    settle = [r['settle_mm'] for r in rows]
    rec = [r['recover_s'] for r in rows]
    keff = [r['k_eff_N_per_m'] for r in rows]
    force = data.get('force_N', 6.0)
    body = data.get('body', 'hand')

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.2, 4.2), dpi=160)

    # ---------------- 左：让位量 ----------------
    ax1.plot(ks, peak, 'o--', color=ORANGE, lw=1.6, ms=5, label='峰值让位')
    ax1.plot(ks, steady, 'o-', color=BLUE, lw=2.2, ms=6, label='稳态让位')
    ax1.plot(ks, settle, 's-', color=RED, lw=1.4, ms=4, label='撤销后残留')
    for x, y in zip(ks, steady):
        ax1.annotate(f'{y:.1f}', (x, y), textcoords='offset points',
                     xytext=(0, 8), ha='center', fontsize=8, color=BLUE)
    lo, hi = steady[0], steady[-1]
    ax1.text(0.03, 0.94, f'低刚度比高刚度多让位 {lo / hi:.2f} 倍\n'
                         f'（{lo:.1f} mm  vs  {hi:.1f} mm）',
             transform=ax1.transAxes, fontsize=9, color=INK, va='top',
             bbox=dict(boxstyle='round,pad=0.35', fc='#f4f6fa', ec='#c9d1de', lw=0.8))
    ax1.set_xscale('log')
    ax1.set_xticks(ks)
    ax1.set_xticklabels([f'{int(k)}' for k in ks], fontsize=8)
    ax1.xaxis.set_minor_locator(NullLocator())
    ax1.set_xlabel('左臂线性刚度指令 $K_{lin,la}$ (N/m)')
    ax1.set_ylabel('手腕让位量 (mm)')
    ax1.set_title(f'外力 {force:.0f} N 作用在手腕时的柔顺让位', fontsize=11, color=INK)
    ax1.grid(alpha=0.28, color=GRID)
    ax1.legend(fontsize=8, loc='upper right', framealpha=0.9)
    ax1.set_ylim(0, max(peak) * 1.5)
    ax1.set_xlim(ks[0] * 0.82, ks[-1] * 1.30)

    # ---------------- 右：恢复时间 + 等效刚度 ----------------
    ax2.plot(ks, rec, 'o-', color=ORANGE, lw=2.0, ms=6, label='恢复时间')
    ax2.set_xscale('log')
    ax2.set_xticks(ks)
    ax2.set_xticklabels([f'{int(k)}' for k in ks], fontsize=8)
    ax2.xaxis.set_minor_locator(NullLocator())
    ax2.set_xlabel('左臂线性刚度指令 $K_{lin,la}$ (N/m)')
    ax2.set_ylabel('恢复时间 (s)', color=ORANGE)
    ax2.tick_params(axis='y', labelcolor=ORANGE)
    ax2.grid(alpha=0.28, color=GRID)
    ax2.set_ylim(0, max(rec) * 1.3)
    ax2.set_xlim(ks[0] * 0.82, ks[-1] * 1.30)

    ax3 = ax2.twinx()
    ax3.plot(ks, keff, 's--', color=BLUE, lw=1.8, ms=5, label='等效刚度')
    ax3.set_ylabel('等效刚度 $F/\\delta$ (N/m)', color=BLUE)
    ax3.tick_params(axis='y', labelcolor=BLUE)
    ax3.set_ylim(0, max(keff) * 1.35)
    ax3.annotate(f'饱和于 {max(keff):.0f} N/m', xy=(ks[-1], keff[-1]),
                 xytext=(-92, -26), textcoords='offset points', fontsize=8,
                 color=BLUE,
                 arrowprops=dict(arrowstyle='->', color=BLUE, lw=0.9))
    ax2.set_title('撤销外力后的恢复时间与等效刚度', fontsize=11, color=INK)

    lines = ax2.get_lines() + ax3.get_lines()
    ax2.legend(lines, [ln.get_label() for ln in lines], fontsize=8,
               loc='center right', framealpha=0.9)

    fig.suptitle(f'LAC 全身柔顺控制 · G1-23DOF · 作用点 {body}',
                 fontsize=12, color=INK, y=0.99)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    fig.savefig(args.out, bbox_inches='tight')
    print(f'已保存 {args.out}')
    print(f'  刚度 {ks}')
    print(f'  稳态 {[round(v, 1) for v in steady]} mm')
    print(f'  恢复 {[round(v, 2) for v in rec]} s')


if __name__ == '__main__':
    main()
