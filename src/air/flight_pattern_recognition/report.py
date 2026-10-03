"""Render figures from measured model outputs. No results are invented here."""
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
from pattern_data import CLASSES

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'results'
LABELS = ['Line', 'Circle', 'Figure eight', 'Altitude variation', 'Dwell / reversal']


def save(fig, name):
    fig.savefig(OUT / name, dpi=160, bbox_inches='tight', facecolor='white')
    plt.close(fig)


def main():
    OUT.mkdir(exist_ok=True)
    plt.style.use('ggplot')
    colors = ['#16758c', '#e18b31', '#657bc4', '#4b9f72', '#b8618c']
    fig, ax = plt.subplots(figsize=(9, 4))
    for name, style in [('mlp_42', '-'), ('gru_42', '--'), ('mlp_position_only_42', ':')]:
        history = json.loads((ROOT / 'models' / name / 'loss.json').read_text())
        ax.plot([r['epoch'] for r in history], [r['validation_loss'] for r in history], style, label=name+' validation')
        if name == 'mlp_42':
            ax.plot([r['epoch'] for r in history], [r['train_loss'] for r in history], '-', color='#999999', label='mlp_42 training')
    ax.set(xlabel='Epoch', ylabel='Cross-entropy loss', title='Training / validation loss (test flights not used)')
    ax.legend()
    save(fig, 'loss.png')

    measured = json.loads((OUT / 'final/mlp_42.json').read_text())
    matrix = np.array(measured['confusion_matrix'])
    fig, ax = plt.subplots(figsize=(8, 6))
    im = ax.imshow(matrix, cmap='Blues')
    for i in range(5):
        for j in range(5):
            ax.text(j, i, str(matrix[i, j]), ha='center', va='center', color='white' if matrix[i, j] > matrix.max()*.55 else '#222222')
    ax.set_xticks(range(5)); ax.set_xticklabels(LABELS, rotation=25, ha='right')
    ax.set_yticks(range(5)); ax.set_yticklabels(LABELS)
    ax.set(xlabel='Predicted family', ylabel='Collection task family', title='Fresh final flights: raw window predictions\nAccuracy %.1f%% | macro F1 %.3f' % (100*measured['accuracy'], measured['macro_f1']))
    ax.grid(False)
    fig.colorbar(im, ax=ax, label='Windows (overlapping within each flight)')
    save(fig, 'confusion_matrix.png')

    summary = json.loads((OUT / 'final/summary.json').read_text())
    fig, ax = plt.subplots(figsize=(10, 4.5))
    x = np.arange(len(summary))
    ax.bar(x-.18, [r['accuracy'] for r in summary], .36, label='Raw accuracy', color=colors[0])
    ax.bar(x+.18, [r['macro_f1'] for r in summary], .36, label='Macro F1', color=colors[1])
    ax.set_xticks(x); ax.set_xticklabels([r['model'] for r in summary], rotation=18, ha='right')
    ax.set(ylim=(0, 1.12), ylabel='Score', title='Independent final test: seeds, ablation and non-neural baseline')
    ax.legend(loc='upper right')
    save(fig, 'model_comparison.png')

    original = json.loads((OUT / 'original_test/summary.json').read_text())
    fig, ax = plt.subplots(figsize=(10, 4.5))
    x = np.arange(len(summary))
    ax.bar(x-.18, [r['accuracy'] for r in original], .36, label='Original held-out: 5 flights / 70 windows', color=colors[1])
    ax.bar(x+.18, [r['accuracy'] for r in summary], .36, label='Fresh final: 15 flights / 315 windows', color=colors[0])
    ax.set_xticks(x); ax.set_xticklabels([r['model'] for r in summary], rotation=18, ha='right')
    ax.set(ylim=(0, 1.17), ylabel='Raw accuracy', title='Both held-out test sets: generalization depends on flight parameters')
    ax.legend(loc='upper right', fontsize=8)
    save(fig, 'test_set_comparison.png')

    with (OUT / 'final/mlp_42_predictions.csv').open(newline='') as f:
        rows = list(csv.DictReader(f))
    fig, axes = plt.subplots(5, 1, figsize=(10, 9), sharey=True, constrained_layout=True)
    for ax, episode in zip(axes, sorted({r['episode'] for r in rows})[:5]):
        selected = [r for r in rows if r['episode'] == episode]
        x = np.arange(len(selected))
        ax.plot(x, [CLASSES.index(r['truth']) for r in selected], color='#222222', linewidth=3, alpha=.35, label='Collection label')
        ax.step(x, [CLASSES.index(r['prediction']) for r in selected], where='post', color=colors[0], label='Neural prediction')
        rejected = [i for i, r in enumerate(selected) if r['accepted'] == 'False']
        if rejected:
            ax.scatter(rejected, [CLASSES.index(selected[i]['prediction']) for i in rejected], marker='x', color='#bb4e44', label='Low confidence')
        ax.set_yticks(range(5)); ax.set_yticklabels(['Line', 'Circle', 'Eight', 'Altitude', 'Dwell'])
        ax.set_title(episode, loc='left', fontsize=10)
    axes[0].legend(loc='upper right', ncol=3, fontsize=8)
    axes[-1].set_xlabel('Causal prediction window index (about 0.5 s apart)')
    fig.suptitle('Five final flights: collection label vs actual neural predictions')
    save(fig, 'prediction_timeline.png')

    fig, ax = plt.subplots(figsize=(12, 4.4))
    ax.set(xlim=(0, 12), ylim=(0, 4.4))
    ax.axis('off')
    boxes = [(0.1, 2.6, 'AirSim / saved CSV\nposition, velocity, acceleration'),
             (3.1, 2.6, 'ROS2 telemetry\ninteger time + segment reset'),
             (6.1, 2.6, 'Causal 6-second window\n8 geometric features'),
             (9.1, 2.6, 'Time mean + std\nMLP: 16 -> 32 -> 5'),
             (9.1, .5, 'Confidence + 3-step stability\nclass / uncertain / event'),
             (6.1, .5, 'ROS2 topics + JSONL\npredictions and task log'),
             (3.1, .5, 'Live subscriber dashboard\ntrajectory, scores, timeline')]
    for x0, y0, text in boxes:
        ax.add_patch(FancyBboxPatch((x0, y0), 2.7, 1.1, boxstyle='round,pad=0.06', facecolor='#eaf4f6', edgecolor=colors[0]))
        ax.text(x0+1.35, y0+.55, text, ha='center', va='center', fontsize=9)
    for a, b in [((2.85, 3.15), (3.02, 3.15)), ((5.85, 3.15), (6.02, 3.15)), ((8.85, 3.15), (9.02, 3.15)),
                 ((10.45, 2.5), (10.45, 1.7)), ((9.02, 1.05), (8.85, 1.05)), ((6.02, 1.05), (5.85, 1.05))]:
        ax.annotate('', xy=b, xytext=a, arrowprops=dict(arrowstyle='->', lw=1.7, color=colors[0]))
    ax.text(.1, .85, 'Offline: whole-flight split\ntrain-only normalization\nvalidation-only selection', fontsize=10)
    ax.set_title('UAV flight pattern recognition and automatic task log', weight='bold')
    save(fig, 'architecture.png')


if __name__ == '__main__':
    main()
