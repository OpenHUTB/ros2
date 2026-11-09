"""通道归因的离线核对：主导通道是否落在注入通道组里，并与随机基线对照。

注入类型与通道组：position_drift 注入 p*，velocity_spike 注入 v*。
归因只用检测器残差，不看注入标签；标签只用于算命中率。

评估口径（评审指出）：
- 分母用**全部** model_residual 报警步（alarmed_total），并单列 dominant_coverage，
  说明「有多少报警步连主导通道都没给出」，不再把这类步悄悄丢掉
- 同时给出随机基线：dominant 最多 2 个通道、从 6 个里选，随机 1 个的期望命中率是 50%、
  随机 2 个是 80%。命中率必须和这个基线比，才能说明归因是否真的有效
"""
import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

from channels import CHANNELS, POSITION_GROUP, VELOCITY_GROUP
from consistency import ConsistencyDetector
from stream_detector import Detector

EXPECTED = {'position_drift': POSITION_GROUP, 'velocity_spike': VELOCITY_GROUP}
ROOT = Path(__file__).resolve().parent
CONSTANT_REPORT = ['px', 'py']   # 「恒定报位置组」的朴素基线


def chance_rate(chosen):
    """从 6 个通道里随机选 len(chosen) 个，命中 3 元目标组的概率。"""
    k = len(chosen)
    if k <= 0:
        return 0.0
    total = math.comb(len(CHANNELS), k)
    miss = math.comb(len(POSITION_GROUP), k) if k <= len(POSITION_GROUP) else 0
    return 1.0 - miss / total


def scan(detector_factory, records, data):
    stats = defaultdict(lambda: {'alarmed_total': 0, 'with_dominant': 0, 'hit': 0,
                                 'constant_hit': 0, 'chance_sum': 0.0})
    examples = []
    for record in records:
        if record['split'] != 'test' or record['kind'] not in EXPECTED:
            continue
        folder = data / record['path']
        detector = detector_factory()
        with (folder / 'observations.csv').open(encoding='utf-8') as f:
            results = [detector.step(row) for row in csv.DictReader(f)]
        for result in results:
            if not (result['alarm'] and result['reason'] == 'model_residual'):
                continue
            bucket = stats[record['kind']]
            bucket['alarmed_total'] += 1
            bucket['constant_hit'] += int(bool(set(CONSTANT_REPORT) & EXPECTED[record['kind']]))
            dominant = result.get('dominant') or []
            if not dominant:
                continue
            bucket['with_dominant'] += 1
            bucket['chance_sum'] += chance_rate(dominant)
            if set(dominant) & EXPECTED[record['kind']]:
                bucket['hit'] += 1
                if len(examples) < 6:
                    examples.append({'scenario': record['path'].split('/')[-1],
                                     'dominant': dominant,
                                     'score': round(result['score'], 3)})
    return stats, examples


def summarise(stats):
    kinds = {}
    for kind, bucket in stats.items():
        total = bucket['alarmed_total']
        covered = bucket['with_dominant']
        kinds[kind] = {
            'alarmed_total': total,
            'with_dominant': covered,
            'dominant_coverage': round(covered / total, 4) if total else 0.0,
            'hit': bucket['hit'],
            'hit_rate': round(bucket['hit'] / covered, 4) if covered else 0.0,
            'hit_rate_of_all_alarms': round(bucket['hit'] / total, 4) if total else 0.0,
            'chance_baseline': round(bucket['chance_sum'] / covered, 4) if covered else 0.0,
            'constant_position_baseline': round(bucket['constant_hit'] / total, 4) if total else 0.0,
        }
    return kinds


def evaluate(detectors=('gru', 'consistency'), output='results/attribution_eval.json'):
    data = ROOT / 'data/prepared'
    manifest = json.loads((data / 'manifest.json').read_text(encoding='utf-8'))['records']
    factories = {
        'gru': ('models/gru_42', lambda: Detector(ROOT / 'models/gru_42')),
        'consistency': ('models/consistency_42',
                        lambda: ConsistencyDetector(ROOT / 'models/consistency_42')),
    }
    report = {'rule': 'alarmed steps with reason=model_residual; hit = dominant ∩ injected channel group',
              'chance_rule': 'random k of 6 channels vs 3-channel target group: k=1 -> 50%, k=2 -> 80%',
              'detectors': {}}
    examples = []
    for name in detectors:
        model, factory = factories[name]
        stats, found = scan(factory, manifest, data)
        report['detectors'][name] = {'model': model, 'kinds': summarise(stats)}
        examples.extend(found[:3])
    report['examples'] = examples[:6]

    target = ROOT / output
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report['detectors'], indent=2, ensure_ascii=False))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--detectors', default='gru,consistency')
    parser.add_argument('--output', default='results/attribution_eval.json')
    args = parser.parse_args()
    evaluate(tuple(d.strip() for d in args.detectors.split(',') if d.strip()), args.output)
