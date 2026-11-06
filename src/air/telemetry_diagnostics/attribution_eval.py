"""通道归因的离线核对：在测试集场景上统计「主导通道是否落在注入通道组」。

position_drift 注入的是位置通道（p*），velocity_spike 注入的是速度通道（v*）。
归因本身只用检测器残差，不看注入标签；标签只用来算命中率。
"""
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

from stream_detector import Detector

POSITION = {'px', 'py', 'pz'}
VELOCITY = {'vx', 'vy', 'vz'}
EXPECTED = {'position_drift': POSITION, 'velocity_spike': VELOCITY}
ROOT = Path(__file__).resolve().parent


def evaluate(model='models/gru_42', output='results/attribution_eval.json'):
    data = ROOT / 'data/prepared'
    manifest = json.loads((data / 'manifest.json').read_text(encoding='utf-8'))['records']
    stats = defaultdict(lambda: {'alarmed': 0, 'hit': 0})
    examples = []
    for record in manifest:
        if record['split'] != 'test' or record['kind'] not in EXPECTED:
            continue
        folder = data / record['path']
        detector = Detector(ROOT / model)
        with (folder / 'observations.csv').open(encoding='utf-8') as f:
            results = [detector.step(row) for row in csv.DictReader(f)]
        for result in results:
            if not (result['alarm'] and result['reason'] == 'model_residual' and result.get('dominant')):
                continue
            bucket = stats[record['kind']]
            bucket['alarmed'] += 1
            hit = bool(set(result['dominant']) & EXPECTED[record['kind']])
            bucket['hit'] += int(hit)
            if len(examples) < 6:
                examples.append({'scenario': record['path'].split('/')[-1],
                                 'dominant': result['dominant'],
                                 'score': round(result['score'], 3),
                                 'hit': hit})
    report = {'model': model,
              'rule': 'alarmed steps with model_residual; hit = dominant ∩ injected channel group',
              'kinds': {kind: {**value, 'rate': round(value['hit'] / max(value['alarmed'], 1), 4)}
                        for kind, value in stats.items()},
              'examples': examples}
    target = ROOT / output
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report['kinds'], indent=2))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='models/gru_42')
    parser.add_argument('--output', default='results/attribution_eval.json')
    args = parser.parse_args()
    evaluate(args.model, args.output)
