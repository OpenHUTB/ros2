"""Check offline / streaming agreement on identical causal windows."""
import argparse
import json
from pathlib import Path
import numpy as np
import torch
from pattern_data import KEYS, CLASSES, read_flight
from pattern_stream import Recognizer
from pattern_model import PatternNet


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--split', default='final')
    parser.add_argument('--raw', type=Path, default=Path('data/final_raw'))
    args = parser.parse_args()
    data = np.load('data/prepared/'+args.split+'.npz')
    folder = Path('models/mlp_42')
    config = json.loads((folder/'config.json').read_text())
    model = PatternNet(config['variant'])
    model.load_state_dict(torch.load(folder/'model.pt', weights_only=True))
    model.eval()
    with torch.no_grad():
        offline = torch.softmax(model(torch.tensor((data['x']-config['mean'])/config['std'], dtype=torch.float32)), 1).numpy()
    events = []
    checked = 0
    for episode in sorted(set(data['episodes'])):
        stream = Recognizer(folder)
        _, stamps, values = read_flight(args.raw/(episode+'.csv'))
        expected = {int(data['row_indices'][i]): i for i in np.flatnonzero(data['episodes'] == episode)}
        for row_index, (stamp, z) in enumerate(zip(stamps, values)):
            result = stream.update(dict(timestamp_ns=int(stamp), segment_id=str(episode), **dict(zip(KEYS, z.tolist()))))
            if result is not None:
                if row_index not in expected:
                    raise AssertionError('Unexpected streaming window')
                np.testing.assert_allclose(result['probabilities'], offline[expected[row_index]], atol=2e-6)
                checked += 1
                if result['event']:
                    events.append(dict(result['event'], elapsed_s=result['elapsed_s']))
    if checked != len(offline):
        raise AssertionError('Missing streaming windows')
    output = {'split': args.split, 'matching_windows': checked, 'max_allowed_probability_error': 2e-6,
              'events': events, 'event_count': len(events)}
    Path('results/stream_agreement.json').write_text(json.dumps(output, indent=2))
    print('STREAM_AGREEMENT', checked, 'events', len(events))


if __name__ == '__main__':
    main()
