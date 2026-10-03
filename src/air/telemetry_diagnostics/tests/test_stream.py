import json
import unittest
from pathlib import Path
import numpy as np
import torch
from train_evaluate import load_episode
from stream_detector import Detector
import csv


class StreamTests(unittest.TestCase):
    def test_online_equals_offline_for_every_scenario(self):
        root = Path('data/prepared')
        records = json.loads((root / 'manifest.json').read_text())['records']
        for record in records:
            if record['split'] != 'test': continue
            folder = root / record['path']
            episode = load_episode(folder)
            detector = Detector('models/gru_42')
            c = detector.config
            with (folder / 'observations.csv').open() as f:
                results = [detector.step(row) for row in csv.DictReader(f)]
            x = (episode['x'] - np.array(c['xm'], dtype=np.float32)) / np.array(c['xs'], dtype=np.float32)
            with torch.no_grad():
                prediction = detector.model(torch.from_numpy(x)).numpy() * c['ys'] + c['ym']
            expected = np.sqrt(np.mean(((episode['y'] - prediction) / c['residual_scale']) ** 2, axis=1))
            actual = [results[i]['score'] for i in episode['indices']]
            np.testing.assert_allclose(actual, expected, rtol=1e-3, atol=1e-4)


if __name__ == '__main__': unittest.main()
