"""Causal inference and debounced task events, shared by ROS and offline replay."""
import json
from collections import deque
from pathlib import Path
import numpy as np
from pattern_data import CLASSES, KEYS, sample_window


class Recognizer:
    def __init__(self, folder=None, predict=None, config=None):
        if predict is None:
            import torch
            from pattern_model import PatternNet
            torch.set_num_threads(3)
            folder = Path(folder)
            config = json.loads((folder / 'config.json').read_text())
            model = PatternNet(config['variant'])
            model.load_state_dict(torch.load(folder / 'model.pt', map_location='cpu', weights_only=True))
            model.eval()

            def predict(x):
                with torch.no_grad():
                    return torch.softmax(model(torch.from_numpy(x[None])), 1).numpy()[0]
        self.predict = predict
        self.config = config
        self.mean = np.asarray(config['mean'], dtype=np.float32)
        self.std = np.asarray(config['std'], dtype=np.float32)
        self.reset()

    def reset(self):
        self.samples = deque(maxlen=1000)
        self.segment = None
        self.start = None
        self.next_time = None
        self.candidate = None
        self.count = 0
        self.stable = 'unknown'

    def update(self, message):
        try:
            stamp = message['timestamp_ns']
            if isinstance(stamp, bool) or not isinstance(stamp, int):
                raise ValueError('timestamp_ns must be an integer')
            segment = str(message.get('segment_id', 'live'))
            values = np.asarray([message[k] for k in KEYS], dtype=np.float64)
            if not np.isfinite(values).all():
                raise ValueError('Non-finite telemetry')
        except (KeyError, TypeError, ValueError):
            self.reset()
            raise ValueError('Invalid telemetry; history reset') from None
        if self.samples and (segment != self.segment or stamp <= self.samples[-1][0]
                             or stamp - self.samples[-1][0] > 150_000_000):
            self.reset()
        if self.start is None:
            self.start = stamp
            self.next_time = stamp + int(self.config['warmup_s'] * 1e9)
            self.segment = segment
        self.samples.append((stamp, values))
        if stamp < self.next_time:
            return None
        self.next_time = stamp + int(self.config['inference_period_s'] * 1e9)
        stamps, values = zip(*self.samples)
        x = sample_window(stamps, values, stamp)
        if x is None:
            self.reset()
            return None
        probabilities = np.asarray(self.predict(((x - self.mean) / self.std).astype(np.float32)))
        if probabilities.shape != (5,) or not np.isfinite(probabilities).all():
            self.reset()
            raise ValueError('Invalid model output')
        raw = int(probabilities.argmax())
        confidence = float(probabilities[raw])
        accepted = confidence >= self.config['confidence_threshold']
        label = CLASSES[raw] if accepted else 'uncertain'
        if label == self.candidate:
            self.count += 1
        else:
            self.candidate, self.count = label, 1
        event = None
        if self.count >= self.config['stable_predictions'] and label != self.stable:
            event = {'timestamp_ns': stamp, 'segment_id': segment,
                     'previous': self.stable, 'label': label, 'confidence': confidence}
            self.stable = label
        return {'timestamp_ns': stamp, 'segment_id': segment, 'raw_label': CLASSES[raw],
                'label': label, 'confidence': confidence, 'probabilities': probabilities.tolist(),
                'stable_label': self.stable, 'elapsed_s': (stamp - self.start) / 1e9,
                'event': event}
