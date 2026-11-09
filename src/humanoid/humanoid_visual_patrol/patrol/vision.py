"""Separate perception and direct-image control models."""
import numpy as np
import torch
from torch import nn


class FloorNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(nn.Conv2d(3, 16, 5, padding=2), nn.ReLU(),
            nn.Conv2d(16, 16, 3, padding=1), nn.ReLU(), nn.Conv2d(16, 1, 1))

    def forward(self, image):
        return self.net(image)


class ImageController(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(nn.Conv2d(3, 16, 5, stride=2, padding=2), nn.ReLU(),
            nn.Conv2d(16, 32, 3, stride=2, padding=1), nn.ReLU(),
            nn.Conv2d(32, 32, 3, stride=2, padding=1), nn.ReLU(),
            nn.Flatten(), nn.Linear(32*12*16, 64), nn.ReLU(), nn.Linear(64, 2))

    def forward(self, image):
        return self.net(image)


class MaskController(nn.Module):
    """Learned velocity policy operating on a semantic floor mask."""
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(nn.AdaptiveAvgPool2d((12,16)),nn.Flatten(),
            nn.Linear(192,64),nn.ReLU(),nn.Linear(64,32),nn.ReLU(),nn.Linear(32,2))

    def forward(self, mask):
        return self.net(mask)


def mask_command(mask):
    """Reactive floor-following teacher, with no global route/position input.

    Scores candidate image corridors; low front clearance reduces speed.
    This is a heuristic teacher, not a globally optimal navigation planner.
    """
    h, w = mask.shape
    band = mask[int(h*.48):int(h*.82)]
    scores = np.array([band[:, max(0, x-10):min(w, x+11)].mean()
                       for x in range(12, w-12, 4)])
    centers = np.arange(12, w-12, 4)
    scores -= .20*np.abs(centers-w/2)/(w/2)
    best = centers[np.argmax(scores)]
    yaw = float(np.clip((w/2-best)/(w/2)*1.2, -.45, .45))
    clearance = float(mask[int(h*.55):int(h*.85), int(w*.4):int(w*.6)].mean())
    speed = .28*float(np.clip((clearance-.15)/.6, 0, 1))*(1-.6*abs(yaw))
    return np.array([speed, yaw], dtype=np.float32)


def tensor_image(image):
    return torch.from_numpy(image.copy()).permute(2, 0, 1).float().unsqueeze(0)/255.


class VisionPolicy:
    def __init__(self, weights, mode):
        self.mode = mode
        self.net = FloorNet() if mode == 'perception' else ImageController()
        self.net.load_state_dict(torch.load(weights, map_location='cpu', weights_only=True))
        self.net.eval()
        if mode == 'perception':
            from pathlib import Path
            self.controller = MaskController()
            self.controller.load_state_dict(torch.load(Path(weights).with_name('mask_controller.pt'),
                                                       map_location='cpu',weights_only=True))
            self.controller.eval()

    def command(self, image):
        with torch.inference_mode():
            output = self.net(tensor_image(image))
        if self.mode == 'perception':
            mask = (output.sigmoid()[0, 0].numpy()>.5).astype(np.float32)
            with torch.inference_mode():
                command = self.controller(torch.from_numpy(mask)[None,None])[0].numpy()
            # A learned controller can extrapolate a forward command even when
            # segmentation detects no floor. Stop on this unambiguous failure.
            if not mask.any():
                command = np.zeros(2, dtype=np.float32)
            return np.clip(command,[0,-.45],[.3,.45]), mask
        command = output[0].numpy()
        return np.clip(command, [0, -.45], [.3, .45]), None
