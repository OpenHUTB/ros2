"""Tabular-free reinforcement learning baselines implemented with MLPs.

``DQN`` (value based, replay buffer + target network) and ``PolicyGradient``
(REINFORCE with a running baseline) are used to compare two different RL
algorithms on the same discrete-action navigation task.
"""

from __future__ import annotations

import json
import os
from collections import deque
from typing import List, Tuple

import numpy as np

from .losses import softmax
from .mlp import MLP


class ReplayBuffer:
    """Fixed-capacity experience replay buffer."""

    def __init__(self, capacity: int = 50000):
        self.capacity = int(capacity)
        self.data = deque(maxlen=self.capacity)

    def add(self, state, action: int, reward: float, next_state, done: bool):
        self.data.append((np.asarray(state, dtype=np.float64),
                          int(action), float(reward),
                          np.asarray(next_state, dtype=np.float64),
                          bool(done)))

    def __len__(self) -> int:
        return len(self.data)

    def sample(self, batch_size: int, rng: np.random.Generator):
        idx = rng.integers(0, len(self.data), size=int(batch_size))
        batch = [self.data[i] for i in idx]
        s = np.stack([b[0] for b in batch])
        a = np.asarray([b[1] for b in batch], dtype=np.int64)
        r = np.asarray([b[2] for b in batch], dtype=np.float64)
        s2 = np.stack([b[3] for b in batch])
        d = np.asarray([1.0 if b[4] else 0.0 for b in batch],
                       dtype=np.float64)
        return s, a, r, s2, d


class DQN:
    """Deep Q-network with epsilon-greedy exploration and a target network."""

    def __init__(self, state_dim: int, n_actions: int, hidden=(64, 64),
                 seed: int = 2026, lr: float = 5e-4, gamma: float = 0.99,
                 buffer_size: int = 50000, batch_size: int = 32,
                 target_sync: int = 200):
        self.state_dim = int(state_dim)
        self.n_actions = int(n_actions)
        self.hidden = tuple(int(h) for h in hidden)
        sizes = [self.state_dim, *self.hidden, self.n_actions]
        self.net = MLP(sizes, activation="relu", seed=seed, lr=lr)
        self.target = MLP(sizes, activation="relu", seed=seed + 1, lr=lr)
        self.target.copy_params_from(self.net)
        self.gamma = float(gamma)
        self.batch_size = int(batch_size)
        self.target_sync = int(target_sync)
        self.buffer = ReplayBuffer(buffer_size)
        self.rng = np.random.default_rng((seed + 7) & 0xFFFFFFFF)
        self.steps = 0
        self.last_loss = None

    def act(self, state: np.ndarray, epsilon: float = 0.0) -> int:
        if self.rng.random() < float(epsilon):
            return int(self.rng.integers(self.n_actions))
        q = self.net.predict(np.asarray(state, dtype=np.float64))
        return int(np.argmax(q))

    def remember(self, state, action, reward, next_state, done) -> None:
        self.buffer.add(state, action, reward, next_state, done)

    def train_step(self) -> float:
        if len(self.buffer) < self.batch_size:
            return 0.0
        s, a, r, s2, done = self.buffer.sample(self.batch_size, self.rng)
        q = self.net.forward(s)
        q_next = self.target.predict(s2).max(axis=1)
        target = r + self.gamma * (1.0 - done) * q_next
        pred = q[np.arange(len(a)), a]
        diff = pred - target
        loss = float(np.mean(diff * diff))
        dq = np.zeros_like(q)
        dq[np.arange(len(a)), a] = 2.0 * diff / len(a)
        self.net.backward(dq)
        self.net.step()
        self.steps += 1
        if self.target_sync > 0 and self.steps % self.target_sync == 0:
            self.target.copy_params_from(self.net)
        self.last_loss = loss
        return loss

    def update_target(self) -> None:
        self.target.copy_params_from(self.net)

    # ------------------------------------------------------------------- io
    def save(self, path: str, extra: dict = None) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        layers = [l for l in self.net.layers if hasattr(l, "W")]
        payload = {
            "kind": np.asarray("dqn"),
            "state_dim": np.asarray(self.state_dim, dtype=np.int64),
            "n_actions": np.asarray(self.n_actions, dtype=np.int64),
            "hidden": np.asarray(self.hidden, dtype=np.int64),
            "gamma": np.asarray(self.gamma),
            "lr": np.asarray(float(self.net.opt.lr)),
        }
        for i, layer in enumerate(layers):
            payload["W_%d" % i] = layer.W
            payload["b_%d" % i] = layer.b
        np.savez(path, **payload)
        if extra is not None:
            with open(os.path.splitext(path)[0] + ".json", "w",
                      encoding="utf-8") as fh:
                json.dump(extra, fh, indent=2, sort_keys=True)

    @classmethod
    def load(cls, path: str) -> "DQN":
        with np.load(path, allow_pickle=False) as data:
            state_dim = int(data["state_dim"])
            n_actions = int(data["n_actions"])
            hidden = tuple(int(h) for h in data["hidden"])
            gamma = float(data["gamma"])
            lr = float(data["lr"])
            obj = cls(state_dim, n_actions, hidden=hidden, seed=0, lr=lr,
                      gamma=gamma)
            layers = [l for l in obj.net.layers if hasattr(l, "W")]
            for i, layer in enumerate(layers):
                layer.W[...] = data["W_%d" % i]
                layer.b[...] = data["b_%d" % i]
            obj.update_target()
        return obj


class PolicyGradient:
    """REINFORCE with a batch-mean baseline (softmax policy MLP)."""

    def __init__(self, state_dim: int, n_actions: int, hidden=(64, 64),
                 seed: int = 2026, lr: float = 1e-3, gamma: float = 0.99):
        self.state_dim = int(state_dim)
        self.n_actions = int(n_actions)
        self.hidden = tuple(int(h) for h in hidden)
        sizes = [self.state_dim, *self.hidden, self.n_actions]
        self.net = MLP(sizes, activation="tanh", seed=seed, lr=lr)
        self.rng = np.random.default_rng((seed + 11) & 0xFFFFFFFF)
        self.gamma = float(gamma)
        self.baseline = 0.0
        self.baseline_decay = 0.95
        self.last_loss = None

    def act(self, state: np.ndarray) -> Tuple[int, float]:
        logits = self.net.predict(np.asarray(state, dtype=np.float64))
        probs = softmax(logits)
        action = int(self.rng.choice(self.n_actions, p=probs))
        return action, float(probs[action])

    def act_greedy(self, state: np.ndarray) -> int:
        """Deterministic (argmax) action, used for evaluation."""
        logits = self.net.predict(np.asarray(state, dtype=np.float64))
        return int(np.argmax(logits))

    @staticmethod
    def compute_returns(rewards, gamma: float = 0.99) -> np.ndarray:
        out = np.zeros(len(rewards), dtype=np.float64)
        running = 0.0
        for i in range(len(rewards) - 1, -1, -1):
            running = float(rewards[i]) + gamma * running
            out[i] = running
        return out

    def update(self, states: np.ndarray, actions, returns) -> float:
        states = np.asarray(states, dtype=np.float64)
        actions = np.asarray(actions, dtype=np.int64)
        returns = np.asarray(returns, dtype=np.float64)
        if states.ndim == 1:
            states = states[None, :]
        n = states.shape[0]
        # running baseline for variance reduction
        self.baseline = (self.baseline_decay * self.baseline
                         + (1.0 - self.baseline_decay) * float(returns.mean()))
        adv = returns - self.baseline
        std = float(adv.std())
        if std > 1e-6:
            adv = adv / std
        logits = self.net.forward(states)
        probs = softmax(logits)
        onehot = np.zeros_like(probs)
        onehot[np.arange(n), actions] = 1.0
        picked = np.clip(probs[np.arange(n), actions], 1e-12, 1.0)
        loss = float(-np.mean(adv * np.log(picked)))
        # gradient of -mean(adv * log pi(a|s))
        dlogits = -(adv[:, None] * (onehot - probs)) / n
        self.net.backward(dlogits)
        self.net.step()
        self.last_loss = loss
        return loss

    # ------------------------------------------------------------------- io
    def save(self, path: str, extra: dict = None) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        layers = [l for l in self.net.layers if hasattr(l, "W")]
        payload = {
            "kind": np.asarray("pg"),
            "state_dim": np.asarray(self.state_dim, dtype=np.int64),
            "n_actions": np.asarray(self.n_actions, dtype=np.int64),
            "hidden": np.asarray(self.hidden, dtype=np.int64),
            "gamma": np.asarray(self.gamma),
            "lr": np.asarray(float(self.net.opt.lr)),
        }
        for i, layer in enumerate(layers):
            payload["W_%d" % i] = layer.W
            payload["b_%d" % i] = layer.b
        np.savez(path, **payload)
        if extra is not None:
            with open(os.path.splitext(path)[0] + ".json", "w",
                      encoding="utf-8") as fh:
                json.dump(extra, fh, indent=2, sort_keys=True)

    @classmethod
    def load(cls, path: str) -> "PolicyGradient":
        with np.load(path, allow_pickle=False) as data:
            state_dim = int(data["state_dim"])
            n_actions = int(data["n_actions"])
            hidden = tuple(int(h) for h in data["hidden"])
            gamma = float(data["gamma"])
            lr = float(data["lr"])
            obj = cls(state_dim, n_actions, hidden=hidden, seed=0, lr=lr,
                      gamma=gamma)
            layers = [l for l in obj.net.layers if hasattr(l, "W")]
            for i, layer in enumerate(layers):
                layer.W[...] = data["W_%d" % i]
                layer.b[...] = data["b_%d" % i]
        return obj
