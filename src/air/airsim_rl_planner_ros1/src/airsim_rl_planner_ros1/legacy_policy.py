"""Inference-only loader for the inspected Auto_drone 10-D/5-action checkpoint."""
from collections import deque
import numpy as np
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.buffers import RolloutBuffer
from stable_baselines3.common.policies import ActorCriticPolicy


def load_policy(path):
    # Replace pickled Python 3.11/NumPy 2 metadata, not learned weights.
    # These values match the inspected checkpoint; not a generic model converter.
    return PPO.load(path, device='cpu', custom_objects={
        'policy_class': ActorCriticPolicy,
        'observation_space': spaces.Box(0.0, 1.0, shape=(10,), dtype=np.float32),
        'action_space': spaces.Discrete(5),
        '_last_obs': None,
        '_last_episode_starts': None,
        'ep_info_buffer': deque(maxlen=100),
        'ep_success_buffer': deque(maxlen=100),
        'rollout_buffer_class': RolloutBuffer,
        'clip_range': lambda _: 0.2,
        'lr_schedule': lambda _: 0.0003,
    })
