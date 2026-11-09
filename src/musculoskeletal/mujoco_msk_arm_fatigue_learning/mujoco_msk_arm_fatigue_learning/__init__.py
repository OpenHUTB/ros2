"""Fatigue dynamics and online learning for the musculoskeletal arm."""

from .fatigue import MuscleFatigueModel
from .learning import AdaptiveFatigueCompensator

__all__ = [
    'AdaptiveFatigueCompensator',
    'MuscleFatigueModel',
]
