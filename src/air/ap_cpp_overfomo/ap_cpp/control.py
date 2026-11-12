"""Speed shaping: the OverFOMO channel, re-expressed over the belief grid.

The published method adapts the vehicle speed from two online scalars -
``confidence`` of the segmentation and the ``coverage ratio`` of the frame:

    speed = nominal + (1 - 2 * cr_norm) * Q_max

AP-CPP keeps that controller bit-for-bit (see
:meth:`UtilityWeights`-independent :func:`baseline_speed`) and adds a second
channel driven by the *belief*: the planner's planned information gain and the
remaining mission entropy modulate the nominal speed, so that the vehicle also
slows down where the map is still unresolved - even if the current frame
happens to look confident.

The blend is deliberately conservative: the active term is bounded by
``gain_weight * Q_max`` so the vehicle can never be commanded outside the
baseline envelope ``[nominal - Q_max, nominal + Q_max]``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from ap_cpp.utility import UtilityModel

__all__ = ["SpeedCommand", "APCPPSpeedController"]


@dataclass
class SpeedCommand:
    """A speed decision, with the terms that produced it."""

    speed: float
    baseline_speed: float
    active_term: float
    confidence: float
    coverage_ratio: float
    reason: str

    def as_dict(self) -> dict:
        return {
            "speed": self.speed,
            "baseline_speed": self.baseline_speed,
            "active_term": self.active_term,
            "confidence": self.confidence,
            "coverage_ratio": self.coverage_ratio,
            "reason": self.reason,
        }


def baseline_speed(
    coverage_ratio: float,
    nominal_speed: float = 3.0,
    q_max: float = 2.0,
    ratio_min: float = 0.05,
    ratio_max: float = 0.15,
) -> float:
    """The upstream adaptive-speed law.

    Mirrors ``GetSpeed.find_speed`` in ``get_new_speed.py``: the coverage ratio
    is normalised between ``ratio_min`` and ``ratio_max`` and mapped linearly
    onto ``[-Q_max, +Q_max]`` around the nominal speed.  Low vegetation density
    => fly faster; a dense canopy => slow down and image more carefully.
    """
    if ratio_max <= ratio_min:
        raise ValueError("ratio_max must exceed ratio_min")
    normalised = (coverage_ratio - ratio_min) / (ratio_max - ratio_min)
    normalised = float(np.clip(normalised, 0.0, 1.0))
    adjustment = (1.0 - 2.0 * normalised) * q_max
    return float(nominal_speed + adjustment)


class APCPPSpeedController:
    """Combines the baseline speed law with the active-perception signal."""

    def __init__(
        self,
        nominal_speed: float = 3.0,
        q_max: float = 2.0,
        min_speed: float = 1.0,
        max_speed: float = 8.0,
        ratio_min: float = 0.05,
        ratio_max: float = 0.15,
        gain_weight: float = 0.35,
        gain_reference: float = 1.0,
        entropy_weight: float = 0.5,
    ):
        self.nominal_speed = float(nominal_speed)
        self.q_max = float(q_max)
        self.min_speed = float(min_speed)
        self.max_speed = float(max_speed)
        self.ratio_min = float(ratio_min)
        self.ratio_max = float(ratio_max)
        self.gain_weight = float(gain_weight)
        self.gain_reference = float(gain_reference)
        self.entropy_weight = float(entropy_weight)

    def command(
        self,
        coverage_ratio: float,
        measurement_entropy: float,
        planned_gain: float = 0.0,
        mean_entropy: Optional[float] = None,
    ) -> SpeedCommand:
        """Produce a speed command for the current step.

        Parameters
        ----------
        coverage_ratio:
            Vegetated fraction of the latest frame (upstream definition).
        measurement_entropy:
            Normalised entropy reported by the perception back-end.
        planned_gain:
            Information gain the rolling-horizon planner expects from the
            upcoming observation.  Large values mean unexplored terrain ahead.
        mean_entropy:
            Mean belief entropy over the operational area; drives a global
            "still a lot to see" term.
        """
        base = baseline_speed(
            coverage_ratio,
            nominal_speed=self.nominal_speed,
            q_max=self.q_max,
            ratio_min=self.ratio_min,
            ratio_max=self.ratio_max,
        )

        confidence = float(np.clip(1.0 - measurement_entropy, 0.0, 1.0))

        # Active term: slow down when the planner expects a lot of new
        # information, or when the map as a whole is still unresolved.
        gain_term = self.gain_weight * np.tanh(planned_gain / max(self.gain_reference, 1e-9))
        entropy_term = 0.0
        if mean_entropy is not None:
            entropy_term = self.entropy_weight * float(np.clip(mean_entropy, 0.0, 1.0))
        active_term = -self.q_max * (gain_term + entropy_term)

        speed = float(np.clip(base + active_term, self.min_speed, self.max_speed))

        if active_term < -0.25:
            reason = "exploring: high expected information gain"
        elif active_term > 0.25:
            reason = "resolved: map saturated, speeding up"
        else:
            reason = "tracking reference speed"

        return SpeedCommand(
            speed=speed,
            baseline_speed=float(base),
            active_term=active_term,
            confidence=confidence,
            coverage_ratio=float(coverage_ratio),
            reason=reason,
        )

    def g_from_belief(self, mean_entropy: float, coverage_fraction: float) -> float:
        """Evaluate the upstream ``G(x, y)`` with belief-derived arguments.

        Provided so that downstream consumers of the published method (papers,
        plots, the ``check_g_func.py`` test cases) can be reproduced directly
        from the AP-CPP belief grid.
        """
        confidence = UtilityModel.confidence_from_entropy(mean_entropy)
        return UtilityModel.g_function(confidence, coverage_fraction)
