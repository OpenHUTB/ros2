"""Active Perception Coverage Path Planning (AP-CPP).

An extension of the ``OverFOMO`` adaptive coverage path planning pipeline
(Krestenitis et al., 2023) that closes the loop between *where the robot looks*
and *how fast it moves*.

The original scheme re-uses a pre-computed boustrophedon route and only
modulates the flight speed from the semantic content of the incoming frames.
AP-CPP adds a second, orthogonal degree of freedom: the robot maintains a
belief over the field (occupancy + per-cell information entropy), scores every
candidate motion with a utility that trades coverage cost against expected
information gain, and re-optimises the remainder of the route over a rolling
horizon.

Reference
---------
M. Krestenitis, E. K. Raptis, A. C. Kapoutsis, K. Ioannidis, E. B. Kosmatopoulos,
S. Vrochidis, "Overcome the fear of missing out: Active sensing UAV scanning for
precision agriculture", Robotics and Autonomous Systems, 2023.
"""

from ap_cpp.grid_model import GridConfig, CoverageGrid
from ap_cpp.utility import UtilityWeights, UtilityModel
from ap_cpp.planner import (
    PlannerConfig,
    RollingHorizonPlanner,
    PlanResult,
    StepDiagnostics,
)
from ap_cpp.sensor import SensorConfig, SensorModel
from ap_cpp.mission import APCPPMission, DemoPerceptionModel, MissionRecord
from ap_cpp.geo_bridge import GeoBridge

__all__ = [
    "GridConfig",
    "CoverageGrid",
    "UtilityWeights",
    "UtilityModel",
    "PlannerConfig",
    "RollingHorizonPlanner",
    "PlanResult",
    "StepDiagnostics",
    "SensorConfig",
    "SensorModel",
    "APCPPMission",
    "DemoPerceptionModel",
    "MissionRecord",
    "GeoBridge",
]

__version__ = "0.1.0"
