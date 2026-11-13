# Copyright 2026 xxbzy727749
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for power-line inspection route planning."""

import math

import pytest

from uav_powerline_inspection.planner import build_inspection_route, estimate_route


def test_route_inspects_both_sides_of_each_pole():
    """Each pole should receive left and right viewpoints."""
    route = build_inspection_route([(0, 0, 12), (30, 0, 14)], standoff=5, height_margin=2)

    assert len(route) == 4
    assert {point["side"] for point in route} == {"left", "right"}
    assert route[0]["y"] == pytest.approx(5)
    assert route[1]["y"] == pytest.approx(-5)
    assert route[0]["z"] == pytest.approx(14)
    assert abs(route[0]["yaw_deg"]) == pytest.approx(90)


def test_curved_corridor_uses_local_direction():
    """A curved corridor should use the local pole direction."""
    route = build_inspection_route([(0, 0, 10), (10, 10, 10), (10, 20, 10)], standoff=2)

    first = route[0]
    assert first["x"] == pytest.approx(-math.sqrt(2))
    assert first["y"] == pytest.approx(math.sqrt(2))
    assert route[-1]["x"] == pytest.approx(12)


def test_duplicate_middle_poles_are_rejected():
    """Adjacent poles at the same position should be rejected."""
    poles = [(0, 0, 10), (10, 0, 10), (10, 0, 10), (20, 0, 10)]

    with pytest.raises(ValueError, match="adjacent poles"):
        build_inspection_route(poles)


def test_reversing_to_previous_position_is_rejected():
    """A local direction that collapses to zero should be rejected."""
    with pytest.raises(ValueError, match="local pole direction"):
        build_inspection_route([(0, 0, 10), (10, 0, 10), (0, 0, 10)])


def test_estimate_reports_distance_and_time():
    """Route estimates should include distance and ideal travel time."""
    route = build_inspection_route([(0, 0, 10), (30, 0, 10)], standoff=5)
    summary = estimate_route(route, speed=5)

    assert summary["viewpoint_count"] == 4
    assert summary["distance_m"] > 30
    assert summary["ideal_time_s"] == pytest.approx(summary["distance_m"] / 5, abs=0.001)


@pytest.mark.parametrize(
    "poles",
    [[], [(0, 0, 10)], [(0, 0, 10), (0, 0, 12)], [(0, 0, math.nan), (1, 0, 10)]],
)
def test_invalid_pole_layouts_are_rejected(poles):
    """Incomplete and non-finite pole layouts should be rejected."""
    with pytest.raises(ValueError):
        build_inspection_route(poles)
