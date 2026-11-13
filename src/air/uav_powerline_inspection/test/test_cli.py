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

"""Tests for the power-line inspection preview command."""

import json

import pytest

from uav_powerline_inspection.cli import main


@pytest.mark.parametrize("spacing", ["0", "-40", "nan", "inf"])
def test_non_positive_or_non_finite_spacing_is_rejected(tmp_path, spacing):
    """Invalid spacing should fail before a route file is written."""
    output = tmp_path / "route.json"

    with pytest.raises(SystemExit, match="pole-spacing must be a positive finite number"):
        main(["--pole-spacing", spacing, "--output", str(output)])

    assert not output.exists()


def test_valid_spacing_writes_route_preview(tmp_path):
    """Positive spacing should produce a successful route preview."""
    output = tmp_path / "route.json"

    result = main(
        ["--pole-count", "2", "--pole-spacing", "40", "--output", str(output)]
    )

    report = json.loads(output.read_text(encoding="utf-8"))
    assert result == 0
    assert len(report["poles"]) == 2
    assert report["poles"][1][0] == pytest.approx(40.0)
