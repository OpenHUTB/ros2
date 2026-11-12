# Copyright 2026 xxbzy727749

"""Run the ROS 2 flake8 quality check."""

import pytest

from ament_flake8.main import main_with_errors


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Run flake8 lint checks."""
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, (
        f"Found {len(errors)} code style errors / warnings:\n" + "\n".join(errors)
    )
