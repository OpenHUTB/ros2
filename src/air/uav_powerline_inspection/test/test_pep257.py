# Copyright 2026 xxbzy727749

"""Run the ROS 2 docstring quality check."""

import pytest

from ament_pep257.main import main


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Run pep257 docstring checks."""
    rc = main(argv=[".", "test"])
    assert rc == 0, "Found docstring style errors"
