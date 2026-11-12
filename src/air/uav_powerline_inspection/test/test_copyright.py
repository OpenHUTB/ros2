# Copyright 2026 xxbzy727749

"""Run the ROS 2 copyright quality check."""

import pytest

from ament_copyright.main import main


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    """Check source files for copyright notices."""
    rc = main(argv=["."])
    assert rc == 0, "Found copyright errors"
