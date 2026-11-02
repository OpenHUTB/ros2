"""Regression test for the turtle-heart pen sequence without a ROS runtime."""

from __future__ import annotations

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest import mock


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "main.py"
)


def _module(name, **attributes):
    module = types.ModuleType(name)
    for key, value in attributes.items():
        setattr(module, key, value)
    return module


def _load_turtle_heart():
    rospy = _module(
        "rospy",
        init_node=lambda *args, **kwargs: None,
        Publisher=lambda *args, **kwargs: mock.Mock(),
        Subscriber=lambda *args, **kwargs: mock.Mock(),
        Rate=lambda *args, **kwargs: mock.Mock(),
        wait_for_service=lambda *args, **kwargs: None,
        ServiceProxy=lambda *args, **kwargs: mock.Mock(),
        loginfo=lambda *args, **kwargs: None,
    )
    geometry_msgs = _module("geometry_msgs")
    geometry_msgs.msg = _module("geometry_msgs.msg", Twist=type("Twist", (), {}))
    turtlesim = _module("turtlesim")
    turtlesim.msg = _module("turtlesim.msg", Pose=type("Pose", (), {}))
    turtlesim.srv = _module("turtlesim.srv", SetPen=type("SetPen", (), {}))

    module_name = "_test_turtle_heart"
    modules = {
        "rospy": rospy,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs.msg,
        "turtlesim": turtlesim,
        "turtlesim.msg": turtlesim.msg,
        "turtlesim.srv": turtlesim.srv,
    }
    spec = importlib.util.spec_from_file_location(module_name, MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class TurtleHeartPenTest(unittest.TestCase):
    def test_moves_to_start_with_pen_up_then_draws_from_second_point(self):
        module = _load_turtle_heart()
        drawer = object.__new__(module.TurtleDrawer)
        events = []

        drawer.set_pen = lambda down: events.append(("pen", down))
        drawer.move_to = lambda x, y: events.append(("move", x, y))

        with mock.patch.object(module.time, "sleep"):
            drawer.draw_heart()

        self.assertEqual(events[0], ("pen", False))
        self.assertEqual(events[1][0], "move")
        self.assertEqual(events[2], ("pen", True))
        # One pen-up move to the start plus the 60 remaining contour points.
        self.assertEqual(sum(event[0] == "move" for event in events), 61)
        self.assertEqual(events[-1], ("pen", False))


if __name__ == "__main__":
    unittest.main()
