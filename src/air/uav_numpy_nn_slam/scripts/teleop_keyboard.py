#!/usr/bin/env python3
"""UVNN UAV keyboard teleoperation node (raw terminal input).

Runs with the system Python 3.8 and only needs rospy plus the standard library
(termios / tty / select).  Every key press publishes a command for
``~decay`` seconds (0.4 s by default); after that the command decays to zero so
a lost key-up event can never leave the UAV flying away.

    W / S    forward +-        A / D    yaw rate +-
    R / F    climb +-          Q / E    strafe +-
    SPACE    stop              Ctrl-C   quit

The node also publishes ``/uav/key`` (std_msgs/String) so the viewer HUD can
show the last key.  When stdin is not a terminal (e.g. started by roslaunch)
the node logs a hint and keeps running idle instead of failing.
"""

from __future__ import annotations

import os
import select
import sys
import time


def _repo_root() -> str:
    env = os.environ.get("UVNN_ROOT", "")
    if env:
        cand = os.path.realpath(env)
        if os.path.isfile(os.path.join(cand, "uav_numpy_nn_slam", "config.py")):
            return cand
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.dirname(here), here):
        if os.path.isfile(os.path.join(cand, "uav_numpy_nn_slam", "config.py")):
            return cand
    for entry in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if not entry:
            continue
        pkg = os.path.join(entry, "uav_numpy_nn_slam")
        if os.path.isfile(os.path.join(pkg, "package.xml")):
            return os.path.dirname(pkg)
    return os.path.dirname(here)


ROOT = _repo_root()
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from uav_numpy_nn_slam import config as C

MAX_VX = float(getattr(C, "MAX_VX", 8.0))
MAX_VY = float(getattr(C, "MAX_VY", 5.0))
MAX_VZ = float(getattr(C, "MAX_VZ", 3.0))
MAX_YAW_RATE = float(getattr(C, "MAX_YAW_RATE", 1.2))

_BANNER = """
+------------------------------------------------------------------+
|  UVNN UAV keyboard teleoperation                                 |
|                                                                  |
|    W / S : forward / backward       A / D : yaw left / right     |
|    R / F : climb / descend          Q / E : strafe left / right  |
|    SPACE : all stop                 Ctrl-C : quit               |
+------------------------------------------------------------------+
"""

# key -> (forward, strafe, climb, yaw_rate) sign vector
_KEY_SIGNS = {
    "w": (1.0, 0.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0, 0.0),
    "q": (0.0, 1.0, 0.0, 0.0),
    "e": (0.0, -1.0, 0.0, 0.0),
    "r": (0.0, 0.0, 1.0, 0.0),
    "f": (0.0, 0.0, -1.0, 0.0),
    "a": (0.0, 0.0, 0.0, 1.0),
    "d": (0.0, 0.0, 0.0, -1.0),
}
_SCALES = (MAX_VX, MAX_VY, MAX_VZ, MAX_YAW_RATE)


def _parse_script(spec: str):
    """Parse ``"w:3.2,w+d:2.0,r:1.8"`` into ``[(signs, seconds, label), ...]``.

    Each comma-separated chunk is ``keys:seconds``; keys may be combined with
    ``+`` (e.g. ``w+d``).  This is used by ``~script`` for reproducible demo
    recordings - the timeline drives exactly the same command path as a
    physical key press.
    """
    steps = []
    for chunk in str(spec).replace(";", ",").split(","):
        chunk = chunk.strip()
        if not chunk or ":" not in chunk:
            continue
        keys, _, duration = chunk.rpartition(":")
        try:
            seconds = float(duration)
        except ValueError:
            continue
        if seconds <= 0.0:
            continue
        signs = [0.0, 0.0, 0.0, 0.0]
        labels = []
        for key in keys.replace("+", " ").split():
            key = key.strip().lower()
            if key in _KEY_SIGNS:
                sign = _KEY_SIGNS[key]
                for i in range(4):
                    signs[i] += sign[i]
                labels.append(key.upper())
            elif key in ("space", "stop"):
                labels.append("SPACE")
        signs = tuple(max(-1.0, min(1.0, v)) for v in signs)
        steps.append((signs, seconds, "+".join(labels) if labels else "SPACE"))
    return steps


class TeleopKeyboard:
    """Publishes ``/uav/cmd_vel`` from raw keyboard input or a script timeline."""

    def __init__(self) -> None:
        self.rate_hz = max(1.0, float(rospy.get_param("~rate", 20.0)))
        self.decay = max(0.05, float(rospy.get_param("~decay", 0.4)))
        self.script_spec = str(rospy.get_param("~script", "")).strip()
        self.script_loop = bool(rospy.get_param("~script_loop", False))
        self.script = _parse_script(self.script_spec)
        self.pub_cmd = rospy.Publisher("/uav/cmd_vel", Twist, queue_size=5)
        self.pub_key = rospy.Publisher("/uav/key", String, queue_size=5)
        self._signs = (0.0, 0.0, 0.0, 0.0)
        self._stamp = -1e9
        self._key = "-"
        self._fd = None
        self._tty_attrs = None
        print(_BANNER)
        sys.stdout.flush()

    # ------------------------------------------------------------- terminal io
    def _open_terminal(self) -> bool:
        try:
            import termios
            import tty
        except Exception:
            rospy.logwarn("teleop_keyboard: termios unavailable")
            return False
        if not sys.stdin.isatty():
            rospy.logwarn(
                "teleop_keyboard: stdin is not a terminal - started from "
                "roslaunch, so key input is unavailable.  Run this node in a "
                "terminal (rosrun uav_numpy_nn_slam teleop_keyboard.py) or use the "
                "viewer's own keyboard with ~teleop:=true.")
            return False
        try:
            self._fd = sys.stdin.fileno()
            self._tty_attrs = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            return True
        except Exception as exc:
            rospy.logwarn("teleop_keyboard: cannot configure terminal: %s", exc)
            self._fd = None
            return False

    def _restore_terminal(self) -> None:
        if self._fd is not None and self._tty_attrs is not None:
            try:
                import termios
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._tty_attrs)
            except Exception:
                pass
        self._fd = None
        self._tty_attrs = None

    def _read_keys(self) -> None:
        if self._fd is None:
            return
        try:
            ready, _, _ = select.select([self._fd], [], [], 0.0)
            if not ready:
                return
            data = os.read(self._fd, 64)
        except Exception:
            return
        for byte in data:
            char = chr(byte)
            if char == "\x03":          # Ctrl-C
                raise KeyboardInterrupt
            lower = char.lower()
            if lower in _KEY_SIGNS:
                self._signs = _KEY_SIGNS[lower]
                self._stamp = time.time()
                self._publish_key(lower.upper())
            elif char == " ":
                self._signs = (0.0, 0.0, 0.0, 0.0)
                self._stamp = time.time()
                self._publish_key("SPACE")

    # -------------------------------------------------------------------- loop
    def _publish_key(self, label: str) -> None:
        if label != self._key:
            self._key = label
            self.pub_key.publish(String(data=label))

    def _cmd(self) -> Twist:
        if time.time() - self._stamp > self.decay:
            signs = (0.0, 0.0, 0.0, 0.0)
        else:
            signs = self._signs
        msg = Twist()
        msg.linear.x = float(signs[0] * _SCALES[0])
        msg.linear.y = float(signs[1] * _SCALES[1])
        msg.linear.z = float(signs[2] * _SCALES[2])
        msg.angular.z = float(signs[3] * _SCALES[3])
        return msg

    def _spin_script(self) -> None:
        """Play the ``~script`` key timeline through the normal press path."""
        schedule = []
        t_end = 0.0
        for signs, seconds, label in self.script:
            t_end += seconds
            schedule.append((t_end, signs, label))
        rospy.loginfo("teleop_keyboard: scripted teleop, %d steps, %.1f s%s",
                      len(schedule), t_end,
                      " (looping)" if self.script_loop else "")
        start = time.time()
        rate = rospy.Rate(self.rate_hz)
        try:
            while not rospy.is_shutdown():
                elapsed = time.time() - start
                if self.script_loop and t_end > 0.0:
                    elapsed = elapsed % t_end
                active = None
                for end, signs, label in schedule:
                    if elapsed < end:
                        active = (signs, label)
                        break
                if active is None:
                    # timeline finished: release the key and decay to zero
                    self._publish_key("-")
                else:
                    signs, label = active
                    self._signs = signs
                    self._stamp = time.time()
                    self._publish_key(label)
                self.pub_cmd.publish(self._cmd())
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    break
        finally:
            self._restore_terminal()
            try:
                if not rospy.is_shutdown():
                    self.pub_cmd.publish(Twist())
                    self._publish_key("-")
            except Exception:
                pass

    def spin(self) -> None:
        if self.script:
            self._spin_script()
            return
        interactive = self._open_terminal()
        rate = rospy.Rate(self.rate_hz)
        try:
            while not rospy.is_shutdown():
                if interactive:
                    self._read_keys()
                if time.time() - self._stamp > self.decay and self._key != "-":
                    self._publish_key("-")
                self.pub_cmd.publish(self._cmd())
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    break
        finally:
            self._restore_terminal()
            # always leave the vehicle stopped
            try:
                if not rospy.is_shutdown():
                    self.pub_cmd.publish(Twist())
                    self._publish_key("-")
            except Exception:
                pass


def main() -> int:
    rospy.init_node("teleop_keyboard", anonymous=False)
    node = TeleopKeyboard()
    try:
        node.spin()
    except KeyboardInterrupt:
        rospy.loginfo("teleop_keyboard: interrupted by user")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
