#!/usr/bin/env python3
"""MSK-Arm keyboard teleoperation node (raw terminal input).

Runs with the system Python 3.8 and only needs rospy plus the standard library
(termios / tty / select).  Muscle activations are *ramped* while a key is held
and stay where they were released (a muscle keeps its contraction), which makes
the Hill-muscle arm pleasant to drive:

    Q / E    shoulder flexor / extensor activation up
    A / D    elbow flexor / extensor activation up
    W        co-contraction (all four muscles +)
    S        relaxation (all four muscles -)
    SPACE    all activations to zero        Ctrl-C   quit

The node publishes ``/msk/muscle_cmd`` (std_msgs/Float32MultiArray, four
activations in ``[0, 1]``) and ``/msk/key`` (std_msgs/String) so the viewer HUD
can show the last key.  When stdin is not a terminal (e.g. started by
roslaunch) the node logs a hint and keeps running idle instead of failing,
which is what makes the scripted ``~script`` timeline useful for recordings.
"""

from __future__ import annotations

import os
import select
import sys
import time


def _repo_root() -> str:
    env = os.environ.get("MSK_ROOT", "")
    if env:
        cand = os.path.realpath(env)
        if os.path.isfile(os.path.join(cand, "msk_arm_nn", "config.py")):
            return cand
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.dirname(here), here):
        if os.path.isfile(os.path.join(cand, "msk_arm_nn", "config.py")):
            return cand
    for entry in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if not entry:
            continue
        pkg = os.path.join(entry, "msk_arm_nn")
        if os.path.isfile(os.path.join(pkg, "package.xml")):
            return os.path.dirname(pkg)
    return os.path.dirname(here)


ROOT = _repo_root()
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import rospy
from std_msgs.msg import Float32MultiArray, String

from msk_arm_nn import config as C

N_MUSCLES = len(getattr(C, "MUSCLES", (0, 0, 0, 0)))

_BANNER = """
+------------------------------------------------------------------+
|  MSK-Arm musculoskeletal arm keyboard teleoperation              |
|                                                                  |
|    Q / E : shoulder flexor / extensor                            |
|    A / D : elbow flexor / extensor                               |
|    W     : co-contraction (all muscles +)                        |
|    S     : relaxation (all muscles -)                            |
|    SPACE : all activations to zero        Ctrl-C : quit          |
+------------------------------------------------------------------+
"""

# single-muscle ramp rates (activation per second)
_RAMP_MUSCLE = 1.20
_RAMP_COACT = 0.60
_RAMP_RELAX = 0.90

# key -> muscle index for the four single-muscle keys
_KEY_MUSCLE = {"q": 0, "e": 1, "a": 2, "d": 3}


def _parse_script(spec: str):
    """Parse ``"w:0.5,q:0.8,space:0.4"`` into ``[(keys, seconds, label), ...]``.

    ``keys`` is a tuple of held key characters for that slice of the timeline;
    ``+`` combines keys (e.g. ``q+w``).  The timeline drives exactly the same
    ramp code path as a physical key press, which makes the recorded demo
    reproducible.
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
        held = []
        labels = []
        for key in keys.replace("+", " ").split():
            key = key.strip().lower()
            if key in _KEY_MUSCLE or key in ("w", "s"):
                held.append(key)
                labels.append(key.upper())
            elif key in ("space", "stop"):
                held.append(" ")
                labels.append("SPACE")
        steps.append((tuple(held), seconds, "+".join(labels) if labels else "SPACE"))
    return steps


class TeleopKeyboard:
    """Publishes ramped ``/msk/muscle_cmd`` activations."""

    def __init__(self) -> None:
        self.rate_hz = max(1.0, float(rospy.get_param("~rate", 20.0)))
        self.script_spec = str(rospy.get_param("~script", "")).strip()
        self.script_loop = bool(rospy.get_param("~script_loop", False))
        self.script = _parse_script(self.script_spec)
        self.pub_cmd = rospy.Publisher("/msk/muscle_cmd",
                                       Float32MultiArray, queue_size=5)
        self.pub_key = rospy.Publisher("/msk/key", String, queue_size=5)
        self._u = [0.0] * N_MUSCLES
        self._held = set()
        self._key = "-"
        self._last_tick = time.time()
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
                "terminal (rosrun msk_arm_nn teleop_keyboard.py) or use the "
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
            if lower in _KEY_MUSCLE or lower in ("w", "s"):
                self._held = {lower}
                self._publish_key(lower.upper())
            elif char == " ":
                self._held = {" "}
                self._publish_key("SPACE")
                self._u = [0.0] * N_MUSCLES

    # -------------------------------------------------------------------- loop
    def _publish_key(self, label: str) -> None:
        if label != self._key:
            self._key = label
            self.pub_key.publish(String(data=label))

    def _tick(self) -> None:
        """Ramp the activations for the currently held keys."""
        now = time.time()
        dt = max(1e-3, min(0.1, now - self._last_tick))
        self._last_tick = now
        if " " in self._held:
            self._u = [0.0] * N_MUSCLES
        else:
            if "w" in self._held:
                for i in range(N_MUSCLES):
                    self._u[i] += _RAMP_COACT * dt
            if "s" in self._held:
                for i in range(N_MUSCLES):
                    self._u[i] -= _RAMP_RELAX * dt
            for key in ("q", "e", "a", "d"):
                if key in self._held and _KEY_MUSCLE[key] < N_MUSCLES:
                    self._u[_KEY_MUSCLE[key]] += _RAMP_MUSCLE * dt
        self._u = [float(min(1.0, max(0.0, v))) for v in self._u]

    def _publish(self) -> None:
        msg = Float32MultiArray()
        msg.data = [float(v) for v in self._u]
        self.pub_cmd.publish(msg)

    def _spin_script(self) -> None:
        """Play the ``~script`` key timeline through the normal ramp path."""
        total = sum(step[1] for step in self.script)
        rospy.loginfo("teleop_keyboard: scripted teleop, %d steps, %.1f s%s",
                      len(self.script), total,
                      " (looping)" if self.script_loop else "")
        start = time.time()
        rate = rospy.Rate(self.rate_hz)
        schedule = []
        t_end = 0.0
        for keys, seconds, label in self.script:
            t_end += seconds
            schedule.append((t_end, keys, label))
        try:
            while not rospy.is_shutdown():
                elapsed = time.time() - start
                if self.script_loop and t_end > 0.0:
                    elapsed = elapsed % t_end
                active = None
                for end, keys, label in schedule:
                    if elapsed < end:
                        active = (keys, label)
                        break
                if active is None:
                    self._held = set()
                    self._publish_key("-")
                else:
                    keys, label = active
                    self._held = set(keys)
                    self._publish_key(label)
                self._tick()
                self._publish()
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    break
        finally:
            self._restore_terminal()
            try:
                if not rospy.is_shutdown():
                    self._u = [0.0] * N_MUSCLES
                    self._publish()
                    self._publish_key("-")
            except Exception:
                pass

    def spin(self) -> None:
        if self.script:
            self._spin_script()
            return
        interactive = self._open_terminal()
        self._last_tick = time.time()
        rate = rospy.Rate(self.rate_hz)
        try:
            while not rospy.is_shutdown():
                if interactive:
                    self._read_keys()
                self._tick()
                self._publish()
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    break
        finally:
            self._restore_terminal()
            # always leave the arm relaxed
            try:
                if not rospy.is_shutdown():
                    self._u = [0.0] * N_MUSCLES
                    self._publish()
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
