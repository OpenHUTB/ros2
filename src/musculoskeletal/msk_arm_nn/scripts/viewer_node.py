#!/usr/bin/env python3
"""MSK-Arm top-down viewer with optional keyboard teleoperation.

Runs with the *system* Python 3.8 (rospy + python3-opencv + python3-pil); it
never imports the project venv.  The node subscribes to:

    /msk/state    (std_msgs/String, JSON)   joints, activations, EE, target
    /msk/plan     (nav_msgs/Path)           optional EE plan in the workspace

and renders the frame with ``msk_arm_nn.render.ArmRenderer``: two-link arm with
muscle colouring, activation bars, EE trail, target marker and the HUD fields
MODE / KEY / ERR / TQ / REACH / HIT / COVER.

Keys (window must have focus):

    Q / E    shoulder flexor / extensor
    A / D    elbow flexor / extensor
    W        co-contraction (all muscles +)
    S        relaxation (all muscles -)
    SPACE    all activations to zero
    P        save a PNG screenshot to results/
    ESC      quit

With ``~teleop:=true`` the keys are published on ``/msk/muscle_cmd``.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time


def _repo_root() -> str:
    env = os.environ.get("MSK_ROOT", "")
    if env:
        cand = os.path.realpath(env)
        if os.path.isfile(os.path.join(cand, "msk_arm_nn", "scenes.py")):
            return cand
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.dirname(here), here):
        if os.path.isfile(os.path.join(cand, "msk_arm_nn", "scenes.py")):
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

import numpy as np
import rospy
from PIL import ImageDraw

try:
    import cv2
except Exception as _cv_exc:  # pragma: no cover - environment dependent
    cv2 = None
    _CV_ERROR = _cv_exc

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, String

from msk_arm_nn import config as C
from msk_arm_nn.render import ArmRenderer
from msk_arm_nn.scenes import get_scene

WIN_NAME = "MSK ARM"
N_MUSCLES = len(getattr(C, "MUSCLES", (0, 0, 0, 0)))
RAMP_MUSCLE = 1.20
RAMP_COACT = 0.60
RAMP_RELAX = 0.90
_KEY_MUSCLE = {"q": 0, "e": 1, "a": 2, "d": 3}


class ViewerNode:
    """Renders the musculoskeletal arm workspace in an OpenCV window."""

    def __init__(self) -> None:
        self.scene_name = str(rospy.get_param("~scene", "reach"))
        self.teleop = bool(rospy.get_param("~teleop", False))
        self.display = str(rospy.get_param("~display", ":0"))
        self.fps = max(1.0, float(rospy.get_param("~fps", 15.0)))
        self.render_width = max(320, int(rospy.get_param("~render_width", 760)))
        self.window_x = int(rospy.get_param("~window_x", 400))
        self.window_y = int(rospy.get_param("~window_y", 20))
        self.auto_snapshot = str(rospy.get_param("~auto_snapshot", ""))
        self.snapshot_delay = float(rospy.get_param("~snapshot_delay", 3.0))
        self.show_window = bool(rospy.get_param("~display_window", True))
        if self.display:
            os.environ["DISPLAY"] = self.display
        # GNOME keeps the X cookie in /run/user/<uid>/gdm/Xauthority; when
        # roslaunch is started from a non-login shell XAUTHORITY may be unset.
        auth = os.environ.get("XAUTHORITY", "")
        if not auth or not os.path.isfile(auth):
            for cand in ("/run/user/%d/gdm/Xauthority" % os.getuid(),
                         os.path.expanduser("~/.Xauthority")):
                if os.path.isfile(cand):
                    os.environ["XAUTHORITY"] = cand
                    rospy.loginfo("viewer_node: using XAUTHORITY=%s", cand)
                    break

        if cv2 is None:
            rospy.logfatal("viewer_node: OpenCV unavailable: %s", _CV_ERROR)
            raise SystemExit(1)

        self.scene = get_scene(self.scene_name)
        self.renderer = ArmRenderer(self.scene, width_px=self.render_width)

        self.state = None
        self.path = None
        self._trail = []
        self._last_frame = None
        self._headless = not self.show_window
        self._started = time.time()
        self._auto_saved = False

        # keyboard teleop state
        self._u = [0.0] * N_MUSCLES
        self._held = set()
        self._key_label = ""
        self._last_tick = time.time()

        self.pub_cmd = rospy.Publisher("/msk/muscle_cmd",
                                       Float32MultiArray, queue_size=5)
        rospy.Subscriber("/msk/state", String, self._on_state, queue_size=5)
        rospy.Subscriber("/msk/plan", Path, self._on_plan, queue_size=2)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("viewer_node: window=%r scene=%s teleop=%s display=%s "
                      "render_width=%d",
                      WIN_NAME, self.scene_name, self.teleop,
                      self.display or "none", self.render_width)

    # ------------------------------------------------------------- callbacks
    def _on_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self.state = data
        trail = data.get("trail")
        if isinstance(trail, list) and trail:
            self._trail = [(float(p[0]), float(p[1])) for p in trail[-300:]
                           if len(p) >= 2]

    def _on_plan(self, msg: Path) -> None:
        try:
            self.path = [(float(p.pose.position.x), float(p.pose.position.y))
                         for p in msg.poses]
        except Exception:
            self.path = None

    # --------------------------------------------------------------- keyboard
    def _handle_key(self, key: int) -> None:
        if key < 0 or key > 255:
            return
        char = chr(key)
        if char == "\x1b":
            rospy.loginfo("viewer_node: ESC pressed, shutting down")
            rospy.signal_shutdown("ESC")
            return
        if char == " ":
            self._held = {" "}
            self._key_label = "SPACE"
            self._u = [0.0] * N_MUSCLES
            return
        lower = char.lower()
        if lower in _KEY_MUSCLE or lower in ("w", "s"):
            self._held = {lower}
            self._key_label = lower.upper()
            return
        if char in ("p", "P"):
            if self._last_frame is not None:
                self._save_snapshot(self._last_frame, tag="manual")

    def _tick_teleop(self) -> None:
        now = time.time()
        dt = max(1e-3, min(0.1, now - self._last_tick))
        self._last_tick = now
        if " " in self._held:
            self._u = [0.0] * N_MUSCLES
        else:
            if "w" in self._held:
                for i in range(N_MUSCLES):
                    self._u[i] += RAMP_COACT * dt
            if "s" in self._held:
                for i in range(N_MUSCLES):
                    self._u[i] -= RAMP_RELAX * dt
            for key in ("q", "e", "a", "d"):
                if key in self._held and _KEY_MUSCLE[key] < N_MUSCLES:
                    self._u[_KEY_MUSCLE[key]] += RAMP_MUSCLE * dt
        self._u = [float(min(1.0, max(0.0, v))) for v in self._u]

    def _publish_teleop(self) -> None:
        if not self.teleop:
            return
        self._tick_teleop()
        msg = Float32MultiArray()
        msg.data = [float(v) for v in self._u]
        self.pub_cmd.publish(msg)

    # ---------------------------------------------------------------- capture
    def _snapshot_path(self, tag: str) -> str:
        directory = os.path.join(ROOT, "results")
        os.makedirs(directory, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        return os.path.join(directory,
                            "ros_%s_%s_%s.png" % (self.scene_name, tag, stamp))

    def _save_snapshot(self, image, tag: str = "snap") -> str:
        path = self._snapshot_path(tag)
        try:
            image.save(path)
            rospy.loginfo("viewer_node: screenshot saved -> %s", path)
        except Exception as exc:  # pragma: no cover - defensive
            rospy.logwarn("viewer_node: cannot save screenshot: %s", exc)
        return path

    # ----------------------------------------------------------------- render
    def _draw_hint(self, frame, map_info: str = "-") -> None:
        """Draw the control hint inside the workspace area (never in the HUD)."""
        try:
            draw = ImageDraw.Draw(frame, "RGBA")
            font = self.renderer.font_small
            margin = self.renderer.margin
            lines = [
                "Q/E shoulder  A/D elbow  W co-contract  S relax  SPACE zero",
                "P snapshot  ESC quit   |   plan %d" % (
                    len(self.path) if self.path else 0),
            ]
            widths = []
            heights = []
            for line in lines:
                try:
                    width, height = draw.textsize(line, font=font)
                except Exception:
                    width, height = 7 * len(line), 13
                widths.append(width)
                heights.append(height)
            x0 = margin + 8
            y0 = margin + 8
            line_h = max(heights) + 4
            x1 = min(margin + self.renderer.map_w - 4, x0 + max(widths) + 12)
            y1 = y0 + line_h * len(lines) + 4
            draw.rectangle([x0 - 5, y0 - 4, x1, y1],
                           fill=(6, 14, 20, 205), outline=(90, 150, 170, 180))
            for i, line in enumerate(lines):
                draw.text((x0, y0 + i * line_h), line, font=font,
                          fill=(180, 210, 222))
        except Exception:  # pragma: no cover - defensive
            pass

    def _render_frame(self, now: float):
        state = self.state or {}
        q = state.get("q") or list(self.scene.start_q)
        try:
            q = [float(v) for v in q[:2]]
        except (TypeError, ValueError):
            q = [float(v) for v in self.scene.start_q]
        activations = state.get("a")
        if not isinstance(activations, list) or len(activations) < N_MUSCLES:
            activations = [0.0] * N_MUSCLES
        ee = state.get("ee") or [0.0, 0.0]
        target = state.get("target") or list(self.scene.targets[0])
        try:
            ee_x, ee_y = float(ee[0]), float(ee[1])
            tx, ty = float(target[0]), float(target[1])
            ee_error = math.hypot(tx - ee_x, ty - ee_y)
        except (TypeError, ValueError, IndexError):
            ee_error = 0.0
            tx, ty = 0.0, 0.0
        sim_time = float(state.get("time", 0.0))
        torque = float(state.get("max_torque", 0.0) or 0.0)
        reached = int(state.get("reached", 0) or 0)
        collisions = int(state.get("collisions", 0) or 0)
        success = bool(state.get("success", False))
        mode = str(state.get("mode", "TELEOP"))
        key_label = str(state.get("key", "") or self._key_label or "-")
        hud = {
            "SUCC": "YES" if success else "no",
            "TGT": "%.3f,%.3f" % (tx, ty),
        }
        frame = self.renderer.render(
            q=q,
            activations=activations,
            path=self.path,
            goal=(tx, ty),
            trail=self._trail,
            occupancy=None,
            hud=hud,
            mode=mode,
            key_label=key_label,
            message="",
            coverage=0.0,
            time=sim_time,
            torque=torque,
            ee_error=ee_error,
            reached=reached,
            collisions=collisions,
        )
        self._draw_hint(frame)
        self._last_frame = frame

        if self._headless:
            return
        try:
            bgr = cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR)
            cv2.imshow(WIN_NAME, bgr)
            key = cv2.waitKey(1) & 0xFF
            if key != 255:
                self._handle_key(key)
        except Exception as exc:  # pragma: no cover - X11 dependent
            rospy.logerr("viewer_node: cannot display window (%s); "
                         "continuing headless", exc)
            self._headless = True

    # -------------------------------------------------------------------- run
    def spin(self) -> None:
        if not self._headless:
            try:
                cv2.namedWindow(WIN_NAME, cv2.WINDOW_AUTOSIZE)
                # keeps the left third of the desktop visible for the terminal
                cv2.moveWindow(WIN_NAME, self.window_x, self.window_y)
            except Exception as exc:  # pragma: no cover - X11 dependent
                rospy.logerr("viewer_node: namedWindow failed (%s), headless",
                             exc)
                self._headless = True
        rate = rospy.Rate(20.0)
        last_render = -1e9
        while not rospy.is_shutdown():
            self._publish_teleop()
            now = time.time()
            if now - last_render >= 1.0 / self.fps:
                last_render = now
                self._render_frame(now)
                if (self.auto_snapshot and not self._auto_saved
                        and self._last_frame is not None
                        and now - self._started >= self.snapshot_delay):
                    path = self.auto_snapshot
                    if not os.path.isabs(path):
                        path = os.path.join(ROOT, path)
                    try:
                        os.makedirs(os.path.dirname(path), exist_ok=True)
                        self._last_frame.save(path)
                        rospy.loginfo("viewer_node: auto screenshot -> %s", path)
                        self._auto_saved = True
                    except Exception as exc:
                        rospy.logwarn("viewer_node: auto snapshot failed: %s",
                                      exc)
                        self._auto_saved = True
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def _on_shutdown(self) -> None:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


def main() -> int:
    rospy.init_node("viewer_node", anonymous=False)
    node = ViewerNode()
    try:
        node.spin()
    finally:
        node._on_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
