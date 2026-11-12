#!/usr/bin/env python3
"""HNN humanoid top-down viewer with optional keyboard teleoperation.

Runs with the *system* Python 3.8 (rospy + python3-opencv + python3-pil); it
never imports the project venv.  The node subscribes to:

    /humanoid/state       (std_msgs/String, JSON)   ground truth + scan snapshot
    /humanoid/map         (nav_msgs/OccupancyGrid)  optional SLAM map
    /humanoid/plan        (nav_msgs/Path)           optional global plan
    /humanoid/slam_pose   (geometry_msgs/PoseStamped) optional SLAM pose

and renders the frame with ``humanoid_walk_nn.render.TopDownRenderer``,
including the gait phase, the balance gauge, the fall counter and the moving
carts.

Keys (window must have focus):

    W / S    walk forward / backward
    A / D    turn left / right
    SPACE    stop
    P        save a PNG screenshot to results/
    ESC      quit

With ``~teleop:=true`` the keys are published on ``/humanoid/cmd`` (20 Hz with
a 0.4 s decay).
"""

from __future__ import annotations

import json
import math
import os
import sys
import time


def _repo_root() -> str:
    env = os.environ.get("HNN_ROOT", "")
    if env:
        cand = os.path.realpath(env)
        if os.path.isfile(os.path.join(cand, "humanoid_walk_nn", "scenes.py")):
            return cand
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.dirname(here), here):
        if os.path.isfile(os.path.join(cand, "humanoid_walk_nn", "scenes.py")):
            return cand
    for entry in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if not entry:
            continue
        pkg = os.path.join(entry, "humanoid_walk_nn")
        if os.path.isfile(os.path.join(pkg, "package.xml")):
            return os.path.dirname(pkg)
    return os.path.dirname(here)


ROOT = _repo_root()
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import numpy as np
import rospy
from PIL import ImageDraw

# Ubuntu 20.04 ships Pillow 7.0, which predates ImageDraw.rounded_rectangle
# (added in Pillow 8.2); shim it so the viewer works on the system interpreter.
if not hasattr(ImageDraw.ImageDraw, "rounded_rectangle"):
    def _rounded_rectangle(self, xy, radius=0, fill=None, outline=None,
                           width=1, **kwargs):
        self.rectangle(xy, fill=fill, outline=outline, width=width)

    ImageDraw.ImageDraw.rounded_rectangle = _rounded_rectangle

try:
    import cv2
except Exception as _cv_exc:  # pragma: no cover - environment dependent
    cv2 = None
    _CV_ERROR = _cv_exc

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String

from humanoid_walk_nn import config as C
from humanoid_walk_nn.render import TopDownRenderer
from humanoid_walk_nn.scenes import get_scene

WIN_NAME = "HUMANOID"
LIDAR_BEAMS = int(getattr(C, "LIDAR_BEAMS", 72))
BEAM_ANGLES = np.linspace(-math.pi, math.pi, LIDAR_BEAMS, endpoint=False)
MAX_SPEED = float(getattr(C, "MAX_SPEED", 1.25))
MAX_BACKWARD = float(getattr(C, "MAX_BACKWARD", 0.5))
MAX_TURN_RATE = float(getattr(C, "MAX_TURN_RATE", 1.5))

# key -> (walk speed m/s, turn rate rad/s)
_KEY_CMD = {
    "w": (1.1, 0.0),
    "s": (-0.40, 0.0),
    "a": (0.0, 1.0),
    "d": (0.0, -1.0),
}


def _yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ViewerNode:
    """Renders the indoor floor plan in an OpenCV window."""

    def __init__(self) -> None:
        self.scene_name = str(rospy.get_param("~scene", "hall"))
        self.teleop = bool(rospy.get_param("~teleop", False))
        self.display = str(rospy.get_param("~display", ":0"))
        self.fps = max(1.0, float(rospy.get_param("~fps", 15.0)))
        self.render_width = max(320, int(rospy.get_param("~render_width", 820)))
        self.window_x = int(rospy.get_param("~window_x", 400))
        self.window_y = int(rospy.get_param("~window_y", 20))
        self.decay = max(0.05, float(rospy.get_param("~decay", 0.4)))
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
        self.renderer = TopDownRenderer(self.scene, width_px=self.render_width)

        self.state = None
        self.slam_pose = None
        self.path = None
        self._occupancy = None
        self._occupied_mask = None
        self._coverage = 0.0
        self._trail = []
        self._phase = 0.0
        self._balance = 0.0
        self._falls = 0
        self._headless = not self.show_window
        self._started = time.time()
        self._auto_saved = False
        self._last_frame = None

        # keyboard teleop state
        self._key_speed = 0.0
        self._key_turn = 0.0
        self._key_stamp = -1e9
        self._key_label = ""

        self.pub_cmd = rospy.Publisher("/humanoid/cmd", Twist, queue_size=5)
        rospy.Subscriber("/humanoid/state", String, self._on_state,
                         queue_size=5)
        rospy.Subscriber("/humanoid/map", OccupancyGrid, self._on_map,
                         queue_size=2)
        rospy.Subscriber("/humanoid/plan", Path, self._on_plan, queue_size=2)
        rospy.Subscriber("/humanoid/slam_pose", PoseStamped,
                         self._on_slam_pose, queue_size=2)
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
        for attr, key in (("_phase", "phase"), ("_balance", "balance")):
            try:
                setattr(self, attr, float(data.get(key, 0.0) or 0.0))
            except (TypeError, ValueError):
                pass
        try:
            self._falls = int(data.get("falls", 0) or 0)
        except (TypeError, ValueError):
            pass
        truth = data.get("truth")
        if truth and len(truth) >= 4:
            if (not self._trail or
                    math.hypot(truth[0] - self._trail[-1][0],
                               truth[1] - self._trail[-1][1]) > 0.05):
                self._trail.append((float(truth[0]), float(truth[1])))
                if len(self._trail) > 900:
                    del self._trail[:300]

    def _on_map(self, msg: OccupancyGrid) -> None:
        try:
            data = np.asarray(msg.data, dtype=np.int16)
            width = int(msg.info.width)
            height = int(msg.info.height)
            if width <= 0 or height <= 0 or data.size != width * height:
                return
            data = data.reshape(height, width)
            unknown = data < 0
            prob = np.where(unknown, 0.5, data / 100.0)
            self._occupancy = prob
            self._occupied_mask = np.where(unknown, False, data >= 65)
            known = float(np.count_nonzero(~unknown))
            self._coverage = known / float(max(1, data.size))
        except Exception as exc:  # pragma: no cover - defensive
            rospy.logwarn_throttle(5.0, "viewer_node: bad map message: %s", exc)

    def _on_plan(self, msg: Path) -> None:
        try:
            self.path = [(float(p.pose.position.x), float(p.pose.position.y))
                         for p in msg.poses]
        except Exception:
            self.path = None

    def _on_slam_pose(self, msg: PoseStamped) -> None:
        try:
            self.slam_pose = (float(msg.pose.position.x),
                              float(msg.pose.position.y),
                              _yaw_from_quaternion(msg.pose.orientation))
        except Exception:
            self.slam_pose = None

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
            self._key_speed = 0.0
            self._key_turn = 0.0
            self._key_stamp = time.time()
            self._key_label = "SPACE"
            return
        lower = char.lower()
        if lower in _KEY_CMD:
            self._key_speed, self._key_turn = _KEY_CMD[lower]
            self._key_stamp = time.time()
            self._key_label = lower.upper()
            return
        if char in ("p", "P"):
            if self._last_frame is not None:
                self._save_snapshot(self._last_frame, tag="manual")

    def _publish_teleop(self) -> None:
        if not self.teleop:
            return
        if time.time() - self._key_stamp > self.decay:
            speed, turn = 0.0, 0.0
        else:
            speed, turn = self._key_speed, self._key_turn
        msg = Twist()
        msg.linear.x = float(np.clip(speed, -MAX_BACKWARD, MAX_SPEED))
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.z = float(np.clip(turn, -MAX_TURN_RATE, MAX_TURN_RATE))
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
        """Draw the control/link hint inside the map area (never in the HUD)."""
        try:
            draw = ImageDraw.Draw(frame, "RGBA")
            font = self.renderer.font_small
            margin = self.renderer.margin
            lines = [
                "W/S walk  A/D turn  SPACE stop   P snapshot  ESC quit",
                "%s  plan %d  step %d" % (
                    map_info, len(self.path) if self.path else 0,
                    int(self.state.get("steps", 0)) if self.state else 0),
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
        truth = state.get("truth")
        if truth and len(truth) >= 4:
            pose = [float(v) for v in truth]
        else:
            pose = [float(v) for v in self.scene.start]
        scan = state.get("scan")
        scan_arr = np.asarray(scan, dtype=np.float64) if scan else None
        if scan_arr is not None and scan_arr.size != LIDAR_BEAMS:
            scan_arr = None

        sim_time = float(state.get("time", 0.0))
        speed = float(state.get("speed", 0.0) or 0.0)
        balance = state.get("balance", self._balance)
        falls = state.get("falls", self._falls)
        collisions = int(state.get("collisions", 0))
        steps = int(state.get("steps", 0))
        phase = state.get("phase", self._phase)
        try:
            balance = float(balance)
        except (TypeError, ValueError):
            balance = 0.0
        try:
            falls = int(falls)
        except (TypeError, ValueError):
            falls = 0
        try:
            phase = float(phase)
        except (TypeError, ValueError):
            phase = 0.0

        hud = {
            "SPD": "%.2f m/s" % speed,
            "BAL": "%+.3f m" % balance,
            "FALL": "%d" % falls,
            "HIT": "%d" % collisions,
            "PHASE": "%.2f" % (phase % 1.0),
            "STEP": "%d" % steps,
        }
        known = "map %s" % ("-" if self._occupancy is None else
                            "%dx%d" % (self._occupancy.shape[1],
                                       self._occupancy.shape[0]))
        mode = str(state.get("mode", "TELEOP"))
        key_label = str(state.get("key", "") or self._key_label or "-")
        try:
            carts = self.scene.cart_positions(sim_time)
        except Exception:  # pragma: no cover - defensive
            carts = None
        frame = self.renderer.render(
            pose=pose,
            scan=scan_arr,
            beam_angles=BEAM_ANGLES if scan_arr is not None else None,
            slam_pose=self.slam_pose,
            occupancy=self._occupancy,
            occupied_mask=self._occupied_mask,
            path=self.path,
            goal=self.scene.goal[:2],
            trail=self._trail,
            hud=hud,
            mode=mode,
            key_label=key_label,
            message="",
            coverage=self._coverage,
            time=sim_time,
            phase=phase,
            balance=balance,
            falls=falls,
            carts=carts,
        )
        # keep our own status hint inside the map area: the renderer's HUD
        # band below `margin + map_h` is reserved for the metric fields
        self._draw_hint(frame, known)
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
                # 820 px wide window at (400, 20): keeps the left third of the
                # desktop visible for the terminal used by the recorded demo.
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
