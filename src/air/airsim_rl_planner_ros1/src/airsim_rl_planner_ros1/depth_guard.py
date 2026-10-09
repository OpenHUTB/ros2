"""Conservative experimental front-camera guard, not a full 3-D planner."""
import math
import numpy as np


class DepthGuard:
    def __init__(self):
        self.blocked = False
        self.side = 0
        self.until = None

    def filter(self, requested, depth, quaternion, now):
        stop = np.zeros(3)
        d = np.asarray(depth)
        if d.ndim != 2 or min(d.shape) < 12 or not np.all(np.isfinite(d)) or np.any(d <= 0):
            return stop, 'invalid depth'
        q = np.asarray(quaternion, dtype=float)
        if q.shape != (4,) or not np.all(np.isfinite(q)) or abs(np.linalg.norm(q)-1) > .05:
            return stop, 'invalid orientation'
        x,y,z,w = q
        roll = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
        pitch = math.asin(np.clip(2*(w*y-z*x),-1,1))
        if max(abs(roll),abs(pitch)) > math.radians(15):
            return stop, 'camera tilted'
        yaw = math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        rotation = np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        v = np.asarray(requested,dtype=float)
        body = rotation.T @ v[:2]
        if np.linalg.norm(v) < 1e-6:
            return stop, 'stop requested'
        # No rear/vertical sensors: reject commands into unobserved space.
        if abs(v[2]) > 1e-6 or body[0] < 0 or abs(body[1]) > .5*max(body[0],0):
            return stop, 'unsupported motion outside forward view'
        h,width = d.shape
        band = d[h//4:3*h//4]
        left = float(band[:, :width//3].min())
        front = float(band[:, width//3:2*width//3].min())
        right = float(band[:, 2*width//3:].min())
        # Hysteresis avoids switching between forward and stop at one threshold.
        if front < 3.0:
            self.blocked = True
        if front > 4.0:
            self.blocked = False
            self.side, self.until = 0, None
        if not self.blocked:
            v = v.copy()
            v[:2] *= min(1., .5/max(np.linalg.norm(v[:2]),1e-9))
            return v, 'clear'
        if front < 1.5:
            return stop, 'emergency clearance'
        if self.until is None:
            if max(left,right) < 4.0 or abs(left-right) < .5:
                return stop, 'wall: no distinct open side'
            self.side = -1 if left > right else 1
            self.until = now + 2.0
        if now >= self.until:
            return stop, 'bypass time limit: inspect scene'
        if (left if self.side < 0 else right) < 4.0:
            return stop, 'chosen side blocked'
        out = stop.copy()
        out[:2] = rotation @ np.array([0., self.side*.2])
        return out, 'limited left bypass' if self.side < 0 else 'limited right bypass'
