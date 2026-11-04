#!/usr/bin/env python3
"""ROS Noetic closed-loop waypoint patrol; wall-clock safety deadlines."""
import json
import csv
import os
import math
import threading
import time
import signal
import sys
import hashlib
import faulthandler

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from turtlesim.msg import Pose


class Patrol:
    def __init__(self, exit_requested=lambda: False):
        self.lock = threading.RLock()
        self.pose = None
        self.received = None
        self.started = time.monotonic()
        self.index = 0
        self.exit_requested = exit_requested
        self.trace = None
        self.closed = False
        self.state = 'RUNNING'
        self.goals = rospy.get_param('~waypoints', [[7, 5.5], [7, 7], [5.5, 7]])
        if not isinstance(self.goals, list) or not self.goals:
            raise ValueError('waypoints must be a nonempty list')
        for goal in self.goals:
            if (not isinstance(goal, list) or len(goal) != 2 or
                    any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                        not math.isfinite(v) or not 0.5 <= v <= 10.5 for v in goal)):
                raise ValueError('each waypoint must contain two finite coordinates in [0.5,10.5]')
        self.trace = None
        trace_path = rospy.get_param('~trace_path', '')
        if trace_path:
            self.trace = open(os.path.expanduser(trace_path), 'x', newline='')
            self.writer = csv.writer(self.trace)
            self.writer.writerow(['elapsed_s', 'x', 'y', 'theta', 'state', 'completed', 'target_error'])
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
        self.status = rospy.Publisher('~status', String, queue_size=1, latch=True)
        self.sub = rospy.Subscriber('/turtle1/pose', Pose, self.on_pose, queue_size=1)
        for name in ('pause', 'resume', 'cancel'):
            rospy.Service('~' + name, Trigger, lambda req, action=name: self.command(action))

    def on_pose(self, pose):
        with self.lock:
            self.pose = pose
            self.received = time.monotonic()
            if self.trace and not self.closed:
                goal = self.goals[min(self.index, len(self.goals)-1)]
                self.writer.writerow([self.received-self.started, pose.x, pose.y,
                                      pose.theta, self.state, self.index,
                                      math.hypot(goal[0]-pose.x, goal[1]-pose.y)])
                self.trace.flush()

    def stop(self):
        # Main loop and services publish under the same control lock.
        # No sleeping or shutdown calls inside a service callback.
        self.pub.publish(Twist())

    def close(self):
        # A finally block owns cleanup; signal handlers never call ROS or take locks.
        print('EXIT cleanup_begin', file=sys.stderr, flush=True)
        with self.lock:
            self.closed = True
            self.stop()
        self.sub.unregister()
        with self.lock:
            if self.trace:
                self.trace.close()
                self.trace = None
        print('EXIT trace_closed', file=sys.stderr, flush=True)
        # Keep transport alive briefly so turtlesim can receive the last zero.
        time.sleep(0.1)
        print('EXIT ros_shutdown_begin', file=sys.stderr, flush=True)
        rospy.signal_shutdown('patrol exited')
        print('EXIT ros_shutdown_done', file=sys.stderr, flush=True)

    def command(self, action):
        with self.lock:
            if action == 'cancel' and self.state in ('RUNNING', 'PAUSED'):
                self.state = 'CANCELED'
            elif action == 'pause' and self.state == 'RUNNING':
                self.state = 'PAUSED'
            elif action == 'resume' and self.state == 'PAUSED':
                self.state = 'RUNNING'
            else:
                return TriggerResponse(False, self.state)
            self.stop()
            return TriggerResponse(True, self.state)

    def run(self):
        try:
            while not rospy.is_shutdown() and not self.exit_requested():
                with self.lock:
                    now = time.monotonic()
                    cmd = Twist()
                    if self.state in ('RUNNING', 'PAUSED'):
                        if now - self.started > 120:
                            self.state = 'TASK_TIMEOUT'
                        elif (self.received is None and now - self.started > 3) or (self.received is not None and now - self.received > 0.5):
                            self.state = 'POSE_TIMEOUT'
                    if self.state == 'RUNNING' and self.pose is not None:
                        x, y = self.goals[self.index]
                        distance = math.hypot(x-self.pose.x, y-self.pose.y)
                        if distance <= 0.08:
                            self.index += 1
                            if self.index == len(self.goals):
                                self.state = 'COMPLETED'
                        else:
                            error = math.atan2(y-self.pose.y, x-self.pose.x)-self.pose.theta
                            error = math.atan2(math.sin(error), math.cos(error))
                            cmd.angular.z = max(-2.0, min(2.0, 4*error))
                            cmd.linear.x = min(1.5, distance)*max(0, math.cos(error)) if abs(error) < 0.6 else 0
                    self.pub.publish(cmd)
                    self.status.publish(json.dumps(dict(state=self.state, completed=self.index, total=len(self.goals))))
                time.sleep(0.05)
        finally:
            self.close()


def main():
    requested = [False]

    def request_exit(signum, frame):
        requested[0] = True

    # Explicitly own signals BEFORE init_node. Do not replace rospy handlers later.
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, request_exit)
    faulthandler.enable()
    if hasattr(signal, 'SIGUSR1'):
        faulthandler.register(signal.SIGUSR1, all_threads=True)
    with open(__file__, 'rb') as source:
        digest = hashlib.sha256(source.read()).hexdigest()
    print('START pid={} source={} sha256={}'.format(
        os.getpid(), os.path.realpath(__file__), digest), file=sys.stderr, flush=True)
    rospy.init_node('waypoint_patrol', disable_signals=True)
    patrol = Patrol(lambda: requested[0])
    patrol.run()


if __name__ == '__main__':
    main()
