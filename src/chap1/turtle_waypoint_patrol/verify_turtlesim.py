#!/usr/bin/env python3
"""Real ROS/turtlesim integration tests. Run against an otherwise idle turtlesim."""
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import unittest
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger, Empty
from turtlesim.msg import Pose

ROOT = Path(__file__).resolve().parent
OUTPUT = Path(os.environ.get('PATROL_EVIDENCE_DIR', str(ROOT / 'evidence')))


class PatrolChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        OUTPUT.mkdir(parents=True, exist_ok=False)
        rospy.init_node('patrol_checks', anonymous=True)
        rospy.wait_for_service('/reset', timeout=10)
        cls.poses, cls.statuses, cls.commands = [], [], []
        cls.subs = [
            rospy.Subscriber('/turtle1/pose', Pose,
                lambda p: cls.poses.append((time.monotonic(), p.x, p.y)), queue_size=100),
            rospy.Subscriber('/waypoint_patrol/status', String,
                lambda m: cls.statuses.append(json.loads(m.data)), queue_size=100),
            rospy.Subscriber('/turtle1/cmd_vel', Twist,
                lambda m: cls.commands.append((time.monotonic(), m.linear.x, m.angular.z)), queue_size=100)]

    def setUp(self):
        self.process = None
        self.log = None
        rospy.ServiceProxy('/reset', Empty)()
        self.poses.clear()
        self.statuses.clear()
        self.commands.clear()
        self.wait(lambda: bool(self.poses), 5)

    def wait(self, check, timeout=40):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if check():
                return
            if rospy.is_shutdown():
                self.fail('ROS shut down while waiting')
            time.sleep(.05)
        self.fail('condition timed out; inspect ' + str(OUTPUT))

    def start(self, *args):
        name = self.id().split('.')[-1]
        self.log = (OUTPUT / (name + '.log')).open('w')
        goals = [] if any(a.startswith('_waypoints:=') for a in args) else [
            '_waypoints:=[[7,5.5],[7,7],[5.5,7]]']
        trace = OUTPUT / (name + '.csv')
        self.process = subprocess.Popen([
            'python3', str(ROOT / 'main.py'), '__name:=waypoint_patrol',
            '_trace_path:=' + str(trace)] + goals + list(args),
            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        return self.process

    def call(self, action):
        rospy.wait_for_service('/waypoint_patrol/' + action, timeout=5)
        return rospy.ServiceProxy('/waypoint_patrol/' + action, Trigger)()

    def active(self):
        self.wait(lambda: self.statuses and self.statuses[-1]['state'] == 'RUNNING')
        self.wait(lambda: any(v > .1 for _, v, _ in self.commands))

    def still(self):
        time.sleep(.3)
        self.assertTrue(self.poses)
        first = self.poses[-1]
        time.sleep(1)
        # Do not mistake a disconnected observer for a stationary turtle.
        self.assertGreater(self.poses[-1][0], first[0] + .5)
        delta = math.hypot(self.poses[-1][1]-first[1], self.poses[-1][2]-first[2])
        self.assertLess(delta, .03)
        print(self.id(), 'stationary displacement', delta, flush=True)

    def stop(self, sig=signal.SIGTERM):
        p = self.process
        if p is None or p.poll() is not None:
            return
        sent = time.monotonic()
        p.send_signal(sig)
        try:
            result = p.wait(timeout=8)
        except subprocess.TimeoutExpired:
            # Ask THIS child for its Python stacks before killing it. Preserve failure.
            p.send_signal(signal.SIGUSR1)
            time.sleep(.3)
            p.kill()
            p.wait(timeout=3)
            self.fail('exit timeout: thread stacks saved in ' + str(OUTPUT))
        self.assertEqual(result, 0)
        print('exit signal={} seconds={:.3f}'.format(sig, time.monotonic()-sent), flush=True)

    def tearDown(self):
        try:
            self.stop()
        finally:
            if self.log:
                self.log.close()
            # Stop this test's actuator even after a failed controller shutdown.
            safety = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
            time.sleep(.1)
            for _ in range(3):
                safety.publish(Twist())
                time.sleep(.05)
            safety.unregister()

    def test_01_pause_resume_completion(self):
        self.start()
        self.active()
        self.assertTrue(self.call('pause').success)
        self.wait(lambda: self.statuses[-1]['state'] == 'PAUSED')
        self.still()
        index = self.statuses[-1]['completed']
        self.assertTrue(self.call('resume').success)
        self.wait(lambda: self.statuses[-1]['state'] == 'COMPLETED', 60)
        self.assertEqual(self.statuses[-1]['completed'], 3)
        self.assertLess(index, 3)
        self.assertFalse(self.call('resume').success)

    def test_02_cancel(self):
        self.start('_waypoints:=[[2,2]]')
        self.active()
        self.assertTrue(self.call('cancel').success)
        self.assertFalse(self.call('resume').success)
        self.still()

    def test_03_invalid_waypoint(self):
        p = self.start('_waypoints:=[[100,2]]')
        self.assertNotEqual(p.wait(timeout=8), 0)

    def test_04_missing_pose(self):
        self.start('/turtle1/pose:=/missing_pose')
        self.wait(lambda: self.statuses and self.statuses[-1]['state'] == 'POSE_TIMEOUT', 8)
        self.assertTrue(self.commands)
        self.assertTrue(all(v == 0 and w == 0 for _, v, w in self.commands))

    def test_05_sigterm_during_motion(self):
        self.start('_waypoints:=[[2,9]]')
        self.active()
        self.stop(signal.SIGTERM)
        self.still()

    def test_06_pose_interruption(self):
        enabled = [True]
        pub = rospy.Publisher('/patrol_pose_relay', Pose, queue_size=1)
        sub = rospy.Subscriber('/turtle1/pose', Pose,
            lambda p: pub.publish(p) if enabled[0] else None, queue_size=1)
        try:
            self.start('_waypoints:=[[9,2]]', '/turtle1/pose:=/patrol_pose_relay')
            self.active()
            enabled[0] = False
            self.wait(lambda: self.statuses[-1]['state'] == 'POSE_TIMEOUT', 5)
            self.still()
            self.assertTrue(all(v == 0 and w == 0 for _, v, w in self.commands[-5:]))
        finally:
            sub.unregister()
            pub.unregister()

    def test_07_sigint_during_motion(self):
        self.start('_waypoints:=[[2,9]]')
        self.active()
        self.stop(signal.SIGINT)
        self.still()


if __name__ == '__main__':
    print('source SHA256', hashlib.sha256((ROOT/'main.py').read_bytes()).hexdigest(), flush=True)
    unittest.main(verbosity=2)
