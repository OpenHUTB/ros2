#!/usr/bin/env python3
"""Publish successive NED goals for the mapping flight."""

import os
import sys

import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from lidar_features import advance_waypoint


class WaypointGoalNode(object):
    def __init__(self):
        self.waypoints = [(float(x), float(y)) for x, y in
                          rospy.get_param('~waypoints', [[0.0, 20.0], [16.0, 20.0]])]
        if not self.waypoints:
            raise ValueError('waypoints must not be empty')
        self.altitude = float(rospy.get_param('~altitude', 4.0))
        self.tolerance = float(rospy.get_param('~tolerance_m', 1.5))
        self.index = 0
        self.publisher = rospy.Publisher('/drone/goal', PointStamped, queue_size=1,
                                         latch=True)
        rospy.Subscriber('/drone/state', Float32MultiArray, self._on_state)

    def _on_state(self, msg):
        if len(msg.data) < 2:
            return
        next_index = advance_waypoint(self.index, msg.data[:2],
                                      self.waypoints, self.tolerance)
        if next_index != self.index:
            self.index = next_index
            rospy.loginfo('建图航点切换到 %s', self.waypoints[self.index])

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            x, y = self.waypoints[self.index]
            goal = PointStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = 'world_ned'
            goal.point.x, goal.point.y, goal.point.z = x, y, -self.altitude
            self.publisher.publish(goal)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('waypoint_goal')
    WaypointGoalNode().run()
