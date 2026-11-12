#!/usr/bin/env python3
"""Launch standalone simulation/learning tasks through a ROS launch file."""
import subprocess
import rospy


def main():
    rospy.init_node('humanoid_task_runner')
    args = [
        rospy.get_param('~sim_python'), rospy.get_param('~sim_entry'),
        '--weights', rospy.get_param('~weights'),
        '--output', rospy.get_param('~output'),
    ]
    child = subprocess.Popen(args)
    try:
        while child.poll() is None and not rospy.is_shutdown():
            rospy.rostime.wallsleep(.1)
        if child.poll() not in (None,0):
            raise RuntimeError('Simulation task failed with exit code %s'%child.returncode)
    finally:
        if child.poll() is None:
            child.terminate()
            child.wait(timeout=10)


if __name__ == '__main__':
    main()
