#!/usr/bin/env python3
"""Plan on the live SLAM map with a learned tie breaker; execute via move_base."""
import json
import subprocess
import threading

import actionlib
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal


def main():
    rospy.init_node('neural_patrol_planner')
    latest_map=[None];pending=[None]
    rospy.Subscriber('/map',OccupancyGrid,lambda msg:latest_map.__setitem__(0,msg),queue_size=1)
    rospy.Subscriber('/move_base_simple/goal',PoseStamped,lambda msg:pending.__setitem__(0,msg),queue_size=1)
    result_pub=rospy.Publisher('/neural_patrol/result',String,queue_size=1,latch=True)
    path_pub=rospy.Publisher('/neural_patrol/path',Path,queue_size=1,latch=True)
    listener=tf.TransformListener()
    client=actionlib.SimpleActionClient('/move_base',MoveBaseAction)
    while not rospy.is_shutdown():
        if pending[0] is None or latest_map[0] is None:
            rospy.rostime.wallsleep(.1);continue
        goal=pending[0];pending[0]=None
        try:
            if goal.header.frame_id!='map': raise ValueError('Goal must use map frame')
            listener.waitForTransform('map','base_link',rospy.Time(0),rospy.Duration(5))
            position,_=listener.lookupTransform('map','base_link',rospy.Time(0))
            grid=latest_map[0]
            request=dict(map=list(grid.data),width=grid.info.width,height=grid.info.height,
                resolution=grid.info.resolution,origin=[grid.info.origin.position.x,grid.info.origin.position.y],
                start=list(position[:2]),goal=[goal.pose.position.x,goal.pose.position.y])
            command=[rospy.get_param('~sim_python'),rospy.get_param('~sim_entry'),
                     '--mode','plan_stdio','--assets',rospy.get_param('~assets'),
                     '--weights',rospy.get_param('~weights')]
            output=subprocess.run(command,input=json.dumps(request),stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE,universal_newlines=True,timeout=30,check=True)
            report=json.loads(output.stdout)
            report['request_id']=goal.header.stamp.to_nsec()
            path=Path();path.header.frame_id='map';path.header.stamp=rospy.Time.now()
            for x,y in report['waypoints']:
                pose=PoseStamped();pose.header=path.header
                pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.orientation.w=1
                path.poses.append(pose)
            path_pub.publish(path)
            if not client.wait_for_server(rospy.Duration(10)):raise RuntimeError('move_base unavailable')
            report['execution']=[]
            for pose in path.poses:
                target=MoveBaseGoal();target.target_pose=pose
                target.target_pose.header.stamp=rospy.Time.now()
                client.send_goal(target)
                completed=client.wait_for_result(rospy.Duration(45))
                state=client.get_state()
                report['execution'].append(dict(goal=[pose.pose.position.x,pose.pose.position.y],state=state))
                if not completed or state!=3:
                    client.cancel_goal();break
            report['success']=len(report['execution'])==len(path.poses) and all(p['state']==3 for p in report['execution'])
            result_pub.publish(String(data=json.dumps(report)))
        except Exception as exc:
            client.cancel_goal()
            detail=getattr(exc,'stderr',None) or str(exc)
            rospy.logerr('Neural planning failed: %s',detail)
            result_pub.publish(String(data=json.dumps(dict(success=False,error=detail,request_id=goal.header.stamp.to_nsec()))))


if __name__=='__main__':
    main()
