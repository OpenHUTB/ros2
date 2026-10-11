#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

def send_goal(x,y,yaw):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = yaw
    rospy.loginfo(f"🚚 小车前往快递点：x={x},y={y}")
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("导航失败！")
        return False
    else:
        if client.get_state() == 3:
            rospy.loginfo("✅ 到达配送点，等待2秒继续下一单")
            rospy.sleep(2)
            return True
        else:
            rospy.logwarn("到达目标失败")
            return False

if __name__ == '__main__':
    rospy.init_node('delivery_task_node')
    # 快递任务序列：起点仓库 → 一号快递点 →二号快递点 → 返回仓库
    task_list = [
        [0.0, 0.0, 1.0],
        [2.5, 0.0, 1.0],
        [0.0, 3.5, 1.0],
        [0.0, 0.0, 1.0]
    ]
    rospy.loginfo("====快递配送任务启动====")
    for point in task_list:
        send_goal(point[0],point[1],point[2])
    rospy.loginfo("🎉全部快递配送完成！任务结束")

