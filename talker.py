#!/usr/bin/env python3
# 导入ros python库
import rospy
# 导入ros标准字符串消息类型
from std_msgs.msg import String

def talker():
    # 创建发布对象：话题名称chatter，消息类型String，消息队列最大保存10条
    pub = rospy.Publisher('chatter', String, queue_size=10)
    # 初始化ros节点，节点名字talker；anonymous=True自动加后缀，防止重名冲突
    rospy.init_node('talker', anonymous=True)
    # 设置发布频率 10Hz，每秒循环10次
    rate = rospy.Rate(10)

    # 只要ros没有关闭，持续循环发布消息
    while not rospy.is_shutdown():
        # 定义要发送的字符串消息
        msg_str = "ROS入门作业：节点发布测试"
        # 在终端打印日志，方便查看运行状态
        rospy.loginfo("发布的消息：%s", msg_str)
        # 将消息发布到chatter话题
        pub.publish(msg_str)
        # 休眠，控制循环频率
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    # 捕获ctrl+c终止程序的异常
    except rospy.ROSInterruptException:
        pass