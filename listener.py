#!/usr/bin/env python3
# 导入ros python库
import rospy
# 导入ros标准字符串消息类型
from std_msgs.msg import String

# 回调函数：一旦收到话题chatter的消息，自动执行这个函数
def callback(data):
    # 打印收到的消息内容，data.data就是消息本体
    rospy.loginfo("订阅节点收到数据：%s", data.data)

def listener():
    # 初始化订阅节点，节点名称listener
    rospy.init_node('listener', anonymous=True)
    # 订阅chatter话题，消息类型String，收到消息交给callback处理
    rospy.Subscriber("chatter", String, callback)
    # 保持节点持续运行，等待接收消息，循环监听
    rospy.spin()

if __name__ == '__main__':
    rospy.loginfo("订阅节点已经启动，等待接收消息...")
    listener()