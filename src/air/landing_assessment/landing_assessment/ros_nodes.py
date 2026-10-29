import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from .inference import Estimator
from .simulation import rotation

class Replay(Node):
    def __init__(self):
        super().__init__('landing_replay')
        self.declare_parameter('files','');self.rows=[]
        offset=0.
        for file in self.get_parameter('files').value.split(';'):
            raw=np.load(file)['raw']
            for row in raw:self.rows.append((offset+row[0]-raw[0,0],row))
            offset+=raw[-1,0]-raw[0,0]+.5
        self.pub=self.create_publisher(Odometry,'/uav/odom',100)
        self.done=self.create_publisher(String,'/landing/replay_done',10)
        self.start=time.monotonic()+2;self.i=0;self.finished=False
        self.create_timer(.01,self.tick)
    def tick(self):
        if self.i>=len(self.rows):
            if time.monotonic()-self.start < self.rows[-1][0]+1.:return
            self.done.publish(String(data='complete'));self.finished=True;return
        elapsed=time.monotonic()-self.start
        if elapsed-self.rows[self.i][0]>.15:
            self.start+=elapsed-self.rows[self.i][0];elapsed=self.rows[self.i][0]
        while self.i<len(self.rows) and self.rows[self.i][0]<=elapsed:
            _,row=self.rows[self.i];msg=Odometry();msg.header.frame_id='map';msg.child_frame_id='base_link'
            ns=int(round(row[0]*1e9));msg.header.stamp.sec=ns//10**9;msg.header.stamp.nanosec=ns%10**9
            msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=map(float,row[1:4])
            q=msg.pose.pose.orientation;q.w,q.x,q.y,q.z=map(float,row[7:11])
            # nav_msgs/Odometry twist is expressed in child_frame_id (FLU body).
            v=msg.twist.twist.linear;v.x,v.y,v.z=map(float,rotation(row[7:11]).T@row[4:7])
            w=msg.twist.twist.angular;w.x,w.y,w.z=map(float,row[11:14])
            self.pub.publish(msg);self.i+=1

class Assess(Node):
    def __init__(self):
        super().__init__('landing_assessment')
        self.declare_parameter('model','');self.declare_parameter('log','results/ros_predictions.jsonl')
        self.estimator=Estimator(self.get_parameter('model').value)
        self.pub=self.create_publisher(String,'/landing/assessment',100)
        self.create_subscription(Odometry,'/uav/odom',self.receive,100)
        self.create_subscription(String,'/landing/replay_done',self.end,10)
        path=Path(self.get_parameter('log').value);path.parent.mkdir(parents=True,exist_ok=True)
        self.log=path.open('w');self.finished=False;self.count=0
    def receive(self,msg):
        stamp=msg.header.stamp;t=stamp.sec+stamp.nanosec/1e9
        p=msg.pose.pose.position;v=msg.twist.twist.linear;q=msg.pose.pose.orientation;w=msg.twist.twist.angular
        quat=[q.w,q.x,q.y,q.z]
        result=self.estimator.step(t,[p.x,p.y,p.z],rotation(quat)@np.array([v.x,v.y,v.z]),quat,[w.x,w.y,w.z])
        self.pub.publish(String(data=json.dumps(result)));self.log.write(json.dumps(result)+'\n');self.log.flush();self.count+=1
    def end(self,msg):
        self.get_logger().info(f'Landing replay complete: {self.count} observations assessed')
        self.finished=True

def spin(cls):
    rclpy.init();node=cls()
    try:
        while rclpy.ok() and not node.finished:rclpy.spin_once(node,timeout_sec=.1)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if hasattr(node,'log'):node.log.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
def replay_main():spin(Replay)
def assessment_main():spin(Assess)
