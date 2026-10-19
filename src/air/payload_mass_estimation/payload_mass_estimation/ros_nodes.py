"""Stamped synchronized observations and payload prediction ROS2 nodes."""
import argparse,json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .data import Estimator
from .experiment import ROOT,predictor

class Replay(Node):
    def __init__(self):
        super().__init__('payload_replay');self.pub=self.create_publisher(String,'/payload/observation',100)
        self.rows=[]
        for seed in [40120,40121,40122]:
            with np.load(ROOT/f'data/episode_{seed}.npz') as a:self.rows.extend(a['observed'].tolist())
        self.i=0;self.start=time.monotonic();self.done=None;self.timer=self.create_timer(.05,self.tick)
    def tick(self):
        if time.monotonic()-self.start<2:return
        if self.i>=len(self.rows):
            if self.done is None:self.done=time.monotonic()
            if time.monotonic()-self.done>2:self.finished=True
            return
        t,z,v,f=self.rows[self.i];msg=String();msg.data=json.dumps({'stamp_s':t,'height_m':z,'velocity_z_mps':v,'command_thrust_n':f});self.pub.publish(msg);self.i+=1

class Predict(Node):
    def __init__(self):
        super().__init__('payload_estimator');self.est=Estimator(predictor());self.pub=self.create_publisher(String,'/payload/estimate',100)
        self.sub=self.create_subscription(String,'/payload/observation',self.receive,100);self.last=time.monotonic();self.count=0;self.results=[];self.create_timer(.2,self.watch)
    def receive(self,msg):
        self.last=time.monotonic()
        try:
            a=json.loads(msg.data);row=[a[k] for k in ['stamp_s','height_m','velocity_z_mps','command_thrust_n']]
            self.count+=1;res=self.est.step(row)
            if res:
                res['stamp_s']=row[0];out=String();out.data=json.dumps(res);self.pub.publish(out);self.results.append(res)
        except (ValueError,KeyError,TypeError):self.est.rows=[]
    def watch(self):
        if self.count and time.monotonic()-self.last>3:
            (ROOT/'results').mkdir(exist_ok=True);(ROOT/'results/ros_node.json').write_text(json.dumps({'observations':self.count,'predictions':len(self.results),'results':self.results},indent=2));self.finished=True

def run(cls):
    rclpy.init();node=cls()
    try:
        while rclpy.ok() and not getattr(node,'finished',False):rclpy.spin_once(node,timeout_sec=.2)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

def replay():run(Replay)
def estimate():run(Predict)
