import json,time
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .experiment import ROOT

def main():
    rclpy.init();node=Node('payload_plot');rows=[]
    last=[None]
    def receive(msg):
        rows.append(json.loads(msg.data));last[0]=time.monotonic()
    node.create_subscription(String,'/payload/estimate',receive,100)
    deadline=time.monotonic()+90
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
        if last[0] is not None and time.monotonic()-last[0]>5:break
    if not rows:
        node.destroy_node();rclpy.shutdown();raise RuntimeError('No ROS estimates received')
    fig,ax=plt.subplots(figsize=(10,4))
    ax.plot([r['mass_kg'] for r in rows],label='MLP');ax.plot([r['physics_kg'] for r in rows],label='Physics impulse',alpha=.7)
    ax.set(xlabel='Received prediction index',ylabel='Estimated payload (kg)',title='Actual ROS2 subscriber: three flight replays');ax.legend();ax.grid(alpha=.25);fig.tight_layout()
    out=ROOT/'results';out.mkdir(exist_ok=True);fig.savefig(out/'ros_subscriber.png',dpi=150);(out/'ros_subscriber.json').write_text(json.dumps({'messages':len(rows),'rows':rows},indent=2));node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
