"""Display actual ROS assessment messages, never offline labels."""
import argparse
import json
import time
from pathlib import Path
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .data import PHASES

def main():
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=35);p.add_argument('--output',default='results/ros_dashboard.png');a=p.parse_args()
    rclpy.init();node=Node('landing_dashboard');rows=[]
    node.create_subscription(String,'/landing/assessment',lambda m:rows.append(json.loads(m.data)),100)
    plt.ion();fig,axes=plt.subplots(2,1,figsize=(10,6));fig.suptitle('Landing assessment — actual ROS2 subscriber')
    until=time.monotonic()+a.seconds
    next_draw=0.
    try:
        while time.monotonic()<until and plt.fignum_exists(fig.number):
            # Drain queued ROS messages before drawing; slow VM graphics must not
            # limit the subscriber to one message per expensive redraw.
            for _ in range(100):rclpy.spin_once(node,timeout_sec=0.)
            if time.monotonic()<next_draw:
                plt.pause(.02);continue
            next_draw=time.monotonic()+.3
            for ax in axes:ax.clear()
            ready=[r for r in rows if r.get('ready')]
            if ready:
                axes[0].plot(range(len(ready)),[r['height_m'] for r in ready],label='Foot clearance')
                axes[1].step(range(len(ready)),[r['phase_index'] for r in ready],where='post',label='Neural phase')
                impacts=[r['predicted_peak_load_factor'] for r in ready if 'predicted_peak_load_factor' in r]
                axes[0].set_title('Predicted peak loads (body weights): '+', '.join(f'{v:.2f}' for v in impacts),fontsize=11)
            axes[0].set_ylabel('Clearance (m)');axes[1].set_yticks(range(4));axes[1].set_yticklabels(PHASES);axes[1].set_xlabel('Received observation index')
            for ax in axes:ax.grid(alpha=.2)
            fig.tight_layout(rect=[0,0,1,.93]);plt.pause(.02)
        output=Path(a.output);output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(output,dpi=150)
        output.with_suffix('.json').write_text(json.dumps({'source':'actual /landing/assessment subscriber','messages':len(rows),'ready_messages':sum(r.get('ready',False) for r in rows),'impact_predictions':sum('predicted_peak_load_factor' in r for r in rows)},indent=2))
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        plt.close(fig)

if __name__=='__main__':main()
