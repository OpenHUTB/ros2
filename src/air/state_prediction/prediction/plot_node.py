"""Desktop visualization from actual ROS2 topics, for systems without working RViz."""
from collections import deque
import json
import tkinter as tk
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String


class TopicPlot(Node):
    def __init__(self):
        super().__init__('uav_topic_plot')
        self.truth=deque(maxlen=600); self.future=[]; self.errors={}; self.error_series={h:deque(maxlen=300) for h in (0.25,0.5,1.0)}
        self.create_subscription(String,'/uav/state',self.on_state,100)
        self.create_subscription(Path,'/uav/predicted_path',self.on_path,10)
        self.create_subscription(String,'/uav/errors',self.on_error,100)
        self.root=tk.Tk(); self.root.title('OpenHUTB UAV state prediction | ROS2 live analysis')
        self.root.geometry('1100x700')
        self.figure=Figure(figsize=(11,7),dpi=100,facecolor='#f8fafc')
        self.axes=[self.figure.add_subplot(221),self.figure.add_subplot(222),self.figure.add_subplot(212)]
        self.canvas=FigureCanvasTkAgg(self.figure,master=self.root); self.canvas.get_tk_widget().pack(fill='both',expand=True)
        self.status=tk.StringVar(value='Waiting for ROS2 state data...')
        tk.Label(self.root,textvariable=self.status,font=('Arial',11)).pack(fill='x')
        self.root.protocol('WM_DELETE_WINDOW',self.close); self.root.after(20,self.poll)

    def on_state(self,msg):
        d=json.loads(msg.data); p=d['state'][:3]; self.truth.append((d['timestamp_ns']*1e-9,p[1],p[0],-p[2]))

    def on_path(self,msg):
        self.future=[(p.pose.position.x,p.pose.position.y,p.pose.position.z) for p in msg.poses]

    def on_error(self,msg):
        d=json.loads(msg.data); h=d['horizon_s']; self.errors[h]=d
        self.error_series[h].append((d['target_timestamp_ns']*1e-9,d['position_rmse_m']))

    def poll(self):
        for _ in range(30): rclpy.spin_once(self,timeout_sec=0)
        if self.truth: self.draw()
        self.root.after(250,self.poll)

    def draw(self):
        xy,z,err=self.axes
        for ax in self.axes: ax.clear(); ax.grid(True,color='#dfe6ee')
        recent=list(self.truth)
        xy.plot([p[1] for p in recent],[p[2] for p in recent],color='#14966a',label='Measured path',linewidth=2)
        if self.future:
            xy.plot([p[0] for p in self.future],[p[1] for p in self.future],color='#ef8c23',marker='o',label='GRU: 0.25 / 0.5 / 1 s',linewidth=2)
        xy.set(xlabel='East (m)',ylabel='North (m)',title='ROS2 trajectory: measured and predicted'); xy.legend(loc='best')
        t0=recent[0][0]; z.plot([p[0]-t0 for p in recent],[p[3] for p in recent],color='#14966a',label='Measured height')
        if self.future:
            now=recent[-1][0]-t0; z.plot([now]+[now+h for h in (0.25,0.5,1.0)], [p[2] for p in self.future],color='#ef8c23',marker='o',label='Forecast height')
        z.set(xlabel='Simulation time (s)',ylabel='Height (m)',title='Altitude'); z.legend(loc='best')
        for h,points in self.error_series.items():
            if points:
                start=points[0][0]; err.plot([p[0]-start for p in points],[p[1] for p in points],label='%g s'%h)
        err.set(xlabel='Matched target time (s)',ylabel='Cumulative position RMSE (m)',title='Delayed truth-aligned error'); err.legend(loc='best')
        self.status.set('Measured states: %d | 1 s matched forecasts: %d | 1 s RMSE: %s m' % (len(recent),self.errors.get(1.,{}).get('matched_count',0),
            '%.4f'%self.errors[1.]['position_rmse_m'] if 1. in self.errors else 'pending'))
        self.figure.tight_layout(pad=2); self.canvas.draw_idle()

    def close(self):
        self.root.destroy(); self.destroy_node(); rclpy.shutdown()


def main():
    rclpy.init(); node=TopicPlot(); node.root.mainloop()


if __name__=='__main__': main()
