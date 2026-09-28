"""ROS2 transport, fixed-horizon forecasts and target-time-aligned errors.

Internal data use AirSim NED/FRD. All RViz positions use ENU in map.
The String state topic is a versioned JSON schema, preserving integer nanoseconds.
"""
import argparse
from collections import deque
import csv
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from nav_msgs.msg import Path as RosPath, Odometry
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from .data import DT,HISTORY,HORIZONS,STATE_FIELDS,features


def stamp(ns):
    from builtin_interfaces.msg import Time
    return Time(sec=int(ns//1000000000),nanosec=int(ns%1000000000))


def enu(p):
    return float(p[1]),float(p[0]),float(-p[2])


def quat_product(a,b):
    w,x,y,z=a; v,i,j,k=b
    return np.array([w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v])


def enu_flu_orientation(q):
    # R_ENU_FLU = R_ENU_NED * R_NED_FRD * R_FRD_FLU.
    return quat_product(quat_product([0,2**-0.5,2**-0.5,0],q),[0,1,0,0])


def pose(p,ns):
    msg=PoseStamped(); msg.header.frame_id='map'; msg.header.stamp=stamp(ns)
    msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=enu(p)
    msg.pose.orientation.w=1.0
    return msg


class StateSource(Node):
    def __init__(self,args):
        super().__init__('uav_state_source'); self.args=args
        self.pub=self.create_publisher(String,'/uav/state',10)
        self.odom_pub=self.create_publisher(Odometry,'/uav/odometry',10)
        self.path_pub=self.create_publisher(RosPath,'/uav/truth_path',10)
        self.history=deque(maxlen=600); self.index=0; self.previous_ns=None
        if args.csv:
            with Path(args.csv).open() as f: self.rows=list(csv.DictReader(f))
            self.initial_ns=int(self.rows[0]['timestamp_ns']); self.start_wall=time.monotonic()
            self.timer=self.create_timer(0.005,self.replay)
        else:
            import airsim
            self.client=airsim.MultirotorClient(ip=args.host,timeout_value=2)
            if not self.client.ping(): raise RuntimeError('AirSim ping failed')
            self.timer=self.create_timer(DT,self.live)
        self.get_logger().info('State source ready: '+('replay' if args.csv else 'live AirSim'))

    def emit(self,ns,state,episode):
        if ns==self.previous_ns: return
        self.previous_ns=ns
        self.pub.publish(String(data=json.dumps(dict(schema_version=1,timestamp_ns=ns,state=state,episode_id=episode,frame='NED_FRD'))))
        odom=Odometry(); odom.header.frame_id='map'; odom.header.stamp=stamp(ns); odom.child_frame_id='drone_flu'
        odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=enu(state[:3])
        q=enu_flu_orientation(state[9:13]); odom.pose.pose.orientation.w,odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z=map(float,q)
        # Odometry twist is in child_frame_id: rotate world-NED velocity into FRD, then FLU.
        w,x,y,z=state[9:13]
        rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        velocity=(rotation.T@np.array(state[3:6]))*np.array([1,-1,-1])
        odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z=map(float,velocity)
        odom.twist.twist.angular.x,odom.twist.twist.angular.y,odom.twist.twist.angular.z=map(float,np.array(state[13:16])*[1,-1,-1])
        self.odom_pub.publish(odom)
        self.history.append(pose(state[:3],ns)); path=RosPath(); path.header=odom.header; path.poses=list(self.history); self.path_pub.publish(path)

    def live(self):
        from .collect import pack_state
        try:
            s=self.client.getMultirotorState(vehicle_name=self.args.vehicle)
            self.emit(int(s.timestamp),pack_state(s)[1:],'live')
        except Exception as e:
            self.get_logger().error('AirSim state read failed: '+str(e))

    def replay(self):
        elapsed=(time.monotonic()-self.start_wall)*self.args.speed
        while self.index<len(self.rows):
            row=self.rows[self.index]; ns=int(row['timestamp_ns'])
            if (ns-self.initial_ns)*1e-9>elapsed: return
            self.emit(ns,[float(row[k]) for k in STATE_FIELDS],Path(self.args.csv).stem)
            self.index+=1
        self.timer.cancel(); self.get_logger().info('Replay complete; final markers remain visible')


class ForecastNode(Node):
    def __init__(self,args):
        super().__init__('uav_state_predictor')
        import torch
        from .model import load_predictor
        torch.set_num_threads(2)
        self.model,self.ckpt=load_predictor(args.model)
        self.history=deque(maxlen=80); self.pending=[]; self.episode=None; self.last_prediction_ns=0
        self.squared=np.zeros((3,6)); self.absolute=np.zeros((3,6)); self.count=np.zeros(3,dtype=int)
        self.paths=self.create_publisher(RosPath,'/uav/predicted_path',10)
        self.markers=self.create_publisher(MarkerArray,'/uav/prediction_markers',10)
        self.forecasts=self.create_publisher(String,'/uav/forecast',10)
        self.errors=self.create_publisher(String,'/uav/errors',10)
        self.sub=self.create_subscription(String,'/uav/state',self.on_state,100)
        self.error_file=None
        if args.error_log:
            p=Path(args.error_log); p.parent.mkdir(parents=True,exist_ok=True); self.error_file=p.open('w')
        self.get_logger().info('Predictor ready: '+args.model)

    def on_state(self,msg):
        from .model import predict
        payload=json.loads(msg.data)
        if payload.get('schema_version')!=1 or payload.get('frame')!='NED_FRD': return
        ns=int(payload['timestamp_ns']); state=np.array(payload['state'],dtype=np.float32)
        if state.shape!=(16,) or not np.isfinite(state).all(): return
        episode=payload['episode_id']
        if self.history and (episode!=self.episode or ns<=self.history[-1][0] or ns-self.history[-1][0]>150000000):
            self.history.clear(); self.pending.clear(); self.last_prediction_ns=0
        self.episode=episode
        if self.history:
            prev_ns,prev=self.history[-1]
            remaining=[]
            for target,hi,forecast in self.pending:
                if target>ns: remaining.append((target,hi,forecast)); continue
                if target<prev_ns: continue
                alpha=(target-prev_ns)/(ns-prev_ns)
                truth=prev[:6]*(1-alpha)+state[:6]*alpha
                error=forecast-truth
                self.squared[hi]+=error*error; self.absolute[hi]+=np.abs(error); self.count[hi]+=1
                report=dict(target_timestamp_ns=target,horizon_s=float(HORIZONS[hi]),matched_count=int(self.count[hi]),
                    position_rmse_m=float(np.sqrt(self.squared[hi,:3].sum()/(3*self.count[hi]))),
                    velocity_rmse_mps=float(np.sqrt(self.squared[hi,3:].sum()/(3*self.count[hi]))),
                    position_mae_m=float(self.absolute[hi,:3].sum()/(3*self.count[hi])))
                self.errors.publish(String(data=json.dumps(report)))
                if self.error_file: self.error_file.write(json.dumps(report)+'\n'); self.error_file.flush()
            self.pending=remaining
            if np.dot(state[9:13],prev[9:13])<0: state[9:13]*=-1
        self.history.append((ns,state))
        if ns-self.history[0][0]<int((HISTORY-1)*DT*1e9) or ns-self.last_prediction_ns<100000000: return
        times=np.array([(t-ns)*1e-9 for t,_ in self.history]); values=np.stack([s for _,s in self.history])
        grid=np.arange(-(HISTORY-1),1)*DT
        hist=np.column_stack([np.interp(grid,times,values[:,i]) for i in range(16)]).astype(np.float32)
        hist[:,9:13]/=np.linalg.norm(hist[:,9:13],axis=1,keepdims=True)
        start=time.perf_counter(); forecast=predict(self.model,self.ckpt,features(hist)[None],state[None])[0]
        elapsed=(time.perf_counter()-start)*1000; self.last_prediction_ns=ns
        path=RosPath(); path.header.frame_id='map'; path.header.stamp=stamp(ns); path.poses=[pose(state[:3],ns)]
        for hi,h in enumerate(HORIZONS):
            target=ns+int(round(float(h)*1e9)); self.pending.append((target,hi,forecast[hi].copy())); path.poses.append(pose(forecast[hi,:3],target))
        self.paths.publish(path)
        self.forecasts.publish(String(data=json.dumps(dict(timestamp_ns=ns,horizons_s=HORIZONS.tolist(),state_ned=forecast.tolist(),inference_ms=elapsed))))
        spheres=Marker(); spheres.header=path.header; spheres.ns='forecast'; spheres.id=0; spheres.type=Marker.SPHERE_LIST; spheres.action=Marker.ADD
        spheres.pose.orientation.w=1.; spheres.scale.x=spheres.scale.y=spheres.scale.z=0.16
        spheres.color.r=1.; spheres.color.g=0.35; spheres.color.a=1.
        spheres.points=[Point(x=p.pose.position.x,y=p.pose.position.y,z=p.pose.position.z) for p in path.poses[1:]]
        label=Marker(); label.header=path.header; label.ns='metrics'; label.id=1; label.type=Marker.TEXT_VIEW_FACING; label.action=Marker.ADD
        label.pose.position.z=11.; label.pose.orientation.w=1.; label.scale.z=0.4; label.color.r=label.color.g=label.color.b=label.color.a=1.
        rmse=np.sqrt(self.squared[2,:3].sum()/(3*max(self.count[2],1)))
        label.text='GRU | %.1f ms\n1 s position RMSE: %.3f m\nMatched forecasts: %d'%(elapsed,rmse,self.count[2])
        self.markers.publish(MarkerArray(markers=[spheres,label]))


def source_main():
    p=argparse.ArgumentParser(); p.add_argument('--host',default='192.168.239.1'); p.add_argument('--vehicle',default='PredictionDrone'); p.add_argument('--csv',default=''); p.add_argument('--speed',type=float,default=1)
    args=p.parse_args(remove_ros_args()[1:]); rclpy.init(); node=StateSource(args)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()


def predictor_main():
    p=argparse.ArgumentParser(); p.add_argument('--model',required=True); p.add_argument('--error-log',default='')
    args=p.parse_args(remove_ros_args()[1:]); rclpy.init(); node=ForecastNode(args)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if node.error_file: node.error_file.close()
        node.destroy_node(); rclpy.shutdown()
