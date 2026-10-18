"""Observe a running pipeline and require measured states, predictions and delayed errors."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    p=argparse.ArgumentParser(); p.add_argument('--seconds',type=float,default=25); p.add_argument('--output',default='results/ros_flow_check.json'); a=p.parse_args()
    rclpy.init(); n=Node('uav_pipeline_observer')
    report={'state_count':0,'forecast_count':0,'error_count':0,'horizons_seen':[],'inference_ms':[]}
    def state(msg):
        d=json.loads(msg.data); assert d['frame']=='NED_FRD'; assert len(d['state'])==16
        assert np.isfinite(d['state']).all(); report['state_count']+=1
    def forecast(msg):
        d=json.loads(msg.data); assert np.array(d['state_ned']).shape==(3,6)
        assert np.isfinite(d['state_ned']).all(); report['forecast_count']+=1; report['inference_ms'].append(d['inference_ms'])
    def error(msg):
        d=json.loads(msg.data); assert d['matched_count']>0; assert np.isfinite(d['position_rmse_m'])
        report['error_count']+=1
        if d['horizon_s'] not in report['horizons_seen']: report['horizons_seen'].append(d['horizon_s'])
    subscriptions=[n.create_subscription(String,'/uav/state',state,100),n.create_subscription(String,'/uav/forecast',forecast,100),n.create_subscription(String,'/uav/errors',error,100)]
    end=time.monotonic()+a.seconds
    try:
        while time.monotonic()<end: rclpy.spin_once(n,timeout_sec=0.1)
    finally: n.destroy_node(); rclpy.shutdown()
    latency=report.pop('inference_ms'); report['median_inference_ms']=float(np.median(latency)) if latency else None
    report['passed']=report['state_count']>20 and report['forecast_count']>5 and set(report['horizons_seen'])=={0.25,0.5,1.0}
    path=Path(a.output); path.parent.mkdir(parents=True,exist_ok=True); path.write_text(json.dumps(report,indent=2)); print(json.dumps(report))
    if not report['passed']: raise SystemExit('ROS pipeline check failed')


if __name__=='__main__': main()
