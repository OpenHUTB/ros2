"""Exercise malformed JSON and real subscriber watchdog on an isolated ROS domain."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from ros_diagnostics import Diagnostics
from telemetry_data import CHANNELS


def main():
    model=str(Path('models/consistency_42').resolve())
    rclpy.init(args=['--ros-args','-p','model:='+model])
    detector=Diagnostics(); observer=Node('smoke_observer')
    results=[]; diagnoses=[]
    observer.create_subscription(String,'/uav/anomaly_score',lambda m:results.append(json.loads(m.data)),100)
    observer.create_subscription(DiagnosticArray,'/diagnostics',lambda m:diagnoses.append(m),100)
    pub=observer.create_publisher(String,'/uav/telemetry',100)
    executor=SingleThreadedExecutor();executor.add_node(detector);executor.add_node(observer)
    def pump(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:executor.spin_once(timeout_sec=.01)
    try:
        pump(1.)
        pub.publish(String(data='{broken json'));pump(.2)
        row={'arrival_ns':10**18,'source_ns':10**18,'delivered':1,**{k:0 for k in CHANNELS}}
        pub.publish(String(data=json.dumps(dict(row,px=float('nan')))));pump(.2)
        for i in range(20):
            pub.publish(String(data=json.dumps(dict(row,arrival_ns=10**18+i*50_000_000,source_ns=10**18+i*50_000_000))))
            pump(.05)
        pump(.6)
        assert sum(r['reason']=='invalid_payload' for r in results)>=2
        assert any(r['reason']=='missing_packet' and r['alarm'] for r in results)
        assert any(r['model_ready'] for r in results)
        assert any(m.status[0].level==DiagnosticStatus.ERROR for m in diagnoses)
        out=Path('results/ros_robustness.json');out.parent.mkdir(exist_ok=True)
        out.write_text(json.dumps({'passed':True,'score_messages':len(results),'diagnostic_arrays':len(diagnoses),
                                  'checks':['malformed JSON','NaN payload','model warmup','no-packet watchdog','DiagnosticStatus.ERROR']},indent=2))
        print(out.read_text())
    finally:
        executor.shutdown();detector.destroy_node();observer.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
