"""JSON lines over stdio: a process boundary between ROS and simulator Python."""
import json
import sys
import base64
from pathlib import Path

from .scene import IndoorSimulator


def serve(assets, seed=0, controller='navigation',weights=None):
    sim = IndoorSimulator(assets,seed)
    if controller in ('perception','end2end'):
        from .vision import VisionPolicy
        policy=VisionPolicy(Path(weights)/f'{controller}.pt',controller)
    else:
        policy=None
    try:
        for line in sys.stdin:
            request = json.loads(line)
            if request.get('stop'):
                break
            command = request.get('command',[0,0,0])
            if policy and sim.data.time>=2:
                prediction,_=policy.command(sim.image())
                command=[float(prediction[0]),0,float(prediction[1])]
            sim.set_command(*command)
            previous_yaw = sim.odom[2]
            previous_time = sim.data.time
            for _ in range(10):
                sim.step()
                if sim.fallen:
                    break
            rgb=sim.image()
            result = dict(time=float(sim.data.time), odom=sim.odom.tolist(),
                scan=sim.scan(), velocity=[float(sim.data.qvel[0]),float(sim.data.qvel[1])],
                angular_velocity=float((sim.odom[2]-previous_yaw)/(sim.data.time-previous_time)),
                fallen=bool(sim.fallen), joints=sim.data.qpos[7:].tolist(),
                joint_names=[sim.model.joint(i).name for i in range(1,sim.model.njnt)],
                image=base64.b64encode(rgb.tobytes()).decode('ascii'),
                image_shape=list(rgb.shape))
            print(json.dumps(result),flush=True)
    finally:
        sim.close()
