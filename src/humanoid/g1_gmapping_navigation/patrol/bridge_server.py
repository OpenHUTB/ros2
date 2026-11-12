"""JSON lines over stdio: a process boundary between ROS and simulator Python."""
import json
import sys
import base64

from .simulator import IndoorSimulator


def serve(assets, seed=0):
    sim = IndoorSimulator(assets, seed)
    try:
        for line in sys.stdin:
            request = json.loads(line)
            if request.get('stop'):
                break
            command = request.get('command',[0,0,0])
            sim.set_command(*command)
            previous_yaw = sim.odom[2]
            previous_time = sim.data.time
            for _ in range(10):
                sim.step()
                if sim.fallen:
                    break
            rgb=sim.image()
            camera_position, camera_rotation = sim.camera_transform()
            result = dict(time=float(sim.data.time), odom=sim.odom.tolist(),
                scan=sim.scan(), velocity=sim.data.qvel[:2].tolist(),
                angular_velocity=float((sim.odom[2] - previous_yaw) / (sim.data.time - previous_time)),
                camera_position=camera_position, camera_rotation=camera_rotation,
                fallen=bool(sim.fallen), joints=sim.data.qpos[7:].tolist(),
                joint_names=[sim.model.joint(i).name for i in range(1,sim.model.njnt)],
                image=base64.b64encode(rgb.tobytes()).decode('ascii'),
                image_shape=list(rgb.shape))
            print(json.dumps(result),flush=True)
    finally:
        sim.close()
