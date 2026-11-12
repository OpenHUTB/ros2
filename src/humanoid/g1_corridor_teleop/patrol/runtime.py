import json
from pathlib import Path
import time

import numpy as np

from .scene import IndoorSimulator


def teleop(assets,seed=0,seconds=45,output=None):
    """Interactive MuJoCo viewer; arrow/WASD commands expire after 0.6s."""
    import mujoco.viewer
    sim = IndoorSimulator(assets,seed)
    command = np.zeros(2)
    speed = [.25]
    last_key = [0.]
    events = []
    def key(code):
        if code in (87,265): command[:] = [speed[0],0]
        elif code in (83,264): command[:] = [-speed[0],0]
        elif code in (65,263): command[:] = [0,.35]
        elif code in (68,262): command[:] = [0,-.35]
        elif code == 32: command[:] = 0
        elif code in (61,334): speed[0] = min(.4,speed[0]+.05)
        elif code in (45,333): speed[0] = max(.1,speed[0]-.05)
        last_key[0] = time.monotonic()
        events.append(dict(key=code,command=command.tolist(),time=float(sim.data.time)))
    print('WASD/arrows: move, space: stop, +/-: speed; repeat presses to keep moving.',flush=True)
    try:
        with mujoco.viewer.launch_passive(sim.model,sim.data,key_callback=key) as viewer:
            while viewer.is_running() and not sim.fallen and sim.data.time<seconds:
                start = time.monotonic()
                if time.monotonic()-last_key[0]>.6: command[:] = 0
                sim.set_command(command[0],wz=command[1])
                sim.step()
                viewer.sync()
                time.sleep(max(0,.02-(time.monotonic()-start)))
    finally:
        sim.set_command()
        if output:
            output=Path(output)
            output.mkdir(parents=True,exist_ok=True)
            (output/'teleop_check.json').write_text(json.dumps(dict(events=events,
                final_position=sim.data.qpos[:3].tolist(),fallen=bool(sim.fallen),
                simulated_seconds=float(sim.data.time)),indent=2),encoding='utf-8')
        sim.close()
