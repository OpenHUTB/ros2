#!/usr/bin/env python3
"""Measure the deployed image controller near the end wall in MuJoCo."""
import argparse
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from patrol.scene import IndoorSimulator
from patrol.vision import VisionPolicy


def run(assets, weights, distance):
    sim = IndoorSimulator(assets, 7)
    try:
        # End wall's near surface is x=9.88. Distance is pelvis to wall,
        # not foot or body clearance. Settle before enabling the policy.
        sim.data.qpos[0] = 9.88 - distance
        mujoco.mj_forward(sim.model, sim.data)
        policy = VisionPolicy(Path(weights) / 'end2end.pt', 'end2end')
        for _ in range(100):
            sim.set_command(0, 0, 0)
            sim.step()
        if sim.fallen:
            return dict(distance_m=distance, settled=False, fallen=True)
        start = sim.data.qpos[:3].copy()
        commands = []
        contacts = 0
        previous = np.zeros(2)
        for tick in range(150):
            if tick % 10 == 0:
                command, _ = policy.command(sim.image())
                previous = .6 * previous + .4 * command
                sim.set_command(float(previous[0]), wz=float(previous[1]))
                commands.append(command.tolist())
            sim.step()
            for contact in sim.data.contact:
                names = [mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or ''
                         for g in (contact.geom1, contact.geom2)]
                if 'end_wall' in names:
                    contacts += 1
                    break
            if sim.fallen:
                break
        return dict(distance_m=distance, settled=True, start_position=start.tolist(),
                    final_position=sim.data.qpos[:3].tolist(), fallen=bool(sim.fallen),
                    wall_contact_steps=contacts, commands=commands)
    finally:
        sim.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', required=True)
    parser.add_argument('--weights', default=str(Path(__file__).resolve().parents[1] / 'models'))
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    results = [run(args.assets, args.weights, d) for d in (1.5, 1.0, .6)]
    Path(args.output).write_text(json.dumps(results, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(results, indent=2))
