"""Re-run a recorded seed and render the real MuJoCo scene; verify identical physics."""
from pathlib import Path
import numpy as np
from landing_assessment import simulation
original = simulation.make_model
def visual_model(seed):
    model,info=original(seed)
    # Only the offscreen framebuffer changes; the frozen physics model is untouched.
    model.vis.global_.offwidth=900
    model.vis.global_.offheight=600
    return model,info
simulation.make_model=visual_model
original_update=simulation.mujoco.Renderer.update_scene
def closer_view(self,data,camera=-1,**kwargs):
    if isinstance(camera,simulation.mujoco.MjvCamera):
        camera.distance=1.6
        camera.lookat[:]=[0,0,float(data.qpos[2])*.7]
        camera.elevation=-18
    return original_update(self,data,camera=camera,**kwargs)
simulation.mujoco.Renderer.update_scene=closer_view
Path('results').mkdir(exist_ok=True)
raw,info=simulation.episode(1096,Path('results/simulation_render.png'))
np.testing.assert_array_equal(raw,np.load('data/raw/episode_1096.npz')['raw'])
print('Actual MuJoCo rendering saved; rerun matches recorded physics samples exactly.')
