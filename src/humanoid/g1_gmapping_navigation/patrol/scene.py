"""Generate indoor geometry without changing the upstream robot assets."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np



def build_scene(assets, seed=0):
    asset_dir = Path(assets).resolve() / 'resources/robots/g1_description'
    tree = ET.parse(asset_dir/'g1_12dof.xml')
    root = tree.getroot()
    compiler = root.find('compiler')
    if compiler is None:
        compiler = ET.Element('compiler')
        root.insert(0, compiler)
    compiler.set('meshdir', (asset_dir / 'meshes').as_posix())
    world = root.find('worldbody')
    if world is None:
        raise ValueError('G1 XML lacks worldbody')
    pelvis = world.find("body[@name='pelvis']")
    if pelvis is None:
        raise ValueError('G1 XML lacks pelvis body')
    for geom in pelvis.iter('geom'):
        geom.set('group', '1')
    ET.SubElement(pelvis, 'camera', name='front_camera', pos='0.16 0 0.35',
                  xyaxes='0 -1 0 0.35 0 0.94', fovy='75')
    visual = root.find('visual')
    if visual is None:
        visual = ET.SubElement(root, 'visual')
    global_settings = visual.find('global')
    if global_settings is None:
        global_settings = ET.SubElement(visual, 'global')
    global_settings.attrib.update(offwidth='640', offheight='480')
    headlight_settings = visual.find('headlight')
    if headlight_settings is None:
        headlight_settings = ET.SubElement(visual, 'headlight')
    headlight_settings.attrib.update(ambient='0.4 0.4 0.4', diffuse='0.6 0.6 0.6')
    rng = np.random.default_rng(seed)
    brightness = float(rng.uniform(.6, .95))
    ET.SubElement(world, 'light', pos='3 0 4', diffuse=f'{brightness} {brightness} {brightness}')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='15 15 .1', rgba='.65 .65 .60 1')
    boxes = [('left_wall', [4, 2, 1], [6, .12, 1]),
             ('right_wall', [4, -2, 1], [6, .12, 1]),
             ('back_wall', [-2, 0, 1], [.12, 2, 1]),
             ('end_wall', [10, 0, 1], [.12, 2, 1]),
             ('door_left', [5, 1.65, .9], [.12, .35, .9]),
             ('door_right', [5, -1.65, .9], [.12, .35, .9]),
             ('cabinet', [3, .95 + float(rng.uniform(-.15, .15)), .45], [.3, .3, .45])]
    for name, pos, size in boxes:
        rgba = [*rng.uniform(.25, .75, 3), 1]
        ET.SubElement(world, 'geom', name=name, type='box',
                      pos=' '.join(map(str, pos)), size=' '.join(map(str, size)),
                      rgba=' '.join(map(str, rgba)))
    ET.SubElement(world, 'geom', name='inspection_marker', type='box',
                  pos='9.84 0 1', size='.02 .35 .35', rgba='.1 .7 .8 1',
                  contype='0', conaffinity='0')
    return ET.tostring(root, encoding='unicode')
