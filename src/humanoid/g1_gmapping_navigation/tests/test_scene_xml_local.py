import sys
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from patrol.scene import build_scene


class SceneXmlTest(unittest.TestCase):
    def test_generates_scene_without_simulator_or_assets(self):
        for compiler in ('', '<compiler angle="radian"/>'):
            with self.subTest(compiler=compiler), TemporaryDirectory() as directory:
                assets = Path(directory)
                description = assets / 'resources/robots/g1_description'
                description.mkdir(parents=True)
                (description / 'g1_12dof.xml').write_text(
                    f'<mujoco>{compiler}<worldbody><body name="pelvis"/></worldbody></mujoco>',
                    encoding='utf-8',
                )
                xml = build_scene(assets, seed=7)
                self.assertEqual(xml, build_scene(assets, seed=7))
                root = ET.fromstring(xml)
                self.assertEqual(root.find('compiler').get('meshdir'),
                                 (description / 'meshes').as_posix())
                if compiler:
                    self.assertEqual(root.find('compiler').get('angle'), 'radian')
                names = {geom.get('name') for geom in root.findall('worldbody/geom')}
                self.assertTrue({'floor', 'left_wall', 'right_wall', 'back_wall', 'end_wall',
                                 'door_left', 'door_right', 'inspection_marker'} <= names)
                self.assertIsNotNone(root.find("worldbody/body/camera[@name='front_camera']"))


if __name__ == '__main__':
    unittest.main()
