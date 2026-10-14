import os
import py_compile
import unittest
import xml.etree.ElementTree as ET

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PKG = os.path.join(ROOT, 'src', 'turtlebot3_keyboard_teleop_sim')
MAIN = os.path.join(PKG, 'scripts', 'main.py')
LAUNCH = os.path.join(PKG, 'launch', 'keyboard_teleop_sim.launch')
PKGXML = os.path.join(PKG, 'package.xml')


class TestKeyboardTeleopSimLocal(unittest.TestCase):
    def test_main_compiles(self):
        py_compile.compile(MAIN, doraise=True)

    def test_launch_has_world_and_teleop(self):
        text = open(LAUNCH, encoding='utf-8').read()
        self.assertIn('turtlebot3_world.launch', text)
        self.assertIn('turtlebot3_teleop_key', text)

    def test_package_xml(self):
        root = ET.parse(PKGXML).getroot()
        self.assertEqual(root.find('name').text,
                         'turtlebot3_keyboard_teleop_sim')
        deps = [e.text for e in root.findall('exec_depend')]
        self.assertIn('turtlebot3_gazebo', deps)
        self.assertIn('turtlebot3_teleop', deps)


if __name__ == '__main__':
    unittest.main()
