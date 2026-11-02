"""Static checks for the required Gazebo and launch configuration."""

from __future__ import annotations

from pathlib import Path
import unittest
from xml.etree import ElementTree


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class TestSimulationAssets(unittest.TestCase):
    def test_both_worlds_define_required_gpu_lidar(self) -> None:
        for filename in ('obstacle_world.sdf', 'obstacle_world_dynamic.sdf'):
            with self.subTest(filename=filename):
                root = ElementTree.parse(
                    PACKAGE_ROOT / 'worlds' / filename
                ).getroot()
                sensor = root.find(".//sensor[@type='gpu_lidar']")
                self.assertIsNotNone(sensor)
                self.assertEqual(sensor.findtext('update_rate'), '5')
                self.assertEqual(
                    sensor.findtext('lidar/scan/horizontal/samples'),
                    '360',
                )
                self.assertEqual(sensor.findtext('lidar/range/min'), '0.05')
                self.assertEqual(sensor.findtext('lidar/range/max'), '30.0')
                visualize = root.find(".//gui/plugin[@filename='VisualizeLidar']")
                self.assertIsNotNone(visualize)
                self.assertEqual(
                    root.findtext(".//gui/plugin[@filename='MinimalScene']/engine"),
                    'ogre',
                )

    def test_main_launch_exposes_headless_argument(self) -> None:
        launch_source = (
            PACKAGE_ROOT / 'launch' / 'main.launch.py'
        ).read_text(encoding='utf-8')
        self.assertIn("'headless': headless", launch_source)
        self.assertIn("'headless'", launch_source)

        sim_source = (
            PACKAGE_ROOT / 'launch' / 'sim.launch.py'
        ).read_text(encoding='utf-8')
        self.assertIn("'--render-engine'", sim_source)
        self.assertIn("'ogre'", sim_source)
        self.assertIn("'render_engine'", sim_source)
