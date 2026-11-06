import os
import unittest
from ament_index_python.packages import get_package_share_directory


class TestPackage(unittest.TestCase):
    def test_launch_files_exist(self):
        pkg = get_package_share_directory('carla_rl_driving')
        launch_dir = os.path.join(pkg, 'launch')
        self.assertTrue(os.path.isdir(launch_dir))
        self.assertGreaterEqual(len(os.listdir(launch_dir)), 1)


if __name__ == '__main__':
    unittest.main()
