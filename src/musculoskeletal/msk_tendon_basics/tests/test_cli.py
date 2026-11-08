# -*- coding: utf-8 -*-
"""命令行入口测试：roslaunch 追加的私有参数不能被 argparse 拒绝。"""

from __future__ import annotations

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import main as cli  # noqa: E402


class CliTest(unittest.TestCase):
    def test_ros_remaps_are_ignored(self):
        with mock.patch.object(cli, "run") as run:
            code = cli.main([
                "--quick",
                "--outdir", "demo_test",
                "__name:=msk_tendon_basics",
                "__log:=/tmp/msk.log",
            ])
        self.assertEqual(code, 0)
        run.assert_called_once_with("demo_test", quick=True)

    def test_normal_args_still_work(self):
        with mock.patch.object(cli, "run") as run:
            code = cli.main(["--outdir", "demo_test"])
        self.assertEqual(code, 0)
        run.assert_called_once_with("demo_test", quick=False)


if __name__ == "__main__":
    unittest.main()
