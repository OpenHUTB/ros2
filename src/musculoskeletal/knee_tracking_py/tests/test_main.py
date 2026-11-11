"""CLI 冒烟测试：--quick --no-render 必须退出 0 并产出文件。"""

from __future__ import annotations

import contextlib
import io
import tempfile
import unittest
from pathlib import Path

import main as app


class TestMainCli(unittest.TestCase):
    def test_quick_no_render(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            old_dir = app.RESULTS_DIR
            app.RESULTS_DIR = Path(tmp)
            try:
                with contextlib.redirect_stdout(io.StringIO()):
                    code = app.main(["--quick", "--no-render"])
            finally:
                app.RESULTS_DIR = old_dir
            self.assertEqual(code, 0)
            for name in (
                "tracking_eval.png",
                "tracking_step.png",
                "tracking.csv",
                "metrics.txt",
            ):
                path = Path(tmp) / name
                self.assertTrue(path.is_file(), name)
                self.assertGreater(path.stat().st_size, 0, name)
            self.assertFalse((Path(tmp) / "tracking.gif").exists())
            text = (Path(tmp) / "metrics.txt").read_text(encoding="utf-8")
            self.assertIn("重力前馈对照", text)
            self.assertIn("稳态误差", text)

    def test_bad_config_returns_2(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            bad = Path(tmp) / "bad.yaml"
            bad.write_text("joint:\n  inertia: -1\n", encoding="utf-8")
            with contextlib.redirect_stdout(io.StringIO()):
                with contextlib.redirect_stderr(io.StringIO()):
                    code = app.main(["--no-render", "--config", str(bad)])
            self.assertEqual(code, 2)


if __name__ == "__main__":
    unittest.main()
