"""配置解析与校验测试。"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from src.config import (
    ConfigError,
    load_config,
    parse_yaml,
    to_dict,
    validate_config,
)

ROOT = Path(__file__).resolve().parents[1]


class TestYamlParser(unittest.TestCase):
    def test_nested_mapping_and_scalars(self) -> None:
        data = parse_yaml(
            "a:\n"
            "  x: 1\n"
            "  y: 2.5\n"
            "  flag: true\n"
            "  name: 'knee'\n"
            "  none: null\n"
        )
        self.assertEqual(data["a"]["x"], 1)
        self.assertEqual(data["a"]["y"], 2.5)
        self.assertIs(data["a"]["flag"], True)
        self.assertEqual(data["a"]["name"], "knee")
        self.assertIsNone(data["a"]["none"])

    def test_comment_and_inline_list(self) -> None:
        data = parse_yaml(
            "# header\n"
            "sim:\n"
            "  dt: 0.005   # step\n"
            "  gains: [1, 2.5, 3]\n"
            "  quoted: 'a # not comment'\n"
        )
        self.assertEqual(data["sim"]["dt"], 0.005)
        self.assertEqual(data["sim"]["gains"], [1, 2.5, 3])
        self.assertEqual(data["sim"]["quoted"], "a # not comment")

    def test_duplicate_key_rejected(self) -> None:
        with self.assertRaises(ConfigError):
            parse_yaml("a:\n  x: 1\n  x: 2\n")

    def test_tab_indentation_rejected(self) -> None:
        with self.assertRaises(ConfigError):
            parse_yaml("a:\n\tx: 1\n")

    def test_missing_colon_rejected(self) -> None:
        with self.assertRaises(ConfigError):
            parse_yaml("a:\n  just_text\n")

    def test_non_finite_rejected(self) -> None:
        with self.assertRaises(ConfigError):
            parse_yaml("a: .nan\n")


class TestLoadConfig(unittest.TestCase):
    def test_project_config_loads(self) -> None:
        cfg = load_config(ROOT / "config" / "params.yaml")
        self.assertEqual(cfg.joint.inertia, 0.06)
        self.assertEqual(cfg.controller.kp, 30.0)
        self.assertAlmostEqual(cfg.muscle.max_force * cfg.muscle.moment_arm, 17.5)
        self.assertEqual(cfg.simulation.dt, 0.005)

    def test_to_dict_roundtrip(self) -> None:
        cfg = load_config(ROOT / "config" / "params.yaml")
        data = to_dict(cfg)
        self.assertEqual(data["controller"]["kp"], 30.0)
        self.assertEqual(data["muscle"]["tau_deact"], 0.05)

    def test_unknown_section_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bad.yaml"
            path.write_text("mystery:\n  x: 1\n", encoding="utf-8")
            with self.assertRaises(ConfigError):
                load_config(path)

    def test_unknown_key_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bad.yaml"
            path.write_text("joint:\n  inertia: 1.0\n  wrong: 2.0\n", encoding="utf-8")
            with self.assertRaises(ConfigError):
                load_config(path)

    def test_sine_out_of_limits_rejected(self) -> None:
        cfg = load_config(ROOT / "config" / "params.yaml")
        from dataclasses import replace

        bad = replace(
            cfg,
            trajectory=replace(
                cfg.trajectory, sine_bias_deg=70.0, sine_amplitude_deg=40.0
            ),
        )
        with self.assertRaises(ConfigError):
            validate_config(bad)

    def test_invalid_dt_rejected(self) -> None:
        cfg = load_config(ROOT / "config" / "params.yaml")
        from dataclasses import replace

        bad = replace(
            cfg, simulation=replace(cfg.simulation, dt=0.0)
        )
        with self.assertRaises(ConfigError):
            validate_config(bad)


if __name__ == "__main__":
    unittest.main()
