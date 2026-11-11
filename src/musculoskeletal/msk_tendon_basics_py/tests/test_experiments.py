"""三组实验结果、CSV/PNG 输出与无 matplotlib 分支测试。"""

from __future__ import annotations

import csv
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import main as main_module  # noqa: E402
from scripts import plotting  # noqa: E402
from scripts.experiments import (  # noqa: E402
    RELEASE_REFERENCE,
    STIFFNESS_MULTIPLIERS,
    STIFFNESS_REFERENCE,
    build_summary,
    run_all,
    run_isometric,
    run_release,
    run_tendon_stiffness,
)
from scripts.muscle_tendon import MuscleTendonParams, rigid_force_limit  # noqa: E402


class TestIsometricExperiment(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.r = run_isometric(quick=False)

    def _at(self, a):
        idx = int(np.argmin(np.abs(self.r["activations"] - a)))
        return idx

    def test_reference_points_half_activation(self):
        i = self._at(0.5)
        self.assertAlmostEqual(self.r["elastic_force"][i], 52.2, delta=0.1)
        self.assertAlmostEqual(self.r["rigid_force"][i], 53.7, delta=0.1)
        self.assertAlmostEqual(self.r["loss_pct"][i], 2.7, delta=0.1)
        self.assertAlmostEqual(self.r["elastic_fiber_length"][i], 0.833, delta=0.001)
        self.assertAlmostEqual(self.r["elastic_strain_pct"][i], 3.5, delta=0.1)
        self.assertAlmostEqual(self.r["elastic_energy"][i], 0.045, delta=0.001)

    def test_reference_points_full_activation(self):
        i = self._at(1.0)
        self.assertAlmostEqual(self.r["elastic_force"][i], 101.6, delta=0.1)
        self.assertAlmostEqual(self.r["rigid_force"][i], 107.4, delta=0.2)
        self.assertAlmostEqual(self.r["elastic_fiber_length"][i], 0.816, delta=0.001)
        self.assertAlmostEqual(self.r["elastic_strain_pct"][i], 6.8, delta=0.1)
        self.assertAlmostEqual(self.r["elastic_energy"][i], 0.172, delta=0.001)

    def test_force_monotonic_with_activation(self):
        self.assertTrue(np.all(np.diff(self.r["elastic_force"]) > 0.0))
        self.assertTrue(np.all(np.diff(self.r["rigid_force"]) > 0.0))

    def test_every_equilibrium_has_positive_tendon_strain(self):
        self.assertTrue(np.all(self.r["elastic_strain_pct"] >= 0.0))

    def test_rigid_limit_function_agrees_with_experiment(self):
        p = MuscleTendonParams()
        limit = rigid_force_limit(p, activation=1.0)
        idx = self._at(1.0)
        self.assertAlmostEqual(limit, self.r["rigid_force"][idx], places=9)
        self.assertGreater(limit, self.r["elastic_force"][idx])


class TestStiffnessExperiment(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.r = run_tendon_stiffness()

    def test_all_reference_rows(self):
        for i, multiplier in enumerate(STIFFNESS_MULTIPLIERS):
            ref_force, ref_strain, ref_energy = STIFFNESS_REFERENCE[float(multiplier)]
            self.assertAlmostEqual(self.r["force"][i], ref_force, delta=0.1)
            self.assertAlmostEqual(self.r["strain_pct"][i], ref_strain, delta=0.1)
            self.assertAlmostEqual(self.r["energy"][i], ref_energy, delta=0.001)

    def test_stiffer_tendon_transmits_more_force(self):
        self.assertTrue(np.all(np.diff(self.r["force"]) > 0.0))

    def test_stiffer_tendon_stores_less_energy(self):
        self.assertTrue(np.all(np.diff(self.r["energy"]) < 0.0))

    def test_stiffer_tendon_has_less_strain(self):
        self.assertTrue(np.all(np.diff(self.r["strain_pct"]) < 0.0))


class TestReleaseExperiment(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.r = run_release(quick=True)
        cls.e = cls.r["elastic"]
        cls.g = cls.r["rigid"]

    def test_pre_release(self):
        self.assertAlmostEqual(self.e.pre_force, RELEASE_REFERENCE["pre_force"], delta=0.2)
        self.assertAlmostEqual(self.e.pre_energy, RELEASE_REFERENCE["pre_energy"], delta=0.002)

    def test_min_energy_and_fraction(self):
        self.assertAlmostEqual(self.e.min_energy, RELEASE_REFERENCE["min_energy"], delta=0.002)
        self.assertAlmostEqual(
            100.0 * self.e.release_fraction,
            RELEASE_REFERENCE["release_fraction"],
            delta=2.0,
        )

    def test_peak_angles(self):
        self.assertAlmostEqual(
            self.e.peak_angle, RELEASE_REFERENCE["peak_angle"], delta=0.01
        )
        self.assertAlmostEqual(
            self.g.peak_angle, RELEASE_REFERENCE["rigid_peak_angle"], delta=0.01
        )


class TestOutputPipeline(unittest.TestCase):
    def test_run_all_generates_required_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = run_all(outdir=tmp, quick=True)
            out = Path(tmp)
            for name in (
                "isometric.csv",
                "tendon_stiffness.csv",
                "release.csv",
                "summary.txt",
            ):
                self.assertTrue((out / name).is_file(), msg=name)
                self.assertGreater((out / name).stat().st_size, 0, msg=name)
            if plotting.HAS_MPL:
                for name in ("isometric.png", "tendon_stiffness.png", "release.png"):
                    self.assertTrue((out / name).is_file(), msg=name)
            summary = (out / "summary.txt").read_text(encoding="utf-8")
            self.assertIn("实验 1", summary)
            self.assertIn("实验 2", summary)
            self.assertIn("实验 3", summary)
            self.assertIn("复现说明", summary)
            self.assertGreater(result["isometric"]["elastic_force"][-1], 0.0)

    def test_csv_headers_and_rows(self):
        with tempfile.TemporaryDirectory() as tmp:
            run_all(outdir=tmp, quick=True)
            out = Path(tmp)
            with (out / "isometric.csv").open(encoding="utf-8", newline="") as f:
                rows = list(csv.reader(f))
            self.assertGreaterEqual(len(rows), 6)
            self.assertEqual(rows[0][0], "activation")
            self.assertEqual(rows[0][1], "elastic_force_N")
            with (out / "release.csv").open(encoding="utf-8", newline="") as f:
                rows = list(csv.reader(f))
            self.assertGreater(len(rows), 100)
            self.assertEqual(rows[0][0], "time_s")
            self.assertIn(rows[1][1], ("hold", "release"))

    def test_plotting_skips_without_matplotlib(self):
        with mock.patch.object(plotting, "HAS_MPL", False):
            self.assertFalse(
                plotting.plot_isometric({}, Path("unused.png"))
            )

    def test_main_exit_zero_without_matplotlib(self):
        with tempfile.TemporaryDirectory() as tmp:
            with mock.patch.object(plotting, "HAS_MPL", False):
                code = main_module.main(["--quick", "--outdir", tmp])
            self.assertEqual(code, 0)
            out = Path(tmp)
            for name in ("isometric.csv", "tendon_stiffness.csv",
                         "release.csv", "summary.txt"):
                self.assertTrue((out / name).is_file(), msg=name)
            for name in ("isometric.png", "tendon_stiffness.png", "release.png"):
                self.assertFalse((out / name).exists(), msg=name)

    def test_summary_builder_mentions_rigid_note(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = run_all(outdir=tmp, quick=True)
            text = build_summary(
                result["isometric"], result["stiffness"], result["release"],
                result["plots"], quick=True,
            )
            self.assertIn("107.0", text)
            self.assertIn("二分法", text)


if __name__ == "__main__":
    unittest.main()
