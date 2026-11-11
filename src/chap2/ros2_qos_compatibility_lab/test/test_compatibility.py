"""Tests for the pure-Python QoS compatibility rules."""

from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ros2_qos_compatibility_lab.compatibility import (  # noqa: E402
    Durability,
    EndpointQoS,
    Reliability,
    compare_qos,
)


class CompatibilityTest(unittest.TestCase):
    """Cover the DDS offered/requested compatibility matrix."""

    def test_reliable_publisher_matches_reliable_subscriber(self) -> None:
        report = compare_qos(
            EndpointQoS(Reliability.RELIABLE, Durability.VOLATILE),
            EndpointQoS(Reliability.RELIABLE, Durability.VOLATILE),
        )

        self.assertTrue(report.compatible)
        self.assertEqual(report.issues, ())

    def test_reliable_publisher_matches_best_effort_subscriber(self) -> None:
        report = compare_qos(
            EndpointQoS(Reliability.RELIABLE, Durability.VOLATILE),
            EndpointQoS(Reliability.BEST_EFFORT, Durability.VOLATILE),
        )

        self.assertTrue(report.compatible)

    def test_best_effort_publisher_cannot_match_reliable_subscriber(
        self,
    ) -> None:
        report = compare_qos(
            EndpointQoS(Reliability.BEST_EFFORT, Durability.VOLATILE),
            EndpointQoS(Reliability.RELIABLE, Durability.VOLATILE),
        )

        self.assertFalse(report.compatible)
        self.assertEqual(report.summary, "incompatible")
        self.assertEqual(len(report.issues), 1)
        self.assertEqual(report.issues[0].policy, "reliability")
        self.assertIn("RELIABLE publisher", report.issues[0].recommendation)

    def test_transient_local_publisher_matches_volatile_subscriber(
        self,
    ) -> None:
        report = compare_qos(
            EndpointQoS(
                Reliability.RELIABLE,
                Durability.TRANSIENT_LOCAL,
            ),
            EndpointQoS(Reliability.RELIABLE, Durability.VOLATILE),
        )

        self.assertTrue(report.compatible)

    def test_volatile_publisher_cannot_match_transient_local_subscriber(
        self,
    ) -> None:
        report = compare_qos(
            EndpointQoS(Reliability.RELIABLE, Durability.VOLATILE),
            EndpointQoS(
                Reliability.RELIABLE,
                Durability.TRANSIENT_LOCAL,
            ),
        )

        self.assertFalse(report.compatible)
        self.assertEqual(report.issues[0].policy, "durability")

    def test_reports_both_incompatible_policies(self) -> None:
        report = compare_qos(
            EndpointQoS(
                Reliability.BEST_EFFORT,
                Durability.VOLATILE,
            ),
            EndpointQoS(
                Reliability.RELIABLE,
                Durability.TRANSIENT_LOCAL,
            ),
        )

        self.assertEqual(
            [issue.policy for issue in report.issues],
            ["reliability", "durability"],
        )

    def test_accepts_ros_style_enum_names(self) -> None:
        report = compare_qos(
            EndpointQoS("ReliabilityPolicy.BEST_EFFORT", "VOLATILE"),
            EndpointQoS("ReliabilityPolicy.RELIABLE", "VOLATILE"),
        )

        self.assertFalse(report.compatible)
        self.assertEqual(
            report.publisher.reliability,
            Reliability.BEST_EFFORT,
        )

    def test_rejects_unknown_policy(self) -> None:
        with self.assertRaisesRegex(
            ValueError,
            "unsupported reliability policy",
        ):
            compare_qos(
                EndpointQoS("sometimes", Durability.VOLATILE),
                EndpointQoS(Reliability.RELIABLE, Durability.VOLATILE),
            )


if __name__ == "__main__":
    unittest.main()
