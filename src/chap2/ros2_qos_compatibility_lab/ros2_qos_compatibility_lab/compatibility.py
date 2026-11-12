"""Pure-Python ROS 2 QoS compatibility rules.

Only Reliability and Durability participate in the compatibility checks
implemented here. History, depth, deadline, lifespan, and liveliness are
reported by ROS 2 but are outside this module's scope.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Tuple, Union


PolicyValue = Union[str, Enum, object]


class Reliability(str, Enum):
    """QoS reliability policy."""

    RELIABLE = "reliable"
    BEST_EFFORT = "best_effort"


class Durability(str, Enum):
    """QoS durability policy."""

    VOLATILE = "volatile"
    TRANSIENT_LOCAL = "transient_local"


@dataclass(frozen=True)
class EndpointQoS:
    """The subset of endpoint QoS used by compatibility checks."""

    reliability: Reliability = Reliability.RELIABLE
    durability: Durability = Durability.VOLATILE


@dataclass(frozen=True)
class CompatibilityIssue:
    """One incompatible QoS policy between a publisher and a subscriber."""

    policy: str
    publisher_value: str
    subscriber_value: str
    recommendation: str


@dataclass(frozen=True)
class CompatibilityReport:
    """The compatibility result for one publisher/subscriber pair."""

    publisher: EndpointQoS
    subscriber: EndpointQoS
    issues: Tuple[CompatibilityIssue, ...]

    @property
    def compatible(self) -> bool:
        """Return True when every checked policy is compatible."""

        return not self.issues

    @property
    def summary(self) -> str:
        """Return a concise human-readable verdict."""

        return "compatible" if self.compatible else "incompatible"


_RELIABILITY_RANK = {
    Reliability.BEST_EFFORT: 0,
    Reliability.RELIABLE: 1,
}
_DURABILITY_RANK = {
    Durability.VOLATILE: 0,
    Durability.TRANSIENT_LOCAL: 1,
}


def _policy_name(value: PolicyValue) -> str:
    name = getattr(value, "name", value)
    normalized = str(name).strip().lower().replace("-", "_")
    return normalized.rsplit(".", 1)[-1]


def _reliability(value: PolicyValue) -> Reliability:
    name = _policy_name(value)
    try:
        return Reliability(name)
    except ValueError as error:
        raise ValueError(
            f"unsupported reliability policy: {value!r}"
        ) from error


def _durability(value: PolicyValue) -> Durability:
    name = _policy_name(value)
    try:
        return Durability(name)
    except ValueError as error:
        raise ValueError(
            f"unsupported durability policy: {value!r}"
        ) from error


def compare_qos(
    publisher_qos: EndpointQoS,
    subscriber_qos: EndpointQoS,
) -> CompatibilityReport:
    """Compare QoS offered by a publisher with QoS requested by a subscriber.

    ROS 2 QoS compatibility follows the DDS offered/requested model:

    * A RELIABLE publisher can satisfy either subscriber policy, while a
      BEST_EFFORT publisher cannot satisfy a RELIABLE subscriber.
    * A TRANSIENT_LOCAL publisher can satisfy either subscriber policy, while
      a VOLATILE publisher cannot satisfy a TRANSIENT_LOCAL subscriber.
    """

    offered_reliability = _reliability(publisher_qos.reliability)
    requested_reliability = _reliability(subscriber_qos.reliability)
    offered_durability = _durability(publisher_qos.durability)
    requested_durability = _durability(subscriber_qos.durability)
    normalized_publisher = EndpointQoS(
        reliability=offered_reliability,
        durability=offered_durability,
    )
    normalized_subscriber = EndpointQoS(
        reliability=requested_reliability,
        durability=requested_durability,
    )
    issues = []

    if (
        _RELIABILITY_RANK[offered_reliability]
        < _RELIABILITY_RANK[requested_reliability]
    ):
        issues.append(
            CompatibilityIssue(
                policy="reliability",
                publisher_value=offered_reliability.value,
                subscriber_value=requested_reliability.value,
                recommendation=(
                    "Use a RELIABLE publisher or a BEST_EFFORT subscriber."
                ),
            )
        )

    if (
        _DURABILITY_RANK[offered_durability]
        < _DURABILITY_RANK[requested_durability]
    ):
        issues.append(
            CompatibilityIssue(
                policy="durability",
                publisher_value=offered_durability.value,
                subscriber_value=requested_durability.value,
                recommendation=(
                    "Use a TRANSIENT_LOCAL publisher or a VOLATILE "
                    "subscriber."
                ),
            )
        )

    return CompatibilityReport(
        publisher=normalized_publisher,
        subscriber=normalized_subscriber,
        issues=tuple(issues),
    )
