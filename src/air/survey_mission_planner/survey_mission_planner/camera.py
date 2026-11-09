"""Camera geometry for photogrammetric survey planning."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CameraSpec:
    sensor_width_mm: float
    sensor_height_mm: float
    focal_length_mm: float
    image_width_px: int
    image_height_px: int

    def __post_init__(self) -> None:
        for name in (
            "sensor_width_mm",
            "sensor_height_mm",
            "focal_length_mm",
            "image_width_px",
            "image_height_px",
        ):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")

    def gsd_m(self, altitude_m: float) -> float:
        """Ground sample distance in metres/pixel."""
        return (
            float(altitude_m)
            * self.sensor_width_mm
            / (self.focal_length_mm * self.image_width_px)
        )

    def altitude_for_gsd_m(self, target_gsd_m: float) -> float:
        """Flight altitude above ground for a requested GSD."""
        if target_gsd_m <= 0:
            raise ValueError("target_gsd_m must be positive")
        return (
            float(target_gsd_m)
            * self.focal_length_mm
            * self.image_width_px
            / self.sensor_width_mm
        )

    def footprint_m(self, altitude_m: float) -> tuple[float, float]:
        """Ground footprint width and height in metres."""
        width = float(altitude_m) * self.sensor_width_mm / self.focal_length_mm
        height = float(altitude_m) * self.sensor_height_mm / self.focal_length_mm
        return width, height

    def as_dict(self) -> dict:
        return {
            "sensor_width_mm": self.sensor_width_mm,
            "sensor_height_mm": self.sensor_height_mm,
            "focal_length_mm": self.focal_length_mm,
            "image_width_px": self.image_width_px,
            "image_height_px": self.image_height_px,
        }
