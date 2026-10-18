"""CSV, KML and DJI WPML export for survey plans."""

from __future__ import annotations

import csv
from pathlib import Path
from xml.sax.saxutils import escape

from .planner import SurveyPlan


def export_csv(plan: SurveyPlan, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["index", "east_m", "north_m", "latitude", "longitude", "altitude_m"])
        for index, ((east, north), (lat, lon, altitude)) in enumerate(
            zip(plan.waypoints_enu, plan.waypoints_latlon())
        ):
            writer.writerow([index, f"{east:.3f}", f"{north:.3f}", f"{lat:.8f}", f"{lon:.8f}", f"{altitude:.3f}"])


def export_kml(plan: SurveyPlan, path: str | Path) -> None:
    coordinates = "\n".join(
        f"{lon:.8f},{lat:.8f},{altitude:.3f}"
        for lat, lon, altitude in plan.waypoints_latlon()
    )
    content = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>survey_mission</name>
    <Placemark>
      <name>survey_path</name>
      <LineString>
        <altitudeMode>relativeToGround</altitudeMode>
        <coordinates>
{coordinates}
        </coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>
"""
    Path(path).write_text(content, encoding="utf-8")


def export_wpml(plan: SurveyPlan, path: str | Path) -> None:
    """Export a minimal DJI WPML file.

    The schema changes between DJI Pilot versions; validate the file in the
    target controller before flight. The values here are intentionally simple
    and readable rather than vendor-specific advanced actions.
    """
    placemarks = []
    for index, (lat, lon, altitude) in enumerate(plan.waypoints_latlon()):
        placemarks.append(
            f"""    <Placemark>
      <Point><coordinates>{lon:.8f},{lat:.8f},{altitude:.3f}</coordinates></Point>
      <wpml:index>{index}</wpml:index>
      <wpml:executeHeight>{altitude:.3f}</wpml:executeHeight>
      <wpml:waypointSpeed>8.0</wpml:waypointSpeed>
      <wpml:waypointHeadingParam>
        <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>
      </wpml:waypointHeadingParam>
    </Placemark>"""
        )
    content = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.2">
  <Document>
    <wpml:missionConfig>
      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>
      <wpml:finishAction>goHome</wpml:finishAction>
      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>
      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
      <wpml:globalTransitionalSpeed>8</wpml:globalTransitionalSpeed>
      <wpml:droneInfo><wpml:droneEnumValue>68</wpml:droneEnumValue></wpml:droneInfo>
    </wpml:missionConfig>
    <Folder>
      <wpml:templateId>0</wpml:templateId>
      <wpml:waylineCoordinateSysParam>
        <wpml:coordinateMode>WGS84</wpml:coordinateMode>
      </wpml:waylineCoordinateSysParam>
      <wpml:autoFlightSpeed>8</wpml:autoFlightSpeed>
{chr(10).join(placemarks)}
    </Folder>
  </Document>
</kml>
"""
    Path(path).write_text(content, encoding="utf-8")
