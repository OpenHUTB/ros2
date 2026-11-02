"""Regenerate the three lightweight MJCF scenes / 重新生成三个轻量 MJCF 场景。"""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "assets" / "mjcf"
LAYOUTS = {
    "sparse": {
        "columns": (-5.0, 0.0, 5.0),
        "rows": (-3.5, 1.0),
        "panel_half_size": (2.0, 1.15),
        "obstacles": (),
    },
    "dense": {
        "columns": (-7.2, -2.4, 2.4, 7.2),
        "rows": (-4.6, -1.55, 1.55, 4.6),
        "panel_half_size": (2.05, 1.15),
        "obstacles": (),
    },
    "occluded": {
        "columns": (-6.3, -2.1, 2.1, 6.3),
        "rows": (-4.5, -1.5, 1.5, 4.5),
        "panel_half_size": (1.75, 1.05),
        "obstacles": (
            ("skylight", "box", (0.0, 0.0, 2.0), (1.6, 1.1, 0.4), "0.45 0.72 0.78 1"),
            ("hvac_west", "box", (-6.8, 0.0, 2.2), (1.0, 1.2, 0.6), "0.72 0.72 0.68 1"),
            ("hvac_east", "box", (6.8, 0.0, 2.2), (1.0, 1.2, 0.6), "0.72 0.72 0.68 1"),
        ),
    },
}


def _numbers(values: tuple[float, ...]) -> str:
    return " ".join(f"{value:g}" for value in values)


def build_scene(name: str, layout: dict[str, object]) -> ET.ElementTree:
    root = ET.Element("mujoco", {"model": f"urban_rooftop_{name}"})
    ET.SubElement(root, "compiler", {"angle": "radian"})
    ET.SubElement(root, "option", {"timestep": "0.01", "gravity": "0 0 -9.81"})
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", {"offwidth": "640", "offheight": "480"})
    ET.SubElement(visual, "map", {"znear": "0.05", "zfar": "60"})
    asset = ET.SubElement(root, "asset")
    for material_name, rgba in {
        "roof": "0.58 0.60 0.61 1",
        "pv": "0.04 0.16 0.30 1",
        "frame": "0.65 0.67 0.69 1",
        "uav": "0.92 0.24 0.12 1",
    }.items():
        ET.SubElement(asset, "material", {"name": material_name, "rgba": rgba})

    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", {"pos": "0 -4 18", "dir": "0 0 -1", "diffuse": "0.9 0.9 0.85"})
    ET.SubElement(world, "geom", {"name": "ground", "type": "plane", "size": "35 35 0.1", "rgba": "0.30 0.34 0.31 1"})
    ET.SubElement(world, "geom", {"name": "roof", "type": "box", "pos": "0 0 0.75", "size": "10 7 0.75", "material": "roof"})
    parapets = {
        "parapet_n": ("0 6.8 1.65", "10 0.2 0.15"),
        "parapet_s": ("0 -6.8 1.65", "10 0.2 0.15"),
        "parapet_e": ("9.8 0 1.65", "0.2 6.6 0.15"),
        "parapet_w": ("-9.8 0 1.65", "0.2 6.6 0.15"),
    }
    for geom_name, (position, size) in parapets.items():
        ET.SubElement(world, "geom", {"name": geom_name, "type": "box", "pos": position, "size": size, "material": "frame"})

    half_x, half_y = layout["panel_half_size"]
    panel_index = 0
    for y_value in layout["rows"]:
        for x_value in layout["columns"]:
            if name == "occluded" and abs(x_value) < 2.2 and abs(y_value) < 2.0:
                continue
            ET.SubElement(
                world,
                "geom",
                {
                    "name": f"pv_panel_{panel_index:02d}",
                    "type": "box",
                    "pos": _numbers((x_value, y_value, 1.62)),
                    "size": _numbers((half_x, half_y, 0.07)),
                    "material": "pv",
                },
            )
            panel_index += 1
    for obstacle_name, obstacle_type, position, size, rgba in layout["obstacles"]:
        ET.SubElement(world, "geom", {"name": obstacle_name, "type": obstacle_type, "pos": _numbers(position), "size": _numbers(size), "rgba": rgba})

    uav = ET.SubElement(world, "body", {"name": "uav", "pos": "0 0 8"})
    ET.SubElement(uav, "freejoint", {"name": "uav_free_joint"})
    ET.SubElement(uav, "geom", {"name": "uav_body", "type": "box", "size": "0.28 0.18 0.08", "material": "uav", "mass": "1.2"})
    ET.SubElement(uav, "geom", {"name": "uav_arm_x", "type": "box", "size": "0.62 0.035 0.025", "material": "frame", "mass": "0.05"})
    ET.SubElement(uav, "geom", {"name": "uav_arm_y", "type": "box", "size": "0.035 0.62 0.025", "material": "frame", "mass": "0.05"})
    for site_name, position in {
        "rotor_fl": "0.55 0.55 0",
        "rotor_fr": "0.55 -0.55 0",
        "rotor_rl": "-0.55 0.55 0",
        "rotor_rr": "-0.55 -0.55 0",
    }.items():
        ET.SubElement(uav, "site", {"name": site_name, "pos": position, "size": "0.04"})
    ET.SubElement(uav, "camera", {"name": "downward_rgb", "pos": "0 0 -0.12", "mode": "fixed", "fovy": "62"})

    actuator = ET.SubElement(root, "actuator")
    for motor_name, site_name, yaw_gear in (
        ("motor_fl", "rotor_fl", "0.18"),
        ("motor_fr", "rotor_fr", "-0.18"),
        ("motor_rl", "rotor_rl", "-0.18"),
        ("motor_rr", "rotor_rr", "0.18"),
    ):
        ET.SubElement(actuator, "motor", {"name": motor_name, "site": site_name, "gear": f"0 0 1 0 0 {yaw_gear}", "ctrlrange": "0 8", "ctrllimited": "true"})
    ET.indent(root, space="  ")
    return ET.ElementTree(root)


def main() -> int:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    for name, layout in LAYOUTS.items():
        tree = build_scene(name, layout)
        target = OUTPUT / f"rooftop_{name}.xml"
        tree.write(target, encoding="utf-8", xml_declaration=False, short_empty_elements=True)
        with target.open("a", encoding="utf-8", newline="\n") as stream:
            stream.write("\n")
        print(f"scene={name} path={target.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
