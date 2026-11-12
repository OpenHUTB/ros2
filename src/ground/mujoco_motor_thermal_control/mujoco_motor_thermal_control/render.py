"""Render the actual MuJoCo state and save RGB pixels as a PNG."""

from __future__ import annotations

from pathlib import Path
import struct
import zlib

import mujoco


def render_snapshot(model: mujoco.MjModel, data: mujoco.MjData, output: str | Path) -> None:
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.lookat[:] = (data.qpos[0], data.qpos[1], 0.08)
    camera.distance = 0.85
    camera.azimuth = 90
    camera.elevation = -8
    with mujoco.Renderer(model, height=480, width=720) as renderer:
        renderer.update_scene(data, camera=camera)
        rgb = renderer.render().copy()

    # PNG has one unfiltered RGB row per scanline; no plotting dependency is needed.
    def chunk(kind: bytes, payload: bytes) -> bytes:
        checksum = zlib.crc32(kind + payload) & 0xFFFFFFFF
        return struct.pack("!I", len(payload)) + kind + payload + struct.pack("!I", checksum)

    height, width, _ = rgb.shape
    scanlines = b"".join(b"\x00" + row.tobytes() for row in rgb)
    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack("!2I5B", width, height, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(scanlines))
    png += chunk(b"IEND", b"")
    target = Path(output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(png)
