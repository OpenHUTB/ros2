"""Live Tk viewer for the CA-Sim closed loop, with optional screen capture.

The window shows exactly what the pipeline renders; every displayed frame can be
grabbed from the X server (``xwd``) so the resulting GIF is a genuine screen
recording of the running program rather than an offline re-encode.

Usage (inside the Ubuntu VM, with DISPLAY set)::

    python3 tools/screen_demo.py --scene dense --out /home/user/ca/rec --capture
"""

from __future__ import annotations

import argparse
import os
import struct
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import tkinter as tk  # noqa: E402
from PIL import Image  # noqa: E402

from ca_drone.pipeline import NeuralStack, run_episode  # noqa: E402


def read_xwd(path: str) -> Image.Image:
    """Parse an XWD dump produced by ``xwd -id`` (TrueColor, 24/32 bpp)."""
    with open(path, "rb") as f:
        hdr = f.read(100)
        vals = struct.unpack(">25I", hdr)
        header_size, _, _, _, width, height = vals[:6]
        bits_per_pixel, bytes_per_line = vals[11], vals[12]
        ncolors = vals[19]
        f.seek(header_size + ncolors * 12)
        raw = f.read(height * bytes_per_line)
    bpp = bits_per_pixel // 8
    arr = Image.frombytes("RGB", (bytes_per_line // bpp, height), raw, "raw", "BGRX" if bpp == 4 else "BGR")
    return arr.crop((0, 0, width, height))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="dense")
    ap.add_argument("--mode", default="full")
    ap.add_argument("--render-every", type=int, default=8)
    ap.add_argument("--hold", type=float, default=0.10, help="seconds each frame stays on screen")
    ap.add_argument("--out", default="")
    ap.add_argument("--capture", action="store_true")
    ap.add_argument("--weights", default="weights")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    if args.out:
        os.makedirs(args.out, exist_ok=True)

    root = tk.Tk()
    root.title("Neural CA Drone - CA-Sim")
    root.geometry("960x600+240+120")
    root.configure(bg="#0b0e14")
    label = tk.Label(root, bd=0, bg="#0b0e14")
    label.pack(fill="both", expand=True)

    wid = hex(root.winfo_id())
    print("window", wid, flush=True)

    shots = []
    live = os.path.join(args.out or ".", "_live.png")
    os.makedirs(os.path.dirname(live) or ".", exist_ok=True)

    def on_frame(img):
        # Tk 8.6 reads PNG directly, so no ImageTk / PIL.imagetk is needed.
        img.save(live)
        photo = tk.PhotoImage(file=live)
        label.configure(image=photo)
        label.image = photo
        root.update()
        if args.capture and args.out:
            tmp = os.path.join(args.out, "_frame.xwd")
            subprocess.run(["xwd", "-id", wid, "-silent", "-out", tmp], check=True)
            shot = read_xwd(tmp)
            path = os.path.join(args.out, f"frame_{len(shots):03d}.png")
            shot.save(path)
            shots.append(path)
        time.sleep(args.hold)
        root.update()

    stack = NeuralStack(args.weights)
    result = run_episode(args.scene, args.mode, stack, render_every=args.render_every,
                         seed=args.seed, on_frame=on_frame)
    print(f"steps={result.steps} success={result.success} collision={result.collision} "
          f"minclr={result.min_clearance:.2f} shots={len(shots)}", flush=True)
    time.sleep(0.6)
    root.destroy()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
