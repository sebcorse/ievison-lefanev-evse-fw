#!/usr/bin/env python3
"""Animated GIF of the charging screen from the frames of `simui anim` (tools/sim/ui_anim_*.ppm)."""
import glob, os, sys
from PIL import Image
here = os.path.dirname(os.path.abspath(__file__))
frames = [Image.open(p).convert("RGB").resize((256, 320), Image.NEAREST)
          for p in sorted(glob.glob(os.path.join(here, "ui_anim_*.ppm")))]
if not frames:
    sys.exit("no ui_anim_*.ppm: run ./simui anim first")
out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "ui_charging.gif")
frames[0].save(out, save_all=True, append_images=frames[1:], duration=100, loop=0)
print("wrote", out, len(frames), "frames")
