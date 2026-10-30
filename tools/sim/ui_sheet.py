#!/usr/bin/env python3
"""Tile the tools/sim/ui_*.ppm screenshots written by simui into docs/ui-preview.png (x3)."""
import os
from PIL import Image, ImageDraw
ORDER = ["boot", "ready_in", "ready_greet", "ready_greet_b", "ready_out", "ready_rest", "ready_rest_b", "noclock", "connected", "charging_a",
         "charging_b", "charging_sun", "charging_edf", "charging_man", "charging_man_ct", "charging_forced",
         "charging_fallback", "paused", "paused_sun", "nolink", "fault_e1", "fault_e3", "fault_e10",
         "disabled", "diag"]
here = os.path.dirname(os.path.abspath(__file__))
files = [(n, os.path.join(here, f"ui_{n}.ppm")) for n in ORDER if os.path.exists(os.path.join(here, f"ui_{n}.ppm"))]
S, PAD, LABEL, COLS = 3, 12, 18, 5
W, H = 128 * S, 160 * S
rows = (len(files) + COLS - 1) // COLS
sheet = Image.new("RGB", (COLS * (W + PAD) + PAD, rows * (H + LABEL + PAD) + PAD), (40, 40, 40))
d = ImageDraw.Draw(sheet)
for i, (n, p) in enumerate(files):
    im = Image.open(p).resize((W, H), Image.NEAREST)
    x = PAD + (i % COLS) * (W + PAD); y = PAD + (i // COLS) * (H + LABEL + PAD)
    d.text((x, y + 2), n, fill=(230, 230, 230))
    sheet.paste(im, (x, y + LABEL))
out = os.path.join(here, "..", "..", "docs", "ui-preview.png")
sheet.save(out); print("wrote", out, sheet.size)
