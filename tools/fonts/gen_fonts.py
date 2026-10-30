#!/usr/bin/env python3
"""Generate the bitmap fonts and icons used by src/display.cpp -> src/ui_assets.h

Fonts are rendered from DejaVu Sans Mono Bold (Bitstream Vera licence, free to
embed and modify) into fixed cells, 1 bit per pixel, MSB first, one row after
another (row-major, each row padded to a byte boundary):
  - small : 8x16 cells, ASCII 32..126 + slot 127 = degree sign
  - big   : 24x36 cells, "0123456789.A-" (current display)
  - clock : 14x30 cells, "0123456789-:" (time of day on the READY screen, beside
            the plug icon): DejaVu Sans Condensed Bold 37 condensed to 14 px,
            digits ~28 px high, drawn 3 px apart; ':' is meant to be drawn narrow
            (centre half)
Two texts of the READY screen are rendered whole, as 1 bpp bitmaps: "READY"
(the 14x22 look of DejaVu Sans Mono Bold 23) and the greeting, as large as fits
on the line: DejaVu Sans Condensed Bold 26, condensed to 122 px when wider,
letters ~20 px high (the small font's are ~10). Icons (Type 2 socket + plug, two frames; a sun)
are drawn here with PIL primitives so they can be tuned without pixel editing.
Run from the repo root (Pillow; DejaVu 2.37 fonts):
  python3 tools/fonts/gen_fonts.py path/to/DejaVuSansMono-Bold.ttf \
      [path/to/DejaVuSansCondensed-Bold.ttf]   (default: next to the first one)
and look at tools/fonts/preview.png. The greeting defaults to "Hello!";
--greeting "..." changes it. With --local, only src/greeting_local.h is written:
your own greeting, not tracked by git, which ui_assets.h uses when it exists.
"""
import sys, os, math, argparse
from PIL import Image, ImageDraw, ImageFont

ap = argparse.ArgumentParser(description="Generate src/ui_assets.h (fonts, icons, READY screen texts).")
ap.add_argument("ttf", nargs="?", default="DejaVuSansMono-Bold.ttf")
ap.add_argument("ttf_cond", nargs="?", help="DejaVuSansCondensed-Bold.ttf (default: next to ttf)")
ap.add_argument("--greeting", default="Hello!", help="greeting of the READY screen")
ap.add_argument("--local", action="store_true",
                help="write only src/greeting_local.h (your own greeting, not tracked by git)")
ARGS = ap.parse_args()
TTF = ARGS.ttf
TTF_COND = ARGS.ttf_cond or os.path.join(os.path.dirname(TTF), "DejaVuSansCondensed-Bold.ttf")
OUT = os.path.join(os.path.dirname(__file__), "..", "..", "src", "ui_assets.h")
PREVIEW = os.path.join(os.path.dirname(__file__), "preview.png")

def render_glyph(font, ch, cw, chh, yoff):
    """Render one character centred horizontally in a cw x chh cell."""
    img = Image.new("L", (cw * 3, chh * 3), 0)
    d = ImageDraw.Draw(img)
    # draw at an offset so glyphs that overhang the cell are not clipped first
    d.text((cw, yoff), ch, font=font, fill=255)
    # crop the cell: horizontal centring on the glyph's advance box
    adv = font.getlength(ch)
    x0 = cw + (adv - cw) / 2.0
    cell = img.crop((int(round(x0)), 0, int(round(x0)) + cw, chh))
    return cell.point(lambda p: 255 if p >= 128 else 0)

def pack(cell):
    w, h = cell.size
    rowbytes = (w + 7) // 8
    out = []
    px = cell.load()
    for y in range(h):
        for b in range(rowbytes):
            v = 0
            for bit in range(8):
                x = b * 8 + bit
                if x < w and px[x, y]:
                    v |= 0x80 >> bit
            out.append(v)
    return out

def c_array(name, data, per_line=16):
    lines = [f"static const uint8_t {name}[{len(data)}] = {{"]
    for i in range(0, len(data), per_line):
        lines.append("  " + ", ".join(f"0x{b:02X}" for b in data[i:i + per_line]) + ",")
    lines.append("};")
    return "\n".join(lines)

# ---------------------------------------------------------------- fonts
SMALL_W, SMALL_H = 8, 16
BIG_W, BIG_H = 24, 36
small_font = ImageFont.truetype(TTF, 13)
big_font = ImageFont.truetype(TTF, 37)

small_chars = [chr(c) for c in range(32, 127)] + ["°"]   # slot 127 = degree
big_chars = list("0123456789.A-")

# vertical offset: centre the ascender box (digits/capitals) in the cell
def yoff_for(font, cell_h):
    asc, desc = font.getmetrics()
    bbox = font.getbbox("0")           # (x0, y0, x1, y1) of a digit
    glyph_h = bbox[3] - bbox[1]
    return (cell_h - glyph_h) // 2 - bbox[1]

small_cells = [render_glyph(small_font, c, SMALL_W, SMALL_H, yoff_for(small_font, SMALL_H)) for c in small_chars]
big_cells = [render_glyph(big_font, c, BIG_W, BIG_H, yoff_for(big_font, BIG_H)) for c in big_chars]

# ---------------------------------------------------------------- icons
ICON_W, ICON_H = 40, 56

def icon(frame):
    """frame 0 = plug inserted in the socket, 1 = plug detached below it."""
    im = Image.new("1", (ICON_W, ICON_H), 0)
    d = ImageDraw.Draw(im)
    # socket: rounded body with a thinner inner outline and 7 contacts
    d.rounded_rectangle((1, 0, 38, 30), radius=7, outline=1, width=2)
    d.rounded_rectangle((6, 4, 33, 26), radius=5, outline=1, width=1)
    for (x, y) in [(13, 8), (25, 8), (11, 14), (19, 14), (27, 14), (14, 20), (24, 20)]:
        d.rectangle((x, y, x + 2, y + 2), fill=1)
    if frame == 0:
        # plug body inserted (overlapping the lower half of the socket), cable below
        d.rectangle((13, 17, 26, 35), fill=0)                 # clear behind the plug
        d.rounded_rectangle((13, 17, 26, 35), radius=3, outline=1, width=2)
        d.rectangle((18, 36, 21, 55), fill=1)                 # cable
    else:
        # plug detached: two prongs, body, cable
        d.rectangle((16, 35, 17, 40), fill=1)
        d.rectangle((22, 35, 23, 40), fill=1)
        d.rounded_rectangle((13, 40, 26, 50), radius=3, outline=1, width=2)
        d.rectangle((18, 51, 21, 55), fill=1)
    return im

BOLT_W, BOLT_H = 12, 20
def bolt():
    im = Image.new("1", (BOLT_W, BOLT_H), 0)
    d = ImageDraw.Draw(im)
    d.polygon([(7, 0), (1, 11), (6, 11), (4, 19), (11, 7), (6, 7)], fill=1)
    return im

# Sun, alternating with the plug icon while the supply is off-grid (same 40x56 cell)
def sun():
    im = Image.new("1", (ICON_W, ICON_H), 0)
    d = ImageDraw.Draw(im)
    cx, cy = 19.5, 27.5
    d.ellipse((cx - 9, cy - 9, cx + 9, cy + 9), fill=1)
    for k in range(8):                                        # 8 rays, 45 degrees apart
        a = k * math.pi / 4
        d.line((cx + 13 * math.cos(a), cy + 13 * math.sin(a),
                cx + 19.5 * math.cos(a), cy + 19.5 * math.sin(a)), fill=1, width=4)
    return im

icons = [icon(0), icon(1)]
bolt_im = bolt()
sun_im = sun()

# ---------------------------------------------------------------- READY screen
CLOCK_W, CLOCK_H = 14, 30
clock_chars = list("0123456789-:")
def clock_cells(size=37, thr=128):
    """Glyphs centred on the digit advance, then condensed to CLOCK_W columns."""
    f = ImageFont.truetype(TTF_COND, size)
    a = round(f.getlength("0"))
    x0, top, x1, bottom = f.getbbox("0")
    yoff = (CLOCK_H - (bottom - top)) // 2 - top
    cells = []
    for c in clock_chars:
        im = Image.new("L", (a + 8, CLOCK_H), 0)
        ImageDraw.Draw(im).text((4 + (a - f.getlength(c)) / 2, yoff), c, font=f, fill=255)
        im = im.crop((4, 0, 4 + a, CLOCK_H)).resize((CLOCK_W, CLOCK_H), Image.LANCZOS)
        cells.append(im.point(lambda p: 255 if p >= thr else 0))
    return cells
clock_cells_im = clock_cells()

def word(text, size, cw, ch, cap_row):
    """Monospace cells cw x ch, capitals from cap_row (room for accents above)."""
    f = ImageFont.truetype(TTF, size)
    yoff = cap_row - f.getbbox("0")[1]
    im = Image.new("L", (cw * len(text), ch), 0)
    for i, c in enumerate(text):
        im.paste(render_glyph(f, c, cw, ch, yoff), (i * cw, 0))
    return im
ready_im = word("READY", 23, 14, 22, 4)

GREETING = ARGS.greeting
def greeting(size=26, width=122, thr=128):
    """The whole line, cropped to its ink, condensed horizontally if wider than `width`."""
    f = ImageFont.truetype(TTF_COND, size)
    W, H = int(f.getlength(GREETING)) + 20, size * 2
    im = Image.new("L", (W, H), 0)
    ImageDraw.Draw(im).text((10, 0), GREETING, font=f, fill=255)
    x0, _, x1, _ = im.getbbox()
    w = min(width, x1 - x0)
    im = im.crop((x0, 0, x1, H)).resize((w, H), Image.LANCZOS).point(lambda p: 255 if p >= thr else 0)
    _, y0, _, y1 = im.getbbox()
    return im.crop((0, y0, w, y1))
greet_im = greeting()

if ARGS.local:
    LOCAL = os.path.join(os.path.dirname(__file__), "..", "..", "src", "greeting_local.h")
    with open(LOCAL, "w") as f:
        f.write("// Generated by tools/fonts/gen_fonts.py --local: your own READY screen greeting.\n")
        f.write("// Not tracked by git; ui_assets.h uses it instead of its default greeting.\n")
        f.write("#pragma once\n#include <stdint.h>\n#define GREETING_LOCAL 1\n")
        f.write(f"#define GREET_W {greet_im.width}\n#define GREET_H {greet_im.height}\n\n")
        f.write(c_array("TXT_GREET", pack(greet_im)) + "\n")
    print("wrote", os.path.normpath(LOCAL), f"({greet_im.width}x{greet_im.height})")
    sys.exit(0)

# ---------------------------------------------------------------- output
with open(OUT, "w") as f:
    f.write("// Generated by tools/fonts/gen_fonts.py - do not edit by hand.\n")
    f.write("// Fonts rendered from DejaVu Sans Mono Bold (Bitstream Vera licence).\n")
    f.write("// 1 bpp, MSB first, row-major, rows padded to a byte.\n#pragma once\n#include <stdint.h>\n\n")
    f.write(f"#define FONT_SMALL_W {SMALL_W}\n#define FONT_SMALL_H {SMALL_H}\n")
    f.write(f"#define FONT_SMALL_FIRST 32\n#define FONT_SMALL_COUNT {len(small_chars)}   // 32..126 + 127 = degree sign\n")
    f.write(f"#define FONT_BIG_W {BIG_W}\n#define FONT_BIG_H {BIG_H}\n")
    f.write(f"#define FONT_BIG_CHARS \"{''.join(big_chars)}\"\n")
    f.write(f"#define ICON_W {ICON_W}\n#define ICON_H {ICON_H}\n#define BOLT_W {BOLT_W}\n#define BOLT_H {BOLT_H}\n")
    f.write(f"#define FONT_CLOCK_W {CLOCK_W}\n#define FONT_CLOCK_H {CLOCK_H}\n")
    f.write(f"#define FONT_CLOCK_CHARS \"{''.join(clock_chars)}\"\n")
    f.write(f"#define READY_W {ready_im.width}\n#define READY_H {ready_im.height}\n")
    f.write("\n")
    data = []
    for c in small_cells: data += pack(c)
    f.write(c_array("FONT_SMALL", data) + "\n\n")
    data = []
    for c in big_cells: data += pack(c)
    f.write(c_array("FONT_BIG", data) + "\n\n")
    f.write(c_array("ICON_PLUGGED", pack(icons[0])) + "\n\n")
    f.write(c_array("ICON_UNPLUGGED", pack(icons[1])) + "\n\n")
    f.write(c_array("ICON_BOLT", pack(bolt_im)) + "\n\n")
    f.write(c_array("ICON_SUN", pack(sun_im)) + "\n\n")
    data = []
    for c in clock_cells_im: data += pack(c)
    f.write(c_array("FONT_CLOCK", data) + "\n\n")
    f.write(c_array("TXT_READY", pack(ready_im)) + "\n\n")
    f.write("// The greeting: src/greeting_local.h when it exists (your own, not tracked by git),\n"
            "// else the default below. Last, so the other arrays keep their place in flash.\n"
            "#if __has_include(\"greeting_local.h\")\n#include \"greeting_local.h\"\n#else\n")
    f.write(f"#define GREET_W {greet_im.width}\n#define GREET_H {greet_im.height}\n")
    f.write(c_array("TXT_GREET", pack(greet_im)) + "\n#endif\n")

# preview sheet (x4)
cols = 16
rows = (len(small_cells) + cols - 1) // cols
sheet = Image.new("L", (cols * SMALL_W, rows * SMALL_H + BIG_H + 4 + ICON_H + 4
                         + CLOCK_H + 4 + ready_im.height + 4 + greet_im.height), 0)
for i, c in enumerate(small_cells):
    sheet.paste(c, ((i % cols) * SMALL_W, (i // cols) * SMALL_H))
y = rows * SMALL_H + 4
for i, c in enumerate(big_cells[:5]):
    sheet.paste(c, (i * BIG_W, y))
y += BIG_H + 4
sheet.paste(icons[0].convert("L").point(lambda p: 255 if p else 0), (0, y))
sheet.paste(icons[1].convert("L").point(lambda p: 255 if p else 0), (ICON_W + 8, y))
sheet.paste(bolt_im.convert("L").point(lambda p: 255 if p else 0), (2 * ICON_W + 16, y))
sheet.paste(sun_im.convert("L").point(lambda p: 255 if p else 0), (2 * ICON_W + 16 + BOLT_W + 8, y))
y += ICON_H + 4
x = 0
for k in (1, 4, 11, 3, 2):                                # "14:32", narrow ':'
    c = clock_cells_im[k]
    if clock_chars[k] == ":": c = c.crop((CLOCK_W // 4, 0, CLOCK_W // 4 + CLOCK_W // 2, CLOCK_H))
    sheet.paste(c, (x, y)); x += c.width
y += CLOCK_H + 4
sheet.paste(ready_im, (0, y))
y += ready_im.height + 4
sheet.paste(greet_im.convert("L"), (0, y))
sheet.resize((sheet.width * 4, sheet.height * 4), Image.NEAREST).save(PREVIEW)
print(f"wrote {OUT} and {PREVIEW}: small {len(small_cells)} glyphs, big {len(big_cells)}, clock {len(clock_cells_im)}, "
      f"icons 2 + bolt + sun, READY {ready_im.size}, greeting {greet_im.size}")
