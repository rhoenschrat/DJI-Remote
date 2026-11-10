#!/usr/bin/env python3
"""Rasterise an SVG icon into the firmware's 1-bit icon format.

Developer tool, not part of the firmware build. Prints a C array in the
format used by main/icons.c to stdout and an ASCII preview to stderr:

- SIZE x SIZE pixels (default 24, multiple of 8), SIZE*SIZE/8 bytes,
  MSB-first, row-major (24x24 = 72 bytes, 32x32 = 128 bytes)
- inverted polarity: bit 1 = background, bit 0 = icon shape
  (lvgl_icons_init() inverts it into LV_COLOR_FORMAT_A1 at startup)

Usage:
    python3 -I svg_to_icon.py <icon.svg> <array_name> [--threshold 0.5] [--size 24]

Dependencies (install into a virtualenv outside the project tree):
    pip install resvg-py Pillow

resvg-py bundles its renderer and needs no system libraries. If it is not
installed, the script falls back to cairosvg (pip install cairosvg Pillow),
which requires the system cairo library.

Icons that use fill="currentColor" (e.g. Tabler) are rendered with a forced
black fill. The alpha channel is thresholded (default 50%) to decide which
pixels belong to the shape. Do not hand-edit the generated array; adjust the
threshold and run the script again instead.
"""

import argparse
import io
import sys

from PIL import Image

DEFAULT_SIZE = 24


def render_rgba(svg_text: str, size: int) -> Image.Image:
    """Render the SVG at size x size and return an RGBA image."""
    try:
        import resvg_py
        png = resvg_py.svg_to_bytes(svg_string=svg_text, width=size, height=size)
        png = bytes(png)
    except ImportError:
        import cairosvg
        png = cairosvg.svg2png(bytestring=svg_text.encode("utf-8"),
                               output_width=size, output_height=size)
    img = Image.open(io.BytesIO(png)).convert("RGBA")
    if img.size != (size, size):
        img = img.resize((size, size), Image.LANCZOS)
    return img


def to_shape_mask(img: Image.Image, threshold: float) -> list:
    """Return one row per pixel row of booleans, True where the icon shape is."""
    limit = int(round(threshold * 255))
    alpha = img.getchannel("A")
    w, h = img.size
    return [[alpha.getpixel((x, y)) >= limit for x in range(w)]
            for y in range(h)]


def pack_inverted(mask: list) -> list:
    """Pack rows MSB-first; bit 1 = background, bit 0 = shape."""
    out = []
    for row in mask:
        for byte_start in range(0, len(row), 8):
            value = 0
            for bit in range(8):
                if not row[byte_start + bit]:
                    value |= 0x80 >> bit
            out.append(value)
    return out


def format_array(name: str, data: list) -> str:
    lines = []
    for i in range(0, len(data), 8):
        chunk = ",".join(f"0x{b:02x}" for b in data[i:i + 8])
        lines.append("    " + chunk)
    body = ",\n".join(lines)
    return f"const uint8_t {name}[{len(data)}] = {{\n{body}\n}};"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("svg", help="path to the SVG file")
    parser.add_argument("name", help="C array name, e.g. alert_icon")
    parser.add_argument("--threshold", type=float, default=0.5,
                        help="alpha threshold 0..1 (default 0.5)")
    parser.add_argument("--size", type=int, default=DEFAULT_SIZE,
                        help="icon width and height in pixels, multiple of 8 "
                             "(default 24)")
    args = parser.parse_args()
    if args.size <= 0 or args.size % 8:
        parser.error("--size must be a positive multiple of 8")

    with open(args.svg, encoding="utf-8") as f:
        svg_text = f.read()
    svg_text = svg_text.replace("currentColor", "#000000")

    mask = to_shape_mask(render_rgba(svg_text, args.size), args.threshold)

    for row in mask:
        print("".join("#" if px else "." for px in row), file=sys.stderr)

    print(format_array(args.name, pack_inverted(mask)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
