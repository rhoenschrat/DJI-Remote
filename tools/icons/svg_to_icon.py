#!/usr/bin/env python3
"""Rasterise an SVG icon into the firmware's 24x24 1-bit icon format.

Developer tool, not part of the firmware build. Prints a C array in the
format used by main/icons.c to stdout and an ASCII preview to stderr:

- 24x24 pixels, 72 bytes, MSB-first, row-major
- inverted polarity: bit 1 = background, bit 0 = icon shape
  (lvgl_icons_init() inverts it into LV_COLOR_FORMAT_A1 at startup)

Usage:
    python3 -I svg_to_icon.py <icon.svg> <array_name> [--threshold 0.5]

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

SIZE = 24


def render_rgba(svg_text: str) -> Image.Image:
    """Render the SVG at SIZE x SIZE and return an RGBA image."""
    try:
        import resvg_py
        png = resvg_py.svg_to_bytes(svg_string=svg_text, width=SIZE, height=SIZE)
        png = bytes(png)
    except ImportError:
        import cairosvg
        png = cairosvg.svg2png(bytestring=svg_text.encode("utf-8"),
                               output_width=SIZE, output_height=SIZE)
    img = Image.open(io.BytesIO(png)).convert("RGBA")
    if img.size != (SIZE, SIZE):
        img = img.resize((SIZE, SIZE), Image.LANCZOS)
    return img


def to_shape_mask(img: Image.Image, threshold: float) -> list:
    """Return SIZE rows of booleans, True where the icon shape is."""
    limit = int(round(threshold * 255))
    alpha = img.getchannel("A")
    return [[alpha.getpixel((x, y)) >= limit for x in range(SIZE)]
            for y in range(SIZE)]


def pack_inverted(mask: list) -> list:
    """Pack rows MSB-first; bit 1 = background, bit 0 = shape."""
    out = []
    for row in mask:
        for byte_start in range(0, SIZE, 8):
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
    args = parser.parse_args()

    with open(args.svg, encoding="utf-8") as f:
        svg_text = f.read()
    svg_text = svg_text.replace("currentColor", "#000000")

    mask = to_shape_mask(render_rgba(svg_text), args.threshold)

    for row in mask:
        print("".join("#" if px else "." for px in row), file=sys.stderr)

    print(format_array(args.name, pack_inverted(mask)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
