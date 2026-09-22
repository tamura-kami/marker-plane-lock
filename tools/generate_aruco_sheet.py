#!/usr/bin/env python3
"""Generate a printable ArUco DICT_4X4_50 marker sheet without extra packages."""

from pathlib import Path


MARKERS = {
    0: ["111111", "101001", "110101", "111001", "111011", "111111"],
    1: ["111111", "111111", "100001", "101101", "101011", "111111"],
    2: ["111111", "111001", "111001", "111011", "100101", "111111"],
    3: ["111111", "101101", "101101", "110111", "110011", "111111"],
}

MM = 72.0 / 25.4
PAGE_W, PAGE_H = 210 * MM, 297 * MM
MARKER_SIZE = 30 * MM
QUIET = 5 * MM
CUT_SIZE = MARKER_SIZE + 2 * QUIET
POSITIONS = [(52 * MM, 205 * MM), (158 * MM, 205 * MM),
             (52 * MM, 100 * MM), (158 * MM, 100 * MM)]


def escape_pdf_text(value: str) -> str:
    return value.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")


def text(commands, x, y, size, value, centered=False):
    if centered:
        x -= len(value) * size * 0.25
    commands.append(
        f"BT /F1 {size:.2f} Tf {x:.2f} {y:.2f} Td ({escape_pdf_text(value)}) Tj ET"
    )


def draw_marker(commands, marker_id, center_x, center_y):
    left = center_x - MARKER_SIZE / 2
    bottom = center_y - MARKER_SIZE / 2
    cell = MARKER_SIZE / 6
    commands.append("0 g")
    for row, bits in enumerate(MARKERS[marker_id]):
        for col, bit in enumerate(bits):
            if bit == "1":
                x = left + col * cell
                y = bottom + (5 - row) * cell
                commands.append(f"{x:.3f} {y:.3f} {cell:.3f} {cell:.3f} re f")

    cut_left = center_x - CUT_SIZE / 2
    cut_bottom = center_y - CUT_SIZE / 2
    commands.extend([
        "0.72 G 0.5 w [3 3] 0 d",
        f"{cut_left:.3f} {cut_bottom:.3f} {CUT_SIZE:.3f} {CUT_SIZE:.3f} re S",
        "[] 0 d 0 G",
    ])
    text(commands, center_x, cut_bottom - 15, 11,
         f"DICT_4X4_50 - ID {marker_id}", centered=True)


def make_pdf(path: Path):
    commands = ["1 J 1 j"]
    text(commands, PAGE_W / 2, PAGE_H - 45, 18,
         "ArUco Marker Sheet - DICT_4X4_50", centered=True)
    text(commands, PAGE_W / 2, PAGE_H - 66, 10,
         "Print at 100% / Actual size. Do not use Fit to page.", centered=True)
    text(commands, PAGE_W / 2, PAGE_H - 81, 9,
         "Each black marker is 30 mm. Keep the white margin when cutting.", centered=True)

    for marker_id, (x, y) in enumerate(POSITIONS):
        draw_marker(commands, marker_id, x, y)

    bar_x, bar_y, bar_w = 68 * MM, 31 * MM, 50 * MM
    commands.extend([
        "0 G 1 w",
        f"{bar_x:.3f} {bar_y:.3f} m {bar_x + bar_w:.3f} {bar_y:.3f} l S",
        f"{bar_x:.3f} {bar_y - 5:.3f} m {bar_x:.3f} {bar_y + 5:.3f} l S",
        f"{bar_x + bar_w:.3f} {bar_y - 5:.3f} m {bar_x + bar_w:.3f} {bar_y + 5:.3f} l S",
    ])
    text(commands, bar_x + bar_w / 2, bar_y + 9, 10,
         "50 mm print check", centered=True)

    stream = ("\n".join(commands) + "\n").encode("ascii")
    objects = [
        b"<< /Type /Catalog /Pages 2 0 R >>",
        b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
        (f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {PAGE_W:.3f} {PAGE_H:.3f}] "
         "/Resources << /Font << /F1 5 0 R >> >> /Contents 4 0 R >>").encode("ascii"),
        f"<< /Length {len(stream)} >>\nstream\n".encode("ascii") + stream + b"endstream",
        b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>",
    ]
    output = bytearray(b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n")
    offsets = [0]
    for number, obj in enumerate(objects, start=1):
        offsets.append(len(output))
        output.extend(f"{number} 0 obj\n".encode("ascii"))
        output.extend(obj)
        output.extend(b"\nendobj\n")
    xref = len(output)
    output.extend(f"xref\n0 {len(objects) + 1}\n".encode("ascii"))
    output.extend(b"0000000000 65535 f \n")
    for offset in offsets[1:]:
        output.extend(f"{offset:010d} 00000 n \n".encode("ascii"))
    output.extend(
        f"trailer\n<< /Size {len(objects) + 1} /Root 1 0 R >>\n"
        f"startxref\n{xref}\n%%EOF\n".encode("ascii")
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(output)


if __name__ == "__main__":
    make_pdf(Path("docs/aruco_dict4x4_50_ids_0-3_a4.pdf"))
