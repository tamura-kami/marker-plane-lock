#!/usr/bin/env python3
"""Generate an A4 sheet of DICT_4X4_50 marker ID 0 at several sizes."""

from pathlib import Path


MARKER_ID_0 = ["111111", "101001", "110101", "111001", "111011", "111111"]
SIZES_MM = [10, 15, 20, 30, 40, 50]
MM = 72.0 / 25.4
PAGE_W, PAGE_H = 210 * MM, 297 * MM
QUIET_MM = 5
POSITIONS_MM = [(55, 235), (155, 235), (55, 160), (155, 160),
                (55, 85), (155, 85)]


def escape_pdf_text(value: str) -> str:
    return value.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")


def text(commands, x, y, size, value, centered=False):
    if centered:
        x -= len(value) * size * 0.25
    commands.append(
        f"BT /F1 {size:.2f} Tf {x:.2f} {y:.2f} Td ({escape_pdf_text(value)}) Tj ET"
    )


def draw_marker(commands, size_mm, center_x_mm, center_y_mm):
    size = size_mm * MM
    center_x = center_x_mm * MM
    center_y = center_y_mm * MM
    left = center_x - size / 2
    bottom = center_y - size / 2
    cell = size / 6
    commands.append("0 g")
    for row, bits in enumerate(MARKER_ID_0):
        for col, bit in enumerate(bits):
            if bit == "1":
                x = left + col * cell
                y = bottom + (5 - row) * cell
                commands.append(f"{x:.3f} {y:.3f} {cell:.3f} {cell:.3f} re f")

    cut_size = size + 2 * QUIET_MM * MM
    cut_left = center_x - cut_size / 2
    cut_bottom = center_y - cut_size / 2
    commands.extend([
        "0.72 G 0.5 w [3 3] 0 d",
        f"{cut_left:.3f} {cut_bottom:.3f} {cut_size:.3f} {cut_size:.3f} re S",
        "[] 0 d 0 G",
    ])
    text(commands, center_x, cut_bottom - 12, 8,
         f"ID 0 - {size_mm} mm marker", centered=True)


def make_pdf(path: Path):
    commands = ["1 J 1 j"]
    text(commands, PAGE_W / 2, PAGE_H - 32, 17,
         "ArUco Marker ID 0 - DICT_4X4_50", centered=True)
    text(commands, PAGE_W / 2, PAGE_H - 48, 9,
         "Print at 100% / Actual size. Do not use Fit to page.", centered=True)
    text(commands, PAGE_W / 2, PAGE_H - 62, 8,
         "Black square sizes are labeled. Keep the 5 mm white margin when cutting.", centered=True)

    for size_mm, (x, y) in zip(SIZES_MM, POSITIONS_MM):
        draw_marker(commands, size_mm, x, y)

    bar_x, bar_y, bar_w = 80 * MM, 22 * MM, 50 * MM
    commands.extend([
        "0 G 1 w",
        f"{bar_x:.3f} {bar_y:.3f} m {bar_x + bar_w:.3f} {bar_y:.3f} l S",
        f"{bar_x:.3f} {bar_y - 5:.3f} m {bar_x:.3f} {bar_y + 5:.3f} l S",
        f"{bar_x + bar_w:.3f} {bar_y - 5:.3f} m {bar_x + bar_w:.3f} {bar_y + 5:.3f} l S",
    ])
    text(commands, bar_x + bar_w / 2, bar_y + 8, 8,
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
    make_pdf(Path("docs/aruco_dict4x4_50_id0_a4.pdf"))
