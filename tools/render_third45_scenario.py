#!/usr/bin/env python3
"""Run DiagRunner third45 scenario and render comparison SVG."""
from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DIAG = ROOT / "DiagRunner" / "DiagRunner.csproj"
CSV = Path(
    r"C:\Users\AI\AppData\LocalLow\Hovgaard Games\Big Ambitions"
    r"\ModsLocal\VoogleRoute\Data\big_ambitions_enhanced_routes.csv"
)
OUT = ROOT / "docs" / "third45_route_compare.svg"

STYLES = {
    "release_v0.11.7": ("#2196F3", "v0.11.7 release (side OFF, uturn ON)"),
    "side_off_uturn_off": ("#F44336", "side OFF, uturn OFF — huge detour"),
    "side_on_uturn_on": ("#FF9800", "side ON, uturn ON — north loop"),
    "side_on_uturn_off": ("#9C27B0", "side ON, uturn OFF — worst detour"),
}


def run_diag() -> str:
    cmd = ["dotnet", "run", "--project", str(DIAG), "--", str(CSV), "--scenario", "third45"]
    return subprocess.check_output(cmd, text=True, encoding="utf-8", errors="replace")


def parse_routes(text: str) -> dict[str, dict]:
    routes: dict[str, dict] = {}
    current: str | None = None
    meta: dict | None = None
    points: list[tuple[float, float]] = []

    for line in text.splitlines():
        if line.startswith("ROUTE "):
            parts = line.split()
            rid = parts[1]
            current = rid
            meta = {"header": line}
            points = []
            continue
        if line.strip() == "ENDROUTE" and current and meta is not None:
            routes[current] = {"meta": meta["header"], "points": points}
            current = None
            continue
        if current and line.startswith("  "):
            xs, zs = line.split()
            points.append((float(xs.replace(",", ".")), float(zs.replace(",", "."))))
    return routes


def world_to_svg(x: float, z: float, ox: float, oz: float, scale: float) -> tuple[float, float]:
    return (ox + x * scale, oz - z * scale)


def build_svg(routes: dict[str, dict]) -> str:
    all_pts = [(x, z) for r in routes.values() for x, z in r["points"]]
    all_pts += [(220.98, -235.04), (214.21, -136.95)]
    min_x = min(p[0] for p in all_pts) - 15
    max_x = max(p[0] for p in all_pts) + 15
    min_z = min(p[1] for p in all_pts) - 15
    max_z = max(p[1] for p in all_pts) + 15

    w, h = 1100, 900
    margin = 60
    scale = min((w - 2 * margin) / (max_x - min_x), (h - 2 * margin) / (max_z - min_z))
    ox = margin - min_x * scale
    oz = margin + max_z * scale

    def pt(x: float, z: float) -> str:
        sx, sy = world_to_svg(x, z, ox, oz, scale)
        return f"{sx:.1f},{sy:.1f}"

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">',
        '<rect width="100%" height="100%" fill="#1a1f2e"/>',
        '<text x="20" y="28" fill="#e8eaed" font-family="Segoe UI,sans-serif" font-size="16" font-weight="bold">'
        "45 3rd St — offline A* (from your save log)</text>",
        '<text x="20" y="48" fill="#9aa0a6" font-family="Segoe UI,sans-serif" font-size="12">'
        "Car (220.98, -235.04) heading 180° → dest (214.21, -136.95) | building on WEST (x≈214)</text>",
    ]

    # 3rd St corridor bands (approx lane centers)
    for x_lane, label, color in [
        (214.0, "west curb / building", "#4caf50"),
        (221.0, "west travel lane", "#81c784"),
        (225.4, "east travel lane (wrong)", "#ef5350"),
    ]:
        x1, _ = world_to_svg(x_lane, min_z, ox, oz, scale)
        x2, _ = world_to_svg(x_lane, max_z, ox, oz, scale)
        lines.append(
            f'<line x1="{x1:.1f}" y1="{margin}" x2="{x2:.1f}" y2="{h-margin}" '
            f'stroke="{color}" stroke-width="1" stroke-dasharray="6,4" opacity="0.45"/>'
        )
        lines.append(
            f'<text x="{x1+4:.0f}" y="{h-margin-8}" fill="{color}" '
            f'font-family="monospace" font-size="10">{label}</text>'
        )

    # 5th Ave approx (horizontal at z~-353 in detour routes)
    z5 = -353.0
    p1 = world_to_svg(min_x, z5, ox, oz, scale)
    p2 = world_to_svg(max_x, z5, ox, oz, scale)
    lines.append(
        f'<line x1="{p1[0]:.1f}" y1="{p1[1]:.1f}" x2="{p2[0]:.1f}" y2="{p2[1]:.1f}" '
        f'stroke="#ffd54f" stroke-width="1.5" stroke-dasharray="8,4" opacity="0.6"/>'
    )
    lines.append(
        f'<text x="{p2[0]-80:.0f}" y="{p2[1]-6:.0f}" fill="#ffd54f" font-size="11">5th Ave (detour leg)</text>'
    )

    for rid, data in routes.items():
        color, label = STYLES.get(rid, ("#fff", rid))
        pts = data["points"]
        if len(pts) < 2:
            continue
        d = "M " + " L ".join(pt(x, z) for x, z in pts)
        lines.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="2.5" opacity="0.9"/>')
        # end marker
        ex, ez = pts[-1]
        cx, cy = world_to_svg(ex, ez, ox, oz, scale)
        lines.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="5" fill="{color}"/>')

    # origin + destination
    ox_, oz_ = 220.98, -235.04
    dx_, dz_ = 214.21, -136.95
    cx, cy = world_to_svg(ox_, oz_, ox, oz, scale)
    lines.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="8" fill="#00e676" stroke="#fff" stroke-width="2"/>')
    lines.append(f'<text x="{cx+12:.0f}" y="{cy+4:.0f}" fill="#00e676" font-size="12">CAR start</text>')
    cx, cy = world_to_svg(dx_, dz_, ox, oz, scale)
    lines.append(f'<rect x="{cx-7:.0f}" y="{cy-7:.0f}" width="14" height="14" fill="#ff4081" stroke="#fff"/>')
    lines.append(f'<text x="{cx+12:.0f}" y="{cy+4:.0f}" fill="#ff4081" font-size="12">45 3rd St (dest)</text>')

    # legend
    ly = 70
    for rid, (color, label) in STYLES.items():
        if rid not in routes:
            continue
        header = routes[rid]["meta"]
        m = re.search(r"poly=(\d+).*cost=([\d,]+)m.*endLane=\(([^)]+)\)", header)
        extra = ""
        if m:
            extra = f" — {m.group(1)} pts, {m.group(2)}m, end lane {m.group(3)}"
        lines.append(f'<line x1="780" y1="{ly}" x2="810" y2="{ly}" stroke="{color}" stroke-width="3"/>')
        lines.append(
            f'<text x="820" y="{ly+4}" fill="#e8eaed" font-family="Segoe UI,sans-serif" font-size="11">'
            f"{label}{extra}</text>"
        )
        ly += 22

    lines.append("</svg>")
    return "\n".join(lines)


def main() -> int:
    text = run_diag() if "--cached" not in sys.argv else Path(sys.argv[sys.argv.index("--cached") + 1]).read_text(encoding="utf-8")
    routes = parse_routes(text)
    svg = build_svg(routes)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(svg, encoding="utf-8")
    print(f"Wrote {OUT} ({len(routes)} routes)")
    for rid, data in routes.items():
        print(data["meta"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
