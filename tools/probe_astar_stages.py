#!/usr/bin/env python3
"""Run staged A* probes via DiagRunner (real WaypointPathfinder)."""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CSV = ROOT / "data" / "big_ambitions_enhanced_routes.csv"
DIAG = ROOT / "DiagRunner" / "DiagRunner.csproj"

STAGES = ("bridge", "industrial", "north", "all")


def main() -> int:
    csv = Path(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else CSV
    stage = "all"
    for arg in sys.argv[1:]:
        if arg.startswith("--stage="):
            stage = arg.split("=", 1)[1]
        elif arg in STAGES:
            stage = arg

    if not csv.is_file():
        print(f"CSV not found: {csv}", file=sys.stderr)
        return 1

    cmd = ["dotnet", "run", "--project", str(DIAG), "--", str(csv), "--stage", stage]
    print(f"# probe_astar_stages.py stage={stage}")
    print(f"# csv={csv}")
    print()
    return subprocess.call(cmd)


if __name__ == "__main__":
    raise SystemExit(main())
