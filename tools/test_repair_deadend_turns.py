import copy
import csv
import json
import math
import tempfile
import unittest
from pathlib import Path

import repair_deadend_turns as repair


class RepairTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = Path(__file__).resolve().parents[1] / "data/big_ambitions_enhanced_routes.csv"
        cls.fields, cls.rows = repair.load(cls.path)

    def test_idempotent_on_shipped_graph(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "graph.csv"
            target.write_bytes(self.path.read_bytes())
            self.assertEqual([], repair.repair(target))
            self.assertEqual(self.path.read_bytes(), target.read_bytes())

    def test_all_turn_curves_stay_behind_terminal_plane(self):
        turns = [r for r in self.rows if r["source"] == repair.SOURCE]
        self.assertEqual(len(repair.REPAIRS), len(turns))
        for terminal, entry, start, end, road, x, z in repair.REPAIRS:
            row = next(r for r in turns if int(r["fromIndex"]) == start)
            self.assertEqual(end, int(row["toIndex"]))
            self.assertEqual(road, row["fromRoad"])
            self.assertEqual(road, row["toRoad"])
            self.assertEqual("uturn", row["maneuver"])
            a, b = repair.xy(repair.point(row, "from")), repair.xy(repair.point(row, "to"))
            c = float(row["controlX"]), float(row["controlZ"])
            normal = repair.graph.v_norm((x - a[0], z - a[1]))
            for step in range(101):
                t = step / 100
                p = tuple((1-t)**2*u + 2*(1-t)*t*v + t*t*w for u, v, w in zip(a, c, b))
                self.assertLessEqual(repair.graph.dot((p[0]-x, p[1]-z), normal), -1.999)
            self.assertLessEqual(abs(float(row["fromY"]) - float(row["toY"])), 1.5)

    def test_partial_or_duplicate_repair_fails(self):
        rows = copy.deepcopy(self.rows)
        turn = next(r for r in rows if r["source"] == repair.SOURCE)
        rows.append(dict(turn))
        with self.assertRaisesRegex(ValueError, "duplicate"):
            repair.plan(rows)

    def baseline_rows(self):
        evidence = self.path.parent.parent / "docs/navigation-deadends/audit.json"
        changes = json.loads(evidence.read_text(encoding="utf-8"))["repairs"]
        return [r for r in self.rows if r["source"] != repair.SOURCE] + [
            r for c in changes for r in c["removed"]]

    def write_rows(self, path, rows):
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=self.fields)
            writer.writeheader()
            writer.writerows(rows)

    def test_reconstruct_baseline_and_repair_matches_shipped_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "graph.csv"
            self.write_rows(path, self.baseline_rows())
            self.assertEqual(6, len(repair.repair(path)))
            self.assertEqual(self.path.read_bytes(), path.read_bytes())

    def test_coordinate_drift_refuses_every_write(self):
        rows = self.baseline_rows()
        for row in rows:
            if int(row["toIndex"]) == 7292:
                row["toX"] = "-3270"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "graph.csv"
            self.write_rows(path, rows)
            original = path.read_bytes()
            with self.assertRaisesRegex(ValueError, "identities changed"):
                repair.repair(path)
            self.assertEqual(original, path.read_bytes())

    def test_new_terminal_exit_requires_reaudit(self):
        rows = self.baseline_rows()
        row = dict(next(r for r in rows if int(r["toIndex"]) == 7292))
        row["fromIndex"] = "7292"
        rows.append(row)
        with self.assertRaisesRegex(ValueError, "already has an exit"):
            repair.plan(rows)


if __name__ == "__main__":
    unittest.main()
