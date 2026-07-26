#!/usr/bin/env python3
"""Per-cell summary table for the en-route ablation.

Aggregates eval/enroute_ablation_results.csv (from enroute_ablation.py) into
(scenario, variant) cells keyed on the HONEST metric semantic_recovery_success
-- never a raw navigation REACHED. Also flags success rates that diverge across
reps (a tiebreak rep is needed).

Usage:
  python3 eval/enroute_ablation.py            # (re)build the CSV first
  python3 eval/enroute_summary.py [csv]       # default enroute_ablation_results.csv
"""
import csv
import os
import re
import sys
from collections import defaultdict


def _b(value):
    """CSV cell -> True/False/None (None = not applicable / blank)."""
    s = str(value).strip()
    if s == "" or s.lower() == "none":
        return None
    return s.lower() == "true"


def _f(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


class AblationSummary:
    """Per-cell aggregation of ablation trial rows."""

    COLUMNS = ("scenario", "variant", "N", "sem_success", "nav_success",
               "dir_correct", "fallback", "tier", "outcome", "mean_s")

    def __init__(self, rows):
        self.rows = list(rows)

    @classmethod
    def from_csv(cls, path):
        with open(path, newline="") as handle:
            return cls(list(csv.DictReader(handle)))

    def _cells(self):
        groups = defaultdict(list)
        for row in self.rows:
            groups[(row.get("scenario", ""), row.get("variant", ""))].append(row)
        return groups

    @staticmethod
    def _rate(rows, key):
        """x/y over the reps where the metric is applicable (non-blank)."""
        vals = [_b(r.get(key)) for r in rows]
        vals = [v for v in vals if v is not None]
        if not vals:
            return "n/a"
        return f"{sum(vals)}/{len(vals)}"

    @staticmethod
    def _mode(rows, key):
        vals = [str(r.get(key, "")).strip() for r in rows
                if str(r.get(key, "")).strip()]
        return max(set(vals), key=vals.count) if vals else ""

    @staticmethod
    def _mean_s(rows):
        xs = [x for x in (_f(r.get("time_to_resolution_s")) for r in rows)
              if x is not None]
        return f"{sum(xs) / len(xs):.1f}" if xs else ""

    def summary(self):
        cells = []
        for (scenario, variant), rows in sorted(self._cells().items()):
            cells.append({
                "scenario": scenario,
                "variant": variant,
                "N": len(rows),
                "sem_success": self._rate(rows, "semantic_recovery_success"),
                "nav_success": self._rate(rows, "navigation_success"),
                "dir_correct": self._rate(rows, "directive_correct"),
                "fallback": sum(
                    1 for r in rows
                    if _b(r.get("outer_fallback_after_semantic_failure"))),
                "tier": self._mode(rows, "resolving_tier"),
                "outcome": self._mode(rows, "terminal_outcome"),
                "mean_s": self._mean_s(rows),
            })
        return cells

    def warnings(self):
        notes = []
        for cell in self.summary():
            tag = f"{cell['scenario']}/{cell['variant']}"
            rate = re.fullmatch(r"(\d+)/(\d+)", cell["sem_success"])
            if rate and rate.group(1) not in (rate.group(2), "0"):
                notes.append(
                    f"{tag}: sem_success {cell['sem_success']} diverges "
                    "-> run a tiebreak rep")
        return notes

    def render(self):
        cells = self.summary()
        widths = {c: len(c) for c in self.COLUMNS}
        for cell in cells:
            for c in self.COLUMNS:
                widths[c] = max(widths[c], len(str(cell[c])))

        def line(values):
            return "  ".join(
                str(v).ljust(widths[c]) for c, v in zip(self.COLUMNS, values))

        out = [line(self.COLUMNS),
               "  ".join("-" * widths[c] for c in self.COLUMNS)]
        out += [line([cell[c] for c in self.COLUMNS]) for cell in cells]

        notes = self.warnings()
        if notes:
            out += ["", "VALIDITY / DIVERGENCE:"] + [f"  ! {n}" for n in notes]
        return "\n".join(out)


def main():
    default = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "enroute_ablation_results.csv")
    path = sys.argv[1] if len(sys.argv) > 1 else default
    print(AblationSummary.from_csv(path).render())


if __name__ == "__main__":
    main()
