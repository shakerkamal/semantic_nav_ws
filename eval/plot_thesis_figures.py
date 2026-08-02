#!/usr/bin/env python3
"""Generate the Evaluation-chapter figures from the frozen result CSVs.

Writes vector PDFs into ~/Thesis/report/figures/. Every number plotted here is
read from a CSV in this directory; nothing is hard-coded except axis labels.

Run with PYTHONNOUSERSITE=1 (a pip-installed numpy 2 in the user site shadows
the apt matplotlib build otherwise).
"""

import csv
import os
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

EVAL = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.expanduser("~/Thesis/report/figures")

# Palette shared with the draw.io architecture figures (figures/README.md).
GEO_FILL, GEO_EDGE = "#dae8fc", "#6c8ebf"      # deterministic geometry
LLM_FILL, LLM_EDGE = "#ffe6cc", "#d79b00"      # LLM semantic reasoning
OK_FILL, OK_EDGE = "#d5e8d4", "#82b366"        # confirmed / clean
BAD_FILL, BAD_EDGE = "#f8cecc", "#b85450"      # blockage / failure

plt.rcParams.update({
    "font.size": 9,
    "axes.titlesize": 9.5,
    "axes.labelsize": 9,
    "legend.fontsize": 8.5,
    "xtick.labelsize": 8.5,
    "ytick.labelsize": 8.5,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "figure.dpi": 150,
})

SCEN = ["S1", "S2", "S3", "S4"]
SCEN_LABEL = {"S1": "S1\ncontrol", "S2": "S2\ndoor", "S3": "S3\nball", "S4": "S4\nperson"}


def load_enroute():
    rows = list(csv.DictReader(open(os.path.join(EVAL, "enroute_ablation_results.csv"))))
    g = defaultdict(list)
    for r in rows:
        g[(r["scenario"], r["variant"])].append(r)
    return g


def istrue(v):
    return str(v).strip().lower() in ("true", "1", "yes")


def fig_enroute_outcomes(g):
    fig, axes = plt.subplots(1, 2, figsize=(6.6, 2.7), sharey=True)
    width = 0.36
    xs = range(len(SCEN))

    for ax, field, title in (
        (axes[0], "navigation_success", "Original target reached"),
        (axes[1], "semantic_recovery_success", "Clean semantic recovery"),
    ):
        for i, (variant, fill, edge, label) in enumerate(
            [("bgeo", GEO_FILL, GEO_EDGE, "B-Geo"), ("bllm", LLM_FILL, LLM_EDGE, "B-Llm")]
        ):
            vals, hatches = [], []
            for s in SCEN:
                rs = g[(s, variant)]
                if s == "S1" and field == "semantic_recovery_success":
                    vals.append(0)          # not applicable: recovery must not fire
                    hatches.append("//")
                else:
                    vals.append(sum(1 for r in rs if istrue(r[field])))
                    hatches.append("")
            off = (i - 0.5) * width
            bars = ax.bar([x + off for x in xs], vals, width,
                          facecolor=fill, edgecolor=edge, linewidth=1.0,
                          label=label if ax is axes[0] else None)
            for b, v, h in zip(bars, vals, hatches):
                cx = b.get_x() + b.get_width() / 2
                if h:
                    ax.text(cx, 0.12, "n/a", ha="center", va="bottom",
                            fontsize=7.5, color="#777", style="italic")
                else:
                    ax.text(cx, v + 0.12, str(v), ha="center", va="bottom",
                            fontsize=8, color="#333" if v else edge)
        ax.set_title(title)
        ax.set_xticks(list(xs))
        ax.set_xticklabels([SCEN_LABEL[s] for s in SCEN])
        ax.set_ylim(0, 6.2)
        ax.set_yticks(range(6))
        ax.grid(axis="y", lw=0.4, alpha=0.4)
        ax.set_axisbelow(True)

    axes[0].set_ylabel("repetitions (of 5)")
    handles, labels_ = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels_, loc="lower center", ncol=2, frameon=False,
               bbox_to_anchor=(0.5, -0.06))
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "eval_enroute_outcomes.pdf"), bbox_inches="tight")
    plt.close(fig)


def fig_enroute_timing(g):
    fig, ax = plt.subplots(figsize=(6.6, 2.9))
    labels, pos = [], []
    x = 0
    for s in SCEN:
        for variant, fill, edge, nm in (
            ("bgeo", GEO_FILL, GEO_EDGE, "B-Geo"),
            ("bllm", LLM_FILL, LLM_EDGE, "B-Llm"),
        ):
            ts = sorted(float(r["time_to_resolution_s"]) for r in g[(s, variant)])
            med = ts[len(ts) // 2]
            ax.scatter([x] * len(ts), ts, s=26, facecolor=fill, edgecolor=edge,
                       linewidth=0.9, zorder=3)
            ax.plot([x - 0.26, x + 0.26], [med, med], color=edge, lw=2.0, zorder=4)
            labels.append(f"{s}\n{nm}")
            pos.append(x)
            x += 1
        x += 0.5
    ax.set_xticks(pos)
    ax.set_xticklabels(labels)
    ax.set_ylabel("time to resolution (s)")
    ax.grid(axis="y", lw=0.4, alpha=0.4)
    ax.set_axisbelow(True)
    ax.set_ylim(0, 122)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "eval_enroute_timing.pdf"), bbox_inches="tight")
    plt.close(fig)


def fig_clearance(g):
    """Per-repetition clearance-gate outcome, grouped by evidence mode.

    Plotted against the number of cached map cells the cleanup service actually
    modified, which is the only direct measure of how much of the stale residual
    the platform managed to re-observe on that repetition.
    """
    cells = [("S2", "S2 door"), ("S3", "S3 ball"), ("S4", "S4 person")]
    fig, ax = plt.subplots(figsize=(6.6, 3.0))

    # Mode bands behind the data.
    ax.axvspan(-0.6, 1.6, facecolor="#f3f3f3", zorder=0)
    ax.axvspan(1.6, 2.6, facecolor="#eef5ee", zorder=0)
    ax.text(0.5, 76.5, "Mode A: map-confirmed change", ha="center", fontsize=8.5, color="#555")
    ax.text(2.0, 76.5, "Mode B: tracked\ndeparture", ha="center", fontsize=8.5, color="#4a7a4a")

    for xi, (scen, _) in enumerate(cells):
        rs = sorted(g[(scen, "bllm")], key=lambda r: int(r["rep"]))
        n = len(rs)
        for j, r in enumerate(rs):
            x = xi + (j - (n - 1) / 2) * 0.13
            y = float(r["cleanup_modified_count"] or 0)
            if istrue(r["barrier_clear_succeeded"]):
                ax.scatter([x], [y], s=52, marker="o", facecolor=OK_FILL,
                           edgecolor=OK_EDGE, linewidth=1.3, zorder=3)
            else:
                ax.scatter([x], [y], s=64, marker="X", facecolor=BAD_FILL,
                           edgecolor=BAD_EDGE, linewidth=1.3, zorder=3)

    ax.scatter([], [], s=52, marker="o", facecolor=OK_FILL, edgecolor=OK_EDGE,
               linewidth=1.3, label="clearance confirmed")
    ax.scatter([], [], s=64, marker="X", facecolor=BAD_FILL, edgecolor=BAD_EDGE,
               linewidth=1.3, label="gate failed, fallback completed the drive")

    ax.set_xlim(-0.6, 2.6)
    ax.set_xticks(range(len(cells)))
    ax.set_xticklabels([lbl for _, lbl in cells])
    ax.set_ylim(0, 82)
    ax.set_ylabel("cached map cells modified by cleanup")
    ax.grid(axis="y", lw=0.4, alpha=0.4)
    ax.set_axisbelow(True)
    handles, labels_ = ax.get_legend_handles_labels()
    fig.legend(handles, labels_, loc="lower center", ncol=2, frameon=False,
               bbox_to_anchor=(0.5, -0.08))
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "eval_clearance_modes.pdf"), bbox_inches="tight")
    plt.close(fig)


ARMS = [("U0", "U0\ngeometric"), ("U1", "U1\ntable"), ("U2", "U2\nopen-set")]


def load_upfront():
    rows = list(csv.DictReader(open(os.path.join(EVAL, "upfront_ablation_results.csv"))))
    g = defaultdict(list)
    for r in rows:
        g[(r["scenario"], r["arm"])].append(r)
    return g


def fig_upfront_outcomes(g):
    """Companion to fig_enroute_outcomes for the up-front lane (Ch7 s7.5).

    Outcome composition per arm: every repetition ends either reached or
    needs-operator, so the stacks always sum to N and a failure cannot be
    hidden by the axis. U-B/U2's single escalation is the clearance-gate veto
    at a drifted centroid discussed in the text.
    """
    panels = [("U-A", "U-A reachable control", 3), ("U-B", "U-B open-set barrier", 5)]
    fig, axes = plt.subplots(1, 2, figsize=(6.6, 2.7))
    width = 0.5
    for ax, (scen, title, n) in zip(axes, panels):
        for x, (arm, _) in enumerate(ARMS):
            rs = g[(scen, arm)]
            reached = sum(1 for r in rs if istrue(r["navigation_success"]))
            escalated = sum(1 for r in rs if r["terminal_outcome"] == "needs-operator")
            if reached:  # zero-height segments still draw an edge line
                ax.bar([x], [reached], width, facecolor=OK_FILL,
                       edgecolor=OK_EDGE, linewidth=1.0)
            if escalated:
                ax.bar([x], [escalated], width, bottom=[reached],
                       facecolor=BAD_FILL, edgecolor=BAD_EDGE, linewidth=1.0)
            if reached:
                ax.text(x, reached / 2, str(reached), ha="center", va="center",
                        fontsize=8, color="#333")
            if escalated:
                ax.text(x, reached + escalated / 2, str(escalated), ha="center",
                        va="center", fontsize=8, color="#333")
        ax.set_title(f"{title} (n={n})")
        ax.set_xticks(range(len(ARMS)))
        ax.set_xticklabels([lbl for _, lbl in ARMS])
        ax.set_ylim(0, n + 0.6)
        ax.set_yticks(range(n + 1))
        ax.grid(axis="y", lw=0.4, alpha=0.4)
        ax.set_axisbelow(True)
    axes[0].set_ylabel("repetitions")
    handles = [
        plt.Rectangle((0, 0), 1, 1, facecolor=OK_FILL, edgecolor=OK_EDGE),
        plt.Rectangle((0, 0), 1, 1, facecolor=BAD_FILL, edgecolor=BAD_EDGE),
    ]
    fig.legend(handles, ["original target reached", "needs operator"],
               loc="lower center", ncol=2, frameon=False,
               bbox_to_anchor=(0.5, -0.06))
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "eval_upfront_outcomes.pdf"), bbox_inches="tight")
    plt.close(fig)


MODELS = [("llama3_8b", "Llama3-8B"), ("qwen3_14b", "Qwen3-14B"), ("qwen3_32b", "Qwen3-32B")]


def fig_affordance():
    axes_names = [
        ("openable", "gt_openable", "inf_openable"),
        ("safety", "gt_safety", "inf_safety"),
        ("clearable", "gt_clearable", "inf_clearable"),
    ]
    data = {}
    for key, _ in MODELS:
        rows = list(csv.DictReader(open(os.path.join(EVAL, f"affordance_holdout_results_{key}.csv"))))
        n = len(rows)
        vals = []
        for _, gt, inf in axes_names:
            vals.append(100.0 * sum(1 for r in rows if r[gt].strip().lower() == r[inf].strip().lower()) / n)
        vals.append(100.0 * sum(1 for r in rows if istrue(r["directive_correct"])) / n)
        data[key] = vals

    names = [a[0] for a in axes_names] + ["directive"]
    fig, ax = plt.subplots(figsize=(6.6, 2.7))
    width = 0.26
    xs = range(len(names))
    styles = [(GEO_FILL, GEO_EDGE), (LLM_FILL, LLM_EDGE), (OK_FILL, OK_EDGE)]
    for i, ((key, label), (fill, edge)) in enumerate(zip(MODELS, styles)):
        off = (i - 1) * width
        bars = ax.bar([x + off for x in xs], data[key], width,
                      facecolor=fill, edgecolor=edge, linewidth=1.0, label=label)
        for b, v in zip(bars, data[key]):
            ax.text(b.get_x() + b.get_width() / 2, v + 1.2, f"{v:.1f}",
                    ha="center", va="bottom", fontsize=7.5)
    ax.axhline(100, color="#999", lw=0.7, ls=":")
    ax.set_xticks(list(xs))
    ax.set_xticklabels(["openable", "safety class", "clearable", "directive\n(per sample)"])
    ax.set_ylabel("accuracy (%)")
    ax.set_ylim(0, 112)
    ax.set_yticks([0, 25, 50, 75, 100])
    ax.grid(axis="y", lw=0.4, alpha=0.4)
    ax.set_axisbelow(True)
    handles, labels_ = ax.get_legend_handles_labels()
    fig.legend(handles, labels_, loc="lower center", ncol=3, frameon=False,
               bbox_to_anchor=(0.5, -0.07))
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "eval_affordance_holdout.pdf"), bbox_inches="tight")
    plt.close(fig)


def fig_ranker():
    want = [("bm25", "-", "lexical"), ("hybrid", "1.0", "hybrid $d\\geq1.0$"), ("llm", "-", "model")]
    markers = {"lexical": "s", "hybrid $d\\geq1.0$": "o", "model": "^"}
    styles = dict(zip([m[1] for m in MODELS], [(GEO_FILL, GEO_EDGE), (LLM_FILL, LLM_EDGE), (OK_FILL, OK_EDGE)]))

    fig, ax = plt.subplots(figsize=(6.6, 3.0))
    for key, label in MODELS:
        rows = list(csv.DictReader(open(os.path.join(EVAL, f"ranker_results_{key}.csv"))))
        g = defaultdict(list)
        for r in rows:
            g[(r["variant"], r["delta"])].append(r)
        fill, edge = styles[label]
        pts = []
        for variant, delta, vname in want:
            rs = g[(variant, delta)]
            acc = 100.0 * sum(1 for r in rs if istrue(r["top_1_correct"])) / len(rs)
            lat = sum(float(r["total_ms"]) for r in rs) / len(rs)
            pts.append((lat, acc, vname))
            ax.scatter([lat], [acc], s=70, marker=markers[vname], facecolor=fill,
                       edgecolor=edge, linewidth=1.2, zorder=3)
        pts.sort()
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=edge, lw=1.0,
                alpha=0.7, zorder=2, label=label)

    for vname in [w[2] for w in want]:
        ax.scatter([], [], s=60, marker=markers[vname], facecolor="white",
                   edgecolor="#555", linewidth=1.1, label=vname)

    ax.set_xscale("log")
    ax.set_xlabel("mean end-to-end ranking latency (ms, log scale)")
    ax.set_ylabel("top-1 accuracy (%)")
    ax.set_ylim(84, 98)
    ax.grid(lw=0.4, alpha=0.4)
    ax.set_axisbelow(True)
    ax.legend(loc="upper left", ncol=2, frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "eval_ranker_scale.pdf"), bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    g = load_enroute()
    fig_enroute_outcomes(g)
    fig_enroute_timing(g)
    fig_clearance(g)
    fig_upfront_outcomes(load_upfront())
    fig_affordance()
    fig_ranker()
    print("wrote figures to", OUT)
