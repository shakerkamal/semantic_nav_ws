"""Generate plots from a ranker_eval CSV.

Usage:
  ros2 run semantic_nav_semantics plot_ranker_eval --csv eval/results.csv --out eval/
"""
import argparse
import csv
import os
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def _load(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def _accuracy_per_variant(rows):
    agg = defaultdict(lambda: [0, 0])
    for r in rows:
        k = (r["variant"], r["delta"])
        agg[k][0] += int(r["top_1_correct"])
        agg[k][1] += 1
    return {k: c / n for k, (c, n) in agg.items() if n > 0}


def _mean_latency(rows, column):
    agg = defaultdict(list)
    for r in rows:
        k = (r["variant"], r["delta"])
        agg[k].append(float(r[column]))
    return {k: (sum(v) / len(v)) for k, v in agg.items()}


def _llm_invoke_fraction(rows):
    agg = defaultdict(lambda: [0, 0])
    for r in rows:
        if not r["variant"].startswith("hybrid"):
            continue
        k = (r["variant"], r["delta"])
        agg[k][0] += 1 if float(r["llm_rank_ms"]) > 0.1 else 0
        agg[k][1] += 1
    return {k: c / n for k, (c, n) in agg.items() if n > 0}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--csv", required=True)
    p.add_argument("--out", default="eval/")
    args = p.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rows = _load(args.csv)
    acc = _accuracy_per_variant(rows)
    lat = _mean_latency(rows, "total_ms")
    inv = _llm_invoke_fraction(rows)

    labels = [f"{v}\nd={d}" if d != "-" else v for (v, d) in acc.keys()]
    values = list(acc.values())
    fig, ax = plt.subplots(figsize=(max(6, len(labels) * 0.8), 4))
    ax.bar(range(len(labels)), [v * 100 for v in values])
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=45, ha="right")
    ax.set_ylabel("Top-1 accuracy (%)")
    ax.set_title("Ranker accuracy across variants")
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "accuracy.png"), dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(max(6, len(labels) * 0.8), 4))
    ax.bar(range(len(labels)), [lat.get(k, 0.0) for k in acc.keys()])
    ax.set_yscale("log")
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=45, ha="right")
    ax.set_ylabel("Mean total latency (ms, log)")
    ax.set_title("Ranker latency across variants")
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "latency.png"), dpi=150)
    plt.close(fig)

    hybrid_points = sorted(
        ((inv[k], acc[k], k[1]) for k in inv.keys()),
        key=lambda t: t[0],
    )
    if hybrid_points:
        fig, ax = plt.subplots(figsize=(5, 4))
        xs = [t[0] for t in hybrid_points]
        ys = [t[1] * 100 for t in hybrid_points]
        labels_h = [f"d={t[2]}" for t in hybrid_points]
        ax.plot(xs, ys, "o-")
        for x, y, lbl in zip(xs, ys, labels_h):
            ax.annotate(lbl, (x, y), textcoords="offset points", xytext=(4, 4))
        ax.set_xlabel("LLM invocation fraction")
        ax.set_ylabel("Top-1 accuracy (%)")
        ax.set_title("Hybrid: accuracy vs LLM cost")
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, "hybrid_pareto.png"), dpi=150)
        plt.close(fig)

    print(f"Plots written to {args.out}")


if __name__ == "__main__":
    main()
