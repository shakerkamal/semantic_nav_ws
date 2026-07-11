# src/semantic_nav_semantics/semantic_nav_semantics/eval/run_ranker_eval.py
"""Offline evaluation harness for caption rankers.

Usage:
  ros2 run semantic_nav_semantics ranker_eval \\
      --variants bm25,bm25+spatial,llm-text,llm-spatial,hybrid \\
      --delta 0.2,0.5,1.0,2.0,inf \\
      --fixtures <path-to-ground_truth.yaml> \\
      --out eval/results_<ts>.csv

LLM variants require /llama/generate_response to be running.
"""
import argparse
import csv
import math
import os
import time
from dataclasses import dataclass
from typing import List

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node

from semantic_nav_semantics.caption_ranker import BM25CaptionRanker
from semantic_nav_semantics.eval.fixture_loader import Fixture, load_fixtures
from semantic_nav_semantics.hybrid_caption_ranker import HybridCaptionRanker
from semantic_nav_semantics.llama_action_client import LlamaActionClient
from semantic_nav_semantics.llm_caption_ranker import LLMCaptionRanker
from semantic_nav_semantics.semantic_store import load_semantic_store
from semantic_nav_semantics.spatial_context import SpatialContextBuilder


_VARIANT_PARTS = {
    "bm25":           {"family": "bm25",   "spatial": False},
    "bm25+spatial":   {"family": "bm25",   "spatial": True},
    "llm-text":       {"family": "llm",    "spatial": False},
    "llm-spatial":    {"family": "llm",    "spatial": True},
    "hybrid":         {"family": "hybrid", "spatial": False},
    "hybrid+spatial": {"family": "hybrid", "spatial": True},
}


@dataclass
class Row:
    variant: str
    delta: str
    fixture_id: str
    tag_class: str
    predicted_key: str
    expected_keys: str
    top_1_correct: bool
    top_3_contains_expected: bool
    spatial_build_ms: float
    bm25_rank_ms: float
    llm_rank_ms: float
    total_ms: float
    top_score: float
    second_score: float
    reasons: str


class _NullClient:
    def call(self, **kwargs): return None


def _evaluate_one(
    family: str, spatial_on: bool, delta_str: str,
    fixture: Fixture, store, spatial_builder, llama_client,
) -> Row:
    candidates = list(store.rows_for_tag(fixture.tag_class))
    if not candidates:
        return Row(
            variant=f"{family}{'+spatial' if spatial_on else ''}",
            delta=delta_str, fixture_id=fixture.id, tag_class=fixture.tag_class,
            predicted_key="(no candidates)", expected_keys=",".join(fixture.expected_object_keys),
            top_1_correct=False, top_3_contains_expected=False,
            spatial_build_ms=0.0, bm25_rank_ms=0.0, llm_rank_ms=0.0, total_ms=0.0,
            top_score=0.0, second_score=0.0, reasons="no_candidates",
        )

    spatial_build_ms = 0.0
    if spatial_on:
        t0 = time.perf_counter()
        from dataclasses import replace
        all_rows = list(store.by_object_key.values())
        navigable = set(store.navigable_tag_vocabulary)
        candidates = [
            replace(c, object_caption=(
                c.object_caption + " | " +
                spatial_builder.build(c, all_rows, robot_xy=(0.0, 0.0), navigable_tags=navigable)
            ))
            for c in candidates
        ]
        spatial_build_ms = (time.perf_counter() - t0) * 1000.0

    bm25_rank_ms = 0.0
    llm_rank_ms = 0.0
    t_total = time.perf_counter()

    if family == "bm25":
        ranker = BM25CaptionRanker(affordances=store.affordances)
        t0 = time.perf_counter()
        ranked = ranker.rank(candidates, intent_hint=fixture.intent_hint, robot_xy=(0.0, 0.0))
        bm25_rank_ms = (time.perf_counter() - t0) * 1000.0
    elif family == "llm":
        ranker = LLMCaptionRanker(llama_client=llama_client)
        t0 = time.perf_counter()
        ranked = ranker.rank(
            candidates, intent_hint=fixture.intent_hint, robot_xy=(0.0, 0.0),
            user_command=fixture.utterance,
        )
        llm_rank_ms = (time.perf_counter() - t0) * 1000.0
    elif family == "hybrid":
        bm = BM25CaptionRanker(affordances=store.affordances)
        llm = LLMCaptionRanker(llama_client=llama_client)
        delta_val = math.inf if delta_str == "inf" else float(delta_str)
        ranker = HybridCaptionRanker(bm25=bm, llm=llm, delta=delta_val, top_k=4)
        t0 = time.perf_counter()
        _ = bm.rank(candidates, intent_hint=fixture.intent_hint, robot_xy=(0.0, 0.0))
        bm25_rank_ms = (time.perf_counter() - t0) * 1000.0
        t1 = time.perf_counter()
        ranked = ranker.rank(
            candidates, intent_hint=fixture.intent_hint, robot_xy=(0.0, 0.0),
            user_command=fixture.utterance,
        )
        llm_rank_ms = max(0.0, (time.perf_counter() - t1) * 1000.0 - bm25_rank_ms)
    else:
        raise ValueError(f"unknown family: {family}")

    total_ms = (time.perf_counter() - t_total) * 1000.0 + spatial_build_ms

    top = ranked[0]
    second = ranked[1] if len(ranked) > 1 else None
    pred_key = top.row.object_key
    top1 = pred_key in fixture.expected_object_keys
    top3_keys = {r.row.object_key for r in ranked[:3]}
    top3 = bool(top3_keys & set(fixture.expected_object_keys))

    variant = f"{family}{'+spatial' if spatial_on else ''}"
    return Row(
        variant=variant, delta=delta_str,
        fixture_id=fixture.id, tag_class=fixture.tag_class,
        predicted_key=pred_key,
        expected_keys=",".join(fixture.expected_object_keys),
        top_1_correct=top1, top_3_contains_expected=top3,
        spatial_build_ms=spatial_build_ms, bm25_rank_ms=bm25_rank_ms,
        llm_rank_ms=llm_rank_ms, total_ms=total_ms,
        top_score=top.score, second_score=second.score if second else 0.0,
        reasons="|".join(top.reasons),
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--variants", default="bm25,bm25+spatial,llm-text,llm-spatial,hybrid")
    p.add_argument("--delta", default="0.5",
                   help="Comma-separated delta values for hybrid; e.g. 0.2,0.5,1.0,2.0,inf")
    p.add_argument("--fixtures", default=None)
    p.add_argument("--map", dest="map_path", default=None,
                   help="Semantic map JSON to rank against (default: installed "
                        "map_v001.json). Use eval/benchmark_v0/map_v0_noisy.json "
                        "for the frozen v0 ranker benchmark.")
    p.add_argument("--affordances", dest="affordances_path", default=None,
                   help="Intent-affordance sidecar matching the map (default: "
                        "installed object_intent_affordances.json).")
    p.add_argument("--out", default=None)
    args = p.parse_args()

    share = get_package_share_directory("semantic_nav_semantics")
    fixtures_path = args.fixtures or os.path.join(share, "eval", "ground_truth.yaml")
    if args.out:
        out_path = args.out
    else:
        ts = time.strftime("%Y%m%d-%H%M%S")
        out_path = f"eval/results_{ts}.csv"

    map_path = args.map_path or os.path.join(share, "config", "map_v001.json")
    sidecar_path = args.affordances_path or os.path.join(
        share, "config", "object_intent_affordances.json")
    store = load_semantic_store(map_path, affordances_path=sidecar_path)

    sb = SpatialContextBuilder()
    fixtures = load_fixtures(fixtures_path)

    rclpy.init()
    node = Node("ranker_eval_harness")
    llama_client = _NullClient()
    needs_llm = any(v.startswith("llm") or v.startswith("hybrid")
                    for v in args.variants.split(","))
    if needs_llm:
        try:
            from llama_msgs.action import GenerateResponse
            ac = ActionClient(node, GenerateResponse, "/llama/generate_response")
            llama_client = LlamaActionClient(action_client=ac, logger=node.get_logger(), node=node)
            node.get_logger().info("Waiting for /llama/generate_response...")
            if not ac.wait_for_server(timeout_sec=30.0):
                node.get_logger().error("llama_ros not running; LLM variants will be skipped.")
                llama_client = None
        except Exception as exc:
            node.get_logger().error(f"llama_msgs import failed: {exc}")
            llama_client = None

    rows: List[Row] = []
    deltas_for_hybrid = args.delta.split(",")
    for variant in args.variants.split(","):
        variant = variant.strip()
        if variant not in _VARIANT_PARTS:
            node.get_logger().error(f"Unknown variant '{variant}', skipping.")
            continue
        parts = _VARIANT_PARTS[variant]
        family = parts["family"]
        spatial = parts["spatial"]
        if family in ("llm", "hybrid") and llama_client is None:
            node.get_logger().warn(f"Skipping {variant}: no LLM client available.")
            continue

        sweep = deltas_for_hybrid if family == "hybrid" else ["-"]
        for delta_str in sweep:
            for fx in fixtures:
                row = _evaluate_one(
                    family=family, spatial_on=spatial, delta_str=delta_str,
                    fixture=fx, store=store, spatial_builder=sb, llama_client=llama_client,
                )
                rows.append(row)
                node.get_logger().info(
                    f"[EVAL] {variant} delta={delta_str} {fx.id} "
                    f"pred={row.predicted_key} ok={row.top_1_correct} "
                    f"total={row.total_ms:.1f}ms"
                )

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "variant", "delta", "fixture_id", "tag_class", "predicted_key", "expected_keys",
            "top_1_correct", "top_3_contains_expected",
            "spatial_build_ms", "bm25_rank_ms", "llm_rank_ms", "total_ms",
            "top_score", "second_score", "reasons",
        ])
        for r in rows:
            w.writerow([
                r.variant, r.delta, r.fixture_id, r.tag_class,
                r.predicted_key, r.expected_keys,
                int(r.top_1_correct), int(r.top_3_contains_expected),
                f"{r.spatial_build_ms:.2f}", f"{r.bm25_rank_ms:.2f}",
                f"{r.llm_rank_ms:.2f}", f"{r.total_ms:.2f}",
                f"{r.top_score:.4f}", f"{r.second_score:.4f}", r.reasons,
            ])
    node.get_logger().info(f"[EVAL] wrote {len(rows)} rows to {out_path}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
