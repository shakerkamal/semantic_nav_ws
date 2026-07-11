# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Held-out open-set affordance-inference accuracy harness (spec 21.4).

Simulates the open-set condition: a set of tags is HELD OUT of the affordance
table (pretend the table never enumerated them), and for every object of those
tags in the semantic map we ask the live LLM (`/infer_affordance`) to infer
`{openable, clearable, safety_class}` from the object's caption. Each inference
is scored against the hand-authored ground truth in `object_action_attributes
.json` (which still holds the true label -- we only *pretend* it is unknown).

Prints per-field and overall accuracy and writes a per-sample CSV. Requires
`navigator_node` + `llama_ros` to be running.

Usage:
    python3 eval/affordance_holdout.py \
        [--tags door,chair,refrigerator,...] \
        [--attrs <object_action_attributes.json>] \
        [--map <map_v001.json>] \
        [--out eval/affordance_holdout_results.csv] \
        [--timeout 60]
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import rclpy
from rclpy.node import Node

from semantic_nav_interfaces.srv import InferAffordance

# Tags exercised by default: the world-grounded tag set assigned to the 94
# cleaned map objects (see docs; tags + captions verified against
# small_house_semantic.world). Spans all three affordance categories so
# accuracy is measured across the full decision surface, not one class.
DEFAULT_TAGS = [
    # openable
    "refrigerator", "cabinet", "door",
    # clearable
    "chair", "trash bin", "vase", "kitchen utensils",
    "seasoning box", "fitness equipment", "tablet",
    # neither
    "bed", "table", "desk", "picture", "range hood",
    "air conditioner", "security camera", "board",
    "television", "stove", "wall",
]


def _norm(tag: str) -> str:
    return (tag or "").strip().lower()


def recovery_directive(openable, clearable, safety_class) -> str:
    """The recovery directive an affordance triple would select, mirroring the
    orchestrator's precedence (safety floor > open > clear > none). This is the
    outcome the pipeline actually acts on -- more meaningful than per-field
    accuracy, since e.g. a door's redundant clearable flag never changes the
    chosen directive (openable already selects open_door_then_replan)."""
    if _norm(safety_class) in ("human", "animal"):
        return "wait_no_move"
    if bool(openable):
        return "open_door_then_replan"
    if bool(clearable):
        return "clear_object_then_replan"
    return "wait_or_approach"


def _default_path(rel: str) -> str:
    """Prefer the installed config; fall back to the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("semantic_nav_semantics")
        cand = os.path.join(share, "config", rel)
        if os.path.exists(cand):
            return cand
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, "..", "src", "semantic_nav_semantics", "config", rel)


def load_ground_truth(attrs_path: str) -> dict:
    with open(attrs_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return {_norm(k): v for k, v in data.get("by_tag", {}).items()}


def load_samples(map_path: str, held_out: set, gt: dict) -> list:
    """Return [(tag, caption, gt_dict)] for every map object whose tag is held
    out AND has a ground-truth entry (so it is scorable)."""
    with open(map_path, "r", encoding="utf-8") as f:
        objects = json.load(f)
    samples = []
    for rec in objects.values():
        tag = _norm(rec.get("object_tag", rec.get("tag", "")))
        if tag not in held_out or tag not in gt:
            continue
        caption = str(rec.get("object_caption", rec.get("caption", "")))
        samples.append((tag, caption, gt[tag]))
    return samples


class AffordanceHoldoutClient(Node):
    def __init__(self, service_name: str):
        super().__init__("affordance_holdout_client")
        self._client = self.create_client(InferAffordance, service_name)

    def infer(self, tag: str, caption: str, timeout_sec: float):
        """Return the InferAffordance.Response, or None on failure/timeout."""
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                f"'{self._client.srv_name}' unavailable after {timeout_sec:.0f}s."
            )
            return None
        req = InferAffordance.Request()
        req.object_tag = tag
        req.object_caption = caption
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            self.get_logger().warn(f"inference timed out for tag='{tag}'.")
            return None
        return future.result()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tags", default=",".join(DEFAULT_TAGS))
    parser.add_argument("--attrs", default=_default_path("object_action_attributes.json"))
    parser.add_argument("--map", default=_default_path("map_v001.json"))
    parser.add_argument("--out", default="eval/affordance_holdout_results.csv")
    parser.add_argument("--service", default="/infer_affordance")
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args(argv)

    held_out = {_norm(t) for t in args.tags.split(",") if t.strip()}
    gt = load_ground_truth(args.attrs)
    samples = load_samples(args.map, held_out, gt)
    if not samples:
        print("No scorable samples (no map objects with the held-out tags).")
        return 1
    print(f"Held-out tags: {sorted(held_out)}")
    print(f"Scorable samples: {len(samples)}")

    rclpy.init()
    node = AffordanceHoldoutClient(args.service)
    rows = []
    n_open = n_clear = n_safe = n_all = n_dir = 0
    try:
        for tag, caption, truth in samples:
            gt_dir = recovery_directive(
                truth["openable"], truth["clearable"], truth["safety_class"])
            resp = node.infer(tag, caption, args.timeout)
            if resp is None or not bool(resp.success):
                rows.append({
                    "tag": tag, "caption": caption,
                    "gt_openable": truth["openable"], "gt_clearable": truth["clearable"],
                    "gt_safety": truth["safety_class"],
                    "inf_openable": "", "inf_clearable": "", "inf_safety": "",
                    "confidence": "", "correct": False,
                    "gt_directive": gt_dir, "inf_directive": "", "directive_correct": False,
                })
                print(f"  [FAIL ] {tag}: no inference")
                continue
            ok_open = bool(resp.openable) == bool(truth["openable"])
            ok_clear = bool(resp.clearable) == bool(truth["clearable"])
            ok_safe = _norm(resp.safety_class) == _norm(truth["safety_class"])
            all_ok = ok_open and ok_clear and ok_safe
            inf_dir = recovery_directive(
                resp.openable, resp.clearable, resp.safety_class)
            ok_dir = inf_dir == gt_dir
            n_open += ok_open
            n_clear += ok_clear
            n_safe += ok_safe
            n_all += all_ok
            n_dir += ok_dir
            rows.append({
                "tag": tag, "caption": caption,
                "gt_openable": truth["openable"], "gt_clearable": truth["clearable"],
                "gt_safety": truth["safety_class"],
                "inf_openable": bool(resp.openable), "inf_clearable": bool(resp.clearable),
                "inf_safety": _norm(resp.safety_class),
                "confidence": int(resp.confidence_percent), "correct": all_ok,
                "gt_directive": gt_dir, "inf_directive": inf_dir,
                "directive_correct": ok_dir,
            })
            mark = "OK  " if all_ok else "MISS"
            print(f"  [{mark}] {tag}: open={bool(resp.openable)} "
                  f"clear={bool(resp.clearable)} safety={_norm(resp.safety_class)} "
                  f"conf={int(resp.confidence_percent)} dir={inf_dir}"
                  f"{'' if ok_dir else ' (!=' + gt_dir + ')'}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    n = len(samples)
    print("\n=== Open-set affordance-inference accuracy ===")
    print(f"openable  acc = {100.0 * n_open / n:5.1f}%  ({n_open}/{n})")
    print(f"clearable acc = {100.0 * n_clear / n:5.1f}%  ({n_clear}/{n})")
    print(f"safety    acc = {100.0 * n_safe / n:5.1f}%  ({n_safe}/{n})")
    print(f"all-3     acc = {100.0 * n_all / n:5.1f}%  ({n_all}/{n})")
    print("--- directive-level (what the recovery pipeline acts on) ---")
    print(f"directive acc = {100.0 * n_dir / n:5.1f}%  ({n_dir}/{n})")
    print(f"CSV written to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
