# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Pure helpers for open-set affordance classification (spec 21.4).

Duplicated verbatim from semantic_nav_orchestrator/affordance_classification.py
(2026-07-15) rather than imported, to avoid a semantics->orchestrator
dependency: orchestrator already depends on semantics for the reverse
direction (SemanticStore etc.), and this module is tiny, pure, and
dependency-free, so duplication is cheaper than the inversion. Used here to
wire open-set inference into the DYNAMIC (live-perceived) object ingestion
path in local_object_query_node -- previously up-front only, leaving any
en-route detection of a genuinely novel tag stuck with restrictive table
defaults (a live detector can report ANY tag, not just the ones already in
object_action_attributes.json).
"""

from __future__ import annotations

from dataclasses import dataclass


def _norm(tag: str) -> str:
    return (tag or "").strip().lower()


def tag_is_classifiable(tag: str, table_tags: set, door_substring: bool = True) -> bool:
    """True if the affordance table (or the door-substring rule) already covers
    this tag -- i.e. no LLM inference is needed."""
    t = _norm(tag)
    if t in table_tags:
        return True
    if door_substring and "door" in t:
        return True
    return False


@dataclass(frozen=True)
class InferredAffordance:
    openable: bool
    clearable: bool
    safety_class: str
    confidence: int


def accept_inference(inf: InferredAffordance, confidence_floor: int) -> bool:
    """Gate: accept the LLM's inferred affordance only at/above the floor."""
    return int(inf.confidence) >= int(confidence_floor)
