# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Pure helpers for open-set affordance classification (spec 21.4)."""

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
