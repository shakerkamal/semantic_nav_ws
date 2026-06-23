"""Object-centric semantic store for semantic navigation.

Responsibilities:
  - Load object-centric map files whose top-level entries are object_N records.
  - Preserve source identity (``object_N``) and expose a stable wire key
    (``normalize(object_tag):id``), e.g. ``refrigerator:9``.
  - Build indices for object-key lookup, source-key lookup, and tag-gated lookup.
  - Load intent-affordance metadata, including aliases and non-navigable tags.
  - Expose a navigable tag vocabulary for LLM intent prompts.

The store does not produce poses. Object-to-standoff-pose planning belongs in
``standoff_planner.py`` in a later milestone.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from builtin_interfaces.msg import Time


_OBJECT_KEY_RE = re.compile(r"^\s*([a-zA-Z0-9_\- ]+)\s*:\s*(-?\d+)\s*$")


@dataclass(frozen=True)
class ObjectRow:
    """One object instance from map_v001.json."""
    source_key: str              # e.g. "object_8"
    object_key: str              # e.g. "refrigerator:9"
    object_id: int               # upstream id field
    object_tag: str              # raw tag after whitespace cleanup
    normalized_tag: str          # normalized tag used for indices
    object_caption: str
    object_state: str            # "static" | "semi-static" | "movable"
    bbox_center: Tuple[float, float, float]
    bbox_extent: Tuple[float, float, float]
    bbox_volume: float


@dataclass(frozen=True)
class IntentTagMetadata:
    """Front-end intent metadata for a tag."""

    navigable: bool
    affordances: Tuple[str, ...] = ()
    query_hints: Tuple[str, ...] = ()
    caption_boost_terms: Tuple[str, ...] = ()


@dataclass(frozen=True)
class ObjectIntentAffordances:
    """Repo-owned metadata for object-intent grounding.

    This is intentionally separate from object_action_attributes.json, which is
    used by the recovery layer for openable/clearable/safety_class.
    """

    defaults: IntentTagMetadata
    by_tag: Dict[str, IntentTagMetadata]
    aliases: Dict[str, str]

    def resolve_alias(self, text: str) -> str:
        normalized = normalize_tag(text)
        return self.aliases.get(normalized, normalized)

    def metadata_for_tag(self, tag: str) -> IntentTagMetadata:
        normalized = normalize_tag(tag)
        return self.by_tag.get(normalized, self.defaults)

    def tag_is_navigable(self, tag: str) -> bool:
        return bool(self.metadata_for_tag(tag).navigable)


@dataclass(frozen=True)
class SemanticStore:
    """Immutable semantic map snapshot.

    db_version is a Python int but must be serialized on ROS interfaces as a
    uint32. The loader derives it from the first four bytes of SHA-256 content
    hash, so it is already in the valid uint32 range.
    """

    db_version: int
    db_stamp: Time
    source_path: str

    by_source_key: Dict[str, ObjectRow]
    by_object_key: Dict[str, ObjectRow]
    by_tag: Dict[str, Tuple[str, ...]]
    tag_vocabulary: Tuple[str, ...]
    navigable_tag_vocabulary: Tuple[str, ...]
    affordances: ObjectIntentAffordances

    def resolve_tag_or_alias(self, tag_or_alias: str) -> Optional[str]:
        normalized = self.affordances.resolve_alias(tag_or_alias)
        if normalized in self.by_tag and normalized in self.navigable_tag_vocabulary:
            return normalized
        return None

    def object_key_exists(self, object_key: str) -> bool:
        return normalize_object_key(object_key) in self.by_object_key

    def target_known(self, object_tag: str = "", target_object_key: str = "") -> bool:
        """Definition used by ParseSemanticCommand.srv migration.

        True iff a navigable tag/alias resolves in this store, or an explicit
        object key resolves to an existing row.
        """
        if target_object_key and self.object_key_exists(target_object_key):
            return True
        if object_tag and self.resolve_tag_or_alias(object_tag) is not None:
            return True
        return False

    def rows_for_tag(self, tag_or_alias: str) -> Tuple[ObjectRow, ...]:
        """Hydrate tag-gated candidates from object keys to ObjectRow objects."""
        normalized = self.resolve_tag_or_alias(tag_or_alias)
        if normalized is None:
            return ()
        return tuple(
            self.by_object_key[k]
            for k in self.by_tag.get(normalized, ())
            if self.by_object_key[k].object_state != "displaced"
        )
    
    def query_window(
        self,
        center_xy: Tuple[float, float],
        radius_m: float,
    ) -> Tuple[ObjectRow, ...]:
        """Return object rows whose bbox center is within radius_m of center_xy.

        Uses plannar distance only; z is ignored. This supports local semantic
        context queries without exposing the full object database.
        """
        if radius_m <= 0.0:
            return ()

        cx, cy = float(center_xy[0]), float(center_xy[1])
        radius_sq = float(radius_m) * float(radius_m)

        hits = []
        for row in self.by_object_key.values():
            dx = float(row.bbox_center[0]) - cx
            dy = float(row.bbox_center[1]) - cy

            if dx * dx + dy * dy <= radius_sq:
                hits.append(row)

        return tuple(
            sorted(
                hits,
                key=lambda row: row.object_key,
            )
        )


class SemanticStoreError(ValueError):
    pass


def normalize_text(text: str) -> str:
    return " ".join(str(text or "").strip().lower().replace("_", " ").split())


def normalize_tag(tag: str) -> str:
    return normalize_text(tag)


def make_object_key(object_tag: str, object_id: int) -> str:
    return f"{normalize_tag(object_tag)}:{int(object_id)}"


def normalize_object_key(object_key: str) -> str:
    match = _OBJECT_KEY_RE.match(str(object_key or ""))
    if not match:
        return normalize_text(object_key)
    tag, object_id = match.group(1), int(match.group(2))
    return make_object_key(tag, object_id)


def looks_like_object_key(text: str) -> bool:
    return _OBJECT_KEY_RE.match(str(text or "")) is not None


def load_object_intent_affordances(path: str) -> ObjectIntentAffordances:
    """Load object_intent_affordances.json.

    Missing/empty path is allowed and produces permissive defaults. The caller
    should still provide the repo-owned sidecar in normal operation.
    """
    default_metadata = IntentTagMetadata(
        navigable=True,
        affordances=(),
        query_hints=(),
        caption_boost_terms=(),
    )

    if not path:
        return ObjectIntentAffordances(
            defaults=default_metadata,
            by_tag={},
            aliases={},
        )

    if not os.path.exists(path):
        raise FileNotFoundError(f"object_intent_affordances.json not found at '{path}'.")

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if not isinstance(data, dict):
        raise SemanticStoreError("object_intent_affordances.json must contain a JSON object.")

    defaults_obj = data.get("defaults", {})
    if defaults_obj is None:
        defaults_obj = {}
    if not isinstance(defaults_obj, dict):
        raise SemanticStoreError("object_intent_affordances.defaults must be an object.")

    defaults = _parse_intent_tag_metadata(defaults_obj, fallback=default_metadata)

    aliases_obj = data.get("aliases", {})
    if aliases_obj is None:
        aliases_obj = {}
    if not isinstance(aliases_obj, dict):
        raise SemanticStoreError("object_intent_affordances.aliases must be an object.")

    aliases: Dict[str, str] = {}
    for raw_alias, raw_target in aliases_obj.items():
        alias = normalize_tag(str(raw_alias))
        target = normalize_tag(str(raw_target))
        if alias and target:
            aliases[alias] = target

    by_tag_obj = data.get("by_tag", {})
    if by_tag_obj is None:
        by_tag_obj = {}
    if not isinstance(by_tag_obj, dict):
        raise SemanticStoreError("object_intent_affordances.by_tag must be an object.")

    by_tag: Dict[str, IntentTagMetadata] = {}
    for raw_tag, raw_metadata in by_tag_obj.items():
        tag = normalize_tag(str(raw_tag))
        if not tag:
            continue
        if raw_metadata is None:
            raw_metadata = {}
        if not isinstance(raw_metadata, dict):
            raise SemanticStoreError(f"metadata for tag '{raw_tag}' must be an object.")
        by_tag[tag] = _parse_intent_tag_metadata(raw_metadata, fallback=defaults)

    return ObjectIntentAffordances(defaults=defaults, by_tag=by_tag, aliases=aliases)


def load_semantic_store_from_string(
    json_payload: str,
    semantic_map_version: str = "",
    stamp: Optional[Time] = None,
    affordances_path: str = "",
) -> SemanticStore:
    """Load a SemanticStore from a JSON string (provider topic payload).

    db_version is derived from a SHA-256 hash of the payload bytes so it
    changes whenever the content changes, consistent with load_semantic_store.
    """
    affordances = load_object_intent_affordances(affordances_path)

    raw_bytes = json_payload.encode("utf-8")
    db_version = _uint32_hash(raw_bytes)
    db_stamp = stamp if stamp is not None else Time()

    try:
        data = json.loads(json_payload)
    except Exception as exc:
        raise SemanticStoreError(
            f"failed to parse SemanticMapUpdate json_payload: {exc}"
        ) from exc

    if not isinstance(data, dict):
        raise SemanticStoreError(
            "SemanticMapUpdate json_payload root must be a JSON object."
        )

    rows = _load_object_rows(data)

    by_source_key: Dict[str, ObjectRow] = {}
    by_object_key: Dict[str, ObjectRow] = {}
    by_tag_lists: Dict[str, List[str]] = {}

    for row in rows:
        if row.source_key in by_source_key:
            raise SemanticStoreError(f"duplicate source_key '{row.source_key}'.")
        if row.object_key in by_object_key:
            raise SemanticStoreError(f"duplicate object_key '{row.object_key}'.")
        by_source_key[row.source_key] = row
        by_object_key[row.object_key] = row
        by_tag_lists.setdefault(row.normalized_tag, []).append(row.object_key)

    by_tag = {
        tag: tuple(sorted(keys, key=_object_key_sort_key))
        for tag, keys in by_tag_lists.items()
    }
    tag_vocabulary = tuple(sorted(by_tag.keys()))
    navigable_tag_vocabulary = tuple(
        tag for tag in tag_vocabulary if affordances.tag_is_navigable(tag)
    )

    return SemanticStore(
        db_version=db_version,
        db_stamp=db_stamp,
        source_path=semantic_map_version or "<provider>",
        by_source_key=by_source_key,
        by_object_key=by_object_key,
        by_tag=by_tag,
        tag_vocabulary=tag_vocabulary,
        navigable_tag_vocabulary=navigable_tag_vocabulary,
        affordances=affordances,
    )


def load_semantic_store(
    map_path: str,
    affordances_path: str = "",
) -> SemanticStore:
    """Load map_v001.json object database."""
    affordances = load_object_intent_affordances(affordances_path)

    if not map_path or not os.path.exists(map_path):
        raise FileNotFoundError(f"semantic map not found at '{map_path}'.")

    selected_path = map_path

    with open(selected_path, "rb") as f:
        raw_bytes = f.read()

    db_version = _uint32_hash(raw_bytes)
    db_stamp = _mtime_to_time(selected_path)

    try:
        data = json.loads(raw_bytes.decode("utf-8"))
    except Exception as exc:
        raise SemanticStoreError(f"failed to parse semantic map '{selected_path}': {exc}") from exc

    if not isinstance(data, dict):
        raise SemanticStoreError("semantic map root must be a JSON object.")

    rows = _load_object_rows(data)

    by_source_key: Dict[str, ObjectRow] = {}
    by_object_key: Dict[str, ObjectRow] = {}
    by_tag_lists: Dict[str, List[str]] = {}

    for row in rows:
        if row.source_key in by_source_key:
            raise SemanticStoreError(f"duplicate source_key '{row.source_key}'.")
        if row.object_key in by_object_key:
            raise SemanticStoreError(f"duplicate object_key '{row.object_key}'.")

        by_source_key[row.source_key] = row
        by_object_key[row.object_key] = row
        by_tag_lists.setdefault(row.normalized_tag, []).append(row.object_key)

    by_tag = {
        tag: tuple(sorted(keys, key=_object_key_sort_key))
        for tag, keys in by_tag_lists.items()
    }

    tag_vocabulary = tuple(sorted(by_tag.keys()))
    navigable_tag_vocabulary = tuple(
        tag for tag in tag_vocabulary if affordances.tag_is_navigable(tag)
    )

    return SemanticStore(
        db_version=db_version,
        db_stamp=db_stamp,
        source_path=selected_path,
        by_source_key=by_source_key,
        by_object_key=by_object_key,
        by_tag=by_tag,
        tag_vocabulary=tag_vocabulary,
        navigable_tag_vocabulary=navigable_tag_vocabulary,
        affordances=affordances,
    )


def _parse_intent_tag_metadata(
    obj: Mapping[str, object],
    fallback: IntentTagMetadata,
) -> IntentTagMetadata:
    navigable = bool(obj.get("navigable", fallback.navigable))
    return IntentTagMetadata(
        navigable=navigable,
        affordances=_string_tuple(obj.get("affordances", fallback.affordances)),
        query_hints=_string_tuple(obj.get("query_hints", fallback.query_hints)),
        caption_boost_terms=_string_tuple(
            obj.get("caption_boost_terms", fallback.caption_boost_terms)
        ),
    )


def _string_tuple(value: object) -> Tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        return (normalize_text(value),) if normalize_text(value) else ()
    if not isinstance(value, Iterable):
        return ()
    out: List[str] = []
    for item in value:
        text = normalize_text(str(item))
        if text:
            out.append(text)
    return tuple(dict.fromkeys(out))


def _load_object_rows(data: Mapping[str, object]) -> Tuple[ObjectRow, ...]:
    rows: List[ObjectRow] = []

    for source_key, raw_record in data.items():
        if not isinstance(raw_record, dict):
            continue

        if not str(source_key).startswith("object_") and "object_tag" not in raw_record:
            # Ignore non-object metadata keys in future map variants.
            continue

        row = _parse_object_record(str(source_key), raw_record)
        rows.append(row)

    if not rows:
        raise SemanticStoreError("semantic map contained no valid object records.")

    return tuple(rows)


def _parse_object_record(source_key: str, record: Mapping[str, object]) -> ObjectRow:
    if "id" not in record:
        raise SemanticStoreError(f"object '{source_key}' missing required id.")
    try:
        object_id = int(record["id"])
    except Exception as exc:
        raise SemanticStoreError(f"object '{source_key}' has invalid id: {record.get('id')}") from exc

    raw_tag = str(record.get("object_tag", "")).strip()
    object_tag = " ".join(raw_tag.split())
    normalized_tag = normalize_tag(object_tag)
    if not normalized_tag:
        raise SemanticStoreError(f"object '{source_key}' missing object_tag.")

    object_caption = str(record.get("object_caption", "") or "").strip()

    object_state = str(
        record.get("object_state", record.get("object-state", "")) or ""
    ).strip()
    if object_state not in {"static", "semi-static", "movable", "displaced"}:
        raise SemanticStoreError(
            f"object '{source_key}' has invalid object_state='{object_state}'."
        )

    bbox_center = _parse_float_triplet(record.get("bbox_center"), source_key, "bbox_center")
    bbox_extent = _parse_float_triplet(record.get("bbox_extent"), source_key, "bbox_extent")

    if bbox_extent[0] < 0.0 or bbox_extent[1] < 0.0 or bbox_extent[2] < 0.0:
        raise SemanticStoreError(f"object '{source_key}' bbox_extent cannot contain negatives.")
    if bbox_extent[0] == 0.0 and bbox_extent[1] == 0.0 and bbox_extent[2] == 0.0:
        raise SemanticStoreError(f"object '{source_key}' bbox_extent cannot be all zeros.")

    try:
        bbox_volume = float(record.get("bbox_volume", 0.0))
    except Exception:
        bbox_volume = 0.0
    if not math.isfinite(bbox_volume):
        bbox_volume = 0.0

    return ObjectRow(
        source_key=source_key,
        object_key=make_object_key(object_tag, object_id),
        object_id=object_id,
        object_tag=object_tag,
        normalized_tag=normalized_tag,
        object_caption=object_caption,
        object_state=object_state,
        bbox_center=bbox_center,
        bbox_extent=bbox_extent,
        bbox_volume=bbox_volume,
    )


def _parse_float_triplet(value: object, source_key: str, field_name: str) -> Tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 3:
        raise SemanticStoreError(f"object '{source_key}' field '{field_name}' must be a length-3 array.")

    try:
        triplet = tuple(float(v) for v in value)
    except Exception as exc:
        raise SemanticStoreError(f"object '{source_key}' field '{field_name}' contains non-numeric values.") from exc

    if not all(math.isfinite(v) for v in triplet):
        raise SemanticStoreError(f"object '{source_key}' field '{field_name}' contains non-finite values.")

    return triplet  # type: ignore[return-value]


def _uint32_hash(raw_bytes: bytes) -> int:
    digest = hashlib.sha256(raw_bytes).digest()
    return int.from_bytes(digest[:4], byteorder="big", signed=False)


def _mtime_to_time(path: str) -> Time:
    ns = int(os.path.getmtime(path) * 1_000_000_000)
    stamp = Time()
    stamp.sec = int(ns // 1_000_000_000)
    stamp.nanosec = int(ns % 1_000_000_000)
    return stamp


def _object_key_sort_key(object_key: str) -> Tuple[str, int, str]:
    match = _OBJECT_KEY_RE.match(object_key)
    if not match:
        return (normalize_text(object_key), 0, object_key)
    return (normalize_tag(match.group(1)), int(match.group(2)), object_key)
