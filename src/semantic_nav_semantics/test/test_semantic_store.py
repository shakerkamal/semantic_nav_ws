"""Unit tests for semantic_nav_semantics.semantic_store.

Run from the package directory with ROS 2 sourced:

    source /opt/ros/humble/setup.bash
    cd src/semantic_nav_semantics
    python3 -m pytest test/test_semantic_store.py -v
"""

import json
import math
import os

import pytest
from builtin_interfaces.msg import Time

from semantic_nav_semantics.semantic_store import (
    ObjectRow,
    SemanticStore,
    SemanticStoreError,
    load_object_intent_affordances,
    load_semantic_store,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_GOOD_OBJECT_STATE = "semi-static"


def _make_object(
    *,
    id: int,
    object_tag: str,
    object_state: str = _GOOD_OBJECT_STATE,
    bbox_center=None,
    bbox_extent=None,
    bbox_volume: float = 1.0,
    object_caption: str = "a test object",
) -> dict:
    """Return a single map record dict."""
    return {
        "id": id,
        "object_tag": object_tag,
        "object_state": object_state,
        "object_caption": object_caption,
        "bbox_center": bbox_center if bbox_center is not None else [1.0, 2.0, 0.5],
        "bbox_extent": bbox_extent if bbox_extent is not None else [0.5, 0.5, 0.5],
        "bbox_volume": bbox_volume,
    }


def _write_map(tmp_path, records: dict, filename: str = "map.json") -> str:
    """Write a map JSON file and return its path."""
    p = tmp_path / filename
    p.write_text(json.dumps(records), encoding="utf-8")
    return str(p)


def _write_affordances(tmp_path, data: dict, filename: str = "affordances.json") -> str:
    """Write an affordances JSON sidecar and return its path."""
    p = tmp_path / filename
    p.write_text(json.dumps(data), encoding="utf-8")
    return str(p)


def _minimal_affordances(*, non_navigable: list = None) -> dict:
    """Return a minimal affordances dict with specified tags as non-navigable."""
    non_navigable = non_navigable or []
    by_tag = {}
    for tag in non_navigable:
        by_tag[tag] = {"navigable": False}
    return {
        "defaults": {"navigable": True},
        "aliases": {},
        "by_tag": by_tag,
    }


def _three_object_map() -> dict:
    """Return a 3-object map fixture: refrigerator id=9, chair id=2, picture id=99."""
    return {
        "object_1": _make_object(id=9, object_tag="refrigerator"),
        "object_2": _make_object(id=2, object_tag="chair"),
        "object_3": _make_object(id=99, object_tag="picture"),
    }


# ---------------------------------------------------------------------------
# Test 1: Indexing
# ---------------------------------------------------------------------------


def test_indexing_by_object_key(tmp_path):
    """by_object_key contains all three objects keyed by tag:id."""
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path)

    assert isinstance(store, SemanticStore)
    assert "refrigerator:9" in store.by_object_key
    assert "chair:2" in store.by_object_key
    assert "picture:99" in store.by_object_key


def test_indexing_by_source_key(tmp_path):
    """by_source_key contains all source keys from the map file."""
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path)

    assert "object_1" in store.by_source_key
    assert "object_2" in store.by_source_key
    assert "object_3" in store.by_source_key


def test_indexing_source_key_on_row(tmp_path):
    """ObjectRow.source_key round-trips back to the original source key."""
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path)

    row = store.by_object_key["refrigerator:9"]
    assert row.source_key == "object_1"


def test_indexing_by_tag_tuple(tmp_path):
    """by_tag maps each tag to a tuple of object keys."""
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path)

    assert store.by_tag["refrigerator"] == ("refrigerator:9",)
    assert store.by_tag["chair"] == ("chair:2",)
    assert store.by_tag["picture"] == ("picture:99",)


def test_indexing_multiple_instances_same_tag(tmp_path):
    """Multiple objects sharing a tag are all listed under that tag."""
    records = {
        "object_1": _make_object(id=1, object_tag="chair"),
        "object_2": _make_object(id=2, object_tag="chair"),
    }
    map_path = _write_map(tmp_path, records)
    store = load_semantic_store(map_path)

    assert set(store.by_tag["chair"]) == {"chair:1", "chair:2"}
    assert len(store.by_tag["chair"]) == 2


# ---------------------------------------------------------------------------
# Test 2: Navigable vocabulary
# ---------------------------------------------------------------------------


def test_navigable_vocabulary_excludes_non_navigable(tmp_path):
    """Tags marked non-navigable appear in tag_vocabulary but not navigable_tag_vocabulary."""
    affordances_data = _minimal_affordances(non_navigable=["picture"])
    affordances_path = _write_affordances(tmp_path, affordances_data)
    map_path = _write_map(tmp_path, _three_object_map())

    store = load_semantic_store(map_path, affordances_path=affordances_path)

    assert "picture" in store.tag_vocabulary
    assert "picture" not in store.navigable_tag_vocabulary


def test_navigable_vocabulary_includes_navigable(tmp_path):
    """Tags NOT marked non-navigable appear in navigable_tag_vocabulary."""
    affordances_data = _minimal_affordances(non_navigable=["picture"])
    affordances_path = _write_affordances(tmp_path, affordances_data)
    map_path = _write_map(tmp_path, _three_object_map())

    store = load_semantic_store(map_path, affordances_path=affordances_path)

    assert "refrigerator" in store.navigable_tag_vocabulary
    assert "chair" in store.navigable_tag_vocabulary


def test_navigable_vocabulary_without_sidecar_all_navigable(tmp_path):
    """With no sidecar (permissive defaults), every tag is navigable."""
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path, affordances_path="")

    for tag in store.tag_vocabulary:
        assert tag in store.navigable_tag_vocabulary, f"tag '{tag}' should be navigable by default"


def test_tag_vocabulary_is_sorted(tmp_path):
    """tag_vocabulary is sorted alphabetically."""
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path)

    assert list(store.tag_vocabulary) == sorted(store.tag_vocabulary)


# ---------------------------------------------------------------------------
# Test 3: rows_for_tag hydration
# ---------------------------------------------------------------------------


def test_rows_for_tag_navigable(tmp_path):
    """rows_for_tag returns ObjectRow instances for a navigable tag."""
    affordances_data = _minimal_affordances(non_navigable=["picture"])
    affordances_path = _write_affordances(tmp_path, affordances_data)
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path, affordances_path=affordances_path)

    rows = store.rows_for_tag("chair")
    assert len(rows) == 1
    assert isinstance(rows[0], ObjectRow)
    assert rows[0].object_key == "chair:2"
    assert rows[0].object_tag == "chair"


def test_rows_for_tag_non_navigable_returns_empty(tmp_path):
    """rows_for_tag returns () for a non-navigable tag."""
    affordances_data = _minimal_affordances(non_navigable=["picture"])
    affordances_path = _write_affordances(tmp_path, affordances_data)
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path, affordances_path=affordances_path)

    rows = store.rows_for_tag("picture")
    assert rows == ()


def test_rows_for_tag_unknown_returns_empty(tmp_path):
    """rows_for_tag returns () for a completely unknown tag."""
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path)

    rows = store.rows_for_tag("doesnotexist")
    assert rows == ()


def test_rows_for_tag_alias_resolved(tmp_path):
    """rows_for_tag resolves an alias to the canonical tag."""
    affordances_data = {
        "defaults": {"navigable": True},
        "aliases": {"fridge": "refrigerator"},
        "by_tag": {},
    }
    affordances_path = _write_affordances(tmp_path, affordances_data)
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path, affordances_path=affordances_path)

    rows = store.rows_for_tag("fridge")
    assert len(rows) == 1
    assert rows[0].object_key == "refrigerator:9"


def test_rows_for_tag_is_tuple(tmp_path):
    """rows_for_tag always returns a tuple, never a list."""
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path)

    result = store.rows_for_tag("chair")
    assert isinstance(result, tuple)


# ---------------------------------------------------------------------------
# Test 4: Validation errors
# ---------------------------------------------------------------------------


def test_validation_error_missing_object_tag(tmp_path):
    """A record with no object_tag raises SemanticStoreError."""
    records = {
        "object_1": {
            "id": 1,
            # object_tag intentionally omitted
            "object_state": "static",
            "object_caption": "no tag",
            "bbox_center": [1.0, 2.0, 0.5],
            "bbox_extent": [0.5, 0.5, 0.5],
            "bbox_volume": 1.0,
        }
    }
    map_path = _write_map(tmp_path, records)
    with pytest.raises(SemanticStoreError):
        load_semantic_store(map_path)


def test_validation_error_empty_object_tag(tmp_path):
    """A record with an empty string object_tag raises SemanticStoreError."""
    records = {
        "object_1": {
            "id": 1,
            "object_tag": "   ",  # whitespace-only → normalizes to ""
            "object_state": "static",
            "object_caption": "empty tag",
            "bbox_center": [1.0, 2.0, 0.5],
            "bbox_extent": [0.5, 0.5, 0.5],
            "bbox_volume": 1.0,
        }
    }
    map_path = _write_map(tmp_path, records)
    with pytest.raises(SemanticStoreError):
        load_semantic_store(map_path)


def test_validation_error_non_finite_bbox_center(tmp_path):
    """A record with math.inf in bbox_center raises SemanticStoreError."""
    records = {
        "object_1": {
            "id": 1,
            "object_tag": "chair",
            "object_state": "static",
            "object_caption": "bad bbox",
            "bbox_center": [math.inf, 0.0, 0.0],
            "bbox_extent": [0.5, 0.5, 0.5],
            "bbox_volume": 1.0,
        }
    }
    map_path = _write_map(tmp_path, records)
    with pytest.raises(SemanticStoreError):
        load_semantic_store(map_path)


def test_validation_error_non_finite_bbox_extent(tmp_path):
    """A record with math.nan in bbox_extent raises SemanticStoreError."""
    records = {
        "object_1": {
            "id": 1,
            "object_tag": "chair",
            "object_state": "static",
            "object_caption": "bad extent",
            "bbox_center": [1.0, 2.0, 0.5],
            "bbox_extent": [math.nan, 0.5, 0.5],
            "bbox_volume": 1.0,
        }
    }
    map_path = _write_map(tmp_path, records)
    with pytest.raises(SemanticStoreError):
        load_semantic_store(map_path)


def test_validation_error_missing_id(tmp_path):
    """A record with no id field raises SemanticStoreError."""
    records = {
        "object_1": {
            # id intentionally omitted
            "object_tag": "chair",
            "object_state": "static",
            "object_caption": "no id",
            "bbox_center": [1.0, 2.0, 0.5],
            "bbox_extent": [0.5, 0.5, 0.5],
            "bbox_volume": 1.0,
        }
    }
    map_path = _write_map(tmp_path, records)
    with pytest.raises(SemanticStoreError):
        load_semantic_store(map_path)


def test_validation_error_invalid_object_state(tmp_path):
    """A record with an unsupported object_state raises SemanticStoreError."""
    records = {
        "object_1": _make_object(id=1, object_tag="chair", object_state="unknown_state"),
    }
    map_path = _write_map(tmp_path, records)
    with pytest.raises(SemanticStoreError):
        load_semantic_store(map_path)


def test_validation_error_file_not_found():
    """load_semantic_store raises FileNotFoundError for a missing map."""
    with pytest.raises(FileNotFoundError):
        load_semantic_store("/nonexistent/path/map_v001.json")


# ---------------------------------------------------------------------------
# Test 5: db_version / db_stamp
# ---------------------------------------------------------------------------


def test_db_version_is_positive_int(tmp_path):
    """db_version is a positive Python int derived from the file content hash."""
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path)

    assert isinstance(store.db_version, int)
    assert store.db_version > 0


def test_db_version_in_uint32_range(tmp_path):
    """db_version fits in a uint32 (0 <= v < 2**32)."""
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path)

    assert 0 < store.db_version < 2**32


def test_db_stamp_is_ros_time(tmp_path):
    """db_stamp is a builtin_interfaces.msg.Time instance."""
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path)

    assert isinstance(store.db_stamp, Time)


def test_db_stamp_sec_non_negative(tmp_path):
    """db_stamp.sec is non-negative (derived from file mtime)."""
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path)

    assert store.db_stamp.sec >= 0


def test_db_version_stable_for_same_content(tmp_path):
    """Two loads of the same content produce the same db_version."""
    data = _three_object_map()
    p1 = _write_map(tmp_path, data, filename="map_a.json")
    p2 = _write_map(tmp_path, data, filename="map_b.json")

    store_a = load_semantic_store(p1)
    store_b = load_semantic_store(p2)

    assert store_a.db_version == store_b.db_version


def test_db_version_changes_with_content(tmp_path):
    """Changing content produces a different db_version."""
    data_a = _three_object_map()
    data_b = _three_object_map()
    # Modify one field to change the hash
    data_b["object_1"]["id"] = 42

    p1 = _write_map(tmp_path, data_a, filename="map_a.json")
    p2 = _write_map(tmp_path, data_b, filename="map_b.json")

    store_a = load_semantic_store(p1)
    store_b = load_semantic_store(p2)

    assert store_a.db_version != store_b.db_version


# ---------------------------------------------------------------------------
# Test 6: Convenience methods
# ---------------------------------------------------------------------------


def test_object_key_exists_true(tmp_path):
    """object_key_exists returns True for a key present in the store."""
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path)

    assert store.object_key_exists("refrigerator:9")
    assert store.object_key_exists("chair:2")


def test_object_key_exists_false(tmp_path):
    """object_key_exists returns False for an absent key."""
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path)

    assert not store.object_key_exists("microwave:5")


def test_resolve_tag_or_alias_navigable(tmp_path):
    """resolve_tag_or_alias returns the normalized tag for a navigable tag."""
    affordances_data = _minimal_affordances(non_navigable=["picture"])
    affordances_path = _write_affordances(tmp_path, affordances_data)
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path, affordances_path=affordances_path)

    assert store.resolve_tag_or_alias("chair") == "chair"
    assert store.resolve_tag_or_alias("refrigerator") == "refrigerator"


def test_resolve_tag_or_alias_non_navigable_returns_none(tmp_path):
    """resolve_tag_or_alias returns None for a non-navigable tag."""
    affordances_data = _minimal_affordances(non_navigable=["picture"])
    affordances_path = _write_affordances(tmp_path, affordances_data)
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path, affordances_path=affordances_path)

    assert store.resolve_tag_or_alias("picture") is None


def test_target_known_by_tag(tmp_path):
    """target_known returns True when an object_tag resolves."""
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path)

    assert store.target_known(object_tag="chair")
    assert not store.target_known(object_tag="nonexistent_tag_xyz")


def test_target_known_by_object_key(tmp_path):
    """target_known returns True when an explicit object_key resolves."""
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path)

    assert store.target_known(target_object_key="refrigerator:9")
    assert not store.target_known(target_object_key="refrigerator:99")


def test_target_known_both_empty(tmp_path):
    """target_known with no arguments (both empty) returns False."""
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path)
    assert not store.target_known()


# ---------------------------------------------------------------------------
# Test 7: load_object_intent_affordances
# ---------------------------------------------------------------------------


def test_affordances_empty_path_permissive():
    """load_object_intent_affordances with empty path returns navigable defaults."""
    aff = load_object_intent_affordances("")
    assert aff.tag_is_navigable("anything")
    # resolve_alias normalizes via normalize_tag (underscores → spaces, lowercase, etc.)
    # so test with a value that survives normalization unchanged.
    assert aff.resolve_alias("someunknowntag") == "someunknowntag"


def test_affordances_resolve_alias(tmp_path):
    """resolve_alias returns the target tag for a known alias."""
    data = {
        "defaults": {"navigable": True},
        "aliases": {"fridge": "refrigerator"},
        "by_tag": {},
    }
    path = _write_affordances(tmp_path, data)
    aff = load_object_intent_affordances(path)

    assert aff.resolve_alias("fridge") == "refrigerator"
    assert aff.resolve_alias("unknown") == "unknown"


def test_affordances_metadata_for_tag(tmp_path):
    """metadata_for_tag returns tag-specific metadata when present."""
    data = {
        "defaults": {"navigable": True},
        "aliases": {},
        "by_tag": {
            "picture": {"navigable": False},
        },
    }
    path = _write_affordances(tmp_path, data)
    aff = load_object_intent_affordances(path)

    assert not aff.tag_is_navigable("picture")
    assert aff.tag_is_navigable("chair")  # defaults apply


def test_affordances_missing_file_raises():
    """load_object_intent_affordances raises FileNotFoundError for a missing path."""
    with pytest.raises(FileNotFoundError):
        load_object_intent_affordances("/nonexistent/path/affordances.json")


# ---------------------------------------------------------------------------
# Test 8: store is frozen (immutable)
# ---------------------------------------------------------------------------


def test_store_is_frozen(tmp_path):
    """SemanticStore is a frozen dataclass — attribute assignment must raise."""
    map_path = _write_map(tmp_path, _three_object_map())
    store = load_semantic_store(map_path)

    with pytest.raises((AttributeError, TypeError)):
        store.db_version = 999  # type: ignore[misc]


# ---------------------------------------------------------------------------
# Test 9: Real-map smoke test (skipped if file absent)
# ---------------------------------------------------------------------------

_REAL_MAP = os.path.join(
    os.path.dirname(__file__), "..", "config", "map_v001.json"
)
_REAL_SIDECAR = os.path.join(
    os.path.dirname(__file__), "..", "config", "object_intent_affordances.json"
)


@pytest.mark.skipif(not os.path.exists(_REAL_MAP), reason="map_v001.json not present")
def test_loads_real_map_v001():
    """Smoke test: the real map_v001.json loads and key assertions hold."""
    store = load_semantic_store(_REAL_MAP, affordances_path=_REAL_SIDECAR)

    assert "refrigerator:9" in store.by_object_key
    assert "picture" in store.tag_vocabulary
    assert "picture" not in store.navigable_tag_vocabulary
    assert "refrigerator" in store.navigable_tag_vocabulary
    assert isinstance(store.db_version, int)
    assert store.db_version > 0
    assert isinstance(store.db_stamp, Time)
    assert store.db_stamp.sec >= 0
    # All objects in by_object_key are also in by_source_key
    for row in store.by_object_key.values():
        assert row.source_key in store.by_source_key


@pytest.mark.skipif(not os.path.exists(_REAL_MAP), reason="map_v001.json not present")
def test_real_map_rows_for_tag_refrigerator():
    """rows_for_tag('refrigerator') on the real map returns the known instance."""
    store = load_semantic_store(_REAL_MAP, affordances_path=_REAL_SIDECAR)

    rows = store.rows_for_tag("refrigerator")
    assert len(rows) >= 1
    keys = [r.object_key for r in rows]
    assert "refrigerator:9" in keys


@pytest.mark.skipif(not os.path.exists(_REAL_MAP), reason="map_v001.json not present")
def test_real_map_alias_fridge_resolves():
    """Alias 'fridge' → 'refrigerator' works on the real map + sidecar."""
    store = load_semantic_store(_REAL_MAP, affordances_path=_REAL_SIDECAR)

    rows = store.rows_for_tag("fridge")
    assert len(rows) >= 1
    assert all(r.normalized_tag == "refrigerator" for r in rows)
