"""Unit tests for SemanticStore.query_window."""

from builtin_interfaces.msg import Time

from semantic_nav_semantics.semantic_store import (
    IntentTagMetadata,
    ObjectIntentAffordances,
    ObjectRow,
    SemanticStore,
)


def _make_row(object_key: str, tag: str, x: float, y: float) -> ObjectRow:
    return ObjectRow(
        source_key=f"object_{object_key}",
        object_key=object_key,
        object_id=int(object_key.split(":")[1]),
        object_tag=tag,
        normalized_tag=tag,
        object_caption="",
        object_state="static",
        bbox_center=(x, y, 0.0),
        bbox_extent=(0.5, 0.5, 0.5),
        bbox_volume=0.125,
    )


def _make_store(rows):
    by_object_key = {r.object_key: r for r in rows}
    by_source_key = {r.source_key: r for r in rows}

    by_tag = {}
    for r in rows:
        by_tag.setdefault(r.normalized_tag, []).append(r.object_key)

    by_tag = {
        tag: tuple(sorted(keys))
        for tag, keys in by_tag.items()
    }

    affordances = ObjectIntentAffordances(
        defaults=IntentTagMetadata(navigable=True),
        by_tag={},
        aliases={},
    )

    return SemanticStore(
        db_version=1,
        db_stamp=Time(sec=0, nanosec=0),
        source_path="<test>",
        by_source_key=by_source_key,
        by_object_key=by_object_key,
        by_tag=by_tag,
        tag_vocabulary=tuple(sorted(by_tag.keys())),
        navigable_tag_vocabulary=tuple(sorted(by_tag.keys())),
        affordances=affordances,
    )


def test_query_window_returns_objects_within_radius():
    rows = [
        _make_row("chair:1", "chair", 0.0, 0.0),
        _make_row("chair:2", "chair", 3.0, 0.0),
        _make_row("chair:3", "chair", 10.0, 0.0),
    ]
    store = _make_store(rows)

    result = store.query_window(center_xy=(0.0, 0.0), radius_m=4.0)

    assert tuple(r.object_key for r in result) == ("chair:1", "chair:2")


def test_query_window_excludes_objects_outside_radius():
    rows = [_make_row("chair:1", "chair", 0.0, 0.0)]
    store = _make_store(rows)

    result = store.query_window(center_xy=(100.0, 100.0), radius_m=1.0)

    assert result == ()


def test_query_window_zero_radius_returns_empty():
    rows = [_make_row("chair:1", "chair", 0.0, 0.0)]
    store = _make_store(rows)

    result = store.query_window(center_xy=(0.0, 0.0), radius_m=0.0)

    assert result == ()


def test_query_window_clamps_negative_radius_to_empty():
    rows = [_make_row("chair:1", "chair", 0.0, 0.0)]
    store = _make_store(rows)

    result = store.query_window(center_xy=(0.0, 0.0), radius_m=-1.0)

    assert result == ()