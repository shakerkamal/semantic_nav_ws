# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Unit tests for DynamicObjectCache — no ROS runtime required."""

from semantic_nav_semantics.dynamic_overlay import DynamicObjectCache


def _cache(
    default_ttl: float = 5.0, max_ttl: float = 10.0
) -> DynamicObjectCache:
    return DynamicObjectCache(default_ttl_sec=default_ttl, max_ttl_sec=max_ttl)


# ---------------------------------------------------------------------------
# update / __len__
# ---------------------------------------------------------------------------

def test_update_adds_entry():
    c = _cache()
    c.update("human:1", 0.0, 0.0, 3.0, {"key": "human:1"}, now_sec=0.0)
    assert len(c) == 1


def test_update_same_key_overwrites():
    c = _cache()
    c.update("human:1", 0.0, 0.0, 3.0, "first", now_sec=0.0)
    c.update("human:1", 1.0, 1.0, 3.0, "second", now_sec=0.0)
    assert len(c) == 1


def test_update_resets_ttl():
    c = _cache()
    c.update("human:1", 0.0, 0.0, 3.0, "v1", now_sec=0.0)
    # Refresh at t=2 with new TTL=3 → expires at t=5
    c.update("human:1", 0.0, 0.0, 3.0, "v2", now_sec=2.0)
    result = c.objects_in_radius(0.0, 0.0, 1.0, now_sec=4.9)
    assert len(result) == 1


# ---------------------------------------------------------------------------
# objects_in_radius — spatial
# ---------------------------------------------------------------------------

def test_object_inside_radius_returned():
    c = _cache()
    c.update("human:1", 2.0, 0.0, 5.0, {"key": "human:1"}, now_sec=0.0)
    result = c.objects_in_radius(0.0, 0.0, 5.0, now_sec=1.0)
    assert len(result) == 1
    assert result[0]["key"] == "human:1"


def test_object_at_exact_radius_boundary_returned():
    c = _cache()
    c.update("human:1", 5.0, 0.0, 5.0, "payload", now_sec=0.0)
    result = c.objects_in_radius(0.0, 0.0, 5.0, now_sec=1.0)
    assert len(result) == 1


def test_object_outside_radius_not_returned():
    c = _cache()
    c.update("human:1", 10.0, 0.0, 5.0, "payload", now_sec=0.0)
    result = c.objects_in_radius(0.0, 0.0, 5.0, now_sec=1.0)
    assert len(result) == 0


def test_multiple_objects_spatial_filter():
    c = _cache()
    c.update("obj:a", 1.0, 0.0, 5.0, "a", now_sec=0.0)
    c.update("obj:b", 8.0, 0.0, 5.0, "b", now_sec=0.0)
    c.update("obj:c", 3.0, 0.0, 5.0, "c", now_sec=0.0)
    result = c.objects_in_radius(0.0, 0.0, 5.0, now_sec=1.0)
    assert len(result) == 2
    assert set(result) == {"a", "c"}


# ---------------------------------------------------------------------------
# objects_in_radius — TTL / expiry
# ---------------------------------------------------------------------------

def test_expired_object_not_returned():
    c = _cache()
    c.update("human:1", 0.0, 0.0, 2.0, "payload", now_sec=0.0)
    # expires at t=2.0; query at t=2.0 is exactly expired
    result = c.objects_in_radius(0.0, 0.0, 5.0, now_sec=2.0)
    assert len(result) == 0


def test_expired_object_is_purged():
    c = _cache()
    c.update("human:1", 0.0, 0.0, 2.0, "payload", now_sec=0.0)
    c.objects_in_radius(0.0, 0.0, 5.0, now_sec=3.0)
    assert len(c) == 0


def test_live_object_returned_before_expiry():
    c = _cache()
    c.update("human:1", 0.0, 0.0, 5.0, "payload", now_sec=0.0)
    result = c.objects_in_radius(0.0, 0.0, 5.0, now_sec=4.99)
    assert len(result) == 1


def test_mix_of_live_and_expired():
    c = _cache()
    c.update("live:1", 0.0, 0.0, 5.0, "live", now_sec=0.0)
    c.update("dead:1", 0.0, 0.0, 1.0, "dead", now_sec=0.0)
    result = c.objects_in_radius(0.0, 0.0, 5.0, now_sec=2.0)
    assert result == ["live"]
    assert len(c) == 1


# ---------------------------------------------------------------------------
# TTL clamping
# ---------------------------------------------------------------------------

def test_zero_ttl_uses_default():
    c = _cache(default_ttl=4.0, max_ttl=10.0)
    c.update("obj:1", 0.0, 0.0, 0.0, "payload", now_sec=0.0)
    result = c.objects_in_radius(0.0, 0.0, 1.0, now_sec=3.9)
    assert len(result) == 1


def test_ttl_clamped_to_max():
    c = _cache(default_ttl=5.0, max_ttl=10.0)
    c.update("obj:1", 0.0, 0.0, 999.0, "payload", now_sec=0.0)
    # Should expire at t=10 (max_ttl), not t=999
    result = c.objects_in_radius(0.0, 0.0, 1.0, now_sec=10.0)
    assert len(result) == 0


def test_ttl_minimum_clamped():
    c = _cache(default_ttl=5.0, max_ttl=10.0)
    # negative TTL → default
    c.update("obj:1", 0.0, 0.0, -1.0, "payload", now_sec=0.0)
    result = c.objects_in_radius(0.0, 0.0, 1.0, now_sec=4.9)
    assert len(result) == 1


# ---------------------------------------------------------------------------
# Dynamic overlay does NOT touch SemanticStore
# ---------------------------------------------------------------------------

def test_cache_has_no_semantic_store_dependency():
    # DynamicObjectCache should be importable and functional
    # with no SemanticStore, no ROS, no filesystem.
    c = DynamicObjectCache(default_ttl_sec=1.0, max_ttl_sec=5.0)
    c.update("dummy:1", 0.0, 0.0, 1.0, "payload", now_sec=0.0)
    assert len(c) == 1
    assert c.objects_in_radius(0.0, 0.0, 1.0, now_sec=0.5) == ["payload"]
