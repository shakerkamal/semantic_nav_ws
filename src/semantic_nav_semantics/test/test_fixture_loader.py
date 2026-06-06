import os
import pytest
import yaml
from semantic_nav_semantics.eval.fixture_loader import Fixture, load_fixtures


def write(tmp_path, payload):
    p = tmp_path / "gt.yaml"
    p.write_text(yaml.safe_dump(payload))
    return str(p)


def test_loads_minimal_fixture(tmp_path):
    p = write(tmp_path, [{
        "id": "gt-001",
        "utterance": "I am hungry",
        "intent_hint": "food storage",
        "tag_class": "refrigerator",
        "expected_object_keys": ["refrigerator:9"],
    }])
    fixtures = load_fixtures(p)
    assert len(fixtures) == 1
    fx = fixtures[0]
    assert isinstance(fx, Fixture)
    assert fx.id == "gt-001"
    assert fx.expected_object_keys == ("refrigerator:9",)


def test_rejects_duplicate_ids(tmp_path):
    p = write(tmp_path, [
        {"id": "x", "utterance": "u", "intent_hint": "h",
         "tag_class": "chair", "expected_object_keys": ["chair:2"]},
        {"id": "x", "utterance": "u2", "intent_hint": "h2",
         "tag_class": "chair", "expected_object_keys": ["chair:39"]},
    ])
    with pytest.raises(ValueError, match="duplicate"):
        load_fixtures(p)


def test_rejects_missing_required_field(tmp_path):
    p = write(tmp_path, [{
        "id": "x", "utterance": "u",
        "tag_class": "chair", "expected_object_keys": ["chair:2"],
    }])
    with pytest.raises(ValueError, match="intent_hint"):
        load_fixtures(p)


def test_rejects_empty_expected_keys(tmp_path):
    p = write(tmp_path, [{
        "id": "x", "utterance": "u", "intent_hint": "h",
        "tag_class": "chair", "expected_object_keys": [],
    }])
    with pytest.raises(ValueError, match="expected_object_keys"):
        load_fixtures(p)


REAL = os.path.join(os.path.dirname(__file__), "..", "semantic_nav_semantics", "eval", "ground_truth.yaml")


@pytest.mark.skipif(not os.path.exists(REAL), reason="ground_truth.yaml not present")
def test_real_ground_truth_loads_cleanly():
    fixtures = load_fixtures(REAL)
    assert len(fixtures) >= 30
