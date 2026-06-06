import json
from semantic_nav_semantics.semantic_store import ObjectRow
from semantic_nav_semantics.llm_caption_ranker import (
    LLMCaptionRanker, build_grammar_for_candidates, build_prompt,
)


def row(key, tag, caption):
    return ObjectRow(
        source_key=f"object_{key.split(':')[1]}",
        object_key=key, object_id=int(key.split(':')[1]),
        object_tag=tag, normalized_tag=tag,
        object_caption=caption, object_state="movable",
        bbox_center=(0.0, 0.0, 0.0), bbox_extent=(1.0, 1.0, 1.0), bbox_volume=1.0,
    )


def test_grammar_enumerates_only_provided_keys():
    cands = [row("chair:2", "chair", "x"), row("chair:39", "chair", "y")]
    g = build_grammar_for_candidates(cands)
    assert '"chair:2"' in g
    assert '"chair:39"' in g
    assert 'selected_object_key' in g
    assert 'confidence' in g
    assert '"refrigerator:9"' not in g


def test_prompt_lists_candidates_and_intent():
    cands = [
        row("chair:2", "chair", "padded wooden chair suitable for dining"),
        row("chair:39", "chair", "blue folding chair"),
    ]
    p = build_prompt(intent_hint="dining seating", user_command="I want to eat", candidates=cands)
    assert "chair:2" in p
    assert "padded wooden chair" in p
    assert "dining seating" in p
    assert "I want to eat" in p


class _FakeClient:
    def __init__(self, payload): self._payload = payload
    def call(self, **kwargs): return self._payload


def test_ranker_returns_chosen_key():
    cands = [
        row("chair:2", "chair", "padded wooden chair suitable for dining"),
        row("chair:39", "chair", "blue folding chair"),
    ]
    fake = _FakeClient(json.dumps({
        "selected_object_key": "chair:2",
        "rationale": "dining caption matches",
        "confidence": 88,
    }))
    r = LLMCaptionRanker(llama_client=fake)
    ranked = r.rank(cands, intent_hint="dining seating", user_command="I want to eat")
    assert ranked[0].row.object_key == "chair:2"
    assert ranked[0].score == 0.88
    assert "llm_pick" in ranked[0].reasons


def test_ranker_falls_back_on_invalid_key():
    cands = [row("chair:2", "chair", "x")]
    fake = _FakeClient(json.dumps({
        "selected_object_key": "chair:9999",
        "rationale": "...", "confidence": 90,
    }))
    r = LLMCaptionRanker(llama_client=fake)
    ranked = r.rank(cands, intent_hint="x", user_command="x")
    assert ranked[0].row.object_key == "chair:2"
    assert "fallback" in " ".join(ranked[0].reasons)


def test_ranker_falls_back_on_missing_client():
    cands = [row("chair:2", "chair", "x")]
    r = LLMCaptionRanker(llama_client=None)
    ranked = r.rank(cands, intent_hint="x", user_command="x")
    assert ranked[0].row.object_key == "chair:2"
    assert "llm_unavailable" in " ".join(ranked[0].reasons)


def test_ranker_falls_back_on_empty_response():
    cands = [row("chair:2", "chair", "x")]
    fake = _FakeClient("")
    r = LLMCaptionRanker(llama_client=fake)
    ranked = r.rank(cands, intent_hint="x", user_command="x")
    assert ranked[0].row.object_key == "chair:2"
    assert "llm_empty_response" in " ".join(ranked[0].reasons)


def test_ranker_falls_back_on_invalid_json():
    cands = [row("chair:2", "chair", "x")]
    fake = _FakeClient("not json at all")
    r = LLMCaptionRanker(llama_client=fake)
    ranked = r.rank(cands, intent_hint="x", user_command="x")
    assert ranked[0].row.object_key == "chair:2"
    assert "llm_invalid_json" in " ".join(ranked[0].reasons)


def test_ranker_handles_null_confidence():
    cands = [row("chair:2", "chair", "x")]
    fake = _FakeClient(json.dumps({
        "selected_object_key": "chair:2",
        "rationale": "ok",
        "confidence": None,
    }))
    r = LLMCaptionRanker(llama_client=fake)
    ranked = r.rank(cands, intent_hint="x", user_command="x")
    assert ranked[0].row.object_key == "chair:2"
    assert ranked[0].score == 0.0


def test_grammar_handles_keys_with_spaces():
    # Keys like "closet door:8" and "sofa chair:86" contain spaces.
    # The GBNF candidate alternatives are string literals, not constrained by
    # the char rule, so spaces are valid. The returned key must be matched back
    # to the candidate list via exact string equality.
    cands = [
        row("closet door:8", "closet door", "wooden closet door"),
        row("sofa chair:86", "sofa chair", "upholstered sofa chair"),
    ]
    g = build_grammar_for_candidates(cands)
    assert '"closet door:8"' in g
    assert '"sofa chair:86"' in g

    fake = _FakeClient(json.dumps({
        "selected_object_key": "closet door:8",
        "rationale": "matches the requested closet",
        "confidence": 75,
    }))
    r = LLMCaptionRanker(llama_client=fake)
    ranked = r.rank(cands, intent_hint="closet door", user_command="go to the closet door")
    assert ranked[0].row.object_key == "closet door:8"
    assert ranked[0].score == 0.75
    assert "llm_pick" in ranked[0].reasons
