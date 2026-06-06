import math
from semantic_nav_semantics.semantic_store import ObjectRow
from semantic_nav_semantics.caption_ranker import RankedObject
from semantic_nav_semantics.hybrid_caption_ranker import HybridCaptionRanker


def row(key, tag="chair"):
    return ObjectRow(
        source_key=f"object_{key.split(':')[1]}", object_key=key,
        object_id=int(key.split(':')[1]),
        object_tag=tag, normalized_tag=tag, object_caption="x",
        object_state="movable", bbox_center=(0.0, 0.0, 0.0),
        bbox_extent=(1.0, 1.0, 1.0), bbox_volume=1.0,
    )


class _BM25Stub:
    def __init__(self, ordering, scores):
        self._ordering = ordering
        self._scores = scores
        self.called = False

    def rank(self, candidates, intent_hint, robot_xy=None):
        self.called = True
        by_key = {c.object_key: c for c in candidates}
        out = []
        for k, s in zip(self._ordering, self._scores):
            out.append(RankedObject(
                row=by_key[k], score=s, lexical_score=s,
                affordance_score=0.0, caption_tag_bonus=0.0,
                caption_boost_bonus=0.0, volume_bonus=0.0,
                distance_bonus=0.0, conflict_penalty=0.0, reasons=("bm25",),
            ))
        return out


class _LLMStub:
    def __init__(self, key, conf=0.85):
        self._key = key
        self._conf = conf
        self.called = False

    def rank(self, candidates, intent_hint, robot_xy=None, user_command=""):
        self.called = True
        by_key = {c.object_key: c for c in candidates}
        head = RankedObject(
            row=by_key[self._key], score=self._conf,
            lexical_score=0.0, affordance_score=0.0, caption_tag_bonus=0.0,
            caption_boost_bonus=0.0, volume_bonus=0.0, distance_bonus=0.0,
            conflict_penalty=0.0, reasons=("llm_pick",),
        )
        return [head]


def test_clear_winner_skips_llm():
    cs = [row("chair:2"), row("chair:39"), row("chair:4")]
    bm = _BM25Stub(["chair:2", "chair:39", "chair:4"], [3.0, 1.0, 0.5])
    llm = _LLMStub(key="chair:39")
    hybrid = HybridCaptionRanker(bm25=bm, llm=llm, delta=0.5, top_k=3)
    ranked = hybrid.rank(cs, intent_hint="dining")
    assert ranked[0].row.object_key == "chair:2"
    assert llm.called is False
    assert "bm25" in " ".join(ranked[0].reasons)


def test_close_call_invokes_llm_tiebreak():
    cs = [row("chair:2"), row("chair:39"), row("chair:4")]
    bm = _BM25Stub(["chair:2", "chair:39", "chair:4"], [2.0, 1.8, 0.5])
    llm = _LLMStub(key="chair:39")
    hybrid = HybridCaptionRanker(bm25=bm, llm=llm, delta=0.5, top_k=3)
    ranked = hybrid.rank(cs, intent_hint="dining")
    assert llm.called is True
    assert ranked[0].row.object_key == "chair:39"


def test_delta_infinity_always_invokes_llm():
    cs = [row("chair:2"), row("chair:39")]
    bm = _BM25Stub(["chair:2", "chair:39"], [10.0, 0.1])
    llm = _LLMStub(key="chair:39")
    hybrid = HybridCaptionRanker(bm25=bm, llm=llm, delta=math.inf, top_k=2)
    ranked = hybrid.rank(cs, intent_hint="x")
    assert llm.called is True
    assert ranked[0].row.object_key == "chair:39"


def test_delta_zero_never_invokes_llm():
    cs = [row("chair:2"), row("chair:39")]
    bm = _BM25Stub(["chair:2", "chair:39"], [1.0, 1.0])
    llm = _LLMStub(key="chair:39")
    hybrid = HybridCaptionRanker(bm25=bm, llm=llm, delta=0.0, top_k=2)
    ranked = hybrid.rank(cs, intent_hint="x")
    assert llm.called is False


def test_single_candidate_skips_llm():
    cs = [row("chair:2")]
    bm = _BM25Stub(["chair:2"], [1.0])
    llm = _LLMStub(key="chair:2")
    hybrid = HybridCaptionRanker(bm25=bm, llm=llm, delta=10.0, top_k=4)
    ranked = hybrid.rank(cs, intent_hint="x")
    assert llm.called is False
    assert ranked[0].row.object_key == "chair:2"
