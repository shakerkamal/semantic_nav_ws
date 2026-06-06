import pytest
from semantic_nav_semantics.ranker_factory import build_ranker, RankerSpec
from semantic_nav_semantics.caption_ranker import BM25CaptionRanker
from semantic_nav_semantics.llm_caption_ranker import LLMCaptionRanker
from semantic_nav_semantics.hybrid_caption_ranker import HybridCaptionRanker


def test_bm25_factory_returns_bm25_ranker():
    spec = RankerSpec(name="bm25", delta=0.5, top_k=4)
    r = build_ranker(spec, affordances=None, llama_client=None)
    assert isinstance(r, BM25CaptionRanker)


def test_llm_factory_requires_client():
    spec = RankerSpec(name="llm", delta=0.5, top_k=4)
    with pytest.raises(ValueError):
        build_ranker(spec, affordances=None, llama_client=None)


def test_llm_factory_builds_llm_ranker():
    spec = RankerSpec(name="llm", delta=0.5, top_k=4)
    r = build_ranker(spec, affordances=None, llama_client=object())
    assert isinstance(r, LLMCaptionRanker)


def test_hybrid_factory_builds_hybrid():
    spec = RankerSpec(name="hybrid", delta=0.5, top_k=4)
    r = build_ranker(spec, affordances=None, llama_client=object())
    assert isinstance(r, HybridCaptionRanker)


def test_unknown_name_raises():
    with pytest.raises(ValueError):
        build_ranker(RankerSpec(name="banana"), affordances=None, llama_client=None)
