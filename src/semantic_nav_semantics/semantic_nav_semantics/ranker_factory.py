import math
from dataclasses import dataclass
from typing import Optional

from semantic_nav_semantics.caption_ranker import BM25CaptionRanker
from semantic_nav_semantics.hybrid_caption_ranker import HybridCaptionRanker
from semantic_nav_semantics.llm_caption_ranker import LLMCaptionRanker


@dataclass(frozen=True)
class RankerSpec:
    name: str = "bm25"
    delta: float = 0.5
    top_k: int = 4


def _parse_delta(d) -> float:
    if isinstance(d, str) and d.lower() in ("inf", "infinity", "infty"):
        return math.inf
    return float(d)


def build_ranker(spec: RankerSpec, affordances, llama_client: Optional[object] = None):
    name = (spec.name or "").strip().lower()
    delta = _parse_delta(spec.delta)

    if name == "bm25":
        return BM25CaptionRanker(affordances=affordances)
    if name == "llm":
        if llama_client is None:
            raise ValueError("ranker=llm requires a llama_client")
        return LLMCaptionRanker(llama_client=llama_client)
    if name == "hybrid":
        if llama_client is None:
            raise ValueError("ranker=hybrid requires a llama_client")
        return HybridCaptionRanker(
            bm25=BM25CaptionRanker(affordances=affordances),
            llm=LLMCaptionRanker(llama_client=llama_client),
            delta=delta,
            top_k=int(spec.top_k),
        )
    raise ValueError(f"Unknown ranker name: '{spec.name}'")
