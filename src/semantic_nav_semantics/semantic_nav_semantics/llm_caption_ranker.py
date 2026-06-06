import json
from dataclasses import dataclass
from typing import List, Optional, Sequence

from semantic_nav_semantics.caption_ranker import RankedObject
from semantic_nav_semantics.semantic_store import ObjectRow


_GBNF_TEMPLATE = (
    'root ::= "{{" ws key-pair "," ws rat-pair "," ws conf-pair ws "}}"\n'
    'key-pair ::= "\\"selected_object_key\\"" ws ":" ws "\\"" candidate "\\""\n'
    'rat-pair ::= "\\"rationale\\"" ws ":" ws string\n'
    'conf-pair ::= "\\"confidence\\"" ws ":" ws integer\n'
    'candidate ::= {alternation}\n'
    'string ::= "\\"" char* "\\""\n'
    'char ::= [a-zA-Z0-9 ,._:-]\n'
    'integer ::= "0" | [1-9] [0-9]*\n'
    'ws ::= ([ \\t\\n])*\n'
)


def build_grammar_for_candidates(candidates: Sequence[ObjectRow]) -> str:
    keys = [c.object_key for c in candidates if c.object_key]
    if not keys:
        raise ValueError("No candidates to build grammar from.")
    alternation = " | ".join(f'"{k}"' for k in keys)
    return _GBNF_TEMPLATE.format(alternation=alternation)


def build_prompt(intent_hint: str, user_command: str, candidates: Sequence[ObjectRow]) -> str:
    lines = []
    for c in candidates:
        cap = (c.object_caption or "").strip().replace("\n", " ")
        lines.append(f"  {c.object_key:24s} {c.object_tag:14s} \"{cap}\"")
    rendered = "\n".join(lines)
    return f"""You are a semantic scene reasoner for a mobile robot. Choose exactly one
candidate object that best satisfies the user's stated need. Return ONLY the
JSON object shown — no prose, no markdown.

Output schema:
{{"selected_object_key":"<one of the keys>","rationale":"<reason>","confidence":0-100}}

User command: "{user_command}"
Intent hint: "{intent_hint}"

Candidates:
{rendered}
"""


@dataclass
class LLMCaptionRanker:
    llama_client: Optional[object]      # LlamaActionClient or any object with .call(**)
    max_tokens: int = 160
    fallback_score: float = 0.0

    def rank(
        self,
        candidates: Sequence[ObjectRow],
        intent_hint: str,
        robot_xy=None,
        user_command: str = "",
    ) -> List[RankedObject]:
        if not candidates:
            return []

        if self.llama_client is None:
            return [self._fallback(candidates, reason="llm_unavailable")]

        prompt = build_prompt(intent_hint, user_command or intent_hint, candidates)
        try:
            grammar = build_grammar_for_candidates(candidates)
        except ValueError:
            return [self._fallback(candidates, reason="llm_empty_candidates")]

        text = self.llama_client.call(
            prompt=prompt, gbnf_grammar=grammar, max_tokens=self.max_tokens,
        )
        if not text:
            return [self._fallback(candidates, reason="llm_empty_response")]

        try:
            data = json.loads(text)
        except json.JSONDecodeError:
            return [self._fallback(candidates, reason="llm_invalid_json")]

        key = str(data.get("selected_object_key", "")).strip()
        try:
            confidence = int(data.get("confidence") or 0)
        except (TypeError, ValueError):
            confidence = 0
        rationale = str(data.get("rationale", "")).strip()

        chosen = next((c for c in candidates if c.object_key == key), None)
        if chosen is None:
            return [self._fallback(candidates, reason="llm_invalid_key")]

        others = sorted(
            (c for c in candidates if c.object_key != key),
            key=lambda c: c.object_key,
        )
        score = max(0.0, min(100, confidence)) / 100.0
        return [
            RankedObject(
                row=chosen, score=score, lexical_score=0.0, affordance_score=0.0,
                caption_tag_bonus=0.0, caption_boost_bonus=0.0,
                volume_bonus=0.0, distance_bonus=0.0, conflict_penalty=0.0,
                reasons=("llm_pick", rationale or ""),
            ),
        ] + [
            RankedObject(
                row=r, score=0.0, lexical_score=0.0, affordance_score=0.0,
                caption_tag_bonus=0.0, caption_boost_bonus=0.0,
                volume_bonus=0.0, distance_bonus=0.0, conflict_penalty=0.0,
                reasons=("llm_unselected",),
            )
            for r in others
        ]

    def _fallback(self, candidates: Sequence[ObjectRow], reason: str) -> RankedObject:
        sorted_cands = sorted(candidates, key=lambda c: c.object_key)
        return RankedObject(
            row=sorted_cands[0], score=self.fallback_score,
            lexical_score=0.0, affordance_score=0.0,
            caption_tag_bonus=0.0, caption_boost_bonus=0.0,
            volume_bonus=0.0, distance_bonus=0.0, conflict_penalty=0.0,
            reasons=("fallback", reason),
        )
