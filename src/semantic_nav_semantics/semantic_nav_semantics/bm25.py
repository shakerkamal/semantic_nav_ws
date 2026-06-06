import math
import re
from collections import Counter
from typing import List, Sequence

_TOKEN_RE = re.compile(r"[a-z0-9]+")

_STOPWORDS = frozenset({
    "a", "an", "the", "of", "and", "or", "to", "in", "on", "at", "for", "with",
    "is", "are", "was", "were", "be", "by", "as", "it", "its", "this", "that",
})


def tokenize(s: str) -> List[str]:
    return [t for t in _TOKEN_RE.findall((s or "").lower()) if t not in _STOPWORDS]


class BM25:
    def __init__(
        self,
        docs: Sequence[str],
        k1: float = 1.5,
        b: float = 0.75,
    ) -> None:
        self.k1 = k1
        self.b = b
        self._tokens: List[List[str]] = [tokenize(d) for d in docs]
        self._doc_lens = [len(t) for t in self._tokens]
        n = len(self._tokens)
        self._avgdl = (sum(self._doc_lens) / n) if n else 0.0

        df = Counter()
        for toks in self._tokens:
            for term in set(toks):
                df[term] += 1
        self._idf = {
            term: math.log(1.0 + (n - dfi + 0.5) / (dfi + 0.5))
            for term, dfi in df.items()
        }

    def scores(self, query: str) -> List[float]:
        q_terms = list(dict.fromkeys(tokenize(query)))
        if not q_terms:
            return [0.0] * len(self._tokens)

        out: List[float] = []
        for toks, dl in zip(self._tokens, self._doc_lens):
            tf = Counter(toks)
            s = 0.0
            denom_norm = 1.0 - self.b + self.b * (dl / self._avgdl if self._avgdl else 0.0)
            for term in q_terms:
                if term not in tf:
                    continue
                idf = self._idf.get(term, 0.0)
                numer = tf[term] * (self.k1 + 1.0)
                denom = tf[term] + self.k1 * denom_norm
                s += idf * (numer / denom if denom else 0.0)
            out.append(s)
        return out
