from semantic_nav_semantics.bm25 import BM25


def test_bm25_ranks_relevant_doc_first():
    docs = [
        "a padded wooden chair suitable for dining or kitchen use",
        "a dark upholstered office chair with wheels",
        "a small cigar box on the side cabinet",
    ]
    b = BM25(docs)
    scores = b.scores("dining kitchen")
    # doc[0] is the only doc containing "dining" and "kitchen"; docs[1] and
    # docs[2] share no query terms so both score 0.  The meaningful guarantee
    # is that the relevant doc outscores both irrelevant ones.
    assert scores[0] > scores[1]
    assert scores[0] > scores[2]


def test_bm25_zero_for_empty_query():
    b = BM25(["anything"])
    assert b.scores("") == [0.0]
    assert b.scores("   ") == [0.0]


def test_bm25_handles_single_doc():
    b = BM25(["the chair"])
    scores = b.scores("chair")
    assert len(scores) == 1
    assert scores[0] > 0.0
