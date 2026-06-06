from dataclasses import dataclass
from typing import List, Tuple

import yaml


@dataclass(frozen=True)
class Fixture:
    id: str
    utterance: str
    intent_hint: str
    tag_class: str
    expected_object_keys: Tuple[str, ...]
    notes: str = ""


_REQUIRED = ("id", "utterance", "intent_hint", "tag_class", "expected_object_keys")


def load_fixtures(path: str) -> List[Fixture]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or []
    if not isinstance(data, list):
        raise ValueError(f"Fixture file must be a YAML list, got {type(data).__name__}.")

    out: List[Fixture] = []
    seen_ids = set()
    for i, raw in enumerate(data):
        if not isinstance(raw, dict):
            raise ValueError(f"Fixture #{i}: must be a mapping.")
        for field in _REQUIRED:
            if field not in raw or raw[field] in (None, ""):
                raise ValueError(f"Fixture #{i}: missing required field '{field}'.")
        if not isinstance(raw["expected_object_keys"], list) or not raw["expected_object_keys"]:
            raise ValueError(
                f"Fixture {raw.get('id')}: expected_object_keys must be a non-empty list."
            )
        fxid = str(raw["id"])
        if fxid in seen_ids:
            raise ValueError(f"Fixture {fxid}: duplicate id.")
        seen_ids.add(fxid)
        out.append(Fixture(
            id=fxid,
            utterance=str(raw["utterance"]),
            intent_hint=str(raw["intent_hint"]),
            tag_class=str(raw["tag_class"]),
            expected_object_keys=tuple(str(k) for k in raw["expected_object_keys"]),
            notes=str(raw.get("notes", "") or ""),
        ))
    return out
