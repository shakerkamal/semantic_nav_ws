"""Pure TTL cache for dynamic semantic overlay observations.

No ROS imports — testable without a running node.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List


@dataclass
class _CacheEntry:
    object_key: str
    center_x: float
    center_y: float
    expires_at_sec: float
    payload: Any


class DynamicObjectCache:
    """TTL-keyed spatial cache for short-lived dynamic object observations."""

    def __init__(
        self,
        default_ttl_sec: float = 3.0,
        max_ttl_sec: float = 10.0,
    ) -> None:
        self._default_ttl = float(default_ttl_sec)
        self._max_ttl = float(max_ttl_sec)
        self._entries: Dict[str, _CacheEntry] = {}

    def update(
        self,
        object_key: str,
        center_x: float,
        center_y: float,
        ttl_sec: float,
        payload: Any,
        now_sec: float,
    ) -> None:
        """Upsert an observation. Resets TTL if the key already exists."""
        if ttl_sec <= 0.0:
            ttl_sec = self._default_ttl
        ttl_sec = min(max(ttl_sec, 0.1), self._max_ttl)
        self._entries[object_key] = _CacheEntry(
            object_key=object_key,
            center_x=center_x,
            center_y=center_y,
            expires_at_sec=now_sec + ttl_sec,
            payload=payload,
        )

    def objects_in_radius(
        self,
        center_x: float,
        center_y: float,
        radius_m: float,
        now_sec: float,
    ) -> List[Any]:
        """Return live entry payloads within radius_m, purging expired ones."""
        expired = [
            k for k, e in self._entries.items() if e.expires_at_sec <= now_sec
        ]
        for k in expired:
            del self._entries[k]

        result = []
        for entry in self._entries.values():
            dx = entry.center_x - center_x
            dy = entry.center_y - center_y
            if math.hypot(dx, dy) <= radius_m:
                result.append(entry.payload)
        return result

    def __len__(self) -> int:
        return len(self._entries)
