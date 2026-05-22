import json
import math
import threading
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Dict, Mapping, Optional, Tuple


@dataclass(frozen=True)
class LocationRecord:
    location_id: str
    frame_id: str
    x: float
    y: float
    yaw: float
    aliases: Tuple[str, ...]


@dataclass(frozen=True)
class ResolvedLocation:
    location_id: str
    frame_id: str
    x: float
    y: float
    yaw: float
    db_version: int
    db_stamp_sec: int
    db_stamp_nanosec: int


@dataclass(frozen=True)
class SemanticSnapshot:
    db_version: int
    db_stamp_sec: int
    db_stamp_nanosec: int
    locations_by_id: Mapping[str, LocationRecord]
    alias_to_location_id: Mapping[str, str]


class SemanticStore:
    """
    Static semantic database store.

    Current design:
      - Load semantic_db.json once at node startup.
      - Build an immutable in-memory snapshot.
      - Resolve queries against normalized location IDs and aliases.
      - Expose db_version and db_stamp for ResolveLocation responses.

    Future live-topic design can reuse the same snapshot replacement model.
    """

    def __init__(self, store_path: str, initial_db_version: int = 1):
        self._store_path = Path(store_path)
        self._lock = threading.Lock()

        if initial_db_version <= 0:
            raise ValueError("initial_db_version must be >= 1")

        snapshot = self._load_snapshot_from_file(initial_db_version)

        with self._lock:
            self._snapshot = snapshot

    @property
    def source_path(self) -> Path:
        return self._store_path

    @property
    def db_version(self) -> int:
        with self._lock:
            return self._snapshot.db_version

    @property
    def db_stamp_sec(self) -> int:
        with self._lock:
            return self._snapshot.db_stamp_sec

    @property
    def db_stamp_nanosec(self) -> int:
        with self._lock:
            return self._snapshot.db_stamp_nanosec

    @property
    def location_count(self) -> int:
        with self._lock:
            return len(self._snapshot.locations_by_id)

    @property
    def alias_count(self) -> int:
        with self._lock:
            return len(self._snapshot.alias_to_location_id)

    def resolve_location(self, query: str) -> Optional[ResolvedLocation]:
        normalized = self._normalize(query)

        if not normalized:
            return None

        with self._lock:
            snapshot = self._snapshot

        location_id = snapshot.alias_to_location_id.get(normalized)
        if location_id is None:
            return None

        record = snapshot.locations_by_id[location_id]

        return ResolvedLocation(
            location_id=record.location_id,
            frame_id=record.frame_id,
            x=record.x,
            y=record.y,
            yaw=record.yaw,
            db_version=snapshot.db_version,
            db_stamp_sec=snapshot.db_stamp_sec,
            db_stamp_nanosec=snapshot.db_stamp_nanosec,
        )

    def _load_snapshot_from_file(self, db_version: int) -> SemanticSnapshot:
        if not self._store_path.exists():
            raise FileNotFoundError(
                f"Semantic database not found at {self._store_path}"
            )

        if not self._store_path.is_file():
            raise ValueError(
                f"Semantic database path is not a file: {self._store_path}"
            )

        with self._store_path.open("r", encoding="utf-8") as f:
            data = json.load(f)

        stat = self._store_path.stat()
        stamp_sec = int(stat.st_mtime)
        stamp_nanosec = int((stat.st_mtime - stamp_sec) * 1_000_000_000)

        return self._build_snapshot(
            data=data,
            db_version=db_version,
            db_stamp_sec=stamp_sec,
            db_stamp_nanosec=stamp_nanosec,
        )

    def _build_snapshot(
        self,
        data: Dict[str, Any],
        db_version: int,
        db_stamp_sec: int,
        db_stamp_nanosec: int,
    ) -> SemanticSnapshot:
        if not isinstance(data, dict):
            raise ValueError("Invalid semantic database: root must be a JSON object")

        locations = data.get("locations")
        if not isinstance(locations, dict):
            raise ValueError(
                "Invalid semantic database format: 'locations' key missing or not a dictionary"
            )

        if not locations:
            raise ValueError("Invalid semantic database: 'locations' cannot be empty")

        locations_by_id: Dict[str, LocationRecord] = {}
        alias_to_location_id: Dict[str, str] = {}

        for raw_location_id, raw_location in locations.items():
            if not isinstance(raw_location_id, str):
                raise ValueError("Invalid semantic database: location IDs must be strings")

            location_id = raw_location_id.strip()
            if not location_id:
                raise ValueError("Invalid semantic database: location ID cannot be empty")

            if location_id in locations_by_id:
                raise ValueError(f"Duplicate location_id='{location_id}'")

            if not isinstance(raw_location, dict):
                raise ValueError(
                    f"Invalid semantic database: location '{location_id}' must be an object"
                )

            frame_id = str(raw_location.get("frame_id", "")).strip()
            if frame_id != "map":
                raise ValueError(
                    f"Invalid semantic database: location '{location_id}' has "
                    f"frame_id='{frame_id}', expected 'map'"
                )

            x = self._finite_float(raw_location.get("x"), f"{location_id}.x")
            y = self._finite_float(raw_location.get("y"), f"{location_id}.y")
            yaw = self._finite_float(raw_location.get("yaw", 0.0), f"{location_id}.yaw")

            raw_aliases = raw_location.get("aliases", [])
            if raw_aliases is None:
                raw_aliases = []

            if not isinstance(raw_aliases, list):
                raise ValueError(
                    f"Invalid semantic database: location '{location_id}' aliases must be a list"
                )

            aliases = []

            normalized_location_id = self._normalize(location_id)
            if normalized_location_id:
                aliases.append(normalized_location_id)

            for alias in raw_aliases:
                if not isinstance(alias, str):
                    raise ValueError(
                        f"Invalid semantic database: location '{location_id}' contains a non-string alias"
                    )

                normalized_alias = self._normalize(alias)
                if normalized_alias:
                    aliases.append(normalized_alias)

            unique_aliases = tuple(dict.fromkeys(aliases))

            if not unique_aliases:
                raise ValueError(
                    f"Invalid semantic database: location '{location_id}' has no resolvable ID or aliases"
                )

            record = LocationRecord(
                location_id=location_id,
                frame_id=frame_id,
                x=x,
                y=y,
                yaw=yaw,
                aliases=unique_aliases,
            )

            locations_by_id[location_id] = record

            for alias in unique_aliases:
                existing = alias_to_location_id.get(alias)
                if existing is not None and existing != location_id:
                    raise ValueError(
                        f"Alias collision: alias='{alias}' maps to both "
                        f"'{existing}' and '{location_id}'"
                    )

                alias_to_location_id[alias] = location_id

        return SemanticSnapshot(
            db_version=int(db_version),
            db_stamp_sec=int(db_stamp_sec),
            db_stamp_nanosec=int(db_stamp_nanosec),
            locations_by_id=MappingProxyType(locations_by_id),
            alias_to_location_id=MappingProxyType(alias_to_location_id),
        )

    @staticmethod
    def _finite_float(value: Any, field_name: str) -> float:
        try:
            parsed = float(value)
        except Exception as exc:
            raise ValueError(f"Invalid semantic database: field '{field_name}' must be numeric") from exc

        if not math.isfinite(parsed):
            raise ValueError(f"Invalid semantic database: field '{field_name}' must be finite")

        return parsed

    @staticmethod
    def _normalize(text: str) -> str:
        return " ".join(text.strip().lower().replace("_", " ").split())