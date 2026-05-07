import json
import threading
from pathlib import Path
from typing import Any, Dict, Optional


class SemanticStore:
    def __init__(self, store_path: str):
        self._store_path = Path(store_path)
        self._lock = threading.Lock()
        self._db_version: int = 0
        self._locations = self._load_locations()

    def _load_locations(self) -> Dict[str, Any]:
        if not self._store_path.exists():
            raise FileNotFoundError(f"Semantic database not found at {self._store_path}")

        with self._store_path.open('r', encoding='utf-8') as f:
            data = json.load(f)

        if "locations" not in data or not isinstance(data["locations"], dict):
            raise ValueError("Invalid semantic database format: 'locations' key missing or not a dictionary")

        return data["locations"]

    @property
    def db_version(self) -> int:
        with self._lock:
            return self._db_version

    def update_from_msg(self, db_version: int, locations: Dict[str, Any]) -> None:
        """Atomically replace the in-memory snapshot with topic data."""
        with self._lock:
            self._locations = locations
            self._db_version = db_version

    def resolve_location(self, query: str) -> Optional[Dict[str, Any]]:
        normalized = self._normalize(query)

        with self._lock:
            locations = self._locations
            version = self._db_version

        for location_id, location in locations.items():
            if normalized == self._normalize(location_id):
                return {"location_id": location_id, "db_version": version, **location}

            for alias in location.get("aliases", []):
                if normalized == self._normalize(alias):
                    return {"location_id": location_id, "db_version": version, **location}

        return None

    @staticmethod
    def _normalize(text: str) -> str:
        return " ".join(text.strip().lower().replace("_", " ").split())
