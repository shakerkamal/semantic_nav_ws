import json
from pathlib import Path
from typing import Any, Dict, List, Optional

from numpy import place

class SemanticStore:
    def __init__(self, store_path: str):
        self._store_path = Path(store_path)
        self._locations = self._load_locations()

    def _load_locations(self) -> Dict[str, Any]:
        if not self._store_path.exists():
            raise FileNotFoundError(f"Semantic database not found at {self._store_path}")
        
        with self._store_path.open('r', encoding='utf-8') as f:
            data = json.load(f)
        
        if "locations" not in data or not isinstance(data["locations"], dict):
            raise ValueError("Invalid semantic database format: 'locations' key missing or not a dictionary")

        return data["locations"]
    
    def reload_locations(self) -> None:
        self._locations = self._load_locations()

    def resolve_location(self, query: str) -> Optional[Dict[str, Any]]:
        normalized = self._normalize(query)

        for location_id, location in self._locations.items():
            if normalized == self._normalize(location_id):
                return {"location_id": location_id, **location}

            for alias in location.get("aliases", []):
                if normalized == self._normalize(alias):
                    return {"location_id": location_id, **location}

        return None

    @staticmethod
    def _normalize(text: str) -> str:
        return " ".join(text.strip().lower().replace("_", " ").split())