"""Pure helpers for versioned semantic map updates.

No rclpy or ROS imports. Safe to use in unit tests without a running node.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
import tempfile
from typing import Optional


@dataclass(frozen=True)
class SemanticMapUpdateResult:
    new_map_path: str
    displaced_object_key: str
    previous_map_path: str


def _parse_object_key(object_key: str) -> Optional[tuple]:
    tag, sep, id_str = object_key.partition(":")
    if not sep:
        return None
    tag = tag.strip()
    if not tag:
        return None
    try:
        object_id = int(id_str)
    except ValueError:
        return None
    return tag, object_id


def write_displaced_semistatic_map(
    *,
    map_path: str,
    object_key: str,
    object_state: str,
    reason: str = "suspected_semi_static_displacement_from_inferred_blockage",
    now: Optional[datetime] = None,
) -> Optional[SemanticMapUpdateResult]:
    """Write a versioned map with a semi-static object marked displaced.

    Only acts when object_state == "semi-static". Returns None for any other
    state, missing key, or if the key is not found in the map.
    The original map file is never modified — the write is atomic via rename.
    """
    if object_state != "semi-static":
        return None
    if not object_key:
        return None
    if not map_path or not os.path.exists(map_path):
        return None

    parsed = _parse_object_key(object_key)
    if parsed is None:
        return None

    target_tag, target_id = parsed

    with open(map_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if not isinstance(data, dict):
        return None

    timestamp_now = now or datetime.now(timezone.utc)
    displaced_at = timestamp_now.isoformat()

    updated = False
    for record in data.values():
        if not isinstance(record, dict):
            continue

        record_tag = str(record.get("object_tag", "")).strip()
        try:
            record_id = int(record.get("id", -1))
        except (TypeError, ValueError):
            continue

        if record_tag == target_tag and record_id == target_id:
            record["object_state"] = "displaced"
            record["displaced_reason"] = reason
            record["displaced_at"] = displaced_at
            record["displaced_from_map"] = os.path.basename(map_path)
            updated = True
            break

    if not updated:
        return None

    timestamp = timestamp_now.strftime("%Y%m%dT%H%M%S_%fZ")
    dir_path = os.path.dirname(map_path)
    new_path = os.path.join(dir_path, f"map_v001_{timestamp}.json")

    fd, tmp_path = tempfile.mkstemp(
        prefix=".map_v001_",
        suffix=".json.tmp",
        dir=dir_path,
        text=True,
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
            f.write("\n")
        os.replace(tmp_path, new_path)
    except Exception:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass
        raise

    return SemanticMapUpdateResult(
        new_map_path=new_path,
        displaced_object_key=object_key,
        previous_map_path=map_path,
    )
