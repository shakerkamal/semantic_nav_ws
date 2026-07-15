"""Shared pure helpers for the en-route ablation harness scripts."""
import math
from typing import Optional, Tuple

import yaml


def load_scenarios(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def planar_dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def robot_map_pose(tf_buffer) -> Optional[Tuple[float, float]]:
    """map->base_footprint translation, or None while TF is not ready."""
    try:
        import rclpy.time
        t = tf_buffer.lookup_transform(
            "map", "base_footprint", rclpy.time.Time())
        return (t.transform.translation.x, t.transform.translation.y)
    except Exception:
        return None
