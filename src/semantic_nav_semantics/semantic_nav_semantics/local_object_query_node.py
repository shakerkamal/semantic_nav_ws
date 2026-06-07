"""ROS 2 node serving /refresh_local_objects from a static SemanticStore.

BT-LR M1 local semantic context provider.

This node loads the active object-centric semantic map once at startup and
returns a windowed subset around a blockage centroid or robot pose.

In M1, source="static_snapshot" because map_v001.json is not live-updated by
perception yet. This is a local semantic context query, not a full database
refresh.
"""

from __future__ import annotations

import json
import os
from typing import Mapping, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node

from semantic_nav_interfaces.msg import ObjectInstance
from semantic_nav_interfaces.srv import RefreshLocalObjects

from semantic_nav_semantics.semantic_store import (
    ObjectRow,
    SemanticStore,
    load_semantic_store,
    normalize_tag,
)


DEFAULT_SAFETY_CLASS = "none"
DEFAULT_OPENABLE = False
DEFAULT_CLEARABLE = False


def _default_config_path(filename: str) -> str:
    share_dir = get_package_share_directory("semantic_nav_semantics")
    return os.path.join(share_dir, "config", filename)


def load_object_action_attributes(path: str) -> Mapping[str, object]:
    """Load object_action_attributes.json.

    Missing file is allowed for M1. Unknown tags default to:
      safety_class="none", openable=False, clearable=False.

    Supported shapes:
      {
        "defaults": {...},
        "by_tag": {
          "door": {...}
        }
      }

    Also tolerates a direct tag-map shape:
      {
        "door": {...},
        "chair": {...}
      }
    """
    if not path or not os.path.exists(path):
        return {
            "defaults": {
                "safety_class": DEFAULT_SAFETY_CLASS,
                "openable": DEFAULT_OPENABLE,
                "clearable": DEFAULT_CLEARABLE,
            },
            "by_tag": {},
        }

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if not isinstance(data, dict):
        return {
            "defaults": {
                "safety_class": DEFAULT_SAFETY_CLASS,
                "openable": DEFAULT_OPENABLE,
                "clearable": DEFAULT_CLEARABLE,
            },
            "by_tag": {},
        }

    defaults = data.get(
        "defaults",
        {
            "safety_class": DEFAULT_SAFETY_CLASS,
            "openable": DEFAULT_OPENABLE,
            "clearable": DEFAULT_CLEARABLE,
        },
    )
    if not isinstance(defaults, dict):
        defaults = {
            "safety_class": DEFAULT_SAFETY_CLASS,
            "openable": DEFAULT_OPENABLE,
            "clearable": DEFAULT_CLEARABLE,
        }

    by_tag = data.get("by_tag")
    if isinstance(by_tag, dict):
        normalized_by_tag = {
            normalize_tag(tag): attrs
            for tag, attrs in by_tag.items()
            if isinstance(attrs, dict)
        }
    else:
        # Tolerate direct tag-map style.
        normalized_by_tag = {
            normalize_tag(tag): attrs
            for tag, attrs in data.items()
            if tag != "defaults" and isinstance(attrs, dict)
        }

    return {
        "defaults": defaults,
        "by_tag": normalized_by_tag,
    }


def attributes_for_tag(
    attrs: Mapping[str, object],
    tag: str,
) -> Tuple[str, bool, bool]:
    defaults = attrs.get("defaults", {})
    by_tag = attrs.get("by_tag", {})

    if not isinstance(defaults, dict):
        defaults = {}
    if not isinstance(by_tag, dict):
        by_tag = {}

    tag_attrs = by_tag.get(normalize_tag(tag), defaults)
    if not isinstance(tag_attrs, dict):
        tag_attrs = defaults

    safety_class = str(
        tag_attrs.get(
            "safety_class",
            defaults.get("safety_class", DEFAULT_SAFETY_CLASS),
        )
    )
    openable = bool(
        tag_attrs.get(
            "openable",
            defaults.get("openable", DEFAULT_OPENABLE),
        )
    )
    clearable = bool(
        tag_attrs.get(
            "clearable",
            defaults.get("clearable", DEFAULT_CLEARABLE),
        )
    )

    return safety_class, openable, clearable


def row_to_object_instance(
    row: ObjectRow,
    attrs: Mapping[str, object],
) -> ObjectInstance:
    msg = ObjectInstance()

    msg.object_key = row.object_key
    msg.source_key = row.source_key
    msg.object_tag = row.object_tag
    msg.object_caption = row.object_caption
    msg.object_state = row.object_state

    safety_class, openable, clearable = attributes_for_tag(attrs, row.object_tag)
    msg.safety_class = safety_class
    msg.openable = openable
    msg.clearable = clearable

    msg.bbox_center = Point(
        x=float(row.bbox_center[0]),
        y=float(row.bbox_center[1]),
        z=float(row.bbox_center[2]),
    )
    msg.bbox_extent = Vector3(
        x=float(row.bbox_extent[0]),
        y=float(row.bbox_extent[1]),
        z=float(row.bbox_extent[2]),
    )
    msg.bbox_volume = float(row.bbox_volume)

    return msg


def _point_is_effectively_zero(point: Point) -> bool:
    return (
        abs(float(point.x)) < 1e-9
        and abs(float(point.y)) < 1e-9
        and abs(float(point.z)) < 1e-9
    )


class LocalObjectQueryNode(Node):
    """Serve local object windows from SemanticStore.query_window()."""

    def __init__(self) -> None:
        super().__init__("local_object_query_node")

        default_map_path = _default_config_path("map_v001.json")
        default_affordances_path = _default_config_path(
            "object_intent_affordances.json"
        )
        default_action_attrs_path = _default_config_path(
            "object_action_attributes.json"
        )

        self.declare_parameter("map_path", default_map_path)
        self.declare_parameter("affordances_path", default_affordances_path)
        self.declare_parameter("action_attributes_path", default_action_attrs_path)
        self.declare_parameter("service_name", "/refresh_local_objects")
        self.declare_parameter("max_radius_m", 8.0)

        map_path = (
            self.get_parameter("map_path")
            .get_parameter_value()
            .string_value
            .strip()
        )
        affordances_path = (
            self.get_parameter("affordances_path")
            .get_parameter_value()
            .string_value
            .strip()
        )
        action_attrs_path = (
            self.get_parameter("action_attributes_path")
            .get_parameter_value()
            .string_value
            .strip()
        )
        service_name = (
            self.get_parameter("service_name")
            .get_parameter_value()
            .string_value
            .strip()
        )
        self._max_radius_m = float(
            self.get_parameter("max_radius_m")
            .get_parameter_value()
            .double_value
        )

        self._store: SemanticStore = load_semantic_store(
            map_path=map_path,
            affordances_path=affordances_path,
        )
        self._action_attrs = load_object_action_attributes(action_attrs_path)

        self._service = self.create_service(
            RefreshLocalObjects,
            service_name,
            self._handle_refresh_local_objects,
        )

        self.get_logger().info(
            "LocalObjectQueryNode initialized: "
            f"service='{service_name}', "
            f"map_path='{map_path}', "
            f"affordances_path='{affordances_path}', "
            f"action_attributes_path='{action_attrs_path}', "
            f"objects={len(self._store.by_object_key)}, "
            f"db_version={self._store.db_version}, "
            f"max_radius_m={self._max_radius_m:.2f}"
        )

    def _handle_refresh_local_objects(
        self,
        request: RefreshLocalObjects.Request,
        response: RefreshLocalObjects.Response,
    ) -> RefreshLocalObjects.Response:
        requested_radius = float(request.radius_m)

        if requested_radius <= 0.0:
            radius = 0.0
        else:
            radius = min(requested_radius, self._max_radius_m)

        # M1 convention:
        # - use blockage_centroid when provided
        # - otherwise fall back to robot_pose
        #
        # Note: a true blockage at map origin is ambiguous with "unknown".
        # This is acceptable for M1 smoke tests; future versions can add an
        # explicit use_blockage_centroid flag if needed.
        if _point_is_effectively_zero(request.blockage_centroid):
            center_x = float(request.robot_pose.pose.position.x)
            center_y = float(request.robot_pose.pose.position.y)
            center_source = "robot_pose"
        else:
            center_x = float(request.blockage_centroid.x)
            center_y = float(request.blockage_centroid.y)
            center_source = "blockage_centroid"

        rows = self._store.query_window(
            center_xy=(center_x, center_y),
            radius_m=radius,
        )

        response.objects = [
            row_to_object_instance(row, self._action_attrs)
            for row in rows
        ]
        response.db_version = int(self._store.db_version)
        response.db_stamp = self._store.db_stamp
        response.source = "static_snapshot"
        response.message = (
            f"returned {len(response.objects)} objects within {radius:.2f} m "
            f"around {center_source}=({center_x:.3f}, {center_y:.3f})"
        )

        self.get_logger().info(
            "[LOCAL_CONTEXT] "
            f"center_source={center_source}, "
            f"center=({center_x:.3f}, {center_y:.3f}), "
            f"radius={radius:.2f}, "
            f"objects={len(response.objects)}, "
            f"source='{response.source}', "
            f"db_version={response.db_version}"
        )

        return response


def main(args=None):
    rclpy.init(args=args)

    node = LocalObjectQueryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            "Keyboard interrupt received. Shutting down local object query node."
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()