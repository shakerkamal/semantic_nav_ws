"""ROS 2 node serving /refresh_local_objects from a SemanticStore.

BT-LR M1 local semantic context provider, extended in M5B with a dynamic
TTL overlay for short-lived observations (humans, animals, obstacles).
"""

from __future__ import annotations

import json
import os
import threading
import time
from typing import Mapping, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Vector3
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from semantic_nav_interfaces.msg import (
    DoorStateArray,
    DynamicObjectArray,
    ObjectInstance,
    SemanticMapUpdate,
    SemanticStoreUpdated,
)
from semantic_nav_interfaces.srv import (
    InferAffordance,
    QuerySemanticRegion,
    RefreshLocalObjects,
)
from semantic_nav_semantics.affordance_classification import (
    InferredAffordance,
    accept_inference,
    tag_is_classifiable,
)
from semantic_nav_semantics.dynamic_overlay import DynamicObjectCache

from semantic_nav_semantics.semantic_store import (
    ObjectRow,
    SemanticStore,
    load_semantic_store,
    load_semantic_store_from_string,
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

    safety_class, openable, clearable = attributes_for_tag(
        attrs, row.object_tag
    )
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

    # M5B runtime metadata — zero/empty for persistent-map objects.
    msg.source = "persistent_map"
    msg.confidence = 0.0
    msg.ttl_sec = 0.0
    msg.state_detail = ""
    msg.traversability = ""

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
        self.declare_parameter(
            "action_attributes_path", default_action_attrs_path
        )
        self.declare_parameter("service_name", "/refresh_local_objects")
        self.declare_parameter("max_radius_m", 8.0)
        self.declare_parameter("provider_query_service", "/semantic_map/query_region")
        self.declare_parameter("provider_query_timeout_sec", 2.0)
        self.declare_parameter("enable_dynamic_overlay", True)
        self.declare_parameter(
            "dynamic_objects_topic", "/semantic_dynamic_objects"
        )
        self.declare_parameter("default_dynamic_ttl_sec", 3.0)
        self.declare_parameter("max_dynamic_ttl_sec", 10.0)
        self.declare_parameter("enable_door_state_overlay", True)
        self.declare_parameter(
            "door_states_topic", "/semantic_door_states"
        )
        self.declare_parameter("default_door_state_ttl_sec", 3.0)
        self.declare_parameter("max_door_state_ttl_sec", 15.0)

        # Open-set affordance inference (spec 21.4) for DYNAMIC (live
        # -perceived) objects. A live detector can report ANY tag, not just
        # the ones already in object_action_attributes.json (the up-front
        # path already gets this via navigation_orchestrator; en-route never
        # did, leaving a genuinely novel detected tag stuck with restrictive
        # table defaults -- found 2026-07-15, user: "we definitely need this
        # enabled by default since we don't know what object will be
        # detected"). Default True per that explicit direction.
        self.declare_parameter("open_set_inference_enabled", True)
        self.declare_parameter("infer_affordance_service", "/infer_affordance")
        self.declare_parameter("affordance_confidence_floor", 60)
        self.declare_parameter("open_set_timeout_sec", 10.0)

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
        provider_query_service = (
            self.get_parameter("provider_query_service")
            .get_parameter_value()
            .string_value
            .strip()
            or "/semantic_map/query_region"
        )
        self._provider_query_timeout_sec = float(
            self.get_parameter("provider_query_timeout_sec")
            .get_parameter_value()
            .double_value
        )
        enable_dynamic = bool(
            self.get_parameter("enable_dynamic_overlay")
            .get_parameter_value()
            .bool_value
        )
        dynamic_topic = (
            self.get_parameter("dynamic_objects_topic")
            .get_parameter_value()
            .string_value
            .strip()
        )
        default_ttl = float(
            self.get_parameter("default_dynamic_ttl_sec")
            .get_parameter_value()
            .double_value
        )
        max_ttl = float(
            self.get_parameter("max_dynamic_ttl_sec")
            .get_parameter_value()
            .double_value
        )
        enable_door = bool(
            self.get_parameter("enable_door_state_overlay")
            .get_parameter_value()
            .bool_value
        )
        door_topic = (
            self.get_parameter("door_states_topic")
            .get_parameter_value()
            .string_value
            .strip()
        )
        default_door_ttl = float(
            self.get_parameter("default_door_state_ttl_sec")
            .get_parameter_value()
            .double_value
        )
        max_door_ttl = float(
            self.get_parameter("max_door_state_ttl_sec")
            .get_parameter_value()
            .double_value
        )

        self._store: SemanticStore = load_semantic_store(
            map_path=map_path,
            affordances_path=affordances_path,
        )
        self._action_attrs = load_object_action_attributes(action_attrs_path)
        self._store_lock = threading.RLock()
        self._map_path = map_path
        self._affordances_path = affordances_path

        self._dynamic_cache = DynamicObjectCache(
            default_ttl_sec=default_ttl,
            max_ttl_sec=max_ttl,
        )
        self._dynamic_lock = threading.Lock()
        self._enable_dynamic = enable_dynamic

        self._door_state_lock = threading.Lock()
        self._door_states: dict = {}
        self._default_door_state_ttl = default_door_ttl
        self._max_door_state_ttl = max_door_ttl
        self._enable_door = enable_door

        self._semantic_map_version: str = ""

        # ReentrantCallbackGroup allows the provider query client to be called
        # from within the RefreshLocalObjects service callback without deadlock.
        self._reentrant_group = ReentrantCallbackGroup()

        self._provider_query_client = self.create_client(
            QuerySemanticRegion,
            provider_query_service,
            callback_group=self._reentrant_group,
        )
        self._provider_query_service_name = provider_query_service

        infer_affordance_service = (
            self.get_parameter("infer_affordance_service")
            .get_parameter_value()
            .string_value
            .strip()
            or "/infer_affordance"
        )
        self._affordance_confidence_floor = int(
            self.get_parameter("affordance_confidence_floor")
            .get_parameter_value()
            .integer_value
        )
        self._open_set_timeout_sec = float(
            self.get_parameter("open_set_timeout_sec")
            .get_parameter_value()
            .double_value
        )
        self._infer_affordance_client = self.create_client(
            InferAffordance,
            infer_affordance_service,
            callback_group=self._reentrant_group,
        )

        store_update_qos = QoSProfile(depth=1)
        store_update_qos.reliability = ReliabilityPolicy.RELIABLE
        store_update_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._store_updated_sub = self.create_subscription(
            SemanticStoreUpdated,
            "/semantic_store_updated",
            self._handle_store_updated,
            store_update_qos,
        )
        self._provider_map_sub = self.create_subscription(
            SemanticMapUpdate,
            "/semantic_map/updates",
            self._handle_provider_map_update,
            store_update_qos,
        )

        self._dynamic_sub = None
        if self._enable_dynamic:
            self._dynamic_sub = self.create_subscription(
                DynamicObjectArray,
                dynamic_topic,
                self._handle_dynamic_objects,
                10,
            )

        self._door_state_sub = None
        if self._enable_door:
            self._door_state_sub = self.create_subscription(
                DoorStateArray,
                door_topic,
                self._handle_door_states,
                10,
            )

        self._service = self.create_service(
            RefreshLocalObjects,
            service_name,
            self._handle_refresh_local_objects,
            callback_group=self._reentrant_group,
        )

        self.get_logger().info(
            "LocalObjectQueryNode initialized: "
            f"service='{service_name}', "
            f"map_path='{map_path}', "
            f"objects={len(self._store.by_object_key)}, "
            f"db_version={self._store.db_version}, "
            f"max_radius_m={self._max_radius_m:.2f}, "
            f"dynamic_overlay={self._enable_dynamic}, "
            f"dynamic_topic='{dynamic_topic}', "
            f"provider_query_service='{provider_query_service}', "
            f"provider_query_timeout_sec={self._provider_query_timeout_sec:.1f}"
        )

    def _handle_store_updated(self, msg: SemanticStoreUpdated) -> None:
        new_path = msg.semantic_map_uri.strip()
        if not new_path:
            self.get_logger().warn(
                "LocalObjectQueryNode: SemanticStoreUpdated with empty "
                "semantic_map_uri. Ignoring."
            )
            return
        try:
            new_store = load_semantic_store(
                map_path=new_path,
                affordances_path=self._affordances_path,
            )
        except Exception as exc:
            self.get_logger().error(
                f"LocalObjectQueryNode: Failed to reload SemanticStore "
                f"from '{new_path}': {exc}"
            )
            return
        with self._store_lock:
            self._store = new_store
            self._map_path = new_path
            self._semantic_map_version = (msg.semantic_map_version or "").strip()
        self.get_logger().info(
            f"LocalObjectQueryNode: Reloaded SemanticStore from '{new_path}': "
            f"objects={len(new_store.by_object_key)}, "
            f"db_version={new_store.db_version}"
        )

    def _handle_provider_map_update(self, msg: SemanticMapUpdate) -> None:
        if not msg.json_payload.strip():
            self.get_logger().warn(
                "LocalObjectQueryNode: SemanticMapUpdate received with "
                "empty json_payload. Ignoring."
            )
            return
        try:
            new_store = load_semantic_store_from_string(
                json_payload=msg.json_payload,
                semantic_map_version=msg.semantic_map_version,
                stamp=msg.header.stamp,
                affordances_path=self._affordances_path,
            )
        except Exception as exc:
            self.get_logger().error(
                f"LocalObjectQueryNode: Failed to load provider SemanticMapUpdate "
                f"(version='{msg.semantic_map_version}'): {exc}"
            )
            return
        with self._store_lock:
            self._store = new_store
            self._map_path = msg.semantic_map_version or "<provider>"
            self._semantic_map_version = (msg.semantic_map_version or "").strip()
        self.get_logger().info(
            f"LocalObjectQueryNode: Loaded provider map: "
            f"version='{msg.semantic_map_version}', "
            f"objects={len(new_store.by_object_key)}, "
            f"db_version={new_store.db_version}"
        )

    def _handle_dynamic_objects(self, msg: DynamicObjectArray) -> None:
        # Classification (including a possible open-set /infer_affordance
        # round trip, up to open_set_timeout_sec) happens OUTSIDE the
        # _dynamic_lock critical section -- holding that lock during a slow
        # LLM call would stall /refresh_local_objects for every OTHER
        # concurrent query for the whole round trip, not just this ingestion.
        classified = []
        for obs in msg.observations:
            obj = obs.object
            key = obj.object_key.strip()
            if not key:
                continue
            # Tag so downstream knows this came from the overlay.
            obj.source = "dynamic_overlay"
            # Affordances are the semantic layer's judgment, not the
            # detector's: a provider reports WHAT it perceives (tag,
            # caption, state, geometry); openable/clearable/safety come
            # from the same table that classifies persistent-map objects
            # (parity with row_to_object_instance and the up-front flow),
            # falling back to open-set LLM inference for a tag the table
            # cannot classify (spec 21.4) -- previously up-front only.
            safety_class, openable, clearable = self._classify_dynamic_object(
                obj.object_tag, obj.object_caption
            )
            obj.safety_class = safety_class
            obj.openable = openable
            obj.clearable = clearable
            classified.append(obj)

        now_sec = self.get_clock().now().nanoseconds / 1e9
        added = 0
        with self._dynamic_lock:
            for obj in classified:
                self._dynamic_cache.update(
                    object_key=obj.object_key.strip(),
                    center_x=float(obj.bbox_center.x),
                    center_y=float(obj.bbox_center.y),
                    ttl_sec=float(obj.ttl_sec),
                    payload=obj,
                    now_sec=now_sec,
                )
                added += 1
        self.get_logger().debug(
            f"[LOCAL_CONTEXT] dynamic overlay: ingested {added} observations, "
            f"cache_size={len(self._dynamic_cache)}"
        )

    def _handle_door_states(self, msg: DoorStateArray) -> None:
        if not self._enable_door:
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        with self._door_state_lock:
            for obs in msg.observations:
                key = (obs.object_key or "").strip()
                if not key:
                    continue
                ttl = float(obs.ttl_sec)
                if ttl <= 0.0:
                    ttl = self._default_door_state_ttl
                ttl = min(max(ttl, 0.1), self._max_door_state_ttl)
                self._door_states[key] = (obs, now_sec + ttl)

    def _apply_door_state_overlay(self, objects: list) -> list:
        """Mutate state_detail/traversability/openable on known mapped doors."""
        if not self._enable_door:
            return objects
        now_sec = self.get_clock().now().nanoseconds / 1e9
        with self._door_state_lock:
            expired = [
                k for k, (_, exp) in self._door_states.items()
                if exp <= now_sec
            ]
            for k in expired:
                del self._door_states[k]
            for obj in objects:
                key = (obj.object_key or "").strip()
                entry = self._door_states.get(key)
                if entry is None:
                    continue
                obs, _ = entry
                obj.source = "persistent_map+door_state_overlay"
                obj.state_detail = obs.door_state
                obj.traversability = obs.traversability
                obj.openable = bool(obs.robot_openable)
                obj.confidence = float(obs.confidence)
                obj.observation_stamp = obs.header.stamp
                obj.ttl_sec = float(obs.ttl_sec)
        return objects

    def _query_provider_region(
        self,
        center_x: float,
        center_y: float,
        base_map_version: str,
        recovery_event_id: str,
    ) -> "QuerySemanticRegion.Response | None":
        """Call the provider's QuerySemanticRegion service synchronously.

        The robot sends only the query center; the provider decides what
        constitutes "local" objects and returns them. Falls back to the local
        store if the provider is unavailable or times out.
        """
        if not self._provider_query_client.service_is_ready():
            self.get_logger().debug(
                f"[LOCAL_CONTEXT] Provider query service "
                f"'{self._provider_query_service_name}' not available — "
                "using local store."
            )
            return None

        req = QuerySemanticRegion.Request()
        req.query_center.x = center_x
        req.query_center.y = center_y
        req.query_center.z = 0.0
        req.frame_id = "map"
        req.base_map_version = base_map_version or ""
        req.include_displaced = True
        req.recovery_event_id = recovery_event_id or ""

        try:
            future = self._provider_query_client.call_async(req)
            # Poll instead of spin_until_future_complete — calling the latter
            # from within a service callback can deadlock even on a
            # MultiThreadedExecutor. The executor's other threads process the
            # response while this thread sleeps.
            deadline = time.monotonic() + self._provider_query_timeout_sec
            while not future.done():
                if time.monotonic() >= deadline:
                    break
                time.sleep(0.005)
        except Exception as exc:
            self.get_logger().warn(
                f"[LOCAL_CONTEXT] Provider query failed with exception: {exc}"
            )
            return None

        if not future.done():
            self.get_logger().warn(
                f"[LOCAL_CONTEXT] Provider query timed out after "
                f"{self._provider_query_timeout_sec:.1f} s — using local store."
            )
            return None

        result = future.result()
        if result is None or not result.success:
            msg = result.message if result else "no result"
            self.get_logger().warn(
                f"[LOCAL_CONTEXT] Provider query returned failure: {msg} — "
                "using local store."
            )
            return None

        return result

    def _open_set_inference_enabled(self) -> bool:
        """Read live (not cached at init) so `ros2 param set` can flip this
        ablation switch without a relaunch, matching the orchestrator's own
        up-front switch of the same name."""
        return bool(
            self.get_parameter("open_set_inference_enabled")
            .get_parameter_value()
            .bool_value
        )

    def _infer_affordance(self, tag: str, caption: str) -> "InferredAffordance | None":
        """Call /infer_affordance synchronously; None on any failure (caller
        falls back to the table default). Same poll pattern as
        _query_provider_region — no spin_until_future_complete, since this
        can be invoked from within the dynamic-objects subscription callback
        on a MultiThreadedExecutor."""
        if not self._infer_affordance_client.service_is_ready():
            self.get_logger().warn(
                "[LOCAL_CONTEXT] infer_affordance service unavailable; "
                "using table default."
            )
            return None

        req = InferAffordance.Request()
        req.object_tag = tag or ""
        req.object_caption = caption or ""

        try:
            future = self._infer_affordance_client.call_async(req)
            deadline = time.monotonic() + self._open_set_timeout_sec
            while not future.done():
                if time.monotonic() >= deadline:
                    break
                time.sleep(0.005)
        except Exception as exc:
            self.get_logger().warn(
                f"[LOCAL_CONTEXT] infer_affordance call failed: {exc}"
            )
            return None

        if not future.done():
            self.get_logger().warn(
                f"[LOCAL_CONTEXT] infer_affordance timed out after "
                f"{self._open_set_timeout_sec:.1f} s; using table default."
            )
            return None

        resp = future.result()
        if resp is None or not resp.success:
            return None

        inf = InferredAffordance(
            bool(resp.openable), bool(resp.clearable),
            str(resp.safety_class or "none"), int(resp.confidence_percent),
        )
        self.get_logger().info(
            f"[LOCAL_CONTEXT] open-set affordance inferred for tag='{tag}': "
            f"openable={inf.openable} clearable={inf.clearable} "
            f"safety={inf.safety_class} conf={inf.confidence}"
        )
        if not accept_inference(inf, self._affordance_confidence_floor):
            self.get_logger().info(
                "[LOCAL_CONTEXT] inference below confidence floor; "
                "using table default."
            )
            return None
        return inf

    def _classify_dynamic_object(self, tag: str, caption: str) -> Tuple[str, bool, bool]:
        """Table lookup first; open-set LLM inference ONLY for a tag the
        table cannot classify (spec 21.4) -- a live detector can report ANY
        tag, unlike the fixed set of persistent-map objects."""
        by_tag = self._action_attrs.get("by_tag", {})
        table_tags = set(by_tag.keys()) if isinstance(by_tag, dict) else set()

        if (self._open_set_inference_enabled()
                and not tag_is_classifiable(tag, table_tags)):
            inferred = self._infer_affordance(tag, caption)
            if inferred is not None:
                return inferred.safety_class, inferred.openable, inferred.clearable

        return attributes_for_tag(self._action_attrs, tag)

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

        # use blockage_centroid when provided, else fall back to robot_pose
        if _point_is_effectively_zero(request.blockage_centroid):
            center_x = float(request.robot_pose.pose.position.x)
            center_y = float(request.robot_pose.pose.position.y)
            center_source = "robot_pose"
        else:
            center_x = float(request.blockage_centroid.x)
            center_y = float(request.blockage_centroid.y)
            center_source = "blockage_centroid"

        # --- Interface 2: query provider for fresh regional data ---
        with self._store_lock:
            current_version = self._semantic_map_version
            store = self._store

        provider_result = None
        if radius > 0.0:
            provider_result = self._query_provider_region(
                center_x=center_x,
                center_y=center_y,
                base_map_version=request.base_map_version or current_version,
                recovery_event_id="",
            )

        if provider_result is not None:
            # Provider returned fresh data — parse it and apply door overlay.
            try:
                fresh_store = load_semantic_store_from_string(
                    json_payload=provider_result.json_payload,
                    semantic_map_version=provider_result.semantic_map_version,
                    stamp=provider_result.db_stamp,
                    affordances_path=self._affordances_path,
                )
                rows = fresh_store.query_window(
                    center_xy=(center_x, center_y),
                    radius_m=radius,
                )
                static_objects = self._apply_door_state_overlay([
                    row_to_object_instance(row, self._action_attrs)
                    for row in rows
                ])
                effective_db_version = int(fresh_store.db_version)
                effective_db_stamp = fresh_store.db_stamp
                effective_map_version = provider_result.semantic_map_version
                data_source = "provider_regional"
            except Exception as exc:
                self.get_logger().warn(
                    f"[LOCAL_CONTEXT] Failed to parse provider regional response: {exc}. "
                    "Falling back to local store."
                )
                provider_result = None

        if provider_result is None:
            # Fallback: serve from in-memory local store.
            rows = store.query_window(
                center_xy=(center_x, center_y),
                radius_m=radius,
            )
            static_objects = self._apply_door_state_overlay([
                row_to_object_instance(row, self._action_attrs)
                for row in rows
            ])
            effective_db_version = int(store.db_version)
            effective_db_stamp = store.db_stamp
            effective_map_version = current_version
            data_source = "local_store"

        dynamic_objects: list = []
        if self._enable_dynamic and radius > 0.0:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            with self._dynamic_lock:
                dynamic_objects = self._dynamic_cache.objects_in_radius(
                    center_x=center_x,
                    center_y=center_y,
                    radius_m=radius,
                    now_sec=now_sec,
                )

        source_tag = (
            "hybrid_provider" if (provider_result and dynamic_objects)
            else "hybrid" if dynamic_objects
            else data_source
        )

        response.objects = static_objects + dynamic_objects
        response.db_version = effective_db_version
        response.db_stamp = effective_db_stamp
        response.semantic_map_version = effective_map_version
        response.source = source_tag
        response.message = (
            f"returned {len(response.objects)} objects "
            f"({len(static_objects)} static, {len(dynamic_objects)} dynamic) "
            f"within {radius:.2f} m "
            f"around {center_source}=({center_x:.3f}, {center_y:.3f}), "
            f"data_source={data_source}"
        )

        self.get_logger().info(
            "[LOCAL_CONTEXT] "
            f"center_source={center_source}, "
            f"center=({center_x:.3f}, {center_y:.3f}), "
            f"radius={radius:.2f}, "
            f"static={len(static_objects)}, "
            f"dynamic={len(dynamic_objects)}, "
            f"data_source={data_source}, "
            f"source='{source_tag}', "
            f"db_version={effective_db_version}"
        )

        return response


def main(args=None):
    rclpy.init(args=args)

    node = LocalObjectQueryNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info(
            "Keyboard interrupt received. "
            "Shutting down local object query node."
        )
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
