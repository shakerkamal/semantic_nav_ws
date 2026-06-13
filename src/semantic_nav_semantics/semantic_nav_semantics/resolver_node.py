import math
import os
from typing import Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped, Vector3
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from semantic_nav_interfaces.srv import ResolveLocation
from semantic_nav_semantics.caption_ranker import RankedObject
from semantic_nav_semantics.semantic_store import (
    SemanticStore,
    load_semantic_store,
    looks_like_object_key,
    normalize_object_key,
)
from semantic_nav_semantics.standoff_planner import StandoffPlanner, StandoffPose


def _yaw_to_quaternion(yaw: float) -> Tuple[float, float]:
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


class ResolverNode(Node):
    def __init__(self):
        super().__init__("resolver_node")

        default_map = os.path.join(
            get_package_share_directory("semantic_nav_semantics"),
            "config", "map_v001.json",
        )
        default_sidecar = os.path.join(
            get_package_share_directory("semantic_nav_semantics"),
            "config", "object_intent_affordances.json",
        )

        self.declare_parameter("semantic_map_path", default_map)
        self.declare_parameter("intent_affordances_path", default_sidecar)
        self.declare_parameter("resolve_service", "/resolve_location")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("require_robot_pose_for_standoff", True)
        self.declare_parameter("robot_footprint_radius", 0.22)
        self.declare_parameter("clearance_margin", 0.20)
        self.declare_parameter("ranker", "bm25")
        self.declare_parameter("spatial_context", False)
        self.declare_parameter("hybrid_tiebreak_delta", 0.5)
        self.declare_parameter("hybrid_top_k", 4)

        map_path = self.get_parameter("semantic_map_path").get_parameter_value().string_value
        sidecar_path = (
            self.get_parameter("intent_affordances_path").get_parameter_value().string_value
        )
        resolve_service = self.get_parameter("resolve_service").get_parameter_value().string_value
        self._robot_frame = self.get_parameter("robot_frame").get_parameter_value().string_value
        self._map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self._require_pose = bool(self.get_parameter("require_robot_pose_for_standoff").value)
        radius = float(self.get_parameter("robot_footprint_radius").value)
        clearance = float(self.get_parameter("clearance_margin").value)
        ranker_name = self.get_parameter("ranker").get_parameter_value().string_value
        self._spatial_on = bool(self.get_parameter("spatial_context").value)
        delta = float(self.get_parameter("hybrid_tiebreak_delta").value)
        top_k = int(self.get_parameter("hybrid_top_k").value)

        # Callback groups for MultiThreadedExecutor.
        # _srv_cb_group:   ReentrantCallbackGroup lets a second resolve request
        #                  be accepted while a slow LLM call is still in flight.
        # _llama_cb_group: ReentrantCallbackGroup lets the ActionClient's goal-
        #                  response and result callbacks fire on the executor's
        #                  thread pool while the service callback polls.
        self._srv_cb_group = ReentrantCallbackGroup()
        self._llama_cb_group = ReentrantCallbackGroup()

        self._store: SemanticStore = load_semantic_store(
            map_path, affordances_path=sidecar_path
        )
        from semantic_nav_semantics.ranker_factory import RankerSpec, build_ranker
        self._llama_client = self._maybe_build_llama_client(ranker_name)
        self._ranker = build_ranker(
            RankerSpec(name=ranker_name, delta=delta, top_k=top_k),
            affordances=self._store.affordances,
            llama_client=self._llama_client,
        )
        self._spatial_builder = None
        if self._spatial_on:
            from semantic_nav_semantics.spatial_context import SpatialContextBuilder
            self._spatial_builder = SpatialContextBuilder()
            self.get_logger().info("[RESOLVER] spatial_context=ON")
        self._standoff = StandoffPlanner(
            robot_footprint_radius=radius, clearance_margin=clearance,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.srv = self.create_service(
            ResolveLocation, resolve_service, self._handle_resolve,
            callback_group=self._srv_cb_group,
        )

        self.get_logger().info(
            f"[RESOLUTION] Ready. map='{map_path}', objects={len(self._store.by_object_key)}, "
            f"navigable_tags={len(self._store.navigable_tag_vocabulary)}, "
            f"db_version={self._store.db_version}"
        )

    # ----- helpers -----

    def _robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception as exc:
            self.get_logger().warn(f"[RESOLUTION] TF lookup failed: {exc}")
            return None

    def _fail(self, response, message: str):
        response.success = False
        response.object_key = ""
        response.source_key = ""
        response.object_tag = ""
        response.selected_caption = ""
        response.bbox_center = Point()
        response.bbox_extent = Vector3()
        response.candidates_considered = 0
        response.top_score = 0.0
        response.pose = PoseStamped()
        response.db_version = self._store.db_version
        response.db_stamp = self._store.db_stamp
        response.message = message
        self.get_logger().warn(f"[RESOLUTION] FAIL: {message}")
        return response

    # ----- main handler -----

    def _handle_resolve(self, request, response):
        robot_xy = self._robot_xy()
        if robot_xy is None and self._require_pose:
            return self._fail(
                response, "robot pose unavailable and require_robot_pose_for_standoff=true"
            )
        if robot_xy is None:
            robot_xy = (0.0, 0.0)
            self.get_logger().warn("[RESOLUTION] Using (0,0) as robot pose fallback")

        # Precedence 1: explicit object key
        if request.target_object_key:
            key = normalize_object_key(request.target_object_key)
            row = self._store.by_object_key.get(key)
            if row is None:
                return self._fail(
                    response, f"unknown target_object_key '{request.target_object_key}'"
                )
            ranked = [self._direct_pick(row, reason="direct object key")]

        # Precedence 2: object_tag + intent_hint (object-centric path)
        elif request.object_tag:
            ranked = self._rank_by_tag(request.object_tag, request.intent_hint, robot_xy)
            if isinstance(ranked, str):
                return self._fail(response, ranked)

        # Precedence 3: legacy/direct query (object_key | tag | alias)
        elif request.query:
            q = request.query.strip()
            if looks_like_object_key(q):
                row = self._store.by_object_key.get(normalize_object_key(q))
                if row is None:
                    return self._fail(response, f"unknown object key query '{q}'")
                ranked = [self._direct_pick(row, reason="direct query object key")]
            else:
                resolved = self._store.affordances.resolve_alias(q)
                if resolved in self._store.by_tag:
                    ranked = self._rank_by_tag(resolved, q, robot_xy)
                    if isinstance(ranked, str):
                        return self._fail(response, ranked)
                else:
                    return self._fail(
                        response,
                        f"query '{q}' did not match object key, object tag, or alias"
                    )
        else:
            return self._fail(response, "empty semantic resolution request")

        pick = ranked[0]
        pose = self._standoff.plan(pick.row, robot_xy=robot_xy)
        return self._fill_success(response, pick=pick, pose=pose, candidates=len(ranked))

    def _rank_by_tag(self, tag_query: str, hint: str, robot_xy):
        rows = self._store.rows_for_tag(tag_query)
        if not rows:
            norm = self._store.affordances.resolve_alias(tag_query)
            return f"no candidates for object_tag '{tag_query}' (resolved='{norm}')"
        if self._spatial_builder is not None:
            rows = self._with_spatial_context(rows, robot_xy)
        return self._ranker.rank(
            rows, intent_hint=hint, robot_xy=robot_xy, user_command=hint,
        )

    def _maybe_build_llama_client(self, ranker_name: str):
        if ranker_name == "bm25":
            return None
        try:
            from llama_msgs.action import GenerateResponse
            from rclpy.action import ActionClient
            from semantic_nav_semantics.llama_action_client import LlamaActionClient
        except ImportError as exc:
            self.get_logger().error(
                f"ranker={ranker_name} requires llama_msgs but import failed: {exc}"
            )
            return None
        # _llama_cb_group (ReentrantCallbackGroup) lets the ActionClient's
        # goal-response and result callbacks be dispatched by the executor's
        # thread pool while the service callback is polling future.done().
        ac = ActionClient(
            self, GenerateResponse, "/llama/generate_response",
            callback_group=self._llama_cb_group,
        )
        # executor_is_running=True: service callback is inside a
        # MultiThreadedExecutor; polling is correct here — do NOT call
        # rclpy.spin_until_future_complete (that would add the node to a
        # second executor causing double-dispatch).
        return LlamaActionClient(
            action_client=ac,
            logger=self.get_logger(),
            node=self,
            executor_is_running=True,
        )

    def _with_spatial_context(self, rows, robot_xy):
        from dataclasses import replace
        all_rows = list(self._store.by_object_key.values())
        navigable = set(self._store.navigable_tag_vocabulary)
        augmented = []
        for r in rows:
            suffix = self._spatial_builder.build(
                r, all_rows, robot_xy=robot_xy, navigable_tags=navigable,
            )
            augmented.append(replace(r, object_caption=f"{r.object_caption} | {suffix}"))
        return augmented

    def _direct_pick(self, row, reason: str) -> RankedObject:
        return RankedObject(
            row=row, score=0.0, lexical_score=0.0, affordance_score=0.0,
            caption_tag_bonus=0.0, caption_boost_bonus=0.0,
            volume_bonus=0.0, distance_bonus=0.0, conflict_penalty=0.0,
            reasons=(reason,),
        )

    def _fill_success(self, response, pick: RankedObject, pose: StandoffPose, candidates: int):
        row = pick.row
        ps = PoseStamped()
        ps.header.frame_id = self._map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(pose.position_xy[0])
        ps.pose.position.y = float(pose.position_xy[1])
        ps.pose.position.z = 0.0
        qz, qw = _yaw_to_quaternion(pose.yaw)
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        bbox_center = Point()
        bbox_center.x, bbox_center.y, bbox_center.z = row.bbox_center
        bbox_extent = Vector3()
        bbox_extent.x, bbox_extent.y, bbox_extent.z = row.bbox_extent

        response.success = True
        response.object_key = row.object_key
        response.source_key = row.source_key
        response.object_tag = row.object_tag
        response.selected_caption = row.object_caption
        response.bbox_center = bbox_center
        response.bbox_extent = bbox_extent
        response.candidates_considered = candidates
        response.top_score = float(pick.score)
        response.pose = ps
        response.db_version = self._store.db_version
        response.db_stamp = self._store.db_stamp
        response.message = (
            f"Resolved object_key='{row.object_key}' from {candidates} candidate(s); "
            f"top_score={pick.score:.3f}; reasons={list(pick.reasons)}"
        )

        self.get_logger().info(
            f"[RETRIEVAL] object_tag='{row.object_tag}' candidates={candidates} "
            f"selected='{row.object_key}' top_score={pick.score:.3f} "
            f"caption='{row.object_caption[:80]}'"
        )
        self.get_logger().info(
            f"[STANDOFF] bbox_center={row.bbox_center} bbox_extent={row.bbox_extent} "
            f"goal=({pose.position_xy[0]:.2f},{pose.position_xy[1]:.2f}) "
            f"yaw={pose.yaw:.3f} d={pose.standoff_distance:.2f}"
        )
        return response


def main():
    rclpy.init()
    node = ResolverNode()
    # MultiThreadedExecutor is required when ranker=llm or ranker=hybrid.
    # The service callback blocks while polling the LLM future; the executor's
    # thread pool processes the ActionClient response callbacks concurrently.
    # It is harmless (just slightly more overhead) when ranker=bm25.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
