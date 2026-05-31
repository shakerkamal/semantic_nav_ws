import math
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Path

from semantic_nav_interfaces.msg import RecoveryTrigger


class PlanIntersectionMonitor(Node):
    """
    Evidence-only monitor for semantic recovery.

    The node watches the current Nav2 global plan and global costmap. If any
    segment of the plan intersects occupied costmap cells, it publishes a
    RecoveryTrigger. It never cancels Nav2, never clears costmaps, and never
    calls /propose_recovery.
    """

    def __init__(self):
        super().__init__('plan_intersection_monitor')

        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('recovery_trigger_topic', '/recovery_trigger')
        self.declare_parameter('global_frame', 'map')

        self.declare_parameter('occupied_threshold', 90)
        self.declare_parameter('treat_unknown_as_blocked', False)
        self.declare_parameter('sample_radius_m', 0.0)
        self.declare_parameter('min_blocked_poses', 1)
        self.declare_parameter('max_plan_poses', 800)
        self.declare_parameter('debounce_sec', 1.0)
        self.declare_parameter('publish_debug_logs', True)

        self._plan_topic = self.get_parameter('plan_topic').get_parameter_value().string_value
        self._costmap_topic = self.get_parameter('costmap_topic').get_parameter_value().string_value
        self._trigger_topic = self.get_parameter('recovery_trigger_topic').get_parameter_value().string_value
        self._global_frame = self.get_parameter('global_frame').get_parameter_value().string_value

        self._occupied_threshold = int(
            self.get_parameter('occupied_threshold').get_parameter_value().integer_value
        )
        self._treat_unknown_as_blocked = bool(
            self.get_parameter('treat_unknown_as_blocked').get_parameter_value().bool_value
        )
        self._sample_radius_m = float(
            self.get_parameter('sample_radius_m').get_parameter_value().double_value
        )
        self._min_blocked_poses = max(
            1,
            int(self.get_parameter('min_blocked_poses').get_parameter_value().integer_value),
        )
        self._max_plan_poses = max(
            1,
            int(self.get_parameter('max_plan_poses').get_parameter_value().integer_value),
        )
        self._debounce_sec = float(
            self.get_parameter('debounce_sec').get_parameter_value().double_value
        )
        self._publish_debug_logs = bool(
            self.get_parameter('publish_debug_logs').get_parameter_value().bool_value
        )

        self._latest_plan: Optional[Path] = None
        self._latest_costmap: Optional[OccupancyGrid] = None
        self._last_publish_by_key = {}

        self._trigger_pub = self.create_publisher(
            RecoveryTrigger,
            self._trigger_topic,
            10,
        )

        self._plan_sub = self.create_subscription(
            Path,
            self._plan_topic,
            self._on_plan,
            10,
        )
        self._costmap_sub = self.create_subscription(
            OccupancyGrid,
            self._costmap_topic,
            self._on_costmap,
            10,
        )

        self.get_logger().info(
            'PlanIntersectionMonitor initialized: '
            f"plan_topic='{self._plan_topic}', "
            f"costmap_topic='{self._costmap_topic}', "
            f"recovery_trigger_topic='{self._trigger_topic}', "
            f"occupied_threshold={self._occupied_threshold}, "
            f"sample_radius_m={self._sample_radius_m:.3f}, "
            f"debounce_sec={self._debounce_sec:.3f}"
        )

    def _on_plan(self, msg: Path) -> None:
        self._latest_plan = msg
        self._evaluate_and_publish()

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self._latest_costmap = msg
        self._evaluate_and_publish()

    def _evaluate_and_publish(self) -> None:
        if self._latest_plan is None or self._latest_costmap is None:
            return

        if not self._frames_are_compatible(self._latest_plan, self._latest_costmap):
            return

        run = self._find_first_blocked_run(
            plan=self._latest_plan,
            costmap=self._latest_costmap,
        )
        if run is None:
            return

        index_lo, index_hi, centroid, extent_m = run
        debounce_key = self._make_debounce_key(index_lo, index_hi, centroid)

        if self._is_debounced(debounce_key):
            return

        msg = RecoveryTrigger()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._latest_costmap.header.frame_id or self._global_frame
        msg.trigger_source = 'plan_intersection_monitor'
        msg.responsible_object_key = ''
        msg.match_type = 'unknown'
        msg.blockage_centroid = centroid
        msg.blocked_plan_index_lo = int(index_lo)
        msg.blocked_plan_index_hi = int(index_hi)
        msg.blockage_extent_m = float(extent_m)
        msg.debounce_key = debounce_key
        msg.note = (
            'global plan intersects occupied global costmap cells; '
            'evidence only, orchestrator owns recovery'
        )

        self._trigger_pub.publish(msg)

        if self._publish_debug_logs:
            self.get_logger().warn(
                '[RECOVERY/MONITOR] Published RecoveryTrigger: '
                f'indices={index_lo}-{index_hi}, '
                f'centroid=({centroid.x:.2f}, {centroid.y:.2f}), '
                f'extent_m={extent_m:.2f}, key={debounce_key}'
            )

    def _frames_are_compatible(self, plan: Path, costmap: OccupancyGrid) -> bool:
        plan_frame = plan.header.frame_id or self._global_frame
        costmap_frame = costmap.header.frame_id or self._global_frame

        if plan_frame != costmap_frame:
            self.get_logger().warn(
                '[RECOVERY/MONITOR] Ignoring plan/costmap with different frames: '
                f"plan='{plan_frame}', costmap='{costmap_frame}'."
            )
            return False

        return True

    def _find_first_blocked_run(
        self,
        plan: Path,
        costmap: OccupancyGrid,
    ) -> Optional[Tuple[int, int, Point, float]]:
        poses = list(plan.poses[:self._max_plan_poses])
        if not poses:
            return None

        blocked_indices: List[int] = []

        for idx, pose_stamped in enumerate(poses):
            x = float(pose_stamped.pose.position.x)
            y = float(pose_stamped.pose.position.y)

            if self._is_world_point_blocked(costmap, x, y):
                blocked_indices.append(idx)

        if not blocked_indices:
            return None

        runs = self._contiguous_runs(blocked_indices)
        for index_lo, index_hi in runs:
            run_len = index_hi - index_lo + 1
            if run_len < self._min_blocked_poses:
                continue

            centroid = self._centroid_for_plan_range(poses, index_lo, index_hi)
            extent_m = self._arc_length_for_plan_range(poses, index_lo, index_hi)
            return index_lo, index_hi, centroid, extent_m

        return None

    @staticmethod
    def _contiguous_runs(indices: List[int]) -> List[Tuple[int, int]]:
        if not indices:
            return []

        runs = []
        start = indices[0]
        previous = indices[0]

        for index in indices[1:]:
            if index == previous + 1:
                previous = index
                continue

            runs.append((start, previous))
            start = index
            previous = index

        runs.append((start, previous))
        return runs

    def _is_world_point_blocked(
        self,
        costmap: OccupancyGrid,
        x: float,
        y: float,
    ) -> bool:
        cell = self._world_to_map(costmap, x, y)
        if cell is None:
            return False

        mx, my = cell
        resolution = float(costmap.info.resolution)
        radius_cells = 0
        if resolution > 0.0 and self._sample_radius_m > 0.0:
            radius_cells = int(math.ceil(self._sample_radius_m / resolution))

        for cy in range(my - radius_cells, my + radius_cells + 1):
            for cx in range(mx - radius_cells, mx + radius_cells + 1):
                if self._is_cell_blocked(costmap, cx, cy):
                    return True

        return False

    @staticmethod
    def _world_to_map(
        costmap: OccupancyGrid,
        x: float,
        y: float,
    ) -> Optional[Tuple[int, int]]:
        info = costmap.info
        resolution = float(info.resolution)
        width = int(info.width)
        height = int(info.height)

        if resolution <= 0.0 or width <= 0 or height <= 0:
            return None

        origin_x = float(info.origin.position.x)
        origin_y = float(info.origin.position.y)

        mx = int(math.floor((x - origin_x) / resolution))
        my = int(math.floor((y - origin_y) / resolution))

        if mx < 0 or my < 0 or mx >= width or my >= height:
            return None

        return mx, my

    def _is_cell_blocked(self, costmap: OccupancyGrid, mx: int, my: int) -> bool:
        width = int(costmap.info.width)
        height = int(costmap.info.height)

        if mx < 0 or my < 0 or mx >= width or my >= height:
            return False

        index = my * width + mx
        if index < 0 or index >= len(costmap.data):
            return False

        value = int(costmap.data[index])
        if value < 0:
            return self._treat_unknown_as_blocked

        return value >= self._occupied_threshold

    @staticmethod
    def _centroid_for_plan_range(poses, index_lo: int, index_hi: int) -> Point:
        count = max(1, index_hi - index_lo + 1)
        sx = 0.0
        sy = 0.0
        sz = 0.0

        for idx in range(index_lo, index_hi + 1):
            p = poses[idx].pose.position
            sx += float(p.x)
            sy += float(p.y)
            sz += float(p.z)

        point = Point()
        point.x = sx / count
        point.y = sy / count
        point.z = sz / count
        return point

    @staticmethod
    def _arc_length_for_plan_range(poses, index_lo: int, index_hi: int) -> float:
        if index_hi <= index_lo:
            return 0.0

        total = 0.0
        previous = poses[index_lo].pose.position
        for idx in range(index_lo + 1, index_hi + 1):
            current = poses[idx].pose.position
            dx = float(current.x) - float(previous.x)
            dy = float(current.y) - float(previous.y)
            total += math.sqrt(dx * dx + dy * dy)
            previous = current

        return total

    @staticmethod
    def _make_debounce_key(index_lo: int, index_hi: int, centroid: Point) -> str:
        rounded_x = round(float(centroid.x), 1)
        rounded_y = round(float(centroid.y), 1)
        return f'plan:{index_lo}-{index_hi}:centroid:{rounded_x:.1f},{rounded_y:.1f}'

    def _is_debounced(self, debounce_key: str) -> bool:
        now = time.monotonic()
        last = self._last_publish_by_key.get(debounce_key)

        if last is not None and (now - last) < self._debounce_sec:
            return True

        self._last_publish_by_key[debounce_key] = now
        return False


def main(args=None):
    rclpy.init(args=args)
    node = PlanIntersectionMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
