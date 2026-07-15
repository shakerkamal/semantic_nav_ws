#!/usr/bin/env python3
"""En-route blockage trigger (eval-only): changes the WORLD, nothing else.

Watches the robot pose; when it crosses the scenario's trigger line, spawns
the blocker at its fixed pose (reproducible mid-route injection). Optionally
deletes it after delete_after_sec (S1 transient, S4 person-leaves). Announces
spawned/deleted on a latched topic so the mock detector knows the object
exists to be perceived.

Usage:
  python3 eval/enroute_blockage_trigger.py --scenario S2
"""
import argparse
import math
import os
import sys
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from enroute_common import load_scenarios, robot_map_pose  # noqa: E402


def crossed(axis: str, threshold: float, direction: str,
            xy: Tuple[float, float]) -> bool:
    value = xy[0] if axis == "x" else xy[1]
    if direction == "increasing":
        return value > threshold
    return value < threshold


def database_include_xml(model_name: str) -> str:
    """SDF payload for a Gazebo model-database spawn.

    Mirrors gazebo_ros's OWN spawn_entity.py MODEL_DATABASE_TEMPLATE exactly:
    <world><include>, with NO pose in the xml -- placement comes entirely
    from the SpawnEntity service's separate initial_pose field (set in
    _spawn() below), the same as the proven close_door.sh/close_partition.sh
    `spawn_entity.py -database ... -x -y -z -Y` invocations. A synthesized
    <model><pose>...<include>...</model> wrapper (the previous approach) is
    not how gazebo_ros resolves database models: the door spawned but never
    actually blocked the corridor (S2 smoke run, 2026-07-15) because it
    landed away from the intended pose.
    """
    return (
        "<sdf version='1.6'><world name='default'><include>"
        "<uri>model://{}</uri></include></world></sdf>"
    ).format(model_name)


class BlockageTrigger(Node):
    def __init__(self, scenario_name: str, config: dict):
        super().__init__("enroute_blockage_trigger")
        sc = config["scenarios"][scenario_name]
        self._trigger = sc["trigger"]
        self._blocker = sc["blocker"]
        self._delete_after = float(sc.get("delete_after_sec") or 0.0)
        self._fired = False
        self._deleted = False
        self._spawn_time = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(
            String, str(config["common"]["blocker_state_topic"]), latched)

        self._spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")
        self._delete_cli = self.create_client(DeleteEntity, "/delete_entity")

        # Operator-confirm scenarios (delete_after_sec=0.0: S2 door "opens",
        # S3 chair "clears") have NOTHING that removes the spawned entity on
        # confirm -- OperatorDecision.srv only reports acknowledged/note, it
        # cannot signal a simulation-specific follow-up (deleting a Gazebo
        # obstacle would wrongly couple the operator interface to
        # simulation-only concerns; a real deployment has no Gazebo at all).
        # OperatorPrompt (2026-07-15) now publishes responsible_object_key on
        # acknowledged=true specifically so eval tooling can react here.
        if self._delete_after <= 0.0:
            self.create_subscription(
                String, "/operator_confirmed_object",
                self._on_operator_confirmed, 10)

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"[TRIGGER] armed for {scenario_name}: "
            f"{self._trigger['axis']} {self._trigger['direction']} "
            f"{self._trigger['threshold']}")

    def _on_operator_confirmed(self, msg: String) -> None:
        if not msg.data:
            return
        if not self._fired or self._deleted:
            return
        self.get_logger().info(
            f"[TRIGGER] operator confirmed '{msg.data}' -- deleting blocker")
        self._deleted = True
        self._delete()

    def _blocker_xml(self) -> str:
        b = self._blocker
        if b["kind"] == "database":
            return database_include_xml(b["model"])
        share = get_package_share_directory("semantic_nav_bringup")
        for sub in ("door_scenario", "person_scenario", "obstacle_scenario"):
            path = os.path.join(share, "models", sub, b["model"])
            if os.path.exists(path):
                return open(path).read()
        raise FileNotFoundError(b["model"])

    def _spawn(self) -> None:
        # Pre-spawn cleanup: a leftover entity with the SAME name from an
        # earlier/aborted trial rep (S2 delete_after_sec=0 means only the
        # operator-confirm path deletes it; if a rep is killed or the
        # operator declines, the entity survives and every later rep's spawn
        # is flatly rejected -- "Entity [...] already exists", found
        # 2026-07-15). Idempotent: a "not found" failure here is EXPECTED
        # and harmless, just logged for visibility, not an error.
        req = DeleteEntity.Request()
        req.name = self._blocker["entity"]
        future = self._delete_cli.call_async(req)
        future.add_done_callback(self._on_precleanup_response)

    def _on_precleanup_response(self, future) -> None:
        try:
            resp = future.result()
            self.get_logger().info(
                f"[TRIGGER] pre-spawn cleanup: success={resp.success} "
                f"message='{resp.status_message}'")
        except Exception as exc:  # noqa: BLE001 -- best-effort, never blocks spawn
            self.get_logger().info(
                f"[TRIGGER] pre-spawn cleanup call failed (ignored): {exc}")
        self._do_spawn()

    def _do_spawn(self) -> None:
        b = self._blocker
        x, y, yaw = (float(v) for v in b["pose"])
        req = SpawnEntity.Request()
        req.name = b["entity"]
        req.xml = self._blocker_xml()
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.orientation.z = math.sin(yaw / 2.0)
        req.initial_pose.orientation.w = math.cos(yaw / 2.0)
        req.reference_frame = "world"
        self._spawn_time = self.get_clock().now()
        self.get_logger().info(
            f"[TRIGGER] spawn requested '{b['entity']}' at ({x:.3f}, {y:.3f})")
        future = self._spawn_cli.call_async(req)
        future.add_done_callback(self._on_spawn_response)

    def _on_spawn_response(self, future) -> None:
        # The service response is the ONLY authoritative signal that Gazebo
        # actually created the entity -- logging immediately after
        # call_async() (the previous behaviour) only proves the REQUEST was
        # sent, not that it was accepted. Publishing blocker_state=spawned is
        # deferred to here so the mock detector never reports perceiving an
        # object Gazebo rejected.
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001 -- log and surface, do not hide
            self.get_logger().error(f"[TRIGGER] spawn service call failed: {exc}")
            return
        self.get_logger().info(
            f"[TRIGGER] spawn result: success={resp.success} "
            f"message='{resp.status_message}'")
        if resp.success:
            self._state_pub.publish(String(data="spawned"))

    def _delete(self) -> None:
        req = DeleteEntity.Request()
        req.name = self._blocker["entity"]
        future = self._delete_cli.call_async(req)
        future.add_done_callback(self._on_delete_response)
        self._state_pub.publish(String(data="deleted"))
        self.get_logger().info(
            f"[TRIGGER] delete requested '{self._blocker['entity']}'")

    def _on_delete_response(self, future) -> None:
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"[TRIGGER] delete service call failed: {exc}")
            return
        self.get_logger().info(
            f"[TRIGGER] delete result: success={resp.success} "
            f"message='{resp.status_message}'")

    def _tick(self) -> None:
        if self._fired:
            if (self._delete_after > 0.0 and not self._deleted
                    and self._spawn_time is not None):
                elapsed = (self.get_clock().now()
                           - self._spawn_time).nanoseconds / 1e9
                if elapsed >= self._delete_after:
                    self._deleted = True
                    self._delete()
            return
        robot = robot_map_pose(self._tf_buffer)
        if robot is None:
            return
        if crossed(self._trigger["axis"], float(self._trigger["threshold"]),
                   self._trigger["direction"], robot):
            self._fired = True
            self._spawn()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True,
                        choices=["S1", "S2", "S3", "S4", "S5"])
    parser.add_argument("--config", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "enroute_scenarios.yaml"))
    args = parser.parse_args()
    rclpy.init()
    node = BlockageTrigger(args.scenario, load_scenarios(args.config))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
