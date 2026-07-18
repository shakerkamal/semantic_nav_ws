#!/usr/bin/env python3
"""Pre-flight guard: refuse to start a trial when a residual obstacle is still
baked into /map at the scenario's blocker coordinate.

RTAB-Map re-asserts old per-node local grids on graph optimization, and
residuals accumulate across reps when several scenarios spawn at the same gap
(S3/S4/S5 all sit at -2.507,-1.350). A residual there silently wedges the robot
in lethal space AFTER recovery has already confirmed the barrier clear
(observed live 2026-07-18: S4 failed on S3's leftover chair, passed on a fresh
map). This turns that silent failure into a loud "reset the map first".

Exit codes: 0 clear, 2 residual present, 3 no /map received (cannot verify).
"""
import argparse
import os
import sys
import time


def lethal_count(data, width, height, resolution, origin_x, origin_y,
                 cx, cy, radius_m, threshold):
    """Count occupancy cells within radius_m of (cx, cy) at/above threshold.

    Pure and ROS-free so it is unit-testable. Returns (lethal, observed,
    unknown) where observed counts known cells (value >= 0) in the window.
    """
    lethal = observed = unknown = 0
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return lethal, observed, unknown
    center_mx = int((cx - origin_x) / resolution)
    center_my = int((cy - origin_y) / resolution)
    cells = int(radius_m / resolution) + 1
    r2 = radius_m * radius_m
    for my in range(center_my - cells, center_my + cells + 1):
        if my < 0 or my >= height:
            continue
        for mx in range(center_mx - cells, center_mx + cells + 1):
            if mx < 0 or mx >= width:
                continue
            wx = origin_x + (mx + 0.5) * resolution
            wy = origin_y + (my + 0.5) * resolution
            if (wx - cx) ** 2 + (wy - cy) ** 2 > r2:
                continue
            value = data[my * width + mx]
            if value < 0:
                unknown += 1
                continue
            observed += 1
            if value >= threshold:
                lethal += 1
    return lethal, observed, unknown


def _blocker_xy(scenario):
    eval_dir = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, eval_dir)
    from enroute_common import load_scenarios  # noqa: E402
    path = os.path.join(eval_dir, "enroute_scenarios.yaml")
    sc = load_scenarios(path)["scenarios"][scenario]
    pose = sc["blocker"]["pose"]
    return float(pose[0]), float(pose[1])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scenario", help="S1..S5; reads the blocker pose from yaml")
    ap.add_argument("--x", type=float)
    ap.add_argument("--y", type=float)
    ap.add_argument("--radius", type=float, default=0.35,
                    help="sampling radius (m) around the blocker coordinate")
    ap.add_argument("--threshold", type=int, default=90,
                    help="occupancy value counted as lethal")
    ap.add_argument("--tolerance", type=int, default=2,
                    help="lethal cells tolerated as sensor noise before abort")
    ap.add_argument("--timeout", type=float, default=8.0,
                    help="seconds to wait for a /map message")
    ap.add_argument("--map-topic", default="/map")
    args = ap.parse_args(argv)

    if args.scenario:
        cx, cy = _blocker_xy(args.scenario)
    elif args.x is not None and args.y is not None:
        cx, cy = args.x, args.y
    else:
        ap.error("provide --scenario or both --x and --y")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                           HistoryPolicy)
    from nav_msgs.msg import OccupancyGrid

    class Probe(Node):
        def __init__(self):
            super().__init__("map_residual_check")
            qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.msg = None
            self.create_subscription(OccupancyGrid, args.map_topic,
                                     self._cb, qos)

        def _cb(self, msg):
            self.msg = msg

    rclpy.init()
    node = Probe()
    deadline = time.time() + args.timeout
    while rclpy.ok() and node.msg is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    msg = node.msg
    node.destroy_node()
    rclpy.shutdown()

    if msg is None:
        print(f"[MAP_RESIDUAL] no {args.map_topic} within {args.timeout:.0f}s "
              "-- cannot verify the corridor is clear", file=sys.stderr)
        return 3

    lethal, observed, unknown = lethal_count(
        msg.data, msg.info.width, msg.info.height, msg.info.resolution,
        msg.info.origin.position.x, msg.info.origin.position.y,
        cx, cy, args.radius, args.threshold)
    print(f"[MAP_RESIDUAL] blocker=({cx:.3f},{cy:.3f}) r={args.radius}m "
          f"lethal(>={args.threshold})={lethal} observed={observed} "
          f"unknown={unknown} tolerance={args.tolerance}")
    if lethal > args.tolerance:
        print(f"[MAP_RESIDUAL] RESIDUAL present ({lethal} lethal cells) -- "
              "reset the map before this trial", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
