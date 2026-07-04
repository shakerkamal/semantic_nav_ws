# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""CLI demo: diagnose a synthetic two-room map with an open vs closed door.

Run: ros2 run semantic_nav_orchestrator diagnose_blockage_demo
Prints the diagnosis for both cases — no ROS graph required.
"""

from semantic_nav_orchestrator.global_blockage_diagnosis import (
    CostGrid,
    diagnose_global_blockage,
)


def _two_rooms(door_value):
    """Build a 5x3 grid; column 2 is a wall with door cell (2,1)=door_value."""
    data = [
        0, 0, 100, 0, 0,
        0, 0, door_value, 0, 0,
        0, 0, 100, 0, 0,
    ]
    return CostGrid(1.0, 5, 3, 0.0, 0.0, data)


def _report(label, door_value):
    """Diagnose one synthetic map and print the result."""
    grid = _two_rooms(door_value)
    d = diagnose_global_blockage(grid, (0.5, 1.5), (4.5, 1.5))
    print(f"[{label}] diagnosis={d.diagnosis}")
    print(f"         barrier_centroid={d.barrier_centroid}")
    print(f"         standoff_pose={d.standoff_pose}")
    print(f"         confidence={d.confidence:.2f}")


def main(args=None):
    """Print diagnoses for open-door and closed-door synthetic maps."""
    _report("door OPEN ", 0)
    _report("door CLOSED", 100)


if __name__ == "__main__":
    main()
