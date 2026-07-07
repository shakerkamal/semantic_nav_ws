# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Adapt a nav_msgs/OccupancyGrid into the pure CostGrid used by diagnosis.

Duck-typed on purpose (no nav_msgs import) so it stays unit-testable and the
diagnosis stack remains ROS-free.
"""

from semantic_nav_orchestrator.global_blockage_diagnosis import CostGrid


def occupancygrid_to_costgrid(msg) -> CostGrid:
    """Build a CostGrid from anything shaped like a nav_msgs/OccupancyGrid."""
    info = msg.info
    return CostGrid(
        resolution=float(info.resolution),
        width=int(info.width),
        height=int(info.height),
        origin_x=float(info.origin.position.x),
        origin_y=float(info.origin.position.y),
        data=list(msg.data),
    )
