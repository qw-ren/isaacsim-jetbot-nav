"""path_validator — reachability via inflated occupancy grid (STAGE 1, stub).

Planned: rasterise obstacles, inflate by (robot_radius + safety_margin),
A*/BFS from start to goal, return path length + detour ratio vs base map.
"""


def is_reachable(spec: dict, config: dict) -> tuple[bool, float]:
    raise NotImplementedError("Stage 1")
