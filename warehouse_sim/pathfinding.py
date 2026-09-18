"""Shortest-path routing on the warehouse grid.

The grid is uniform-cost (every step between adjacent passable cells
costs 1), so breadth-first search already returns an optimal path; it
is the exact result an A* search with any admissible heuristic (e.g.
Manhattan distance) would produce, just without the extra heap
bookkeeping. ``manhattan_distance`` is exposed separately because the
task assigner also uses it as a distance heuristic between robots and
tasks.
"""

from __future__ import annotations

from collections import deque
from typing import Dict, FrozenSet, List, Optional, Tuple

from .grid import WarehouseGrid

Position = Tuple[int, int]


def manhattan_distance(a: Position, b: Position) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def bfs_shortest_path(
    grid: WarehouseGrid,
    start: Position,
    goal: Position,
    avoid: Optional[FrozenSet[Position]] = None,
) -> Optional[List[Position]]:
    """Return the shortest path from ``start`` to ``goal`` as a list of
    positions (including both endpoints), avoiding impassable rack
    cells. Returns ``None`` if no such path exists (the goal is
    unreachable, e.g. fully walled off by racks or off the grid).

    ``avoid`` is an optional set of extra cells to treat as temporarily
    blocked (e.g. cells currently occupied by other robots) -- used to
    let a robot replan around a blockage rather than only around the
    static rack layout. The goal cell itself is never treated as
    blocked, even if it appears in ``avoid``, so a robot can always plan
    a route to its actual destination.
    """
    avoid = avoid or frozenset()

    if not grid.is_passable(start) or not grid.is_passable(goal):
        return None

    if start == goal:
        return [start]

    visited = {start}
    came_from: Dict[Position, Position] = {}
    queue = deque([start])

    while queue:
        current = queue.popleft()
        if current == goal:
            break
        for neighbor in grid.neighbors(current):
            if neighbor in visited:
                continue
            if neighbor in avoid and neighbor != goal:
                continue
            visited.add(neighbor)
            came_from[neighbor] = current
            queue.append(neighbor)

    if goal not in visited:
        return None

    path = [goal]
    while path[-1] != start:
        path.append(came_from[path[-1]])
    path.reverse()
    return path
