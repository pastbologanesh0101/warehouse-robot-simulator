"""The 2D warehouse grid: shelf/rack cells, aisles, and pack stations.

This is a plain data model of a floor plan. Nothing here talks to any
hardware; a "warehouse" is just a list of equal-length strings.
"""

from __future__ import annotations

from typing import Iterable, Iterator, List, Tuple

RACK = "#"      # impassable shelf/rack cell
AISLE = "."     # passable aisle cell
STATION = "P"   # passable pack (pickup/dropoff) station cell

PASSABLE_SYMBOLS = {AISLE, STATION}

Position = Tuple[int, int]

# 4-directional moves (no diagonals): up, down, left, right.
_DELTAS: Tuple[Position, ...] = ((-1, 0), (1, 0), (0, -1), (0, 1))


class WarehouseGrid:
    """A rectangular grid of cells.

    ``layout`` is a list of equal-length strings, one per row, using the
    ``RACK`` / ``AISLE`` / ``STATION`` symbols above.
    """

    def __init__(self, layout: Iterable[str]):
        rows = [list(row) for row in layout]
        if not rows:
            raise ValueError("layout must have at least one row")
        width = len(rows[0])
        if any(len(row) != width for row in rows):
            raise ValueError("all rows in layout must have the same length")
        self.cells: List[List[str]] = rows
        self.rows = len(rows)
        self.cols = width

    def in_bounds(self, pos: Position) -> bool:
        r, c = pos
        return 0 <= r < self.rows and 0 <= c < self.cols

    def symbol_at(self, pos: Position) -> str:
        r, c = pos
        return self.cells[r][c]

    def is_passable(self, pos: Position) -> bool:
        return self.in_bounds(pos) and self.symbol_at(pos) in PASSABLE_SYMBOLS

    def is_rack(self, pos: Position) -> bool:
        return self.in_bounds(pos) and self.symbol_at(pos) == RACK

    def neighbors(self, pos: Position) -> Iterator[Position]:
        r, c = pos
        for dr, dc in _DELTAS:
            candidate = (r + dr, c + dc)
            if self.is_passable(candidate):
                yield candidate

    def station_positions(self) -> List[Position]:
        return [
            (r, c)
            for r in range(self.rows)
            for c in range(self.cols)
            if self.cells[r][c] == STATION
        ]

    def render(self, robot_positions: dict | None = None) -> str:
        """Render the grid as text, optionally overlaying robot ids (0-9)."""
        robot_positions = robot_positions or {}
        overlay = {pos: str(rid % 10) for pos, rid in robot_positions.items()}
        lines = []
        for r in range(self.rows):
            row_chars = []
            for c in range(self.cols):
                pos = (r, c)
                row_chars.append(overlay.get(pos, self.cells[r][c]))
            lines.append("".join(row_chars))
        return "\n".join(lines)


def build_default_layout(rows: int = 15, cols: int = 15) -> List[str]:
    """Build a sample 15x15 warehouse: four horizontal shelf bands with a
    vertical cross-aisle at column 7, an aisle border, and three pack
    stations at the corners of the floor.
    """
    grid = [[AISLE for _ in range(cols)] for _ in range(rows)]

    rack_row_bands = [(2, 3), (5, 6), (8, 9), (11, 12)]
    rack_col_start, rack_col_end = 2, cols - 3  # leave aisle border on both sides
    cross_aisle_col = cols // 2

    for r0, r1 in rack_row_bands:
        for r in range(r0, r1 + 1):
            if r >= rows:
                continue
            for c in range(rack_col_start, rack_col_end + 1):
                if c == cross_aisle_col:
                    continue  # vertical cross-aisle gap through every band
                grid[r][c] = RACK

    grid[0][0] = STATION
    grid[rows - 1][0] = STATION
    grid[0][cols - 1] = STATION

    return ["".join(row) for row in grid]
