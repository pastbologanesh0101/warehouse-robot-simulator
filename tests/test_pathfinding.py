import unittest

from warehouse_sim import WarehouseGrid, bfs_shortest_path


class TestPathfinding(unittest.TestCase):
    def setUp(self):
        # A small 5x5 grid with two rack bands forcing a detour through
        # row 2. The shortest route from (0,0) to (4,4) is a known,
        # hand-computed 9-cell / 8-step path (e.g. down col 0, across
        # open row 2, down col 4).
        self.detour_grid = WarehouseGrid(
            [
                ".....",
                ".###.",
                ".....",
                ".###.",
                ".....",
            ]
        )

    def test_shortest_path_around_rack_obstacle_has_known_length(self):
        path = bfs_shortest_path(self.detour_grid, (0, 0), (4, 4))
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (4, 4))
        # Known answer: 8 steps (9 cells) -- the racks don't add any
        # extra length here because row 2 is fully open.
        self.assertEqual(len(path), 9)
        self.assertEqual(len(path) - 1, 8)

    def test_path_never_steps_on_a_rack_cell(self):
        path = bfs_shortest_path(self.detour_grid, (0, 0), (4, 4))
        self.assertIsNotNone(path)
        for pos in path:
            self.assertFalse(self.detour_grid.is_rack(pos), f"path stepped on rack at {pos}")

    def test_path_is_contiguous_single_steps(self):
        path = bfs_shortest_path(self.detour_grid, (0, 0), (4, 4))
        for (r1, c1), (r2, c2) in zip(path, path[1:]):
            self.assertEqual(abs(r1 - r2) + abs(c1 - c2), 1)

    def test_no_path_when_target_is_fully_walled_off(self):
        # Column 2 is solid rack across all 3 rows, with no gap anywhere
        # in the grid, so (0,4) is completely unreachable from (0,0)
        # even though the target cell itself is passable.
        walled_grid = WarehouseGrid(
            [
                "..#..",
                "..#..",
                "..#..",
            ]
        )
        path = bfs_shortest_path(walled_grid, (0, 0), (0, 4))
        self.assertIsNone(path)

    def test_no_path_when_start_or_goal_is_impassable(self):
        self.assertIsNone(bfs_shortest_path(self.detour_grid, (1, 1), (0, 0)))
        self.assertIsNone(bfs_shortest_path(self.detour_grid, (0, 0), (1, 1)))

    def test_path_from_cell_to_itself_is_trivial(self):
        path = bfs_shortest_path(self.detour_grid, (2, 2), (2, 2))
        self.assertEqual(path, [(2, 2)])


if __name__ == "__main__":
    unittest.main()
