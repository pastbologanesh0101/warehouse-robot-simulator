import unittest

from warehouse_sim import Robot, RobotState, Simulation, Task, TaskStatus, WarehouseGrid


def open_grid(rows, cols):
    return WarehouseGrid(["." * cols for _ in range(rows)])


class TestCollisionAvoidance(unittest.TestCase):
    """Two robots on a direct collision course, driven at the low level
    (bypassing task assignment) so the collision-avoidance mechanics can
    be checked in isolation, tick by tick.
    """

    def _collision_course_sim(self):
        grid = open_grid(1, 5)
        robot_a = Robot(0, (0, 0))
        robot_a.priority = 1
        robot_a.path = [(0, 1), (0, 2), (0, 3)]

        robot_b = Robot(1, (0, 2))
        robot_b.priority = 2
        robot_b.path = [(0, 1)]

        sim = Simulation(grid, [robot_a, robot_b], [])
        return sim, robot_a, robot_b

    def test_robots_never_occupy_the_same_cell_at_the_same_tick(self):
        sim, robot_a, robot_b = self._collision_course_sim()

        for _ in range(2):
            sim._move_robots_with_collision_avoidance()
            self.assertNotEqual(
                robot_a.position,
                robot_b.position,
                "two robots ended up on the same cell in the same tick",
            )

    def test_lower_priority_robot_waits_instead_of_colliding(self):
        sim, robot_a, robot_b = self._collision_course_sim()

        sim._move_robots_with_collision_avoidance()  # tick 1

        # Higher-priority robot A claims (0, 1); robot B must wait in
        # place rather than also move onto (0, 1).
        self.assertEqual(robot_a.position, (0, 1))
        self.assertEqual(robot_b.position, (0, 2))
        self.assertEqual(robot_b.wait_events, 1)
        self.assertEqual(sim.total_wait_events, 1)

    def test_waiting_robot_resumes_progress_once_the_path_clears(self):
        # Robot X blocks a junction cell (1, 1); robot Y needs to pass
        # straight through it. Once X is given somewhere else to go (it
        # turns off, it does not swap places with Y), Y's original,
        # untouched plan resumes on its own on the very next tick.
        grid = open_grid(3, 3)
        robot_x = Robot(0, (1, 1))
        robot_x.priority = 1

        robot_y = Robot(1, (1, 0))
        robot_y.priority = 2
        robot_y.path = [(1, 1), (1, 2)]

        sim = Simulation(grid, [robot_x, robot_y], [])

        sim._move_robots_with_collision_avoidance()  # tick 1: X is parked, Y waits
        self.assertEqual(robot_x.position, (1, 1))
        self.assertEqual(robot_y.position, (1, 0))
        self.assertEqual(robot_y.wait_events, 1)

        robot_x.path = [(2, 1)]  # X heads off elsewhere, freeing (1, 1)
        sim._move_robots_with_collision_avoidance()  # tick 2

        self.assertEqual(robot_x.position, (2, 1))
        self.assertEqual(robot_y.position, (1, 1))  # Y resumed its original plan
        self.assertEqual(robot_y.wait_events, 1)  # no additional waits
        self.assertEqual(list(robot_y.path), [(1, 2)])


class TestRobotLifecycle(unittest.TestCase):
    def test_robot_transitions_idle_to_en_route_to_idle(self):
        grid = open_grid(5, 5)
        robot = Robot(0, (0, 0))
        task = Task(task_id=0, pickup=(0, 2), dropoff=(4, 4))
        sim = Simulation(grid, [robot], [task])

        self.assertEqual(robot.state, RobotState.IDLE)

        sim.tick()
        self.assertEqual(robot.state, RobotState.EN_ROUTE)

        for _ in range(50):
            if robot.task is None:
                break
            sim.tick()

        self.assertEqual(robot.state, RobotState.IDLE)
        self.assertEqual(task.status, TaskStatus.COMPLETE)
        self.assertIn(task, sim.completed_tasks)


class TestSimulationLoopReporting(unittest.TestCase):
    def test_completed_task_count_after_n_ticks_matches_reality(self):
        # Two robots start at opposite ends of a corridor, each assigned
        # a task whose pickup/dropoff geometry gives a known, hand
        # computed completion tick of 4, with no path overlap so no
        # collision waits occur.
        grid = open_grid(1, 9)
        robot0 = Robot(0, (0, 0))
        robot1 = Robot(1, (0, 8))
        task0 = Task(task_id=0, pickup=(0, 2), dropoff=(0, 4))
        task1 = Task(task_id=1, pickup=(0, 6), dropoff=(0, 8))
        sim = Simulation(grid, [robot0, robot1], [task0, task1])

        sim.run(3)
        self.assertEqual(len(sim.completed_tasks), 0)

        sim.run(1)  # tick 4
        self.assertEqual(len(sim.completed_tasks), 2)
        self.assertTrue(sim.all_tasks_complete())
        self.assertEqual(sim.total_wait_events, 0)
        self.assertEqual(sim.total_robot_steps(), 8)
        self.assertEqual(robot0.steps_taken, 4)
        self.assertEqual(robot1.steps_taken, 4)

    def test_run_advances_tick_count(self):
        grid = open_grid(3, 3)
        sim = Simulation(grid, [Robot(0, (0, 0))], [])
        sim.run(5)
        self.assertEqual(sim.tick_count, 5)

    def test_idle_robots_helper_reflects_current_state(self):
        grid = open_grid(3, 3)
        r0 = Robot(0, (0, 0))
        r1 = Robot(1, (2, 2))
        task = Task(task_id=0, pickup=(0, 1), dropoff=(2, 1))
        sim = Simulation(grid, [r0, r1], [task])

        self.assertEqual(len(sim.idle_robots()), 2)
        sim.tick()
        self.assertEqual(len(sim.idle_robots()), 1)


if __name__ == "__main__":
    unittest.main()
