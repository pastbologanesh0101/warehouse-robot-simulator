import unittest

from warehouse_sim import Robot, RobotState, Simulation, Task, WarehouseGrid


def open_grid(rows=6, cols=6):
    return WarehouseGrid(["." * cols for _ in range(rows)])


class TestTaskAssignment(unittest.TestCase):
    def test_nearest_idle_robot_is_chosen(self):
        grid = open_grid()
        far_robot = Robot(0, (0, 0))
        near_robot = Robot(1, (4, 4))
        task = Task(task_id=0, pickup=(5, 5), dropoff=(0, 0))
        sim = Simulation(grid, [far_robot, near_robot], [task])

        sim.assign_tasks()

        self.assertEqual(task.assigned_robot, near_robot.robot_id)
        self.assertEqual(near_robot.state, RobotState.EN_ROUTE)
        self.assertEqual(far_robot.state, RobotState.IDLE)

    def test_busy_robots_are_skipped_in_favor_of_the_only_idle_one(self):
        grid = open_grid()
        busy_robot = Robot(0, (0, 0))
        busy_robot.state = RobotState.EN_ROUTE  # pretend already working
        idle_robot = Robot(1, (5, 5))
        task = Task(task_id=0, pickup=(0, 1), dropoff=(5, 4))
        sim = Simulation(grid, [busy_robot, idle_robot], [task])

        sim.assign_tasks()

        self.assertEqual(task.assigned_robot, idle_robot.robot_id)

    def test_queue_is_not_naively_fifo_blocked(self):
        """Task 0 is far from every robot; task 1 sits right next to
        robot 1. Because assignment scans the whole pending queue and
        matches by nearest idle robot rather than draining the queue
        strictly in order against a single robot, task 1 should still
        get assigned in the same round even though it is queued second.
        """
        grid = open_grid()
        robot0 = Robot(0, (0, 0))
        robot1 = Robot(1, (5, 5))
        task_far = Task(task_id=0, pickup=(0, 5), dropoff=(0, 0))
        task_near_robot1 = Task(task_id=1, pickup=(5, 4), dropoff=(5, 5))
        sim = Simulation(grid, [robot0, robot1], [task_far, task_near_robot1])

        sim.assign_tasks()

        self.assertEqual(task_near_robot1.assigned_robot, robot1.robot_id)
        self.assertEqual(task_far.assigned_robot, robot0.robot_id)
        self.assertEqual(sim.pending_tasks, [])

    def test_task_stays_pending_when_no_robot_is_idle(self):
        grid = open_grid()
        busy_robot = Robot(0, (0, 0))
        busy_robot.state = RobotState.EN_ROUTE
        task = Task(task_id=0, pickup=(0, 1), dropoff=(0, 2))
        sim = Simulation(grid, [busy_robot], [task])

        sim.assign_tasks()

        self.assertIsNone(task.assigned_robot)
        self.assertIn(task, sim.pending_tasks)


if __name__ == "__main__":
    unittest.main()
