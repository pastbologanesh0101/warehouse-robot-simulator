"""The simulation loop: task assignment, pathfinding, movement, and
priority-based collision avoidance.

Collision avoidance rule (documented here because it's the crux of the
whole project): every robot's current cell is reserved for it by
default. Robots that have a next planned step ("movers") are resolved
in priority order (the robot assigned to its current task earliest
wins ties, then by robot id): a mover may step into a cell only if
nothing currently reserves it. When it moves, its old cell is released
(freeing it for a lower-priority robot right behind it) and its new
cell becomes reserved. A robot that loses the race stays put for this
tick (a "wait event") and, if it knows what it's ultimately heading
for, immediately tries to replan a route around whatever is in its way
for the next tick. This guarantees, by construction, that no two robots
ever end up on the same cell at the end of the same tick.

Known limitation: two robots meeting exactly head-on in a corridor
only one cell wide, each needing the cell the other currently occupies,
will both wait for each other rather than swap places -- this
implementation does not solve that class of true deadlock (neither
does it ever violate the no-shared-cell invariant while it happens).
"""

from __future__ import annotations

from typing import Dict, List, Optional

from .grid import WarehouseGrid
from .pathfinding import bfs_shortest_path, manhattan_distance
from .robot import Robot, RobotState
from .task import Task, TaskStatus


class Simulation:
    def __init__(self, grid: WarehouseGrid, robots: List[Robot], tasks: List[Task]):
        self.grid = grid
        self.robots: Dict[int, Robot] = {r.robot_id: r for r in robots}
        self.pending_tasks: List[Task] = list(tasks)
        self.completed_tasks: List[Task] = []
        self.tick_count = 0
        self.total_wait_events = 0
        self._assignment_counter = 0

    # -- queries ---------------------------------------------------------

    def idle_robots(self) -> List[Robot]:
        return [r for r in self.robots.values() if r.state == RobotState.IDLE]

    def total_robot_steps(self) -> int:
        return sum(r.steps_taken for r in self.robots.values())

    def all_tasks_complete(self) -> bool:
        return not self.pending_tasks and all(
            r.task is None for r in self.robots.values()
        )

    # -- task assignment ---------------------------------------------------

    def assign_tasks(self) -> None:
        """Assign each pending task (queue order) to the nearest idle
        robot, so robots pick up new work as soon as they go idle rather
        than being blocked behind an earlier task in the queue.
        """
        still_pending: List[Task] = []
        for task in self.pending_tasks:
            idle = self.idle_robots()
            if not idle:
                still_pending.append(task)
                continue
            nearest = min(idle, key=lambda r: manhattan_distance(r.position, task.pickup))
            self._assign(nearest, task)
        self.pending_tasks = still_pending

    def _assign(self, robot: Robot, task: Task) -> None:
        self._assignment_counter += 1
        robot.priority = self._assignment_counter
        robot.task = task
        robot.phase = "to_pickup"
        robot.state = RobotState.EN_ROUTE
        task.status = TaskStatus.ASSIGNED
        task.assigned_robot = robot.robot_id
        task.assigned_at_tick = self.tick_count
        self._replan(robot, task.pickup)

    def _replan(self, robot: Robot, target) -> None:
        robot.target = target
        path = bfs_shortest_path(self.grid, robot.position, target)
        robot.path = path[1:] if path else []

    # -- movement / collision avoidance ------------------------------------

    def _move_robots_with_collision_avoidance(self) -> None:
        # Every robot's current cell starts out reserved -- including a
        # robot that has a plan but hasn't moved yet -- because until we
        # actually grant it a move, nothing guarantees it will vacate
        # this tick. A mover is only allowed to step into a cell that
        # nothing else currently claims; when it does move, its old cell
        # is released (so a lower-priority robot behind it can advance
        # into the space it just left) and its new cell becomes claimed.
        reserved: Dict[tuple, int] = {r.position: r.robot_id for r in self.robots.values()}

        movers = [r for r in self.robots.values() if r.next_step() is not None]
        movers.sort(key=lambda r: (r.priority if r.priority is not None else float("inf"), r.robot_id))

        for r in movers:
            next_cell = r.next_step()
            holder = reserved.get(next_cell)
            if holder is not None and holder != r.robot_id:
                r.wait_events += 1
                self.total_wait_events += 1
                # r.position stays reserved for r; nothing to change.
                # "waits a tick and replans": try routing around whatever
                # is currently in the way, so a robot doesn't sit stuck
                # behind an obstacle forever when an alternate route
                # exists. If no alternate route exists, this just
                # rediscovers the same path and it retries next tick.
                if r.target is not None:
                    avoid = frozenset(
                        other.position for other in self.robots.values() if other is not r
                    )
                    alt_path = bfs_shortest_path(self.grid, r.position, r.target, avoid=avoid)
                    if alt_path and len(alt_path) > 1:
                        r.path = alt_path[1:]
            else:
                del reserved[r.position]
                reserved[next_cell] = r.robot_id
                r.position = next_cell
                r.path.pop(0)
                r.steps_taken += 1

    # -- arrivals ------------------------------------------------------------

    def _process_arrivals(self) -> None:
        for robot in self.robots.values():
            if robot.task is None or robot.path:
                continue  # still has a task but hasn't reached this phase's target

            if robot.phase == "to_pickup" and robot.position == robot.task.pickup:
                robot.phase = "to_dropoff"
                robot.task.status = TaskStatus.IN_TRANSIT
                self._replan(robot, robot.task.dropoff)
            elif robot.phase == "to_dropoff" and robot.position == robot.task.dropoff:
                task = robot.task
                task.status = TaskStatus.COMPLETE
                task.completed_at_tick = self.tick_count
                self.completed_tasks.append(task)
                robot.task = None
                robot.phase = None
                robot.path = []
                robot.priority = None
                robot.state = RobotState.IDLE
                # A pack station is a single dock cell shared by every
                # task that delivers there. If a newly-idle robot just
                # sat there (or right next to it) indefinitely, a future
                # delivery to the same station could wait forever. So it
                # heads back to its home position instead. It is still
                # RobotState.IDLE the whole time -- idle_robots() still
                # lists it and assign_tasks() will happily redirect it
                # (via a fresh _replan call) the instant a new task
                # needs it, so this never delays taking on new work.
                if robot.position != robot.home:
                    self._replan(robot, robot.home)

    # -- main loop ------------------------------------------------------------

    def tick(self) -> None:
        self.tick_count += 1
        self.assign_tasks()
        self._move_robots_with_collision_avoidance()
        self._process_arrivals()

    def run(self, ticks: int) -> None:
        for _ in range(ticks):
            self.tick()
