"""A single warehouse robot: position, state, and its current plan."""

from __future__ import annotations

from enum import Enum
from typing import List, Optional, Tuple

from .task import Task

Position = Tuple[int, int]


class RobotState(Enum):
    IDLE = "idle"
    EN_ROUTE = "en_route"


class Robot:
    def __init__(self, robot_id: int, position: Position):
        self.robot_id = robot_id
        self.position = position
        self.home = position  # parking spot it returns to when idle with no task
        self.state = RobotState.IDLE

        self.task: Optional[Task] = None
        # 'to_pickup' while heading to the task's pickup cell, then
        # 'to_dropoff' while heading to the task's dropoff cell.
        self.phase: Optional[str] = None
        # remaining planned steps, NOT including the current cell.
        self.path: List[Position] = []
        # the cell the current path is ultimately aimed at (pickup,
        # dropoff, or home); used to replan around a blockage.
        self.target: Optional[Position] = None
        # lower number = assigned earlier = wins ties during collision
        # avoidance. Set when a task is assigned, cleared when idle again.
        self.priority: Optional[int] = None

        self.steps_taken = 0
        self.wait_events = 0

    def next_step(self) -> Optional[Position]:
        return self.path[0] if self.path else None

    def is_idle(self) -> bool:
        return self.state == RobotState.IDLE

    def __repr__(self) -> str:  # pragma: no cover - convenience only
        return (
            f"Robot(id={self.robot_id}, pos={self.position}, "
            f"state={self.state.value}, phase={self.phase})"
        )
