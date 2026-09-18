"""Tasks: "fetch item from shelf location X, deliver to pack station Y"."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple

Position = Tuple[int, int]


class TaskStatus(Enum):
    PENDING = "pending"          # waiting in the queue for a robot
    ASSIGNED = "assigned"        # robot is en route to the pickup location
    IN_TRANSIT = "in_transit"    # item picked up, robot en route to dropoff
    COMPLETE = "complete"        # delivered


@dataclass
class Task:
    task_id: int
    pickup: Position
    dropoff: Position
    status: TaskStatus = field(default=TaskStatus.PENDING)
    assigned_robot: Optional[int] = None
    assigned_at_tick: Optional[int] = None
    completed_at_tick: Optional[int] = None
