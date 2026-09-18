"""Warehouse Robot Simulator.

A pure-Python, 2D grid software simulation of a multi-robot warehouse
fleet. There is no real hardware anywhere in this package: robots are
plain Python objects that move between cells of a grid, one step per
simulated "tick". The parts that ARE real and tested: BFS/A*-style
shortest-path routing around impassable rack cells, nearest-idle-robot
task assignment, and priority-based collision avoidance that guarantees
no two robots ever occupy the same grid cell at the same tick.
"""

from .grid import WarehouseGrid, RACK, AISLE, STATION
from .pathfinding import bfs_shortest_path, manhattan_distance
from .robot import Robot, RobotState
from .task import Task, TaskStatus
from .simulation import Simulation

__all__ = [
    "WarehouseGrid",
    "RACK",
    "AISLE",
    "STATION",
    "bfs_shortest_path",
    "manhattan_distance",
    "Robot",
    "RobotState",
    "Task",
    "TaskStatus",
    "Simulation",
]
