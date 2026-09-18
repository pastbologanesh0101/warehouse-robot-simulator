#!/usr/bin/env python3
"""Demo CLI for the Warehouse Robot Simulator.

This runs a fully software 2D-grid simulation -- there is no real
hardware here. It builds a sample 15x15 warehouse (rack shelves,
aisles, three pack stations), spawns a handful of robots, queues up a
batch of fetch-and-deliver tasks, and steps the simulation forward tick
by tick, printing a summary (and optionally a step-by-step trace) of
what happened: tasks completed, total robot-steps taken, and any
collision-avoidance wait events.

Usage:
    python demo.py                  # run the default scenario
    python demo.py --ticks 80       # run for a different number of ticks
    python demo.py --verbose        # print a per-tick trace of positions
"""

from __future__ import annotations

import argparse

from warehouse_sim import Robot, Simulation, Task, WarehouseGrid
from warehouse_sim.grid import build_default_layout


def build_demo_scenario():
    layout = build_default_layout(rows=15, cols=15)
    grid = WarehouseGrid(layout)

    robots = [
        Robot(0, (0, 1)),
        Robot(1, (14, 1)),
        Robot(2, (7, 1)),
        Robot(3, (0, 13)),
    ]

    stations = grid.station_positions()  # 3 pack stations, corners of the floor

    # A handful of "fetch from shelf-adjacent aisle cell, deliver to a
    # pack station" tasks, deliberately queued in an order that does not
    # line up with which robot happens to be nearest.
    pickups = [(2, 1), (5, 13), (8, 1), (11, 13), (2, 13), (11, 1)]
    tasks = [
        Task(task_id=i, pickup=pickup, dropoff=stations[i % len(stations)])
        for i, pickup in enumerate(pickups)
    ]

    return grid, robots, tasks


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ticks", type=int, default=60, help="number of ticks to simulate")
    parser.add_argument("--verbose", action="store_true", help="print a per-tick trace")
    args = parser.parse_args()

    grid, robots, tasks = build_demo_scenario()
    total_tasks = len(tasks)
    sim = Simulation(grid, robots, tasks)

    print("Warehouse Robot Simulator -- 2D grid software simulation (no real hardware)")
    print(f"Grid: {grid.rows}x{grid.cols}, robots: {len(robots)}, tasks queued: {total_tasks}")
    print("-" * 72)

    for t in range(1, args.ticks + 1):
        sim.tick()
        if args.verbose:
            positions = ", ".join(
                f"R{r.robot_id}@{r.position}[{r.state.value}]" for r in sim.robots.values()
            )
            print(f"tick {t:3d}: {positions}")

    print("-" * 72)
    print(f"Ticks simulated:        {sim.tick_count}")
    print(f"Tasks completed:        {len(sim.completed_tasks)} / {total_tasks}")
    print(f"Total robot-steps:      {sim.total_robot_steps()}")
    print(f"Collision-avoidance waits: {sim.total_wait_events}")
    print()
    for r in sim.robots.values():
        print(
            f"  Robot {r.robot_id}: final pos={r.position}, state={r.state.value}, "
            f"steps={r.steps_taken}, waits={r.wait_events}"
        )

    if not sim.all_tasks_complete():
        remaining = len(sim.pending_tasks) + sum(1 for r in sim.robots.values() if r.task)
        print()
        print(f"Note: {remaining} task(s) still in progress -- try a higher --ticks value.")


if __name__ == "__main__":
    main()
