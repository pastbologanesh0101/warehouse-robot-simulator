# Warehouse Robot Simulator

A small, fully working multi-robot warehouse simulator.

**What this is, plainly stated:** a 2D grid *software simulation* of a
warehouse robot fleet, written in pure Python. There is no real
hardware anywhere in this repository — no motors, no sensors, no
physical robots. A "robot" is a Python object with a position on a
grid; a "tick" is one loop iteration. What *is* real and genuinely
tested: the shortest-path routing, the nearest-idle-robot task
assignment, and the collision-avoidance logic that keeps robots off of
each other. All of that logic is exercised by real unit tests, not
mocked or faked.

## What it simulates

- **Warehouse layout** — a grid of cells: impassable shelf/rack cells,
  passable aisle cells, and one or more pack (pickup/dropoff) stations.
  A sample 15x15 warehouse with four shelf bands and three pack
  stations is built by `warehouse_sim.grid.build_default_layout()`.
- **Fleet** — N robots, each starting idle at some cell.
- **Tasks** — a queue of "fetch the item at shelf-adjacent location X,
  deliver it to pack station Y" jobs.
- **Task assignment** — every tick, each pending task is handed to the
  nearest *idle* robot (grid/Manhattan distance), so robots pick up
  work as soon as they're free rather than the queue draining strictly
  first-in-first-out against a single robot.
- **Pathfinding** — breadth-first search from a robot's current cell to
  its target cell, avoiding rack cells. The grid is uniform-cost (every
  step costs 1), so BFS already returns the same shortest path an A*
  search would; there's no extra heuristic bookkeeping to gain from
  A* here, so pathfinding.py documents that equivalence rather than
  reimplementing a heap-based search that would produce identical
  answers.
- **Collision avoidance** — priority-based. Every robot's current cell
  is reserved by default; robots with a next planned move are resolved
  in priority order (whichever robot was assigned its current task
  earliest goes first). A robot may step into a cell only if nothing
  else currently holds it. The robot that loses out simply waits one
  tick — and, if it knows its ultimate destination, immediately tries
  to replan a different route around the obstacle for its next
  attempt. This guarantees, by construction, that **no two robots are
  ever on the same cell at the end of the same tick**. (Known
  limitation: two robots meeting exactly head-on in a corridor only one
  cell wide will both wait for each other rather than swap places —
  this simple scheme doesn't solve that particular deadlock class,
  though it never lets them collide while it happens.)
- **Simulation loop** — each tick: assign pending tasks to idle robots,
  advance every robot one grid-step along its path (or make it wait, per
  collision avoidance), then check arrivals: a robot that reaches its
  task's pickup cell switches to heading for the dropoff cell, and a
  robot that reaches the dropoff cell completes the task, goes idle,
  and heads back to its home position (freeing up the pack station for
  the next delivery) — ready to be redirected the instant it's given a
  new task.

## Project layout

```
warehouse_sim/
    grid.py          # WarehouseGrid, rack/aisle/station cells, sample layout
    pathfinding.py    # BFS shortest-path (+ optional obstacle avoidance)
    task.py           # Task, TaskStatus
    robot.py          # Robot, RobotState
    simulation.py      # Simulation: assignment, movement, collision avoidance
demo.py                # CLI entry point
tests/                 # unittest suite (17 tests)
```

## Running the demo

```bash
python3 demo.py                 # run the default 15x15 scenario for 60 ticks
python3 demo.py --ticks 100     # simulate longer
python3 demo.py --verbose       # print every robot's position each tick
```

No dependencies — pure standard library. Requires Python 3.11+ (developed and tested on 3.11 and 3.12).

### Example output

```
Warehouse Robot Simulator -- 2D grid software simulation (no real hardware)
Grid: 15x15, robots: 4, tasks queued: 6
------------------------------------------------------------------------
------------------------------------------------------------------------
Ticks simulated:        60
Tasks completed:        6 / 6
Total robot-steps:      134
Collision-avoidance waits: 3

  Robot 0: final pos=(0, 1), state=idle, steps=44, waits=2
  Robot 1: final pos=(14, 1), state=idle, steps=54, waits=1
  Robot 2: final pos=(7, 1), state=idle, steps=24, waits=0
  Robot 3: final pos=(0, 13), state=idle, steps=12, waits=0
```

All 6 queued tasks complete well within the 60-tick budget, and the 3
`Collision-avoidance waits` are genuine instances where a lower-priority
robot yielded a cell to a higher-priority one instead of colliding.

## Running the tests

```bash
python3 -m unittest discover -v tests
```

17 tests, no dependencies beyond the standard library. They cover:

- pathfinding finds the correct, hand-verified shortest path around a
  rack obstacle on a small crafted grid
- pathfinding correctly returns "no path" when the target is fully
  walled off by racks
- paths never step on a rack cell and are always single-step contiguous
- task assignment picks the nearest idle robot in a crafted scenario
- busy robots are skipped by the assigner; tasks stay pending when no
  robot is idle
- the task queue is processed as robots become idle, not naively
  FIFO-blocked against a single robot
- two robots on a direct collision course never end up on the same
  cell at the same tick (asserted directly, tick by tick)
- the lower-priority robot in a conflict waits exactly once, and
  resumes its original plan on a later tick once the path clears
- a robot transitions `idle -> en_route -> idle` as it completes a task
- the simulation's reported completed-task count after N ticks matches
  a hand-computed, deterministic expectation

Run locally before pushing:

```bash
python3 -m unittest discover -v tests
```

A GitHub Actions workflow (`.github/workflows/tests.yml`) runs the same
suite, plus a demo smoke test, on every push and pull request against
Python 3.11 and 3.12.

## Design notes / honesty section

- This is a **discrete-time, discrete-space** simulation: one grid cell
  per step, one tick per movement. It does not model robot kinematics,
  acceleration, battery, sensors, or any physical hardware.
- Pathfinding does not "see" other robots — it only avoids static rack
  cells. Dynamic conflicts between robots are handled entirely by the
  collision-avoidance layer in `simulation.py`, which is where the
  "replan around a blockage" behavior lives.
- Priority is assigned in task-assignment order (earlier assignment =
  higher priority), a simple and deterministic scheme that is easy to
  reason about and test.
