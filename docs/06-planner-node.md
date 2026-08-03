# Planner node

The third node: a map and a goal in, a route out.

- **In:** `/map` — `nav_msgs::msg::OccupancyGrid`
- **In:** `/goal_point` — `geometry_msgs::msg::PointStamped`
- **In:** `/odom/filtered` — `nav_msgs::msg::Odometry`
- **Out:** `/path` — `nav_msgs::msg::Path`

## What it does

Takes the map the last node built, finds the cheapest way across it from where the robot is to
where it was told to go, and publishes that as a list of waypoints.

Cheapest, not shortest. The map's cells carry the buffer the costmap laid around each obstacle, and
crossing a high-cost cell is charged extra. So the route bends away from walls without any rule
telling it to.

The route is thrown away and rebuilt constantly — the robot moves, the map grows, and a plan made
two seconds ago started somewhere the robot no longer is.

## Specs

| | value | why |
|---|---|---|
| goal tolerance | 0.5 m | closer than the map's own cell size can meaningfully resolve |
| replan rate | 500 ms | often enough that the route starts near the robot, cheap enough to ignore |
| give-up timeout | 60 s | a robot that hasn't arrived by now is stuck, and replanning won't fix it |
| heuristic | Euclidean | matches 8-way movement; Manhattan would over-estimate diagonals |
| neighbours | 8 | 4-way can only stair-step, which makes every diagonal 40% too long |
| blocked at | cost 100 | see below |
| cost weight | 4.0 | see below |

## Choices

Little of this node is forced. These are the calls that shape how it drives.

| decision | picked | why |
|---|---|---|
| what counts as blocked | 100 only | 1–99 is the buffer, not a wall. Refusing to enter it would make the robot refuse doorways. |
| unknown cells (−1) | passable, free | otherwise the robot won't move until it has already seen where it's going |
| cost 1–99 | passable, charged extra | the whole point of building an inflation buffer is for something to weigh it |
| how much extra | ×4.0 | high enough to push the route off walls, low enough that it will still squeeze through a gap when that's the only way |
| path frame | taken from the map message | hardcoding a frame name means the day the map moves frames, the route silently lands in the wrong place |

The cost weight is the one worth understanding. A step's price is its length multiplied by
`1 + 4·(cell_cost/100)`. Crossing a cell at full buffer cost therefore costs five times as much as
crossing open ground, so A\* will happily walk five metres around rather than one metre through.
Turn it up and the robot hugs the centre of every gap; turn it down and it starts clipping corners.

What this weight cannot do is push the route further out than the buffer reaches. Past the costmap's
inflation radius every cell costs the same, so the planner has no reason to prefer one over another.
Clearance is set there, in [04](04-costmap-node.md), not here.

## How I built it

Same split as the other two:

- `planner_node.{hpp,cpp}` — subscribers, publisher, timer, and the state machine
- `planner_core.{hpp,cpp}` — A\* and the grid maths, no ROS logic

Order of work:

1. `package.xml` and `CMakeLists.txt` — add `nav_msgs` and `geometry_msgs`
2. `planner_core.hpp` — the four structs, then the functions A\* needs
3. `planner_core.cpp` — the search
4. `planner_node.hpp` — the state enum, callbacks, and the state they share
5. `planner_node.cpp` — the state machine
6. `./watod build robot`, then publish a goal and look at the route

## What runs

The node is a two-state machine. It is either waiting to be given somewhere to go, or trying to get
there.

```
                 ┌──────────────────────┐
      ┌─────────►│   WAITING_FOR_GOAL   │◄─────────┐
      │          └──────────┬───────────┘          │
      │                     │ goal arrives         │
      │                     ▼                      │
      │          ┌──────────────────────┐          │
      │          │ WAITING_FOR_ROBOT_   │          │
      │          │    TO_REACH_GOAL     │          │
      │          └──────────┬───────────┘          │
      │                     │                      │
      │  within 0.5m        │  replan on:          │  60s elapsed
      └─────────────────────┤  - new map           ├──────────────┘
                            │  - every 500ms
                            ▼
                          /path
```

Three things drive it:

- **Goal arrives** — store it, switch state, plan straight away
- **Map arrives** (1 Hz) — store it; replan only if a goal is being chased
- **Timer** (2 Hz) — arrived? go idle. Out of time? give up. Otherwise replan.

Nothing replans while idle. With no goal there is nothing to plan.

## How A\* works here

A\* explores outward from the robot, always expanding whichever cell currently looks most promising.
Each cell carries:

- **g** — what it actually cost to get here from the robot
- **h** — a guess at what's left to the goal, as the crow flies
- **f** — `g + h`, the whole trip if this cell is on the route

The guess is what separates A\* from flooding the map in every direction. Because straight-line
distance can never overstate the real remaining distance, A\* can trust it, aim at the goal, and
still be certain the route it lands on is the cheapest one.

Two details specific to this implementation:

**Cost enters through g, not h.** `g` is charged `step_length × (1 + 4·cell_cost/100)`, so the
buffer around obstacles makes routes through it expensive. `h` stays pure distance — inflating the
guess with cost would break the guarantee that A\* finds the best route.

**The queue holds stale entries.** When a cheaper way to a cell turns up, the old entry isn't
removed, it's just left behind — pulling it out later and skipping it is cheaper than hunting it
down in the heap. That's what the closed set is for.

The finished route is walked backwards from the goal through `came_from`, then reversed.

## Functions

| function | file | what it does |
|---|---|---|
| `goalCallback` | node | stores the goal, starts the clock, switches state, plans |
| `mapCallback` | node | stores the map; replans if a goal is being chased |
| `odomCallback` | node | stores the robot's pose |
| `timerCallback` | node | arrived / timed out / replan |
| `goalReached` | node | straight-line distance against the 0.5m tolerance |
| `planPath` | node | checks everything has arrived, calls core, publishes |
| `planPath` | core | the A\* search; false if there's no route |
| `worldToGrid` | core | metres → cell; false if off the map |
| `gridToWorld` | core | cell → metres, at the cell's centre |
| `inBounds` / `cost` | core | bounds test and cell lookup |

`CellIndex`, `CellIndexHash`, `AStarNode` and `CompareF` sit in the core header. They exist so cells
can be used as hash-map keys and so the priority queue knows to hand back the lowest `f` first.

## Limitations

- **Plans through obstacles it hasn't fully seen.** The map only marks the surfaces the lidar has
  hit, so a large object seen from one side has gaps in its outline that A\* walks straight through.
  A goal in the middle of a solid cylinder plans fine. This is inherited from the costmap, not
  caused here.
- **Ignores the robot's shape and heading.** The route is a chain of cells, so it can contain turns
  no differential-drive robot can take at speed.
- **Replans from scratch every time.** Nothing is reused between plans. Fine on a 200×200 grid,
  wasteful on anything larger.
- **The goal is a point, not a pose.** There's no way to say which way to face on arrival.
- **Constants are compiled in;** `config/params.yaml` is unused.

## Verify

Rebuild and start the stack as in [03](03-test-the-node.md), then in one shell:

```bash
./watod -t robot
ros2 topic pub /goal_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: sim_world}, point: {x: 8.0, y: 8.0}}"
```

`topic pub` repeats at 1 Hz by default, which keeps the 60 s timeout from firing while you look.

In a second shell:

```bash
./watod -t robot
ros2 topic hz /path                      # ~3Hz, the timer plus map updates
ros2 topic echo /path --field header --once   # frame should match the map's
```

In Foxglove, add `/map` and `/path` to the 3D panel. The route should start at the robot, end on the
goal, and bow outward around obstacles rather than skimming them.
