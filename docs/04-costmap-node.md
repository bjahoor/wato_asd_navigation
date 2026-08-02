# Costmap node

Implemented the first of the four nodes: lidar scans in, occupancy grid out.

- **In:** `/lidar` — `sensor_msgs::msg::LaserScan`
- **Out:** `/costmap` — `nav_msgs::msg::OccupancyGrid`

## What it does

A grid laid over the ground around the robot. Each cell holds one number — how bad it is to
drive there. `0` is free, `100` is an obstacle, and the values between are a buffer ringing
each obstacle.

Why a range instead of just blocked/clear: lidar is noisy, and the robot has width. The buffer
means the planner picks its way down the middle of a corridor on its own, with no explicit
"stay off the walls" rule anywhere in the code.

The grid is wiped and rebuilt on every scan, so it only ever shows what the lidar sees right
now. Remembering anything is the map memory node's job.

## Specs

| | value | why |
|---|---|---|
| resolution | 0.1 m/cell | finer than the map, so fusion leaves no holes |
| grid size | 200 × 200 cells | 10m in every direction — a deliberate crop of the lidar's 20m reach |
| origin | (−10, −10) | puts the robot at the centre, not the corner |
| inflation radius | 1.0 m | roughly a robot width of clearance |
| max cost | 100 | `OccupancyGrid` maximum |
| publish rate | ~9 Hz | not chosen — it's whatever the lidar publishes at |

## How I built it

Two pairs of files, following the convention in
[src/samples/README.md](../src/samples/README.md):

- `costmap_node.{hpp,cpp}` — subscriber, publisher, and the callback that sequences everything
- `costmap_core.{hpp,cpp}` — the grid and all the math, no `rclcpp` calls

ROS problems are then only ever in the node file, and the math unit tests without starting ROS.

Order of work:

1. `package.xml` and `CMakeLists.txt` — add `sensor_msgs` and `nav_msgs`, drop `std_msgs`
2. `costmap_node.hpp` — replace the warm-up publisher and timer with the subscriber, publisher,
   and callback
3. `costmap_core.hpp` — declare the grid, the constants, and the five helpers
4. `costmap_core.cpp` — the math
5. `costmap_node.cpp` — the callback, calling core in order
6. `./watod build robot`, then check the topic

## What runs on each scan

```
/lidar ──► [ wipe ──► mark ──► inflate ──► publish ] ──► /costmap
```

1. **Wipe** — every cell back to 0
2. **Mark** — each valid reading becomes a cell, set to 100
3. **Inflate** — cost spreads outward from each marked cell
4. **Publish** — flatten to 1D, fill header and info, send

Two parts of that are worth spelling out.

**Reading → cell is two conversions.** `x = range·cos(angle)` and `y = range·sin(angle)` give
metres; dividing by the resolution gives the cell. Subtracting the origin first is what moves
the robot from the grid's corner to its centre, so readings behind it don't land on negative
indices and get discarded.

**Inflation walks a square but marks a circle.** For each obstacle it scans the square of cells
within the radius, rejects the corners that fall outside it, and scales cost by
`1 − distance/inflation_radius`. Where two obstacles' rings overlap, the higher cost wins.

## Functions

| function | file | what it does |
|---|---|---|
| `lidarCallback` | node | drops invalid readings, then runs the four steps for one scan; publishes nothing if a scan has no valid readings at all |
| `initializeCostmap` | core | resets every cell to 0 and forgets the previous scan's obstacles |
| `convertToGrid` | core | polar reading → cell indices; false if off the grid |
| `markObstacle` | core | sets a cell to 100, remembers it for inflation |
| `inflateObstacles` | core | fades cost outward from every marked cell |
| `toOccupancyGrid` | core | packs the grid into a message |
| `inBounds` / `cell` | core | bounds test and 2D→1D indexing |

## Limitations

- 10m radius. The lidar reaches 20m, so readings past 10m are dropped.
- No memory — wiped every scan. Map memory fixes this.
- One inflation radius, linear falloff, ignores the robot's actual shape.
- Constants are compiled in; `config/params.yaml` is unused.
- Grid is in the sensor frame, so it carries the scan's `frame_id` and rotates with the robot.
  Aligning it to the world is map memory's problem.

## Verify

Rebuild and start the stack as in [03](03-test-the-node.md), then:

```bash
./watod -t robot                            # shell into the running container
ros2 topic hz /costmap                      # ~9Hz, matching the lidar
ros2 topic echo /costmap --field info --once  # resolution 0.1, 200x200, origin -10,-10
```

`--field info` because a plain echo dumps all 40,000 cells.

In Foxglove, add `/costmap` to the 3D panel. Obstacles show as bright cells with a faded halo
around each one.
