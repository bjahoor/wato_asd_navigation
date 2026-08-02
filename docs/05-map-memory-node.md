# Map memory node

The second node: local costmaps in, one map of everything seen so far out.

- **In:** `/costmap` — `nav_msgs::msg::OccupancyGrid`
- **In:** `/odom/filtered` — `nav_msgs::msg::Odometry`
- **Out:** `/map` — `nav_msgs::msg::OccupancyGrid`

## What it does

The costmap is thrown away and rebuilt every scan, and it moves with the robot. This node pins
each costmap to where the robot was standing when it was taken, and stamps it into one grid that
stays put.

That grid is in the world frame, so an obstacle stays where it is even after the robot turns away
from it. The planner needs that — it can't route around a wall the robot stopped looking at.

Stamping happens only after the robot has moved a set distance. Standing still produces the same
costmap over and over, and re-fusing it changes nothing.

## Specs

| | value | why |
|---|---|---|
| resolution | 0.2 m/cell | coarser than the costmap, so its cells land inside map cells and fusion leaves no holes |
| grid size | 200 × 200 cells | 40m across, covering the 30m arena |
| origin | (−20, −20) | puts the world origin at the map's centre |
| frame | `sim_world` | taken from the odometry, since that is what places the costmap |
| fuse distance | 1.5 m | far enough that the new costmap actually shows something new |
| timer rate | 1 Hz | the map is only worth redrawing so often |
| starting value | −1 (unknown) | nothing has been seen yet, and unknown is not the same as clear |

## How I built it

Same two-pair split as the costmap:

- `map_memory_node.{hpp,cpp}` — the two subscribers, the publisher, the timer, and the distance check
- `map_memory_core.{hpp,cpp}` — the map and the merge, no `rclcpp` calls

Order of work:

1. `package.xml` and `CMakeLists.txt` — add `nav_msgs`
2. `map_memory_node.hpp` — subscribers, publisher, timer, and the state they share
3. `map_memory_core.hpp` — declare the map, the constants, and the three functions
4. `map_memory_core.cpp` — the transform and the merge
5. `map_memory_node.cpp` — callbacks and the timer
6. `./watod build robot`, then check the topic

## What runs

Three things fire at different rates and never call each other. They share state instead, which is
why the node holds so much of it.

```
     /costmap  ~9Hz              /odom/filtered  10Hz
         │                              │
         ▼                              ▼
 ┌────────────────┐            ┌────────────────┐
 │ costmapCallback│            │  odomCallback  │
 │  store newest  │            │  x, y, yaw     │
 └────────┬───────┘            │  moved 1.5m?   │
          │                    └────────┬───────┘
          │ latest_costmap_             │ should_update_map_
          └──────────────┬──────────────┘
                         ▼
                ┌────────────────┐
                │   updateMap    │  1Hz timer
                └────────┬───────┘
                         │ if armed
                         ▼
                ┌────────────────┐
                │integrateCostmap│  rotate + shift + merge
                │     (core)     │
                └────────┬───────┘
                         ▼
                    /map  1Hz
```

Costmaps arrive nine times a second, but fusing one is expensive and pointless if the robot hasn't
moved. So the callbacks stay cheap and the timer decides when work actually happens.

Two parts are worth spelling out.

**Placing the costmap is a rotation then a shift.** A costmap cell's position is in metres from
the robot, so it has to be turned by the robot's heading before it can be added to the robot's
position:

```
global_x = robot_x + local_x·cos(yaw) − local_y·sin(yaw)
global_y = robot_y + local_x·sin(yaw) + local_y·cos(yaw)
```

Heading comes out of the odometry quaternion as yaw. Only the heading matters on flat ground.

**Four costmap cells fall into one map cell.** 0.1m cells landing in 0.2m cells means collisions.
The first one to land overwrites what the map remembered; after that only a higher value sticks,
so a free cell can't rub out an obstacle sharing the same map cell.

## Functions

| function | file | what it does |
|---|---|---|
| `costmapCallback` | node | stores the newest costmap |
| `odomCallback` | node | reads position, converts the quaternion to yaw, arms an update after 1.5m |
| `updateMap` | node | timer tick: fuse if armed, then publish |
| `initializeMap` | core | sets every cell to unknown |
| `integrateCostmap` | core | rotates and shifts each costmap cell into the map, keeping the higher value |
| `toOccupancyGrid` | core | packs the map into a message |
| `inBounds` / `cell` | core | bounds test and 2D→1D indexing |

The map is published on every tick, not just after a fusion, so anything that starts late gets a
map immediately instead of waiting for the robot to drive.

## Change to the costmap node

The lidar comes up before the world does, and that first scan has no returns in it. The costmap
was publishing an all-free grid for it, which the map then recorded as "this area is clear".
`lidarCallback` now counts valid readings and publishes nothing when there are none.

## Limitations

- The costmap has no unknown value — anything it doesn't hit is reported free. Re-observing an
  area therefore erases obstacles the map had remembered there. Fixing it properly means marking
  free space only along the lidar beams.
- Fixed 40m map. Drive past its edge and the readings are dropped.
- The gate is distance only. Spinning on the spot reveals new surroundings but never triggers a fuse.
- Odometry is taken as truth. Real drift would smear the map.
- Constants are compiled in; `config/params.yaml` is unused.

## Verify

Rebuild and start the stack as in [03](03-test-the-node.md), then:

```bash
./watod -t robot                             # shell into the running container
ros2 topic hz /map                           # 1Hz
ros2 topic echo /map --field info --once     # resolution 0.2, 200x200, origin -20,-20
```

In Foxglove, add `/map` to the 3D panel and drive the robot. Explored ground fills in behind it
and stays filled after it turns away.
