# wato_asd_navigation

Autonomous navigation for a differential-drive robot in ROS 2 and C++. Give it a point to drive to
and it builds its own picture of the world, plans a route around what it finds, and drives there.

Built step by step, documented in [`docs/`](docs/).

### ▶ [Watch the demo](docs/assets/demo.mp4)

<p align="center">
  <a href="docs/assets/demo.mp4">
    <img src="docs/assets/demo.png" width="560"
         alt="The robot's map, the inflated cost around each obstacle, and the planned path">
  </a>
</p>

## The system

A four-node ROS 2 navigation pipeline, written from scratch. Each node takes what the one before it
produced and turns it into something closer to a wheel command.

```
                                       you, in Foxglove
                                              │
                                         /goal_point
                                              │
                                              ▼
 ┌─────────┐ /costmap ┌────────────┐ /map ┌─────────┐ /path ┌─────────┐
 │ costmap │─────────►│ map memory │─────►│ planner │──────►│ control │
 └─────────┘          └────────────┘      └─────────┘       └─────────┘
      ▲                                                          │
      │ /lidar, /odom/filtered                                   │ /cmd_vel
      │                                                          ▼
      │              ┌───────────────────────────────────────────────┐
      └──────────────┤              Gazebo — the robot               │
                     └───────────────────────────────────────────────┘
```

A closed loop: the wheel speeds move the robot, which changes what the LiDAR sees, which changes
the route. `/odom/filtered` feeds map memory, planner and control as well as the costmap.

- **Costmap** — LiDAR → occupancy grid. What's around the robot right now. [docs/04](docs/04-costmap-node.md)
- **Map memory** — fuse costmaps into a global map. What's been seen so far. [docs/05](docs/05-map-memory-node.md)
- **Planner** — A* from the robot to a goal, over the map. [docs/06](docs/06-planner-node.md)
- **Control** — Pure Pursuit, turning that route into wheel speeds. [docs/07](docs/07-control-node.md)

## Running it

Needs Docker. Full setup in [docs/00](docs/00-setup.md).

```bash
./watod build
./watod up
```

`up` holds the terminal. Open [Foxglove](https://app.foxglove.dev/) and connect to
`ws://localhost:20000`.

Import [`config/wato_asd_training_foxglove_config.json`](config/wato_asd_training_foxglove_config.json)
(Layouts → Import).

The layout has the 3D panel set up, plus a Teleop panel for driving the robot manually.

## Sending a goal

In a second terminal:

```bash
./watod -t robot    # open a shell inside the running robot container

ros2 topic pub /goal_point geometry_msgs/msg/PointStamped "{header: {frame_id: sim_world}, point: {x: -12.0, y: 2.0}}"
```

Pick goals in open ground — a goal inside a large obstacle will still plan a route.
See [docs/06](docs/06-planner-node.md#limitations).

## Docs

| Step | What it covers |
|---|---|
| [00 — Setup](docs/00-setup.md) | Getting the repo running on a fresh machine |
| [01 — Configure watod](docs/01-configure-watod.md) | Which modules launch, and what watod does |
| [02 — Code a node](docs/02-code-a-node.md) | Writing a first publisher |
| [03 — Test the node](docs/03-test-the-node.md) | Build, run, and see it in Foxglove |
| [04 — Costmap](docs/04-costmap-node.md) | LiDAR scans into a local occupancy grid |
| [05 — Map memory](docs/05-map-memory-node.md) | Fusing costmaps into a world-fixed map |
| [06 — Planner](docs/06-planner-node.md) | A\* across the map to a goal |
| [07 — Control](docs/07-control-node.md) | Pure Pursuit, turning the route into wheel speeds |

## Credit

Scaffolded by [WATonomous](https://www.watonomous.ca/) — their
[starter repo](https://github.com/WATonomous/wato_asd_training) and
[spec](https://wiki.watonomous.ca/admission_assignments/asd_admission_assignment/).
Licensed under Apache 2.0.
