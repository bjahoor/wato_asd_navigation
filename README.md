# wato_asd_navigation

My implementation of the [WATonomous ASD admission assignment](https://wiki.watonomous.ca/admission_assignments/asd_admission_assignment/) —
autonomous navigation for a simulated differential-drive robot in ROS 2 (C++).

A fork of [WATonomous/wato_asd_training](https://github.com/WATonomous/wato_asd_training),
used as a personal project and learning journal. I document what I learn as I go in [`docs/`](docs/).

## The system

A four-node ROS 2 navigation pipeline. Each node takes what the one before it produced and turns
it into something closer to a wheel command.

```
   /lidar         /odom/filtered        /goal_pose        /odom/filtered
      │                  │                    │                  │
      ▼                  ▼                    ▼                  ▼
 ┌──────────┐      ┌────────────┐      ┌───────────┐      ┌───────────┐
 │ costmap  │─────►│ map memory │─────►│  planner  │─────►│  control  │─────►
 └──────────┘      └────────────┘      └───────────┘      └───────────┘
            /costmap            /map               /path              /cmd_vel

  what I see        everything I've      a route to        wheel speeds
  right now         seen so far          the goal          to follow it
```

- **Costmap** — LiDAR → occupancy grid. What's around the robot right now. [docs/04](docs/04-costmap-node.md)
- **Map memory** — fuse costmaps into a global map. What's been seen so far. [docs/05](docs/05-map-memory-node.md)
- **Planner** — A* from the robot to a goal, over the map
- **Control** — Pure Pursuit, turning that route into wheel speeds

## Credit

Original assignment and scaffold by [WATonomous](https://www.watonomous.ca/). Licensed under Apache 2.0.
