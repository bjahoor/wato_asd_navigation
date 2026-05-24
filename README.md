# wato_asd_navigation

My implementation of the [WATonomous ASD admission assignment](https://wiki.watonomous.ca/admission_assignments/asd_admission_assignment/) —
autonomous navigation for a simulated differential-drive robot in ROS 2 (C++).

A fork of [WATonomous/wato_asd_training](https://github.com/WATonomous/wato_asd_training),
used as a personal project and learning journal. I document what I learn as I go in [`docs/`](docs/).

## The system

A four-node ROS 2 navigation pipeline:

- **Costmap** — LiDAR → occupancy grid
- **Map memory** — fuse costmaps into a global map
- **Planner** — A* to a goal
- **Control** — Pure Pursuit path following

## Credit

Original assignment and scaffold by [WATonomous](https://www.watonomous.ca/). Licensed under Apache 2.0.
