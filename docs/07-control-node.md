# Control node

The last of the four: a route in, wheel speeds out. This is the node that actually moves the robot.

- **In:** `/path` — `nav_msgs::msg::Path`
- **In:** `/odom/filtered` — `nav_msgs::msg::Odometry`
- **Out:** `/cmd_vel` — `geometry_msgs::msg::Twist`

## What it does

Follows the planner's route using **Pure Pursuit**.

The idea is the one you already use when driving: you don't steer at the tarmac under the bumper,
you pick a point some way down the road and steer at that. Move forward, pick a new point, steer
again. The path comes out smooth without anyone calculating a smooth path.

So, ten times a second: find the point on the route about a metre ahead, work out the curve that
would take the robot there, and send the wheel speeds that follow that curve.

It never looks at the map. It trusts the route completely — checking for obstacles is the job of
the two nodes upstream.

## Specs

| | value | why |
|---|---|---|
| lookahead distance | 1.0 m | far enough to smooth out the grid's stair-steps, near enough to still take the corners |
| goal tolerance | 0.1 m | tighter than the planner's 0.5m, so the approach is finished properly rather than abandoned |
| linear speed | 0.5 m/s | constant — steering does all the work |
| control rate | 10 Hz | fast enough that the aim point moves smoothly, slow enough to be free |
| turn-in-place above | 60° | see below |
| max angular speed | 1.5 rad/s | a ceiling, so a near-reversal doesn't ask for a spin the robot can't deliver |

## Choices

| decision | picked | why |
|---|---|---|
| lookahead point | first waypoint at least 1m out, walking from the start of the path | waypoints behind the robot get skipped automatically as it passes them |
| every waypoint closer than 1m | aim at the last one | means the end of the route is in reach, so drive at the goal itself |
| heading error over 60° | stop and turn on the spot | a differential drive can pivot. Pure pursuit alone would swing out in a wide loop first, which looks wrong and wastes ground. |
| empty path | stop | the planner has nothing for us; coasting on a stale route is worse than standing still |
| arriving | latch, and publish one stop | otherwise the loop keeps re-sending zero speeds forever |

Turn-in-place and the angular ceiling are additions, not part of the standard algorithm. Pure
pursuit assumes a car, which cannot pivot. This robot can, so it should.

## How I built it

Same split as the other three:

- `control_node.{hpp,cpp}` — subscribers, publisher, timer, and the loop
- `control_core.{hpp,cpp}` — the geometry, no ROS logic

Order of work:

1. `package.xml` and `CMakeLists.txt` — add `nav_msgs` and `geometry_msgs`
2. `control_core.hpp` — the four functions the loop needs, and the constants
3. `control_core.cpp` — lookahead selection, then the steering maths
4. `control_node.hpp` — subscribers, publisher, timer, stored state
5. `control_node.cpp` — the loop
6. `./watod build robot`, then send a goal and watch it drive

## What runs

```
/path ──► store
                 ┌──────────────────────────────────────────┐
/odom ──► store  │ arrived? ──► find aim point ──► steer ──► │──► /cmd_vel
                 └──────────────────────────────────────────┘
                                  10Hz timer
```

Both callbacks only store. Everything happens on the timer, so the robot keeps being commanded at a
steady rate no matter how irregularly the path and odometry arrive.

Each tick, in order:

1. **Nothing yet?** No path or no odometry — send nothing at all, don't guess
2. **Arrived?** Within 0.1m of the route's end — stop, and latch so it stays stopped
3. **Pick the aim point** — first waypoint at least a metre away
4. **Steer** — convert that into a forward and a turning speed
5. **Publish**

## How Pure Pursuit works

Two numbers describe where the aim point is relative to the robot:

- **alpha** — the angle between where the robot is pointing and where the aim point is. Zero means
  dead ahead.
- **distance** — how far away it is.

From those, the arc that starts at the robot, matches its current heading, and passes through the
aim point has curvature:

```
curvature = 2·sin(alpha) / distance
```

Curvature is just "how tight the bend is" — the reciprocal of the circle's radius. Big number,
tight turn. Then, since driving at speed `v` along a curve of that tightness means rotating at:

```
angular = v × curvature
```

That's the whole algorithm. Speed stays fixed at 0.5 m/s; only the turn rate changes.

Two details in this implementation:

**alpha is wrapped into ±180°.** Subtracting two headings can produce 350° when the truth is −10°,
which would send the robot spinning the long way round.

**Heading comes from a quaternion.** Odometry reports orientation as four numbers; `extractYaw`
pulls the one rotation that matters on flat ground out of them.

## Functions

| function | file | what it does |
|---|---|---|
| `pathCallback` | node | stores the newest route, and clears the arrived latch |
| `odomCallback` | node | stores the robot's pose |
| `controlLoop` | node | the five steps above, once per tick |
| `stop` | node | publishes an all-zero `Twist` |
| `findLookaheadPoint` | core | first waypoint at least a lookahead out, or the last one |
| `computeVelocity` | core | alpha, curvature, then the two speeds |
| `computeDistance` | core | straight-line distance between two points |
| `extractYaw` | core | quaternion → heading |
| `goalReached` | core | distance to the route's final waypoint against the tolerance |

## Limitations

- **Constant speed through every turn.** Real controllers slow down for tight bends. This one takes
  a hairpin at the same 0.5 m/s as a straight.
- **Fixed lookahead.** It should grow with speed and shrink on tight sections. One metre is a
  compromise that suits neither extreme.
- **Trusts the route completely.** It never reads the map, so if the planner routes through
  something the map never saw, this node drives into it at full speed.
- **No recovery.** Blocked, stuck, or wedged against something, all it can do is keep trying to
  follow the route. There is no reverse and no give-up.
- **Cannot reverse.** Forward speed is never negative, so getting out of a dead end means turning
  around in place.
- **Constants are compiled in;** `config/params.yaml` is unused.

## Verify

Rebuild and start the stack as in [03](03-test-the-node.md). In one shell, send a goal in open
ground:

```bash
./watod -t robot
ros2 topic pub /goal_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: sim_world}, point: {x: -12.0, y: 2.0}}"
```

In a second shell:

```bash
./watod -t robot
ros2 topic echo /cmd_vel            # linear.x 0.5 while driving, all zeros on arrival
ros2 topic echo /odom/filtered --field pose.pose.position
```

The logs should show `Arrived, stopping` from the control node.

In Foxglove, add `/map`, `/path` and the robot to the 3D panel. The robot should pivot on the spot
if the route starts behind it, then follow the line without weaving.

Pick goals in open ground. A goal inside a large obstacle will still plan a route, because the map
only records the surfaces the lidar has actually hit — see the limitations in
[06](06-planner-node.md).
