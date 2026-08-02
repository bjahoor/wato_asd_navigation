# Test the node

Two terminals. `watod up` streams logs and holds the terminal until Ctrl-C.

## Terminal 1 — start everything

```bash
./watod up      # no arguments = every module in ACTIVE_MODULES
```

Starts the robot, Gazebo, and Foxglove. Leave it running. Naming a service (`./watod up robot`)
starts only that one, so Foxglove stays down and `ws://localhost:20000` is unreachable.

```bash
docker ps       # Foxglove is up when its port shows 0.0.0.0:20000->20000/tcp
```

## Terminal 2 — the edit loop

After changing code, restart just the robot so Gazebo and Foxglove keep running:

```bash
./watod down robot
./watod build robot
./watod up robot
```

`costmap_node` should start logging `Publishing: 'Hello, ROS 2!'` twice a second.

## See it in Foxglove

Add a **Raw Messages** panel on `/test_topic`:

```
data  "Hello, ROS 2!"
```
