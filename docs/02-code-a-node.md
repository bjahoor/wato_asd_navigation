# Code a node and see it in Foxglove

## Set up IntelliSense

```bash
./watod --setup-dev-env robot
```

Copies the ROS headers out of the Docker image to `/tmp/deps` and writes `.vscode/` config so VSCode can resolve ROS includes.

## Add a publisher to the costmap node

Make the node publish "Hello, ROS 2!" to `/test_topic` every 500ms. Changed four files in `src/robot/costmap/`:

- `include/costmap_node.hpp` — declare the publisher, a timer, and a `publishMessage()` callback.
- `src/costmap_node.cpp` — create the publisher + 500ms timer in the constructor; `publishMessage()` builds and publishes the string.
- `package.xml` — add `<depend>std_msgs</depend>` (the String message type).
- `CMakeLists.txt` — `find_package(std_msgs)` and add it to `ament_target_dependencies`.

`package.xml` declares the dependency; `CMakeLists.txt` finds and links it.
