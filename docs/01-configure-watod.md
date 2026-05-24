# Configure watod

`watod-config.local.sh` decides which modules launch. (This file is normally gitignored, but I commit it here so the repo is self-contained.)

```bash
ACTIVE_MODULES="robot gazebo vis_tools"
```

- `robot` — the four nodes (costmap, map_memory, planner, control)
- `gazebo` — the simulator
- `vis_tools` — Foxglove visualizer

## Build and run

`watod` wraps `docker compose` and adds the setup glue around it — it generates the `.env` file (image names, ports, tags), handles the build platform (amd64/arm64), and provides shortcuts like `-t` to open a shell inside a running container. It reads `watod-config.local.sh` to know which modules to act on.

```bash
cd ~/wato_asd_navigation
./watod build   # compiles the code inside the containers
./watod up      # starts the simulator, nodes, and Foxglove
```

- **`build`** — get everything ready: package the code into runnable container images. Nothing runs yet.
- **`up`** — the "on" switch: start the containers (simulator, the 4 nodes, Foxglove) and connect them so they talk to each other.

Workflow when editing code: edit → `./watod build` → `./watod up`. Running `up` alone reuses the old image, so always `build` after a code change.

## View in Foxglove

In the Foxglove desktop app: Open connection → Foxglove WebSocket → `ws://localhost:20000`
