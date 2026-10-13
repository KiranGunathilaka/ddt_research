# bri — Bristol FSAI Sim2 CLI

A small bash CLI bundled inside the workspace. Wraps the most common ROS 2
commands so you can do daily work without retyping `ros2 launch` and
`colcon build`.

## Commands

| Command | What it runs |
| --- | --- |
| `bri build [pkg...]` | `colcon build --symlink-install` (whole workspace, or selected packages) |
| `bri sim run [args]` | `ros2 launch eufs_sim2 eufs_sim2.launch.py` (extra args passed through) |
| `bri clean` | `rm -rf build install log` |
| `bri help` | List commands |

The `foxglove_bridge` ROS node is started automatically by the dev container
on `ws://localhost:8765` (see `.devcontainer/devcontainer.json`'s
`postStartCommand`), so `bri sim run` only launches the simulator and its logs
stay clean.

## Adding a new command

Drop an executable bash file into `commands/`:

```bash
# bri_cli/commands/test.sh
#!/usr/bin/env bash
# desc: run all colcon tests

set -euo pipefail
cd "$EUFS_MASTER"
colcon test
colcon test-result --verbose
```

Make it executable (`chmod +x bri_cli/commands/test.sh`) and it shows up in
`bri help` automatically — no dispatcher edits needed. Multi-level commands
(e.g. `bri sim run`) work the same way: the file `commands/sim.sh` handles
all `bri sim ...` invocations.

## Install (manual, for non-devcontainer use)

Inside the dev container `bri` is already on `PATH` (the container's
`postCreateCommand` calls `install.sh`). If you want to use it from a host
shell that has ROS Humble installed:

```bash
export EUFS_MASTER=/path/to/bristol_fsai_sim2-docker
bash "$EUFS_MASTER/bri_cli/install.sh"
```
