# Bristol FSAI Simulator 2 (Docker)

Bristol FSAI workspace built on top of [eufs_sim2](https://gitlab.com/eufs/public/eufs_sim2)
(EUFS's new ROS 2 Humble simulator that replaces the Gazebo-based `eufs_sim`).

This repo bundles every package needed to build and run the simulator, so a teammate
on Windows / macOS / Linux only needs **Docker Desktop + VSCode + the Dev Containers
extension** — no Ubuntu, no ROS install on the host.

## Versions

- OS in container: Ubuntu 22.04 (Jammy)
- ROS: ROS 2 Humble
- Visualisation: [Foxglove Studio](https://foxglove.dev/) via `foxglove_bridge` on port `8765`

## Contents

| Package | Source | Purpose |
| --- | --- | --- |
| `eufs_sim2` | gitlab.com/eufs/public/eufs_sim2 | Simulator core |
| `eufs_msgs` | gitlab.com/eufs/eufs_msgs | Message / service / action types |
| `eufs_logger` | gitlab.com/eufs/public/eufs-logger | Logging library |
| `eufs_gmock_matchers` | gitlab.com/eufs/public/eufs-gmock-matchers | Test matchers |
| `state_lib` | gitlab.com/eufs/public/state_lib | Vehicle state library |
| `map_lib` | gitlab.com/eufs/public/map_lib | Cone map / track library |
| `vehicle_models` | gitlab.com/eufs/public/vehicle_models | Vehicle dynamics models |
| `ros_can` | Bristol FSAI | FS-AI-API bridge for the real car |
| `eufs_sim_foxglove_plugins` | gitlab.com/eufs/public/eufs-sim-foxglove-plugins (patched) | **Foxglove Studio** extension — renders `ConeWithColorProbabilityArray` (eufs_sim2 cone format), `ConeArrayWithCovariance`, and `CarForces`; plus Joystick / Mission State / Set Map panels |
| `bri_cli/` | Bristol FSAI | Convenience CLI — `bri build` / `bri sim run` / `bri clean`. See [bri_cli/README.md](bri_cli/README.md) |

## Setup

1. Install [Docker Desktop](https://www.docker.com/products/docker-desktop/) and
   [VSCode](https://code.visualstudio.com/) with the **Dev Containers** extension.

2. Clone this repo:

   ```bash
   git clone https://github.com/Formula-Student-AI/bristol_fsai_sim2-docker.git
   cd bristol_fsai_sim2-docker
   ```

3. Open in VSCode and run **"Dev Containers: Reopen in Container"**
   (Ctrl/Cmd-Shift-P → type "reopen").

   First build takes ~5–10 min while the image installs ROS Humble + system deps.
   Subsequent opens are instant.

4. Inside the container, build and run via the bundled `bri` CLI (see
   [`bri_cli/README.md`](bri_cli/README.md)):

   ```bash
   bri build       # colcon build --symlink-install
   bri sim run     # ros2 launch eufs_sim2 eufs_sim2.launch.py
   ```

   `bri sim run` also (re)starts `foxglove_bridge` on port `8765` automatically.
   Raw `colcon` / `ros2` commands still work if you prefer them.

5. From your host, open [Foxglove Studio](https://app.foxglove.dev/) and connect to
   `ws://localhost:8765`. Topics like `/odom`, `/cones`, `/imu/data`, `/cmd` should
   appear.

   > **VSCode PORTS tab**: if `ws://localhost:8765` doesn't connect, check that port
   > `8765` appears in the **PORTS** panel (bottom bar) with a green dot. VSCode
   > Dev Containers forwards it automatically.

## Installing the Foxglove extension (one-time, host-side)

The simulator publishes EUFS-specific message types that Foxglove Studio cannot render
natively. Install the bundled extension **into Foxglove Studio on your host** (not into
the dev container — this is a frontend extension, not a ROS package).

**Easy path — drag-and-drop the pre-built `.foxe`:**

1. Open Foxglove Studio.
2. Drag
   [`eufs_sim_foxglove_plugins/edinburghuniversityformulastudent.eufs-sim-foxglove-plugins-0.0.1.foxe`](eufs_sim_foxglove_plugins/)
   onto the Foxglove window.
3. Reload — coloured cones (`ConeWithColorProbabilityArray`), `WheelForceArrows`,
   Joystick, Mission State, and Set Map panels are now available.

> **Note:** this `.foxe` is a Bristol-patched build that adds support for the
> `ConeWithColorProbabilityArray` message type used by eufs_sim2 (the upstream
> extension only handles the older `ConeArrayWithCovariance` format).

**Dev path — build from source** (only needed if you edit the plugin):

```bash
cd eufs_sim_foxglove_plugins
npm install
npm run package        # produces a new .foxe in this directory
# then drag the .foxe into Foxglove Studio
```

## Driving the simulator

After `bri sim run`, use these commands in a new terminal to drive the car:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 1. Select mission (14 = Trackdrive, 11 = Acceleration, 12 = Skidpad, ...)
ros2 service call /set_mission eufs_msgs/srv/SetMission "{mission: 14}"

# 2. Send GO signal (transitions: OFF → READY → DRIVING)
ros2 service call /go std_srvs/srv/Trigger "{}"

# 3. Publish control commands  ← use acceleration, not speed
ros2 topic pub /cmd ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {acceleration: 2.0, steering_angle: 0.0}}" --rate 10
```

> **Control interface:** `/cmd` uses the `acceleration` field (m/s²) and
> `steering_angle` (radians). The `speed` field is not used by the simulator — your
> own velocity controller should convert a speed setpoint to an acceleration command
> before publishing to `/cmd`.

To reset between runs:

```bash
ros2 service call /reset std_srvs/srv/Trigger "{}"
```

In Foxglove Studio, add the **Mission State [local]** and **Joystick [local]** panels
(from the EUFS extension) for a GUI-based alternative to the CLI commands above.

## Updating an EUFS package

Each EUFS package is a normal `git clone`, not a submodule. To pull upstream changes
for one of them:

```bash
cd eufs_sim2 && git pull && cd ..
colcon build --packages-select eufs_sim2 --symlink-install
```

## How to contribute

```bash
git checkout -b your_branch_name
# ... edit files (live, on host — symlink-install picks them up) ...
git add path/to/file
git commit -m "describe change"
git push -u origin your_branch_name
```

## Credit

- EUFS: <https://gitlab.com/eufs>
- Dev container template: [ijnek/ros-devcontainer-template](https://github.com/ijnek/ros-devcontainer-template)
- Original Bristol fork: [bristol_fsai_sim-docker](https://github.com/Formula-Student-AI/bristol_fsai_sim-docker) (eufs_sim 1, Galactic)
