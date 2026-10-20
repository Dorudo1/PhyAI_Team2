# PhyAI Team2: ROS 2 apps for the PIPER arm

**English** | [한국어](README.ko.md)

Develop an app in Docker, test it with MuJoCo, then run its runtime image on the control PC.

[Setup](#1-setup) → [Implement](#2-implement) → [Test](#3-test-with-mujoco) → [Build](#4-build-the-runtime-image) → [Run on the control PC](#5-run-on-the-control-pc)

## 1. Setup

**What:** Open the shared Ubuntu 22.04 + ROS 2 Humble development environment.

**How:** Open this repo in VS Code and run **Dev Containers: Reopen in Container**.

**Expected result:** A container terminal with ROS 2 available; the repo is at `~/ws/src/PhyAI_Team2`.

### Requirements

| Where | Required |
|---|---|
| Dev PC running Docker | Linux (e.g. Ubuntu 24.04), NVIDIA driver, Docker, NVIDIA Container Toolkit |
| PC running VS Code | Dev Containers extension; Remote - SSH extension for remote work |
| Inside the container | App-specific ROS dependencies (`rosdep`) and Python libraries (`pip`) |

The Docker image includes Ubuntu 22.04 and ROS 2 Humble. **The dev PC host can use Ubuntu 24.04**;
it does not need Ubuntu 22.04 or a separate ROS installation. The current devcontainer uses Linux
host networking and an NVIDIA GPU for the simulator view. A laptop used for SSH can run any OS.

### Local or remote access

- **Local:** Clone the repo on the dev PC, open it in VS Code, then reopen in the container.
- **Remote:** Connect to `<user>@<dev-pc>` with **Remote-SSH: Connect to Host...**, open the repo
  on that PC, then reopen in the container. Docker runs on the dev PC.

The first container build takes a few minutes. `~/ws` is the colcon workspace.

### Check the environment

Run in the container:

```bash
ros2 doctor --report | head
nvidia-smi
```

ROS diagnostics and GPU information should appear.

## 2. Implement

**What:** Create an app under `apps/<name>/`.

**How:** Copy the example, rename its ROS package, and edit its Python code.

**Expected result:** Your app has a `start.sh` and sends commands through ROS topics.

```bash
cd ~/ws/src/PhyAI_Team2
cp -r apps/example apps/<name>
mv apps/<name>/example_app apps/<name>/<your_pkg>
```

### Rename and organize the app

Replace `example_app` with `<your_pkg>` in `package.xml`, `setup.py`, `setup.cfg`, `resource/`,
the Python package folder, the launch file, and `start.sh`.

```text
apps/<name>/
├── start.sh                 # starts your app
├── requirements.txt         # optional pip dependencies
└── <your_pkg>/              # one or more ROS packages
    ├── package.xml
    ├── setup.py / setup.cfg / resource/<your_pkg>
    ├── <your_pkg>/*.py
    └── launch/*.launch.py
```

Use lowercase letters, digits, `-`, and `_` for `<name>`. ROS package names must be unique across
the repo. Keep app files inside `apps/<name>/`; runtime builds include that app and the shared
packages under `src/`.

### Robot interface

Your app publishes targets; the simulator or control PC bridge receives them and returns feedback.

| Topic | Message | Direction |
|---|---|---|
| `/joint_ctrl_single` *(provisional)* | `sensor_msgs/JointState`: `joint1`–`joint6` angles (rad), `joint7` gripper opening (0–0.035 m) | app → arm |
| `/joint_states` | `sensor_msgs/JointState`: `joint1`–`joint8` positions and velocities | arm → app |
| Camera topics *(TBD)* | `sensor_msgs/Image`, `CameraInfo` | host → app |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | host → app |
| `/robot_description` | `std_msgs/String` (URDF) | host → app |
| `/sim/scene` *(simulator only)* | `std_msgs/String`: JSON block poses, robot kinematics, simulation time, pause state, restart ID | simulator → example |

The host stack administrator confirms provisional topics and camera names. Arm commands currently
follow the `piper_ros` convention. If the bridge uses custom messages, add their package to the base image.

### Use discovery in your own app

The shared [discovery core](src/phyai_discovery/README.md) accepts calibrated RGB-D
captures directly from Python. ROS apps can use
[`phyai_discovery_ros`](src/phyai_discovery_ros/README.md) for live cameras,
capture-time TF, stationary feedback checks, and workspace/progress topics:

```bash
# From the colcon workspace; no Junh app or simulator is required.
colcon build --symlink-install --packages-up-to phyai_discovery_ros
source install/setup.bash
# From the repository root, after preparing your installation JSON:
bash scripts/run_discovery.sh --config /absolute/path/hardware.json -- auto_scan:=false
```

For SAM2-assisted discovery with automatic dependency/model setup, use
`bash scripts/run_discovery.sh --config /absolute/path/installation.json -- auto_scan:=false`
after sourcing the built workspace. See the shared guide for first-run downloads
and runtime image provisioning. Direct launch with the raw template is depth-only.

Copy the shared package's configuration template and set your measured camera,
table, and sensor error settings first. Enable `auto_scan:=true` with configured
scan poses to let the shared PIPER controller move and capture, or have your app
own movement and request captures. Junh adds simulator-specific geometry planning
to the shared scan controller; physical scan poses and paths still require
installation validation. The shared adapter is available for hardware
integration; real-sensor accuracy and hardware trajectories still need validation.

### Dependencies and installed files

| File | What to declare |
|---|---|
| `package.xml` | Every ROS/rosdep dependency: `<exec_depend>` for runtime, `<build_depend>` for build, `<depend>` for both |
| `requirements.txt` | Non-ROS Python dependencies without rosdep keys; pin versions |
| `install_system_deps.sh` | Optional system libraries or CUDA Toolkit; run as root during image build |
| `setup.py` | Executables in `console_scripts`; launch, config, URDF, and model files in `data_files` |
| `start.sh` | Launch command, using `exec`; LF line endings |

Example executable registration: `'example = example_app.example:main'`.
Keep `setup.cfg` and `resource/<your_pkg>` so ROS can find the executable. Load installed assets with
`get_package_share_directory('<your_pkg>')`. Check rosdep keys with `rosdep resolve <key>`.

Runtime images install only execution dependencies and installed app files. Missing declarations
can cause `ModuleNotFoundError` or missing files after deployment. Add only imported pip libraries;
large packages slow image transfers. Hardware SDKs and drivers belong on the host.

### Architecture and runtime constraints

```text
Control PC host: Ubuntu 22.04 + ROS 2 Humble
├── piper_bridge → piper_sdk → CAN → arm
├── camera driver
└── robot_state_publisher
          ↕ ROS 2 / DDS
Runtime container (--network host)
└── your app
```

The host owns all hardware. Apps use ROS topics and have no device access. Runtime apps run as
`dev`, with ROS and the app sourced, and no display. Dev and runtime images share the same ROS base;
[check the runtime image](#4-build-the-runtime-image) before deployment to verify its dependencies.

## 3. Test with MuJoCo

**What:** Run your app against a simulated arm.

**How:** Run the test script inside the devcontainer, then open the browser URL.

**Expected result:** The example picks one reachable block, lifts it, releases it above
the table, and returns home. The terminal reports the stages and verified outcome.

```bash
cd ~/ws/src/PhyAI_Team2
bash scripts/test_sim.sh example
```

Open **http://localhost:23517**. Press **Ctrl+C** to stop.

### Select your app

Install your app's declared ROS dependencies with `rosdep` and any `requirements.txt` packages first.
Then run:

```bash
bash scripts/test_sim.sh <name>
bash scripts/test_sim.sh          # select from a menu
```

The script builds the selected app and simulator, starts or reuses MuJoCo, then runs
`apps/<name>/start.sh`. Your app controls motion through the [robot interface](#robot-interface).
[example.py](apps/example/example_app/example_app/example.py) sends smooth arm/gripper targets at
50 Hz for a fixed pick-and-drop sequence using the verified joint waypoints in
[pick_drop.py](apps/example/example_app/example_app/pick_drop.py). The known layout uses
`block_3` at (0.40, -0.33) m, 7.5 × 2.5 × 1.5 cm blocks, and the original base pose.
It uses `/sim/scene` to verify the lift and landing; there is no runtime IK search.
Scene-editor pauses also pause the sequence. **Restart simulation** in either browser
page restores the arm and blocks and reruns the example without restarting its process.
Restart keeps active scene settings and does not save or reload the JSON file.
`sim_cmd.sh` is optional for manual commands.

### Browser access

| Access | URL |
|---|---|
| On the dev PC | `http://localhost:23517` |
| Another machine on the LAN | `http://<dev-pc-ip>:23517` |
| Through SSH | Forward the port in VS Code's **Ports** panel, then use `http://localhost:23517` |

Use the printed URL if you set another port. Rendering uses offscreen GPU/EGL and streams video;
no desktop display is needed. The simulator is development-only. The browser includes virtual camera views. Enable `publish_cameras:=true` to publish rendered RGB-D, CameraInfo, and camera/base transforms on `/tf` for perception apps.
Its model comes from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/agilex_piper).

### Lab scene and camera views

The default scene contains four 84 × 54 cm black tabletops in a 2 × 2 layout,
a PIPER arm mounted near the combined table center, and six movable 7.5 × 2.5 × 1.5 cm blocks.
Open the browser and select **Overview**, **Overhead camera**, or **Wrist camera**.
The overview includes an X/Y/Z orientation gizmo; the overhead view shows X/Y.
The arrows indicate positive world directions: X red, Y green, and Z blue.
The overhead lens is 89 cm above the tabletop, looking straight down from
X = -3 cm and Y = +2 cm relative to the combined table center. The wrist camera follows `link6`.
These are ideal RGB browser views, not an SR305 sensor/noise simulation or ROS RGB-D streams.
Four overhead lights, ambient room fill, and camera-facing fill illuminate the scene.
The `lighting` settings in `lab_scene.json` control ceiling height above the tabletop
and ambient/diffuse/specular RGB strengths; `camera_fill` sets the camera-facing diffuse light. These are visual intensities, not calibrated
lux measurements. Stop and relaunch the simulator process after changing lighting;
the browser **Restart simulation** button only resets the robot and blocks.

Edit [lab_scene.json](sim/piper_mujoco_sim/config/lab_scene.json) to adjust the layout.
All distances are in meters. The world origin is the combined tabletop center; +Z points up,
+X follows the 168 cm dimension, and +Y follows the 108 cm dimension.
The robot base and post center are at X = +3 cm and Y = -2 cm relative to that origin,
configured by `post_position`. The overhead lens uses `overhead_position` in the same frame.
The 1 mm gap, 75 cm table height, 4 cm post radius, robot heading, 20 g block mass,
friction, and exact wrist pose are initial estimates.
The camera assembly is rigidly attached to the arm casing (`link5`), before the rotating
`joint6`. Mount coordinates use `gripper_root_mount`, a reference frame aligned with
the gripper root at zero wrist roll and fixed to that casing. Its +X is opposite
zero-roll `link6` +X. `wrist_camera_mount_position` and `wrist_camera_mount_quaternion` move
and rotate the whole assembly, including its pad, housing, and lens.
`wrist_camera_position` and `wrist_camera_quaternion` set the camera lens pose relative
to that mount (meters and `[w, x, y, z]`);
they do not change the robot's wrist pose or joint positions.
The silver camera bar spans perpendicular to the arm and attaches directly to the
wrist casing through a thin pad, with no post. The hierarchy is
`link5` → `wrist_mount_reference` → `gripper_root_mount` → `wrist_camera_mount`
→ `wrist_camera_body` → `wrist` camera. The camera follows the casing exactly as
the arm moves; gripper roll and finger opening do not move it relative to the casing.
Housing dimensions are approximate. The lens looks parallel to the gripper's forward
direction (gripper-root +Y, or `link6` +Z), rather than converging on the fingers,
and the camera looks along its own -Z. `wrist_camera_roll_degrees` adjusts optical roll
independently of the housing; the default is 0 because the mount frame now has
image-up toward gripper-root +X and the bar width along gripper-root Z. Negative roll rotates
the image counterclockwise.
The leader arm is omitted.

The default browser page (`/`) is a read-only viewer. Open `/edit` explicitly for the
**Scene editor**, or launch with `scene_editing:=true` to show the editor at `/` too:

```bash
ros2 launch piper_mujoco_sim sim.launch.py scene_editing:=true
# Or, when starting a new simulator through the helper:
SCENE_EDITING=true bash scripts/test_sim.sh
```

The helper prints both URLs; a reused simulator keeps its existing homepage mode.
Editing APIs are confined to `/edit/scene`; viewing `/` does not activate editing for other viewers.
The editor supports the arm base, camera mount, camera lens,
overhead camera, field of view, and blocks. Positions use centimeters and rotations use
degrees, with the parent frame shown beside each object. Preview applies edits live and
pauses physics and incoming arm commands; **Resume simulation** continues motion.
**Revert** restores the last saved layout, **Save scene** writes the configured JSON file,
and **Reload scene file** loads pose edits made in your text editor. Custom MJCF models are view-only.
Table dimensions and other structural settings still require a restart after manual editing.

Restart the simulator after manually editing structural scene settings. An already running simulator reused by
`test_sim.sh` keeps its old scene until stopped. To inspect without the example app moving the arm:

```bash
cd ~/ws
colcon build --symlink-install --packages-select piper_mujoco_sim
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch piper_mujoco_sim sim.launch.py
```

A different configuration can be selected with `scene_config:=/absolute/path/config.json`.
To load an existing MJCF instead, use `model_path:=/absolute/path/scene.xml`.

### Use block scattering from your own app

Scattering is provided by `piper_mujoco_sim` and does not require the Junh app,
discovery nodes, or SAM 2. Build/source the simulator as above, then launch:

```bash
export ROS_DOMAIN_ID=43
ros2 launch piper_mujoco_sim sim.launch.py \
  http_port:=23518 publish_cameras:=true \
  scatter_on_start:=true scatter_seed:=42 scatter_table:=all
```

Blocks scatter and settle before the first camera frame. Launch your own app in
another sourced terminal using the same ROS domain. With the same scene/model,
`scatter_seed:=42` reproduces the layout; omit it for a new random layout.
`scatter_table` accepts `all`, `0_0`, `0_1`, `1_0`, or `1_1`.
The shared launch defaults to aligned blocks unless `scatter_on_start:=true`.

To change a running scene or restore its aligned startup layout:

```bash
ros2 run piper_mujoco_sim scatter_blocks scatter --seed 17 --url http://localhost:23518
ros2 run piper_mujoco_sim scatter_blocks reset --url http://localhost:23518
```

Both actions also reset the robot and increment `restart_id` on `/sim/scene`.
Your app should clear cached observations and restart its own task when that ID
changes. The simulator does not automatically restart another app's state machine.
Scattering/reset do not rewrite the scene file. This feature requires the lab
scene selected through `scene_config`; custom `model_path` scenes do not support it.

Apps can also POST JSON directly to `/simulation/blocks/scatter` with
`{"seed":17,"table":"all"}`, or `/simulation/blocks/reset` with `{}` on the
simulator's HTTP port. Responses include `restart_id` and block poses for test
evaluation. Those poses are simulator ground truth, separate from camera-based
perception. The HTTP client selects the simulator by URL, independently of ROS domain.

### ROS domain, port, and existing servers

| Setting | Default | Purpose |
|---|---|---|
| `ROS_DOMAIN_ID` | `42` | Shared ROS domain for app and simulator |
| `HTTP_PORT` | `23517` | Fixed browser port |

Give each developer a domain separate from teammates and the real robot:

```bash
ROS_DOMAIN_ID=43 HTTP_PORT=23518 bash scripts/test_sim.sh <name>
# Or set the domain for subsequent commands in this terminal:
export ROS_DOMAIN_ID=43
```

If the port is occupied, the script prints the process name, PID, and command. It reuses our MuJoCo
simulator only on the same ROS domain. Domain mismatches and unrelated servers produce instructions;
the script does not switch ports or stop existing servers. If ownership is unavailable, inspect the
listener on the host.

Ctrl+C stops the app and any simulator the script started. A reused simulator stays running.

### Run components manually

For separate app and simulator terminals, set the same `ROS_DOMAIN_ID` in both. Build and source first:

```bash
cd ~/ws
colcon build --symlink-install --packages-select <your_pkg> piper_mujoco_sim
source install/setup.bash
export ROS_DOMAIN_ID=43
ros2 launch piper_mujoco_sim sim.launch.py
```

In another terminal, source `~/ws/install/setup.bash`, set the same domain, and launch your app:

```bash
ros2 launch <your_pkg> <file>.launch.py
```

For a manual motion check, run from the repo root with the same domain:

```bash
bash scripts/sim_cmd.sh joint1=0.8 joint7=0.03
```

Stop any app continuously sending targets before manual commands, since its targets will overwrite them.

## 4. Build the runtime image

**What:** Package one app for deployment.

**How:** Build from the repo root in the **dev PC host shell**, outside the devcontainer.

**Expected result:** Docker image `phyai/app-<name>:latest`.

```bash
scripts/build_runtime.sh <name>
```

### Include CUDA and PyTorch

**What:** Declare GPU dependencies per app.

**How:** Edit `apps/<name>/requirements.txt` and, if needed, `install_system_deps.sh`, then rebuild.

**Expected result:** The build and runtime images contain the declared packages; no install is needed on startup.

For example, a CUDA 12.4 PyTorch wheel can be declared in `requirements.txt`:

```text
--extra-index-url https://download.pytorch.org/whl/cu124
torch==2.5.1+cu124
```

This is an example version pair; select the wheel for your app and GPU using the
[PyTorch installation instructions](https://pytorch.org/get-started/previous-versions/).
For a separate CUDA Toolkit/compiler, adapt the commented recipe in
[install_system_deps.sh](apps/example/install_system_deps.sh). The hook runs before pip installation,
from a read-only directory containing dependency declarations; use `/tmp` for downloads and keep it self-contained.
Its packages are included in the final image. Shell exports inside the hook do not persist to later stages.

Devcontainer installs are not copied into runtime images. For local testing, apply the same declarations
inside the devcontainer (`sudo bash apps/<name>/install_system_deps.sh`, then
`pip3 install -r apps/<name>/requirements.txt`). The test runner does not install these automatically.
The build needs network access for downloads; a prepared runtime image can run offline.
The control PC still provides a compatible NVIDIA driver and Container Toolkit.

### Check dependencies before building

Inside the devcontainer:

```bash
rosdep install --from-paths ~/ws/src/PhyAI_Team2/apps/<name> --ignore-src -y --simulate
```

### Verify the built image

In the dev PC host shell:

```bash
docker run --rm -it --network host -e ROS_DOMAIN_ID=43 phyai/app-<name>
```

Use your simulator's domain for feedback and motion testing. If the app works in development but
fails here, check [dependencies and installed files](#dependencies-and-installed-files).
`--symlink-install` can hide missing `data_files`, so test the actual runtime image.

## 5. Run on the control PC

**What:** Run a prepared runtime image against the control PC host stack.

**How:** Start the app in the control PC shell.

**Expected result:** The container runs `start.sh` and communicates with the robot through ROS topics.

```bash
# Control PC shell:
~/phyai/run_runtime.sh <name>
```

### Control PC requirements

- Docker; NVIDIA Container Toolkit if the app uses a GPU.
- A user in the `docker` group.
- The image `phyai/app-<name>:latest` and `~/phyai/run_runtime.sh` already available locally.
- The host stack (piper_bridge, camera driver, robot_state_publisher) running on ROS 2 Humble.

`run_runtime.sh` forwards the shell's `ROS_DOMAIN_ID` (default `0`); it must match the host stack.
Only one app may drive the arm at a time; the runner refuses to start if another app is running.

### Stop or debug

Press Ctrl+C, or stop from another terminal. To debug, start a shell instead of the app:

```bash
docker stop phyai-<name>
~/phyai/run_runtime.sh <name> bash
```

### DDS transport

Different host/container user IDs can cause Fast DDS shared-memory transport to drop data even
when topics are visible. Runtime images use [a UDP-only profile](docker/fastdds_udp.xml) to avoid this.
Keep `FASTRTPS_DEFAULT_PROFILES_FILE` set to that profile.
