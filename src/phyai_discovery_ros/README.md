# Shared discovery ROS adapter

Use `phyai_discovery_ros` from any ROS 2 Humble app to turn calibrated RGB-D
observations into a block inventory. It uses `phyai_discovery` and has no Junh,
MuJoCo, camera driver, robot driver, or motion planner dependency. The Junh app
uses this same adapter through its existing compatibility entry point. The package
also supplies an optional shared PIPER scan controller for position-command bridges.

The core and transport are available for hardware integration. Validation so far
uses synthetic/rendered data and simulated transport; real-camera accuracy,
calibration error bounds, and hardware scan trajectories are not yet validated.
Supported scenes remain static blocks of known cuboid dimensions on a
near-horizontal table, with `overhead` and `wrist` camera roles.

## Build and launch

Add `<exec_depend>phyai_discovery_ros</exec_depend>` to your app's `package.xml`.
From your colcon workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to phyai_discovery_ros
source install/setup.bash
cp "$(ros2 pkg prefix --share phyai_discovery_ros)/config/discovery.json" /tmp/hardware.json
# Edit /tmp/hardware.json for your installation before launching.
ros2 launch phyai_discovery_ros discovery.launch.py config_path:=/tmp/hardware.json
```

The default backend is depth only, requiring no model download or GPU. SAM2 is
optional through the core's [configuration](../phyai_discovery/README.md#sam2-configuration).
The repository's app runtime builds include shared packages under `src/`.

Set `joint_states_topic:=/your_arm/joint_states` when needed. The launch also
accepts `namespace` and `recording_path`; relative output/service names below gain
that namespace. Camera topic names come from JSON. Driver and adapter clocks must
agree; use actual acquisition timestamps and capture-time TF.

## Runtime image launch

App runtime images include the shared packages under `src/`. For an app whose
`start.sh` launches discovery, use the normal runtime runner. The Junh test app's
default startup still sources its development SAM2 environment; that startup is
not a ready-to-deploy hardware entry point. To test depth-only shared discovery
from the Junh image, override its command explicitly:

```bash
# On the dev PC host, outside the devcontainer:
bash scripts/build_runtime.sh junh
# Transfer/load the image on the control PC before running the command below.
# Put your measured installation JSON and referenced calibration/mask files here:
docker run --rm -it --network host \
  -e ROS_DOMAIN_ID=43 \
  -v /absolute/path/to/discovery-config:/config:ro \
  phyai/app-junh:latest \
  ros2 launch phyai_discovery_ros discovery.launch.py \
  config_path:=/config/installation.json auto_scan:=false
```

Use the host stack's ROS domain. Start observation-only, then enable
`auto_scan:=true` with a `scan` configuration after physical poses and paths are
validated. The example uses the depth backend; SAM2 needs explicit runtime model
and dependency provisioning. Native package builds and ROS integration are tested;
the Docker image build and physical deployment remain unverified.

## Configure the real installation

The installed JSON is a template, not measured calibration. Set:

- `map_frame` to the frame used by your table measurements and downstream app.
- Camera depth/CameraInfo topics, optical frames through their messages, and
  rectification settings. `16UC1` uses `depth_scale` (default 0.001 m/unit);
  `32FC1` depth is already metres. CameraInfo dimensions/frame must match depth.
- Capture-time TF from each optical frame into `map_frame`, including calibrated
  wrist-to-camera extrinsics and the arm's joint-dependent transforms. Set each
  camera's `calibration_revision` and restart discovery after calibration changes.
- `detector.known_block_size` in metres, measured `table_height_range: [zmin,zmax]`
  in the map frame, and optionally `scope: [xmin,xmax,ymin,ymax]` for the intended
  tabletop. Set `roi: [x0,y0,x1,y1]` to exclude background where useful. A table
  plane is measured from depth; the configured height interval only filters it.
- `detector.depth_error`, `calibration_position_error` in metres and
  `calibration_yaw_error` in radians from your measurements. Core defaults are
  assumptions, not a sensor accuracy guarantee.
- An exclusion mask when the robot or fixtures enter the table view: `mask` loads
  a boolean `.npy` relative to the JSON directory; `mask_topic` accepts `mono8`
  images with nonzero pixels excluded. Dynamic masks must match depth frame,
  dimensions, and acquisition time within 50 ms. The app/robot-mask provider owns
  their generation; this adapter does not project a robot model automatically.
- If using SAM2, registered RGB/depth pixels in the same optical frame and RGB
  within 50 ms of depth. Configure `rgb_topic`, checkpoint, and device explicitly.
- `viewpoints` to labels that your motion controller maps to validated physical
  arm poses, and `search_criteria`/`inspection_rounds` to your observation needs.
  Simulator joint targets and the MuJoCo inspection planner are not hardware
  collision-checked hardware paths. Start with the closed gripper used by PIPER.

Stationarity currently requires fresh `joint1`–`joint7` positions. The first six
are radians; tolerance for the gripper uses the bridge's native feedback units.
Use the same units when selecting the shared `stationary_tolerance`. Missing,
stale, invalid, or moving feedback prevents captures. Stationarity confirms that
motion stopped; your app must also verify it reached the requested physical pose.

## JSON camera intrinsics

Simulation and hardware should publish the same agreed image, feedback and control
topic names. Select the environment with `ROS_DOMAIN_ID`, set before starting the
process; the bridge, camera publishers and consuming app must share that domain.
Final hardware image topics are pending. The topic names in the template are
configurable placeholders until the team agrees on them. The optional shared scan controller uses the same command/feedback contract in
both environments.

CameraInfo delivery is optional when a JSON calibration file is configured:

```json
"overhead": {
  "depth_topic": "/overhead/depth/image_rect_raw",
  "info_topic": null,
  "intrinsics_file": "overhead_intrinsics.json",
  "depth_scale": 0.001,
  "rectified": true
}
```

This is a camera entry inside `cameras` in the main discovery configuration.
Provide a separate file for each camera. File paths are relative to that main
configuration, or absolute. The format is:

```json
{
  "version": 1,
  "frame_id": "overhead_optical",
  "calibration_revision": "installation_v1",
  "intrinsics": {
    "width": 640, "height": 480,
    "fx": 600.0, "fy": 600.0, "cx": 319.5, "cy": 239.5
  },
  "distortion": []
}
```

These numbers are illustrative, not calibration for the real installation.
An installed example is `config/camera_intrinsics.example.json`. Intrinsics use
pixels. `frame_id` and image dimensions must match the depth stream. Optional
`distortion` contains the camera's distortion coefficients. For a rectified
stream whose source calibration reports nonzero distortion, also provide
`projection`, the 12 row-major values of its calibrated rectified projection
matrix; its focal lengths and principal point are used for depth reconstruction.
Raw distorted images are not undistorted by discovery.

The file loads once at startup and takes precedence over `info_topic`; its revision
participates in observation calibration identity. Restart the node to load a new
file, which also starts a fresh inventory. Invalid files fail startup with their
path. Leave `intrinsics_file` null/omitted to retain CameraInfo delivery. A future
hardware API can adapt its response to the same calibration representation without
changing detection. Camera extrinsics still come from capture-time TF; this file
supplies intrinsics only and does not perform camera calibration.

## Automatic PIPER scanning

After the installation-specific scan poses and connecting paths are validated,
launch discovery with its shared scan controller:

```bash
export ROS_DOMAIN_ID=43  # use the domain selected for your simulator or bridge
ros2 launch phyai_discovery_ros discovery.launch.py \
  config_path:=/absolute/path/installation.json auto_scan:=true
```

`auto_scan` defaults to false, retaining observation-only operation. When enabled,
the same configuration must include a `scan` section. The installed
`config/piper_scan.example.json` contains the existing 11 survey poses, eight
reserve poses and PIPER joint limits, with slower motion settings and a closed
gripper. It is an installation example copied from the simulated setup; verify
the physical setup and paths before using it on hardware. Set calibrated camera
inputs, masks and table settings in that file as needed. Joint limits bound
individual joints; the shared controller does not perform collision checking.

The bridge contract assumed here matches MuJoCo:

| Interface | Contract |
| --- | --- |
| `/joint_ctrl_single` | `sensor_msgs/JointState`, `joint1`–`joint6` position targets in radians; `joint7` gripper opening in metres, 0 closed to 0.035 open |
| `/joint_states` | Fresh `sensor_msgs/JointState` positions for `joint1`–`joint7`; additional joints are allowed |

Use `command_topic` and `joint_states_topic` launch arguments to override these
names identically in both environments. The hardware team must confirm that the
bridge executes streamed position targets with this meaning. The controller
publishes at 50 Hz, uses cubic interpolation bounded by configured arm speed and
acceleration, waits for measured arrival (arm error <0.025 rad and gripper error
<0.002 m), settles, then requests a capture. Discovery additionally checks
stationarity. It waits for a command subscriber and refuses competing command
publishers; this is a ROS graph check, not an atomic bridge control lease.

The shared controller needs no `/sim/scene` or MuJoCo installation. It uses fixed
survey/reserve targets from the installation configuration. The Junh simulator
extension uses this same execution logic with simulated time, pause/restart guards,
and its geometry-based targeted inspection planner. Dynamic targeted-view planning
for hardware remains separate work; the shared controller does not substitute
simulator block truth for camera observations.

`/discovery/scan_status` publishes motion status and faults. Call
`/discovery/stop_scan` (`std_srvs/srv/Trigger`) to latch cancellation; it stops the
trajectory stream and sends a measured-position hold when fresh feedback and
exclusive publishing are available. It also requests discovery finish when that
service is available. This is not a hardware emergency-stop command; bridge stop
handling remains authoritative. Stale feedback/progress, command conflicts and
arrival timeout also latch a fault. Restart discovery to establish a new session
before resuming. On normal completion the controller sends its final hold and
stops publishing commands, leaving the inventory available to consumers.

## App interface

All JSON messages use `std_msgs/msg/String`; services use `std_srvs/srv/Trigger`.
The default names at the root namespace are:

| Interface | Purpose |
| --- | --- |
| `/discovery/workspace` | Core `workspace` + `search` snapshot, input status and errors |
| `/discovery/state` | Latched compact discovery state, completion flag and inventory identity |
| `/discovery/get_block_info` | GetBlockInfo service for the finished inventory |
| `/discovery/search_progress` | Scan state, requested `search.target`, inspection targets |
| `/discovery/overhead/overlay`, `/discovery/wrist/overlay` | Annotated RGB, when RGB is supplied |
| `/discovery/capture` | Request the next wrist observation after positioning |
| `/discovery/restart_scan` | Clear inventory and begin a fresh overhead observation |
| `/discovery/finish_scan` | Stop early with the current result |
| `/discovery/end_inspection` | End reserve inspection between captures |
| `/discovery/add_view` | Add a bounded manual view after finishing |

With `auto_scan:=true`, discovery moves and captures automatically; your app waits
for `search_finished` and reads the inventory. With `auto_scan:=false`, the
perception adapter publishes no arm commands, and your app should:

1. Start camera drivers, calibrated TF and fresh joint feedback, then discovery.
   A stationary overhead capture is acquired automatically.
2. Watch `search.state == "wait_for_wrist_view"` and `search.target`. Move the arm
   to that physical view, verify arrival and settling, then call capture:

   ```bash
   ros2 service call /discovery/capture std_srvs/srv/Trigger '{}'
   ```

3. Check the service's `success` field, then wait for the next requested view or
   `search_finished`. During reserve rounds, your app may plan around perceived
   `workspace.inspection_targets` from progress messages. The shared adapter
   supplies those estimates; it does not generate joint trajectories.
4. Pass confirmed tracks, coordinate frame, uncertainty and final status to the
   next stage. Inspect `criteria_met`, remaining hypotheses and per-track bounds.
   `pick_ready` is always false; discovery does not validate a grasp.

To inspect output:

```bash
ros2 topic echo /discovery/search_progress
ros2 topic echo /discovery/workspace
```

For camera-independent Python usage and the inventory contract, see the
[core README](../phyai_discovery/README.md). For transport replay:

```bash
ros2 launch phyai_discovery_ros discovery.launch.py \
  config_path:=/tmp/hardware.json recording_path:=/absolute/path/scan.json
```

The recording manifest contains measured depth arrays, intrinsics, transforms,
acquisition stamps and `stationary_verified: true` per capture. Replay still
requires the configured segmentation backend; use depth mode for depth-only files.

## Query block information after discovery

The service `/discovery/get_block_info` uses
`phyai_discovery_interfaces/srv/GetBlockInfo`. Junh's compatibility entry point
exposes the same service at `/junh/get_block_info`.

```bash
ros2 service call /discovery/get_block_info \
  phyai_discovery_interfaces/srv/GetBlockInfo '{}'
# Optional: select an ID and require the scan session you observed.
ros2 service call /discovery/get_block_info \
  phyai_discovery_interfaces/srv/GetBlockInfo \
  '{block_id: "block_1", session_id: "your-session-id"}'
```

`block_id` empty returns the full inventory; a provided ID selects one confirmed
block. `session_id` empty accepts the current session; providing it rejects a query
if discovery restarted. Before completion, `success` and `finished` are false and
`snapshot_json` is empty. After completion, a successful response contains:

- `discovery_state` (`search_finished`), `finished`, `session_id`, `revision`,
  `frame_id`, and `search_status` (`complete`, `partial`, or `failed`).
- `snapshot_json`, the existing core `workspace`/`search` JSON envelope. Tracks
  include IDs, position in metres, quaternion in **wxyz** order, size in metres,
  uncertainty, support and resolution state. A single-ID query filters `tracks`
  and omits provisional hypotheses; an all-block query retains them separately.

Unknown IDs and session mismatches return `success: false` with a message and no
snapshot. Query success means the inventory was retrieved; it does not imply that
observation criteria were met or a grasp is ready. The snapshot remains valid only
for its static scene/session; restart discovery when the physical scene changes.

`/discovery/state` publishes compact JSON state with the same metadata fields
above, using reliable transient-local QoS so late subscribers can retrieve the
latest published state. State is derived from the existing search state machine,
not maintained as a second lifecycle variable. Subscribe with transient-local QoS
for the latched sample, or watch the existing progress stream.

Junh's browser viewer waits for `search_finished`, calls `GetBlockInfo` once per
finished session/revision, and displays a final inventory table with positions and
yaw. Clicking a row selects the block's full information. It retries unavailable
or failed queries with a delay and rejects responses from an older session or
revision. Live map updates remain available while discovery runs.

## Validate on the physical setup

First record stationary observations of independently measured block positions
and orientations across the tabletop, including edges, touching blocks, and views
partly occluded by the arm. Replay them through this package. Compare world-frame
position, yaw modulo cuboid symmetry, count, missed/extra blocks, coverage and
reported bounds against the measurements. Keep evaluation reference positions out
of perception inputs. Confirm repeatability and set error budgets from those
results before connecting the inventory to picking or stacking.

Build and source the workspace first so the generated `GetBlockInfo` interface
is available. Then run transport tests from the repository root:

```bash
PYTHONPATH=src/phyai_discovery:src/phyai_discovery_ros \
  python3 -m pytest -q src/phyai_discovery_ros/test
```
