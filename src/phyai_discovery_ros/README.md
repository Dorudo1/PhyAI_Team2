# Shared discovery ROS adapter

Use `phyai_discovery_ros` from any ROS 2 Humble app to turn calibrated RGB-D
observations into a block inventory. It uses `phyai_discovery` and has no Junh,
MuJoCo, camera driver, robot driver, or motion planner dependency. The Junh app
uses this same adapter through its existing compatibility entry point.

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
  trajectory implementations. Start with the closed gripper used by PIPER.

Stationarity currently requires fresh `joint1`–`joint7` positions. The first six
are radians; tolerance for the gripper uses the bridge's native feedback units.
Use the same units when selecting the shared `stationary_tolerance`. Missing,
stale, invalid, or moving feedback prevents captures. Stationarity confirms that
motion stopped; your app must also verify it reached the requested physical pose.

## App interface

All JSON messages use `std_msgs/msg/String`; services use `std_srvs/srv/Trigger`.
The default names at the root namespace are:

| Interface | Purpose |
| --- | --- |
| `/discovery/workspace` | Core `workspace` + `search` snapshot, input status and errors |
| `/discovery/search_progress` | Scan state, requested `search.target`, inspection targets |
| `/discovery/overhead/overlay`, `/discovery/wrist/overlay` | Annotated RGB, when RGB is supplied |
| `/discovery/capture` | Request the next wrist observation after positioning |
| `/discovery/restart_scan` | Clear inventory and begin a fresh overhead observation |
| `/discovery/finish_scan` | Stop early with the current result |
| `/discovery/end_inspection` | End reserve inspection between captures |
| `/discovery/add_view` | Add a bounded manual view after finishing |

The adapter publishes no arm commands. Your app should:

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

## Validate on the physical setup

First record stationary observations of independently measured block positions
and orientations across the tabletop, including edges, touching blocks, and views
partly occluded by the arm. Replay them through this package. Compare world-frame
position, yaw modulo cuboid symmetry, count, missed/extra blocks, coverage and
reported bounds against the measurements. Keep evaluation reference positions out
of perception inputs. Confirm repeatability and set error budgets from those
results before connecting the inventory to picking or stacking.

Run transport tests from the repository root after sourcing ROS:

```bash
PYTHONPATH=src/phyai_discovery:src/phyai_discovery_ros \
  python3 -m pytest -q src/phyai_discovery_ros/test
```
