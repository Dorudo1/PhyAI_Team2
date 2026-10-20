# Junh RGB-D block discovery

This app discovers table geometry and blocks from aligned RGB-D measurements, merges
observations from overhead and wrist cameras, and publishes a partial workspace
map for the master. The MuJoCo launch includes an automatic multi-view driver that
moves the simulated arm and captures wrist views. The standalone discovery node
is read-only; hardware search motion and picking/stacking remain unimplemented.
SAM 2 masks and known block geometry support pose estimation. Uncertain
observations remain hypotheses; only confirmed objects receive block IDs.
Finishing the scan does not establish a complete table map or picking readiness.

The implementation follows the [simple search plan](docs/implementation-plan.md).
The independent [safeguard](../safeguard/docs/implementation-plan.md), hardware
motion integration, and picking/stacking remain separate future work.

## Reuse discovery in the integrated app

The reusable feature lives in [`src/phyai_discovery`](../../src/phyai_discovery/README.md).
It owns RGB-D detection, optional SAM2 segmentation, workspace fusion, scan state,
and the workspace/search result contract. It requires NumPy; ROS, MuJoCo, the web
viewer, and the master are adapters in this test app. Both live discovery and
recording replay use the shared `Discovery` lifecycle. Import feature types directly
from `phyai_discovery`; the temporary forwarding modules in `junh_master` have
been removed.

Other apps should depend on `phyai_discovery` and feed calibrated `Capture`
objects to `Discovery`, then consume `snapshot()` for the next pipeline stage.
See the shared package README for Python and ROS integration instructions.
The existing launch files and `/junh/*` interfaces remain available for testing.
SAM2 setup and its checkpoint remain local to this app; another app supplies its
own segmentation configuration and checkpoint path.

## Scatter blocks for discovery tests

`./apps/junh/run_sim.sh` scatters blocks by default before the simulator publishes
its first camera frame or the automatic search begins. Each launch gets a new
static layout. For a reproducible scene or one-table layout, launch from the
repository root with:

```bash
SCATTER_SEED=42 SCATTER_TABLE=0_1 ./apps/junh/run_sim.sh
```

`SCATTER_TABLE` accepts `all`, `0_0`, `0_1`, `1_0`, or `1_1`. Set
`SCATTER_BLOCKS=false` to start with the aligned scene from `lab_scene.json`.
Rebuild once after updating the simulator. With the simulator already running,
these commands start a *new* discovery scan after changing the scene:

```bash
./apps/junh/scatter_blocks.py scatter --seed 42
./apps/junh/scatter_blocks.py scatter --seed 43 --table 0_1
./apps/junh/scatter_blocks.py reset
```

The first command scatters blocks across all four tabletops. `--table 0_1`
restricts placement to the original lower-right table; other tables are `0_0`,
`1_0`, and `1_1`. Omit `--seed` for a fresh random layout, or reuse it to reproduce
a layout with the same scene settings. Use `--url http://localhost:23518` for a
different simulator port; the script also respects `HTTP_PORT`.

Blocks receive random X/Y positions and yaw. The simulator keeps their footprints
inside a single tabletop, avoids overlapping blocks and robot geometry, and drops
them a short distance in a temporary MuJoCo simulation. Gravity, friction, and
contact dynamics settle the blocks before a layout is applied. A failed layout
leaves the running scene unchanged. Scatter and reset restart the robot and
notify discovery/automatic scanning through the existing simulation restart event.

The simulator saves the **aligned configured positions and full rotation
quaternions** before startup scattering. Later scatters leave that reset target
intact. `reset` and the viewer's **Restart simulation** button restore it and
clear block velocities. The target lasts for this simulator process; no scene
configuration file is changed. A new simulator process gets a new random layout
unless a seed or `SCATTER_BLOCKS=false` is specified.

To score a seeded live scan against simulator truth after it finishes, run:

```bash
python3 apps/junh/examples/evaluate_scatter.py --seed 42
```

Add `--table 0_1` when that scatter restriction was used, or `--aligned` after
reset. The evaluator reports confirmed/missing blocks, position and yaw errors,
and coverage for each physical tabletop. It reads truth only for evaluation;
discovery still receives camera measurements without simulator block poses.

## Run live MuJoCo search

From the repository root:

```bash
./apps/junh/setup_sam2.sh  # once
./apps/junh/run_sim.sh
```

The script sources ROS, builds the shared discovery package, app, and simulator, sources the workspace, configures
UDP DDS, and starts the simulator and all three discovery nodes. Run with
`--no-build` to reuse an existing build. Override the port with
`HTTP_PORT=23518 ./apps/junh/run_sim.sh`; the domain defaults to 42 and respects
`ROS_DOMAIN_ID`. Stop an existing simulator in that domain before launching.
Ctrl+C stops the launched processes. No recording is needed.
Open `http://localhost:23517` for the simulated camera views. Click **App workspace
map** in that viewer, or open `http://localhost:23517/workspace`, to see the app
estimates. The link switches views in the same tab. Switching to the map closes
the simulator's browser stream, so its high-resolution rendering stops. Hidden
simulator/editor tabs also disconnect their streams; hidden workspace tabs pause
map polling and drawing. Returning to a view resumes it automatically. Physics
and capture-demand RGB-D rendering continue for discovery. Opening both views
in separate visible windows still renders both.
The script prints both URLs. The map is also available directly at
`http://localhost:23518`. Its backend is localhost-only; the simulator proxies
the map page and snapshots through its own port. With a remote devcontainer,
forwarding simulator port 23517 is sufficient for both views. The launch publishes
actual rendered RGB and metric depth, matching CameraInfo, and capture-time camera
TF in capture-demand bursts at up to 3 Hz, using 320x240 sensor images.
The browser retains 960x720 frames and its 30 FPS target; sensor passes are idle
while the arm moves. Actual rates depend on rendering hardware. The detector receives camera pixels, not
`/sim/scene` block poses. UDP DDS avoids the shared-memory transport issue observed
between separate processes in this environment. Use the same domain/profile and
source the workspace in additional terminals.

The first overhead capture runs automatically after the arm settles. The
simulation-only `sim_scan` node then visits eleven survey poses that vary the
base, shoulder, elbow and wrist joints. These lift the fixed wrist camera into
upright, downward-looking views at different heights and angles. The camera
mount, image roll and overhead installation stay fixed throughout the scan.
At each view it waits for convergence/settling, requests a fresh capture, and waits
for detection/merging before continuing. It stops advancing when search finishes.
No manual positioning or capture calls are needed for the default simulation run.

The startup log prints `AUTO_SEARCH=true` or `false`. The driver reports its
waiting reason, including missing feedback, paused simulation, or non-simulator
command subscribers. Open the browser to watch motion and inspect `/junh/discovery`, `/junh/status`, and
`/junh/scan_status`. The live workspace viewer updates from the accepted captures.
The gripper stays closed (`joint7=0`) throughout survey and inspection.
The survey is fixed; reserve captures adapt to uncertain detected objects within
a bounded budget. A completed sequence can still produce a partial map and uncertain
tracks. Picking readiness remains false.

`config/mujoco.json` holds the survey and reserve `sim_scan.targets`, motion limits,
and the declared search scope across the four tabletops. Discovery still estimates
table geometry from depth. The simulator renders only the camera currently requested by discovery.
`move_seconds` and `approach_seconds` are minimum trajectory durations. Larger
joint changes take longer to keep the cubic interpolation within
`max_joint_speed` (rad/s) and `max_joint_acceleration` (rad/s²). Small moves no
longer incur a fixed four-second trajectory. Capture still requires stationary
joint feedback, independently of the driver's `settle_seconds` delay.
Full workspace snapshots publish at 1 Hz, while the scan driver receives a small
5 Hz progress message. RViz markers are generated only with a viewer subscribed.
Each camera publishes a matching robot mask so arm,
gripper, and camera geometry are excluded without supplying block poses or IDs.
Real camera discovery needs its own robot mask/filter; this simulator mask is not
a hardware perception implementation.

The motion driver requires fresh simulator scene and joint feedback and refuses
command forwarding if command-topic subscribers include anything other than the
MuJoCo simulator. It uses simulator time so pausing freezes the motion sequence.
Joint limits and feedback timeouts are checked. These development checks are not
the planned hardware safeguard. `master.launch.py` never starts this driver.

For manual simulation captures:

```bash
AUTO_SEARCH=false ./apps/junh/run_sim.sh
```

Position the simulated arm through your existing controls and use `/junh/capture`
for each view. Use **Restart simulation** in the simulator browser to reset the physics, clear
the discovery map, and start the automatic search again. `/junh/restart_scan`
repeats discovery without resetting the physical scene.
If the motion driver reports a fault, address its cause and restart discovery
before resuming. Config changes require relaunching the nodes.

Set RViz's Fixed Frame to `world` for simulation. The map reports observed table
patches, unresolved/unknown regions, and provisional object fits. It is not a
collision-free motion authorization map. To run the simulator separately, use
`ros2 launch piper_mujoco_sim sim.launch.py publish_cameras:=true`, then launch
`master.launch.py` with an appropriate discovery config; that separate launch is
manual unless you also explicitly run the simulation driver.

## Try recorded discovery without ROS

From the repository root, create synthetic depth/RGB images and replay them:

```bash
python3 apps/junh/examples/create_recording.py /tmp/junh-recording
PYTHONPATH=src/phyai_discovery:apps/junh python3 -m phyai_discovery.recording \
  /tmp/junh-recording/scan.json --output /tmp/junh-snapshots.jsonl
```

The images depict blocks with different positions and yaw, including a block
outside overhead coverage. The detector receives pixels and calibration, not
block poses. This generated recording is a development example, not a real camera
performance measurement. NumPy is the only non-ROS runtime Python dependency.

## Build and launch ROS discovery

From your colcon workspace, normally `~/ws`:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to junh_master
source install/setup.bash
ros2 launch junh_master master.launch.py \
  recording_path:=/tmp/junh-recording/scan.json
```

This launches discovery, the master subscriber, and the workspace-marker publisher.
Recorded mode processes the capture sequence automatically and retains the final
map for inspection. Recorded acquisition times are shown as scan-relative ages.

For live cameras, copy [config/discovery.json](config/discovery.json) and set your
actual camera topics, calibration revision, ROI/mask, fixed robot frame, and wrist
view labels. Launch with:

```bash
ros2 launch junh_master master.launch.py config_path:=/absolute/path/discovery.json
```

Camera drivers, joint feedback, and calibrated TF are provided by the host. This
app does not launch drivers, create fictitious transforms, or move the arm. The
simulator supplies depth/TF when `publish_cameras` is enabled. The default topic names are placeholders for your installation.

Each camera requires depth `Image` and matching `CameraInfo`. Supported depth
encodings are `16UC1` (configured scale; default millimetres) and `32FC1` (metres).
Use `rectified: true` for rectified depth with its projection intrinsics. Distorted
raw depth is rejected. Depth and calibration must have matching dimensions and
optical frames. Use capture-time TF into `map_frame`; ROS and camera clocks must
agree. No overhead/wrist frame synchronization is required.

Optional RGB overlays require an RGB image aligned to the depth image, with the
same dimensions and optical frame, and a timestamp within 50 ms. If your depth
and color streams differ, configure aligned depth with its matching intrinsics,
or leave `rgb_topic` empty. Geometry works without color. RGB overlays are emitted
for accepted captures, not continuously for uncaptured frames.

A pixel `roi` is `[x0,y0,x1,y1]` with exclusive upper bounds. A camera `mask` is a
NumPy boolean image (`True` excludes pixels), resolved relative to the config
file. Map-space `detector.exclusion_boxes` use
`[xmin,ymin,zmin,xmax,ymax,zmax]`. Configure these to exclude robot geometry or
reject contaminated captures. Excluded regions do not become observed empty space.

## Manual wrist capture workflow

1. Let discovery accept a fresh overhead frame while joint feedback is stationary.
   If overhead captures remain unusable, bounded retries fall back to wrist views.
2. Inspect the requested view in `/junh/discovery`. Position the wrist using your
   existing robot operating procedure. The labels `wrist_1` and `wrist_2` are capture
   labels, not generated arm poses.
3. Request the next capture:

   ```bash
   ros2 service call /junh/capture std_srvs/srv/Trigger '{}'
   ```

4. Discovery waits for stationary `joint1` through `joint7` feedback and then
   accepts a fresh wrist image. By default, positions must remain within 0.003 rad
   (gripper feedback uses its own native units) for 0.5 s, with no feedback gap over
   1 s. Inspect progress and errors before requesting the next view.
5. Repeat for the configured view list. The final outcome is `complete`, `partial`,
   or `failed` for the declared scope. Finishing the capture list is distinct from
   picking readiness, which always remains false in this app.

Other controls:

```bash
ros2 service call /junh/restart_scan std_srvs/srv/Trigger '{}'
ros2 service call /junh/finish_scan std_srvs/srv/Trigger '{}'
ros2 service call /junh/add_view std_srvs/srv/Trigger '{}'
```

`restart_scan` discards the static-scene inventory and invalidates in-flight worker
results. `finish_scan` ends early with explicit achieved coverage. After finishing,
`add_view` appends an inspection label; at most three extra views may be added per
scan. Position that view and use `capture` again. Failed wrist captures return to
manual request with bounded retries; exhausted views are skipped with a partial
or failed result. Frame/processing timeouts default to 15 s.

Keep blocks stationary within one scan. Restart after moving blocks or changing
calibration. Absence in a later camera view does not delete tracks from earlier
views. Conflicting table estimates and changed camera calibration reject captures.

## Observation criteria and additional scanning

The simulation first captures the overhead view and its eleven survey wrist
views. If evidence requirements remain unmet, discovery appends up to eight
inspection captures. With `sim_scan.targeted_inspection=true`, the driver chooses
joint angles around the detected uncertain blocks instead of repeating a fixed
sweep. Provisional objects and missing supporting views take priority over
refining an already supported pose. Equally urgent blocks take turns.

Each chosen pose must keep the expected object extent inside the image, clear
the arm and fixtures along the viewing rays, and pass sampled joint-path contact
checks. Camera position on the arm, image roll and FOV remain fixed. The planner
uses measured block positions and known robot/table/camera geometry; free-body
simulator geometry is disabled in its model so ground-truth block poses cannot
influence planning. A new capture label must also have a physically different
camera pose. Retrying a rejected capture reuses its original pose.

`max_targeted_views_per_block` defaults to two extra views per uncertain block.
Inspection stops early when the criteria pass or no clear novel targeted view
remains within that budget. The latter returns `partial` with stop reason
`targeted_views_exhausted`, preserving unresolved evidence. When only table
coverage remains unmet, configured reserve poses are still available to inspect
the remaining scope. The workspace and block IDs persist across all captures.

To reproduce discovery across seeded layouts without running ROS motion:

```bash
source apps/junh/sam2_env.sh
MUJOCO_GL=egl python3 apps/junh/examples/benchmark_scan.py --backend sam2 \
  --seeds 42 17 43 7 101 --output /tmp/discovery-benchmark.json
```

This renders the configured poses using the fixed camera geometry and actual
RGB-D detector. Ground-truth poses are used only for scoring the resulting
inventory. The report includes per-capture recall, hypotheses, table coverage,
position/yaw errors, and each targeted capture's focus and joint pose. Add
`--fixed-inspection` to compare against the fixed reserve sweep. Its runtime measures rendering and detection; live
motion, settling and transport timing require `run_sim.sh`.

Edit `search_criteria` in `config/mujoco.json` to set:

| Setting | Default | Meaning |
| --- | --- | --- |
| `min_explored_fraction` | 0.90 | Fraction of the configured scope with measured table or object evidence |
| `min_resolved_views` | 2 | Distinct camera/viewpoint pairs with resolved fits for each confirmed block |
| `max_position_uncertainty` | 0.01 m | Maximum reported sampling bound on position |
| `max_yaw_uncertainty` | 0.0873 rad | Maximum reported yaw sampling bound, about 5 degrees |
| `min_coverage_gain` | 0.005 | Coverage gain considered progress, half a percentage point |
| `max_no_progress_rounds` | 1 | Stop after this many additional sweeps without improving evidence |

These are evidence requirements, not calibrated confidence probabilities.
Replaying the same viewpoint does not add a distinct supporting view. Block count
is never a stopping requirement; it is unknown to discovery. More observations
can reduce uncertainty but cannot guarantee that every occluded object is found.

Complete depth fits bypass SAM2 inference; SAM2 handles only unresolved groups.
Depth outside resolved masks remains provisional and blocks the criteria.
Unexplained above-table depth also blocks success; below-table/background depth
does not increase explored coverage. The simulation config limits tabletop fitting
to measured world Z in `[-0.005, 0.005]`. For hardware, configure
`detector.table_height_range` from the installation calibration in its map frame.
Without an adequate height interval or ROI, multiple substantial horizontal
surfaces are rejected as ambiguous.

Pose bounds include configured depth/calibration errors and cross-view residuals.
Contradictory poses invalidate supporting views; two new consistent distinct
camera/viewpoint pairs are required to settle a conflict. Configure sensor error
bounds for the installation as described in the shared discovery README.

`inspection_rounds` lists the reserve viewpoint names; `sim_scan.targets`
contains survey poses and coverage fallback poses. During targeted inspection,
the driver assigns new joint angles to each reserve label from the current
perception evidence. These are project-specific MuJoCo motion checks; hardware
motion remains deferred. `/junh/scan_status` includes `inspection_focus` with the
detected object ID, estimated position and selected joint angles.

The map distinguishes **observed table surface**, **explored scope** (surface
plus uncertain object evidence), and **unobserved scope**. Orange cells underneath
blocks are explored but do not count as observed free table surface. Neither
metric certifies collision-free space. Full table mapping may remain partial
even when configured observation criteria pass.
The simulation scope now spans the physical table edges. Surface hidden under
blocks or the robot cannot be directly observed, so 100% observed table surface
is not a physically attainable stop condition for these views. Unobserved cells
remain visible rather than being counted as free or silently filled by ground truth.

A block's identity, pose resolution, and observation age are separate. A confirmed
ID stays confirmed as a static scan ages; new contradictory evidence can make
its pose uncertain. `fit_quality` describes geometric evidence, while `quality`
and `identity_state` describe inventory state. The selected-block display explains
these fields. Completed static scans retain coverage colors; a stale discovery
publisher still turns the map purple.

The wrist camera attaches to the casing upstream of joint6. Its optical field of
view is fixed by camera calibration; rotating the gripper does not enlarge it or
change this simulated camera pose. All scan targets now keep joint6 at zero.
The remaining shoulder/elbow/casing orientation can rotate the image; its
capture-time transform accounts for that orientation. Camera-upright constraints
would need to be included in a future camera-pose planner rather than inferred
as additional field of view.

## Workspace display and topics

The browser workspace page is a separate top-down view of `/junh/discovery`. It
draws observed/uncertain/unobserved coverage cells and estimated block footprints,
not simulator block poses. Click a footprint to inspect its position, dimensions,
quality, source cameras, and uncertainty reasons. Pan, zoom, or fit the map; the
page shows the current frame, session, revision, search state, and publisher age.
Old observations stay colored after a finished static scan; stale publishers remain purple. Both the browser page and RViz consume the same snapshot.

For 3D inspection, use RViz as below; set Fixed Frame to `world` in simulation.
The web port defaults to the simulator port plus one and can be changed with
`WORKSPACE_PORT`. It is disabled in standalone `master.launch.py` unless
`workspace_port` is supplied.

```bash
ros2 topic echo /junh/discovery
ros2 topic echo /junh/status
rviz2 -d /absolute/path/to/apps/junh/config/workspace.rviz
```

RViz is an optional desktop viewer; it is not bundled into the headless runtime.
Set its Fixed Frame to your configured `map_frame`. The installed RViz config is
also under `share/junh_master/config`. Discovery publishes:

| Interface | Purpose |
| --- | --- |
| `/junh/discovery` (`std_msgs/String`, JSON) | Versioned workspace, capture progress/result, source ages, and errors |
| `/junh/search_progress` (`std_msgs/String`, JSON) | Small 5 Hz scan-control heartbeat, independent of full map publication |
| `/junh/status` (`std_msgs/String`, JSON) | Master handoff, inventory stability, pending policy, and `pick_ready: false` |
| `/junh/workspace_markers` (`visualization_msgs/MarkerArray`) | Surface coverage, block geometry, IDs, quality/age, and scan status |
| `/junh/overhead/overlay`, `/junh/wrist/overlay` (`sensor_msgs/Image`) | Optional accepted RGB images with candidate rectangles |

Green cells show observed tabletop; amber cells show uncertain evidence; grey
cells show unobserved configured scope. Old observations are purple. Blue boxes
are provisional rectangular fits and orange boxes are unresolved groups. A dead
discovery publisher marks the retained map stale. The viewer consumes the same
snapshot as the master; it does not run another detector.

`scope`, when supplied, is `[xmin,xmax,ymin,ymax]` in the fixed robot frame and
specifies where to report coverage. It does not specify table geometry or grant
motion permission. Without a scope the result remains partial. A table plane does
not certify full table boundaries, gaps, hidden regions, or free 3D space.

## Recorded capture format

A version 1 JSON manifest contains `map_frame`, wrist `viewpoints`, optional
`scope`, and a `captures` list. Each capture includes:

```json
{
  "camera": "wrist",
  "viewpoint": "right",
  "stamp": 1720000000.1,
  "depth": "right_depth.npy",
  "depth_unit": "m",
  "optical_frame": "wrist_optical",
  "map_frame": "base_link",
  "calibration_revision": "installation_v1",
  "intrinsics": {"width": 640, "height": 480, "fx": 600, "fy": 600, "cx": 319.5, "cy": 239.5},
  "camera_to_map": [[1,0,0,0],[0,-1,0,0],[0,0,-1,1],[0,0,0,1]],
  "stationary_verified": true
}
```

The transform above is illustrative; replace it with the measured transform at
capture time. Depth arrays must match intrinsics; units must explicitly be `m`
or `mm`. Optional `rgb`, `mask`, and `roi` fields use the same contracts as live
input. Array paths are relative to the manifest. Camera calibration and robot
pose cannot be omitted. `stationary_verified` records that the acquisition was
stationary; replay does not infer that fact from image pixels.

## Detector scope and tuning

Run `./apps/junh/setup_sam2.sh` once before launching. It creates an ignored
`.venv-sam2` environment, installs pinned PyTorch and official SAM 2 source,
and downloads the SAM 2.1 tiny checkpoint into ignored `.models`.
The simulator launcher and `start.sh` enable this environment automatically.
For a direct ROS launch, source `apps/junh/sam2_env.sh` after the ROS workspace
setup. `JUNH_SAM2_CHECKPOINT` can select another local tiny checkpoint.
The source revision, checkpoint family, and dependencies are explicit in
[setup_sam2.sh](setup_sam2.sh); the implementation follows the
[official SAM 2 image predictor](https://github.com/facebookresearch/sam2).

The configured pipeline generates prompts from above-table measured depth,
runs SAM 2 on capture-matched aligned RGB, and fits a table-supported cuboid
with installation-configured dimensions `detector.known_block_size`
(length, width, height in metres). Dimensions are an object specification;
the detector never reads scene-file block poses, identities, or count.
Default dimensions are 75 × 25 × 15 mm and must match the physical blocks.
SAM 2 loads once and runs only for requested stationary captures. Missing model
dependencies, checkpoint, CUDA, or aligned RGB are reported rather than silently
switching detectors. Set `segmentation.backend` to `depth` explicitly to use
the geometric baseline. Set `segmentation.device` to `cpu` explicitly for CPU
inference, whose speed must be measured separately.

Known-shape poses require supported top-face coverage, shape agreement, and
bounded residuals. Clipped, tilted, multi-height, and incomplete evidence stays
unresolved. Full free-standing 6D pose fitting and reliable splitting of
touching/overlapping blocks are still outside this first implementation.
Yaw has 180-degree symmetry for an unmarked rectangular cuboid. Reported
uncertainty bounds describe sampling assumptions, not calibrated probabilities.


The depth baseline uses bounded table-plane RANSAC and sparse grid components.
Tune plane tolerance, minimum point counts, and sampling cell size against
camera measurements. Coarse sampling can fragment objects or merge neighbors.

The workspace keeps `hypotheses` separately from confirmed `tracks`.
Hypotheses have an internal `hypothesis_id` and no block ID. A supported known-shape
fit can confirm in one view; the depth-only baseline requires consistent resolved
evidence from distinct camera/viewpoint pairs. Repeated captures from the same
view do not count as independent confirmation.
Association is conservative and one-to-one; ambiguous or overlapping unmatched
evidence is retained as bounded alternatives instead of minting another block ID.
Confirmed IDs survive later uncertainty. Partial observations do not overwrite a
previously resolved pose. Missing observations do not delete static-scene inventory.
The stacking layer receives confirmed tracks, including their uncertainty,
and rejects placement proposals for unresolved tracks. Proposals never execute
motion here.

## Tests and package layout

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src/phyai_discovery:apps/junh python3 -m pytest -q src/phyai_discovery/test apps/junh/test
```

For actual-model rotated-scene tests, disable unrelated globally installed pytest
plugins; they interfere with Hydra configuration resource lookup in this container:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MUJOCO_GL=egl JUNH_TEST_SAM2_CHECKPOINT="$PWD/apps/junh/.models/sam2.1_hiera_tiny.pt" \
  PYTHONPATH=src/phyai_discovery:apps/junh apps/junh/.venv-sam2/bin/python -m pytest -q src/phyai_discovery/test/test_sam2_rendered.py
```

Build/source the package first for ROS adapter tests. Without ROS, the adapter test
module skips; depth, inventory, replay, and simulator-contract tests still run.
Tests use rendered depth pixels and real ROS publish/subscribe handoff; these are
software integration tests, not physical camera calibration or robot trials.

The app contains adapters and verification UI:

```text
junh_master/
  ros/          Camera/TF discovery adapter, handoff node, scan driver, ROS lifecycle
  viewer/       Browser page, HTTP server, RViz display primitives
  handoff.py    Prototype consumer for the next pipeline stage
  legacy/       Original scene-pose contract demo (not used by live discovery)
```

The `junh_master` package name, ROS executables, launch files, and `/junh/*`
interfaces stay the same. Python imports now follow the folders above; for example,
`junh_master.ros.discovery_node` and `junh_master.handoff` replace the old flat paths.
`setup.py` installs the viewer page with its package. `start.sh` launches the three
read-only nodes. The MuJoCo launch additionally starts `ros/sim_scan.py`.

Shared detection, scan coordination, inventory, replay, and their tests live under
`src/phyai_discovery/`. Offline replay is available as
`python3 -m phyai_discovery.recording`; `ros2 run junh_master replay_scan` remains
an app command pointing to that implementation.

The original scene-pose demo remains available as `search_demo` or
`python3 -m junh_master.legacy.demo`. It exercises simulator ground truth and is
kept separate from camera discovery. App tests cover the ROS adapters, viewer,
handoff, and this legacy example; shared-package tests cover the discovery feature.
