# Block search implementation plan

## Current implemented scope

Discovery is shared across three packages: `phyai_discovery` owns RGB-D detection,
workspace fusion and search state; `phyai_discovery_ros` owns transport and optional
PIPER scan execution; `phyai_discovery_interfaces` defines `GetBlockInfo`.
Junh is a test app with simulator integration, browser visualization and prototype
handoff. Its ROS entry point preserves `/junh/*` names.

The shared controller executes configured survey/reserve joint poses using the
MuJoCo-compatible position-command bridge contract. It verifies feedback freshness,
arrival including the closed gripper, settling and capture acknowledgement. The
shared launch enables it with `auto_scan:=true`; observation-only is the default.
Junh's MuJoCo extension reuses that controller with simulated time, pause/restart
checks and geometry-based targeted inspection. The current example has eleven
survey poses and eight reserve poses, plus the initial overhead capture.

The browser queries `GetBlockInfo` after search completion, displays position/yaw
and full selected-block information, and rejects obsolete session/revision replies.
The state topic and query metadata derive from the existing search state machine.
JSON intrinsics and CameraInfo are both supported; extrinsics remain capture-time TF.

Hardware trajectories, collision/path validation, real-camera accuracy and runtime
Docker deployment remain unverified. Dynamic targeted inspection for hardware,
the independent safeguard and picking/stacking are not implemented. The camera
detector never consumes simulator block poses or IDs.

Use the [Junh README](../README.md) for current test-app commands and the
[shared integration guide](../../../src/phyai_discovery_ros/README.md) for the
bridge contract, calibration, automatic scans, runtime launch and query API.

## Historical design and validation notes

The sections below preserve earlier planning stages and measurements, including
older eight-view scans and read-only hardware assumptions. They are historical
records, not the current launch contract. The current scope above and linked
READMEs supersede those descriptions; historical timing/count results do not
establish performance for the current scan profile or physical hardware.

### Requirements

- Block count, positions, rotations, and arrangement are unknown at startup.
- Overhead field of view is insufficient for whole-table discovery. Wrist views
  extend coverage and refine estimates.
- Estimate table geometry from measurements and update it throughout the scan.
- Use known block dimensions to fit pose; retain incomplete-face and orientation ambiguity.
- Start with separated rectangular blocks. Touching/overlapping or irregular
  clusters remain unresolved groups when separation is not established.
- Keep unobserved and uncertain regions explicit. A finished capture sequence
  does not establish whole-table completeness or picking readiness.
- Use a fixed robot/world frame, acquisition timestamps, calibration revisions,
  map sessions, and revisions throughout the pipeline.
- Keep hardware motion validation and execution in the separate safeguard app.

### Architecture

| Component | Responsibility |
| --- | --- |
| `src/phyai_discovery/phyai_discovery/` | Reusable capture contracts, detector, segmentation, inventory, search, and `Discovery` lifecycle |
| `src/phyai_discovery/phyai_discovery/recording.py` | Offline replay using the shared lifecycle |
| `junh_master/ros/discovery_node.py` | ROS camera input, capture-time TF, bounded worker, capture services, and publication |
| `junh_master/ros/sim_scan.py` | MuJoCo-only interpolated joint scan and automatic capture requests |
| `junh_master/handoff.py`, `junh_master/ros/handoff_node.py` | Prototype policy handoff and publisher freshness checks |
| `junh_master/viewer/`, `junh_master/ros/viewer_node.py` | Shared-snapshot browser and RViz visualization |
| `junh_master/legacy/` | Original simulator-ground-truth contract example |

```text
Overhead / wrist measurements -> detector -> workspace / inventory -> master
                                     ^                |
                                     |                v
                          capture coordinator      viewer
                                     ^
                                     |
                         MuJoCo swivel driver
```

Geometry processing runs in a single worker rather than camera callbacks. Queues
and pending frames are bounded. Perception and visualization do not generate
hardware commands; the simulation driver is absent from `master.launch.py`.

### Inputs and calibration

Require matching camera intrinsics, depth scale, overhead-to-robot extrinsics,
and wrist-camera-to-mount calibration. Use timestamped joint feedback and the
wrist transform at capture time. Missing calibration or transforms make a capture
unusable, not an empty-table observation.

All accumulated geometry uses a fixed robot frame (`world` in the supplied
MuJoCo profile). Table-plane refinement must not move old tracks or installation
limits silently. Lengths are metres, angles radians. The core uses `[w,x,y,z]`
quaternions; ROS geometry fields use named `x,y,z,w`.

Overhead and wrist frames need not be synchronized with one another. Each capture
is validated and transformed independently. Rectified streams use their matching
projection intrinsics. Distorted raw depth is rejected. Robot exclusion masks
must match the image frame, dimensions, and acquisition time.

The simulator publishes a mask based only on known robot geometry. It does not
provide block geometry to perception. Real-camera use requires its own robot
mask/filter or rejection of contaminated views; full hardware self-filtering
remains deferred.

### Capture procedure

```text
WAIT_FOR_INPUTS -> CAPTURE_OVERHEAD -> DETECT_AND_MERGE
                                           |
                                 WAIT_FOR_WRIST_VIEW
                                           |
                               WAIT_FOR_STATIONARY_ARM
                                           |
                                   CAPTURE_WRIST
                                           |
                                  DETECT_AND_MERGE
                                           |
                             NEXT_VIEW or SEARCH_FINISHED
```

1. Acquire a usable overhead capture while the arm is stationary. Initialize
   table geometry and inventory if surface evidence permits it. Otherwise retain
   unknown geometry and continue with wrist views.
2. Select the next entry in the predefined view list. Hardware development uses
   manually positioned or recorded views. MuJoCo uses configured joint targets.
3. Wait for positioning and stationary feedback within a timeout. Request a fresh
   wrist capture associated with that view and acquisition pose.
4. Detect candidates, merge observations, update coverage, and refresh the viewer.
5. Advance through the list. Allow bounded recaptures for unusable frames and up
   to three operator-requested additional views after a completed scan.
6. Finish with inventory, achieved scope, unresolved groups/regions, and completion
   reason. The result can be complete, partial, or failed for its declared scope;
   finishing the list does not imply readiness for picking.

The automatic simulator driver smoothly approaches a hover configuration and
swivels joint1 through eight targets. It checks simulator identity, exclusive
simulator command subscribers, fresh feedback/progress, and joint limits. It
waits for convergence and settling before requesting each capture, avoids duplicate
requests, and freezes interpolation when simulation is paused. Faults halt the
scan; these development checks are not a hardware safeguard.

### Detection and incremental merging

For each capture:

1. Reject invalid depth, calibration mismatch, stale data, or missing transforms.
2. Back-project valid depth and transform it into the fixed map frame.
3. Apply conservative pixel/3D exclusion regions and any capture-matched robot
   mask. Excluded pixels must not become observed empty space.
4. Estimate a near-horizontal table plane from supported surface evidence.
   Report inconsistent measurements instead of quietly changing frames.
5. Generate SAM 2 point/box prompts from above-table depth proposals and segment
   capture-matched aligned RGB. Load the SAM 2.1 tiny model once, with pinned
   source/runtime dependencies through `setup_sam2.sh`. No online model calls.
6. Fit known length/width/height from installation configuration to supported
   table-top geometry. Require top-face coverage, dimensional agreement, and
   bounded residuals before treating the pose as resolved. Do not assume object
   count or arrangement; never consume scene poses or IDs.
7. Keep provisional hypotheses with internal references, without block IDs.
   Associate observations one-to-one using shared-frame geometry and uncertainty.
   Retain ambiguous alternatives rather than confirming overlapping new objects.
8. Assign a stable block ID when a known-shape observation supplies sufficient
   evidence, or after consistent resolved observations from distinct views in
   the depth-only baseline. Preserve IDs if later evidence becomes uncertain.
9. Expose confirmed inventory and pose uncertainty to the stacking layer.
   Reject placement proposals using unresolved tracks.

Known-shape fitting currently targets flat table-supported cuboids. Partial,
clipped, tilted, stacked, and touching groups remain hypotheses when evidence is
insufficient. SAM 2 segmentation does not itself guarantee separate instances.
Unmarked rectangular blocks retain 180-degree yaw symmetry. Sampling-based pose
uncertainty is explicitly not a calibrated probability. The depth-only baseline
remains available by explicit configuration; a requested SAM 2 backend never
silently falls back.

Keep the scene static during one scan. Moving blocks, grasp/release, or calibration
changes require a restart or explicit invalidation. Absence from a later view
alone does not prove that a previously seen block vanished.

### Minimal workspace map

Maintain the table plane, bounds of observed surface patches, a coarse top-down
coverage grid, block tracks, unresolved groups, source timestamps, session ID,
revision, and input status.

Coverage states are observed, unobserved, and uncertain. Valid surface evidence
can establish an observed empty patch. Invalid depth, occlusion, and gaps leave
uncertainty. A fitted plane or enclosing rectangle does not establish the table's
full boundary. Table coverage is not collision-free 3D evidence.

Keep accumulated scan coverage separate from live camera freshness. Earlier views
remain history, visibly marked with age; they need not all be simultaneously fresh.
A configured scope specifies where coverage is reported, not a known table shape
or motion permission. Without adequate scope/quality evidence the result is partial.

### Contracts

| Record | Fields |
| --- | --- |
| Capture | Camera/view ID, acquisition time, optical/map frames, calibration revision, depth/RGB, intrinsics, transform, ROI/mask |
| Track | Persistent ID, centre/orientation, dimension estimates/ranges, quality, source cameras, last observation, resolved flag and reasons |
| Workspace | Frame, session, revision, table estimate, coverage, confirmed tracks, provisional hypotheses, warnings, acquisition time |
| Search progress | View index, target, capture state, accepted/rejected counts, retry count, reason/result, simulator restart ID |
| Search result | Snapshot revision, complete/partial/failed outcome, completion reason, `pick_ready: false` |

Use compact structured records and avoid images/clouds inside status messages.
Final shared ROS interfaces with the safeguard and teammate executor are a later
integration decision. The master tolerates measured geometry variation, rejects
old sessions, and exposes inventory stability and partial results. Proposals do
not execute motion or increase verified stack height.

### Performance boundaries

- Use the newest pending observation and bound camera/mask histories.
- Retry a pending frame when its matching mask or capture-time TF arrives; expire
  it by acquisition age rather than accumulating old images.
- Run geometry work outside callbacks, crop observations, and use vectorized
  routines. Fit the plane coarsely while retaining detail around candidates.
- Merge detections rather than rebuilding all historical clouds.
- In simulation, render 320x240 RGB-D/mask bursts only for requested stationary
  captures, up to 3 Hz. Keep the browser's 960x720 frames and 30 FPS target.
- Send small `/junh/search_progress` heartbeats at 5 Hz for scan control. Publish
  full map snapshots at 1 Hz so large geometry messages cannot gate motion.
- Generate RViz markers only when a viewer subscribes. Browser-map polling is
  read-only and independent of camera rendering.

### Visualization

RViz markers and the local browser page both consume the same discovery snapshot.
The browser shows top-down estimated footprints and coverage, with pan/zoom,
selectable candidate details, quality, source cameras, uncertainty, scan state,
session/revision, and stale-publisher indication. It contains no simulator scene
poses. RViz remains available for 3D inspection with Fixed Frame `world` in simulation.

Default local URLs are simulator `http://localhost:23517` and app workspace
`http://localhost:23517/workspace`. The simulator proxies the page and map API
from the localhost-only viewer backend, so remote devcontainers need only the
simulator port forwarded. The simulator header switches views in the same tab.
The backend is also directly available at `http://localhost:23518`.
`WORKSPACE_PORT` can override the default backend port (simulator port plus one). The standalone camera launch enables the web view only when configured.

### Coordinated restart

The simulator's Restart simulation button resets physics and increments its
restart ID. Discovery observes that ID, creates a new map session, clears tracks
and coverage, resets capture progress, and invalidates old worker generations.
Scan progress is fenced by the simulator restart ID, including the case where
progress and scene messages arrive in different orders. Automatic searching resumes
only for the new epoch with fresh feedback.

`/junh/restart_scan` restarts discovery without resetting physics. A simulation
restart does not need that separate manual service call. The browser map clears
old selections on session changes and shows the new search progress.

### Migration and milestones

The scene-based `discover()` is isolated in the simulator adapter. Real discovery
and the master use sensor-independent contracts; simulator clocks/restart details
stay at the adapter boundaries. Exact dimension equality is replaced by tolerant
geometry comparisons. Hardware and simulation launches select their own inputs.

| Phase | Deliverable | Acceptance criteria |
| --- | --- | --- |
| 1. Observation contracts/adapters | Recorded and live calibrated inputs | No scene-file block data; invalid depth/TF rejected |
| 2. Single-view detector | Table plane and separated-block fits | Varied count, position, yaw, and uncertainty handled |
| 3. Inventory/viewer | Persistent IDs, coverage, shared-snapshot displays | Repeated views merge; unknown regions/groups remain visible |
| 4. Multi-view scan | Manual capture list and automatic MuJoCo swivel driver | Wrist views extend observations; timeouts/retries and partial completion explicit |
| 5. Master handoff | Stable snapshot and search outcome | Model absence does not block discovery; capture completion distinct from picking readiness |

Validate with synthetic depth, recorded observations, ROS integration, and live
MuJoCo rendering. Sensor-specific accuracy and threshold tuning require measured
camera recordings and installation validation.

### Verification evidence

Tests cover depth stride/encoding, calibration/transform validation, unknown
count and yaw from pixels, noise/ROI/masks, clipped/irregular groups, track association,
missing-view retention, table conflicts, partial coverage, stationary feedback,
retries/timeouts, extra views, old worker invalidation, real ROS handoff, and marker
publication. Driver tests cover interpolation, command ownership, request races,
and restart epoch ordering. HTTP tests check the local map and session clearing.

A complete eight-view MuJoCo scan was verified, including capture-matched robot
masking. Results remain partial and object fits provisional. Installed launch and
Ctrl+C cleanup have been verified. An end-to-end browser restart request was checked
to change the map session and resume automatic search.

A 32-second browser/search comparison measured:

| Build | Browser FPS | Physics real-time ratio | Actual arm motion startup |
| --- | --- | --- | --- |
| Earlier continuous-camera build | 1.0 | 1.03 | Not observed within 32 seconds |
| Demand-driven installed launch | 28.5 | 1.00 | 3.3 seconds |
| Demand-driven script, including build | 28.7 | 1.00 | 5.5 seconds |

These are development-machine measurements for one browser stream, not hardware
latency or accuracy guarantees. The baseline detector-only synthetic 640x480
benchmark measured 104.9 ms median and 132.5 ms maximum over six runs, excluding
camera/DDS/TF delays. The current combined regression suite has 86 passing tests with real SAM 2
inference enabled and unrelated pytest plugin autoload disabled. The standard
model-free suite skips the two explicitly enabled model integration cases.
The browser JavaScript's drawing, selection, and stale indication were exercised
with a DOM/canvas harness; on-screen browser inspection remains useful during use.

### Deferred work and remaining installation decisions

- Hardware motion planning/execution through the independent safeguard.
- Collision/free-space mapping, adaptive viewpoint selection, and boundary exploration.
- Full hardware robot self-filtering, probabilistic association, and hypothesis fusion.
- Reliable touching/overlapping separation and difficult full 6D pose recovery.
- Picking/stacking execution and verified attachment/contact transitions.

The nominal 0.89 m overhead height and environment guard policy remain in the
safeguard plan. Do not enable hardware motion with an always-permit substitute.
Agree on actual camera topics/frames, calibration, expected block shape range,
fit/association thresholds, search scope, and measured acceptance tolerances before
hardware deployment. The simulator's fixed swivel targets are development fixtures,
not hardware trajectories or proof of complete workspace coverage.

### Discovery identity correction validation

The SAM 2 live MuJoCo scan accepts nine captures and produces six confirmed block
IDs, zero remaining provisional hypotheses, and no extra identities for the six
current scene blocks. Simulation ground truth is used only in evaluation, not
in the perception pipeline. This validates the current flat-block layout; rotated
and varying-count rendered scenes have separate model integration tests.

Run the real-model tests with `MUJOCO_GL=egl`, an explicit
`JUNH_TEST_SAM2_CHECKPOINT`, and the local SAM 2 environment. Ordinary unit tests
do not download model weights. Restart clears confirmed tracks, hypotheses,
alternatives, support views, and counters together.

### Bounded evidence-driven inspection

The simulation now has an initial eight-view sweep and two reserve sweeps with
different shoulder/elbow configurations. Search compares explored coverage and
per-block resolved-view support/pose bounds to configurable `search_criteria`.
Unmet criteria append new configured viewpoints while preserving all identities.
Passing the criteria during inspection ends motion early. No-progress and view
budgets prevent indefinite retries; unmet criteria remain explicit in the partial
result and the shared workspace UI.

Explored scope includes measured table and uncertain object evidence; observed
table scope excludes objects. The 90% default applies to exploration and is not
a guarantee of full visibility, inventory completeness, or free space.
Confirmation, resolved-pose evidence, and observation age remain separate.
All scan targets use zero joint6 roll because the casing-mounted camera is
upstream of that joint. Capture-time camera transforms handle image orientation.
This is still configured MuJoCo motion; general next-best-view planning and
hardware safeguard integration remain deferred.

Live inspection validation: the initial scan is followed automatically by novel
configured poses. The revised scan meets all observation requirements after
22 accepted captures (out of a 25-capture maximum), with 90.5% explored scope,
87.9% observed table surface, six stable confirmed IDs, and no provisional
hypotheses. Every block has at least two resolved camera/viewpoint observations.
The driver stops early with `stop_reason=criteria_met`; the result remains
partial because full map completeness and picking authorization are separate.
The complete suite has 86 passing tests, including actual-model inference.
