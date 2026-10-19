# Shared discovery feature

`phyai_discovery` is the reusable discovery stage for the team's integrated app.
It runs without ROS or MuJoCo. NumPy is the only runtime dependency for depth
processing; SAM2/PyTorch are loaded only when SAM2 segmentation is configured.

```text
camera / recording adapter -> Capture -> Discovery -> workspace + search result
                                            |
                            optional SAM2 -> measured depth geometry

integrated app: discovery -> planning -> safeguard -> execution
junh test app:  discovery + ROS transport + MuJoCo scan driver + workspace viewer
```

The final integrated app owns stage ordering, camera synchronization, robot motion,
and the lifetime of each feature. Create one `Discovery` per scan session and reuse
it for all captures so SAM2 loads once. `reset()` starts a new inventory session
without reloading the model. Changes to calibration or the static scene require a
new scan. Detection remains limited to the existing overhead/wrist camera roles,
near-horizontal tables, and block geometry; this extraction preserves those limits.

Complete, separated depth fits bypass segmentation. SAM2 is called for groups
whose depth geometry is unresolved, such as touching blocks. Within those groups,
depth outside every resolved segment stays provisional. Above-table evidence too
small to fit, or outside the supported object height,
remains unexplained and blocks the observation criteria. Other uncertain depth,
including below-table background, does not increase explored coverage.

For installations where the camera can see the floor or multiple horizontal
surfaces, configure `detector.table_height_range: [zmin, zmax]` in metres in the
map frame. The interval restricts the measured points used to estimate the table;
it never supplies a plane when depth support is missing. Multiple substantial
parallel surfaces within that interval remain ambiguous. The MuJoCo adapter uses
`[-0.005, 0.005]` because its world-frame tabletop is at Z = 0. Hardware adapters
must supply their own measured interval or exclude background with a camera ROI.

Resolved pose bounds include sampling, measured table fit error, and configurable
`detector.depth_error` (default 0.001 m), `calibration_position_error` (0.001 m),
and `calibration_yaw_error` (0.01745 rad). These are assumed error bounds, not
calibrated probabilities; configure them for the actual sensor and calibration.
Consistent independent view poses are averaged, accounting for cuboid symmetry;
cross-view residuals enlarge the reported bounds around that fused pose.
Measurements that disagree beyond their combined position or orientation bounds
invalidate previous support.
Recovery requires resolved fits from two distinct camera/viewpoint pairs agreeing
within their measurement bounds; replaying one view cannot settle a conflict.

## Install

For ROS applications, add `<exec_depend>phyai_discovery</exec_depend>` to the
consumer's `package.xml`. From the colcon workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to junh_master
source install/setup.bash
```

For a Python application, install this package directly (no ROS needed):

```bash
python3 -m pip install -e src/phyai_discovery
```

For source-only development from the repository root:

```bash
PYTHONPATH=src/phyai_discovery python3 -c 'from phyai_discovery import Discovery; print(Discovery().snapshot())'
```

## Call from the main pipeline

The camera adapter constructs `Capture` objects with depth in metres, calibrated
`Intrinsics`, a capture-time `camera_to_map` transform, camera role, viewpoint,
acquisition timestamp, and calibration revision. Optional RGB must be aligned to
the depth image. An exclusion mask marks robot/invalid regions as `True`.
The adapter must verify synchronization, freshness, and stationarity before
passing `stationary=True`. Use `Stationarity` for the current seven-joint arm's
feedback, or supply equivalent verification for your hardware.

```python
from phyai_discovery import Discovery, SearchState

feature = Discovery({
    'map_frame': 'base_link',
    'viewpoints': ['left', 'right'],
    'detector': {'known_block_size': [.075, .025, .015]},
})

# overhead_capture and wrist_capture come from your camera/recording adapter.
feature.process(overhead_capture, stationary=True)

# The main app moves to feature.search.target, then requests observation.
feature.request_capture(now_monotonic)
feature.tick(now_monotonic, stationary=True)  # after verified settling
feature.process(wrist_capture, stationary=True)

# Repeat request/tick/process for remaining requested viewpoints.
if feature.search.state == SearchState.FINISHED:
    handoff = feature.snapshot()
    result = handoff['search']['result']
    tracks = handoff['workspace']['tracks']
    # Pass inventory and uncertainty to the planning stage.
```

Drive `tick(now, stationary=...)` periodically with a monotonic clock, including
while waiting for sensors or processing, to enforce timeouts. Timestamped
`Capture.stamp` uses the sensor acquisition clock; keep it separate from the
monotonic timeout clock. The initial overhead observation is requested automatically.
Wrist observations require `request_capture()` and a stationary `tick()` first.
`finish()` ends a scan early; `add_view()` adds a bounded manual inspection view
after finishing. Automatic inspection rounds and search criteria use the existing
`search_criteria` and `inspection_rounds` configuration keys.

For a worker-based adapter, call `job = feature.begin(capture, stationary=True)`
on the owner thread, run `feature.detector.detect(job.capture)` on one worker,
then call `feature.complete(job, detection)` on the owner thread. On failure call
`feature.fail(job, error)`. Old jobs return `False` after timeout, reset, or finish;
they cannot merge into a newer scan. Do not run the same detector concurrently or
mutate a capture's arrays while a worker uses them. Lifecycle calls belong to one
owner thread. `process()` provides the same lifecycle synchronously and rejects
failed detection jobs before re-raising their exception.

## Result contract

`snapshot()` returns the existing `workspace` and `search` envelope. ROS/app
metadata such as `input_mode`, stream ages, and simulator restart IDs belongs to
adapters. Existing `/junh/discovery` consumers continue to receive their envelope.

| Field | Meaning for the next pipeline stage |
| --- | --- |
| `workspace.session_id`, `revision`, `frame` | Inventory identity, version, coordinate frame |
| `workspace.table`, `coverage` | Measured table evidence and observation coverage |
| `workspace.tracks` | Confirmed IDs, geometry, uncertainty, and resolution state |
| `workspace.hypotheses`, `warnings` | Evidence requiring further observation |
| `search.state`, `target` | Scan state and requested physical viewpoint |
| `search.result.status` | `failed`, `partial`, or `complete` after scan termination |
| `search.result.criteria_met` | Whether configured observation criteria were met |
| `search.result.pick_ready` | Always false; discovery does not validate a grasp |

A finished sequence may produce partial results. Planning must inspect uncertainty
and scan outcome; discovery alone does not authorize picking or bypass safeguard.
The existing `junh_master.handoff.Master` remains a prototype consumer in the test app.

## SAM2 configuration

Supply the checkpoint path through the feature config. Other apps need not source
`junh` scripts or use its environment variable:

```python
feature = Discovery({
    'segmentation': {
        'backend': 'sam2',
        'checkpoint': '/path/to/sam2.1_hiera_tiny.pt',
        'device': 'cuda',  # or explicitly 'cpu'
    },
})
```

Omit segmentation or set `backend: depth` to use measured depth alone. For the
current pinned SAM2 installation recipe, see
[`apps/junh/setup_sam2.sh`](../../apps/junh/setup_sam2.sh). Its existing environment
and checkpoint remain at `apps/junh/.venv-sam2` and `apps/junh/.models`; this
refactor does not move or download model files. The integrated app can provision
one shared runtime/checkpoint and inject its path into the same config.

## Recorded captures

Replay measured captures without installing the junh app:

```bash
python3 -m phyai_discovery.recording /path/to/scan.json --output /tmp/snapshots.jsonl
```

The installed Python command `replay_scan` runs the same implementation.

## Verify

From the repository root:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/phyai_discovery:apps/junh \
  python3 -m pytest -q src/phyai_discovery/test apps/junh/test
```

With MuJoCo and a working EGL renderer installed, prefix that command with
`MUJOCO_GL=egl` to include randomized rendered-depth tests with a visible floor,
multiple camera positions, stable identities, and checks against ground truth.

Source ROS and a built workspace to include ROS adapter tests. Actual SAM2 tests
additionally require the local SAM2 environment and checkpoint, as documented in
[`apps/junh/README.md`](../../apps/junh/README.md).
