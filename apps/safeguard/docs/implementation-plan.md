# Project safeguard implementation plan

## Purpose and ownership

Create `apps/safeguard/` as an independent project app that validates and monitors
all wrist-search, picking, and stacking motion before forwarding commands to the
existing ROS 2 bridge. This app owns environment-specific constraints. The bridge
remains native and general purpose; this work does not add table, camera, or
stacking rules to it.

The safeguard remains active across task transitions and task-app restarts.
It has its own launch, dependencies, tests, configuration, and lifecycle. This
is a planning document; no safeguard runtime or enforcement is implemented yet.
The discovery and workspace publisher remain in
[`apps/junh/`](../../junh/docs/implementation-plan.md).

## System boundary

```text
Junh wrist-search planner ──────┐
                               ├─> Project safeguard ─> Native ROS 2 bridge
Teammate pick/stack executor ───┘                              |
                                                     Robot SDK / CAN
                                                             |
                                                  Robot internal controller
```

The safeguard consumes workspace geometry and robot feedback through explicit
interfaces; it does not import `junh_master` internals. The Junh app owns camera
processing, discovery, map refinement, search viewpoint selection, and display.
The teammate app owns picking/stacking policy, grasp/release verification, and
trajectory generation. The safeguard owns permission and command forwarding.

Deploy validation workers separately from the bounded monitor/gateway process.
They may run on the same physical computer. Keep them separate from perception,
model inference, planning, and visualization. Determine deployment location from
latency measurements and bridge integration; a control-PC service is a candidate.

## Installation and environment configuration

Establish robot collision geometry, joint limits, timestamped feedback, camera
attachment geometry, a verified starting configuration, a conservative initial
search volume, overhead obstacles, controller stop behavior, and freshness limits.
Keep installation constraints in the fixed robot reference frame. Table geometry
and observed free space arrive from the workspace publisher and refine the
permitted volume without automatically relaxing installation constraints.

Project environment configuration belongs in this app, with a measured profile
for this block-stacking installation. A future project can supply its own profile
or safeguard implementation without changing the ROS 2 bridge.

The overhead camera is approximately **0.89 m above the tabletop**. This is a
nominal installation value, not a measured clearance for the entire assembly.
Measure the lowest obstruction, including the housing, mount, and cables.
Choose a conservative ceiling using:

```text
permitted ceiling = lowest overhead obstruction - clearance allowance
```

The allowance must cover geometry and calibration uncertainty, latency, and
stopping motion. Do not select a numerical allowance before those are measured.
Represent the overhead assembly as explicit obstacles as well as retaining the
initial conservative ceiling. Perception cannot automatically relax installation
limits. If the table estimate changes, recompute clearance conservatively in the
fixed robot frame.

## Shared motion and workspace interfaces

Define a common ROS interface package for task clients and the safeguard, rather
than duplicating message definitions inside each app. Agree on trajectory,
workspace, attachment, feedback, cancellation, and fault semantics before choosing
message/action types. Use metres, radians, explicit frames, acquisition timestamps,
session IDs, and revisions; document quaternion conversion at ROS boundaries.

| Record | Required information |
| --- | --- |
| Guard decision | Trajectory ID, map and calibration revisions, expiry, permit or reject, reason and limiting geometry |
| Motion request | Client and request IDs, session, timestamp, time-parameterized trajectory, initial state, task phase, carried-object geometry, requested contacts, map and calibration revisions |
| Execution feedback | Request ID, accepted/running/stopped/completed/rejected/fault state, timestamp, reason, permit state, stop acknowledgement |
| Workspace input | Fixed frame, session and revision, table geometry and uncertainty, obstacle geometry, relevant free-space evidence and its age, calibration revision |
| Attachment update | Object ID and geometry, attachment transform, evidence and verification state, timestamp and revision |

The repository builds one selected app directory into a runtime image. Specify
how the shared interface package is installed in each app image before
implementation; placing it outside `apps/safeguard/` alone will not include it in
that build. Keep the same interface version in the safeguard and task images.

## Guard and runtime monitor

The safeguard is a shared motion authority for wrist search, approach, grasp,
lift, transfer, placement, and retreat. Task code requests motion through it;
the safeguard owns validation, permission, monitoring, command forwarding, and
stop handling. A client cannot opt out by labeling a movement as picking or
stacking.

Represent the permitted workspace in 3D, including lateral boundaries, lower
clearance, and the overhead ceiling. Define the base and mounting region
separately from regions allowed for moving links. Check full robot collision
geometry, wrist camera and attachments, and carried-object geometry. A gripper
centre inside the table footprint is insufficient.

The guard should check:

- Full trajectory geometry, including swept motion between checked states.
- Table-edge clearance, permitted height, fixed obstacles, and self-collision.
- Joint position, velocity, and acceleration limits.
- Collision and calibration uncertainty allowances.
- Clearance needed for stopping, based on measured controller behavior.
- Valid map, calibration, feedback, trajectory revision, and permit expiry.

Use a defined whole-path checking strategy with resolution and clearance bounds;
checking sparse waypoints alone can miss a violation. MoveIt's planning scene is
a candidate backend for robot/environment collision checking. Custom workspace
containment, freshness, permit handling, and stop integration still need explicit
implementation. See the [MoveIt Humble planning scene documentation](https://moveit.picknik.ai/humble/doc/examples/planning_scene/planning_scene_tutorial.html).

The runtime monitor checks actual state and the expected stopping trajectory,
not just planned commands. Detect boundary approach early enough to stop within
the permitted volume. On stale inputs, tracking error, revoked permits, or a
constraint violation, request a controller-supported stop and confirm the robot
has stopped. Merely ceasing publication is not a defined stopping mechanism.
Specify fault behavior if acknowledgement or stopping exceeds its deadline.
Preserve a supported gripper state during stopping; do not automatically release
a carried block.

All app motion requests must use the command gateway. Other publishers to the
robot command topic would bypass an app-level guard; verify exclusive command
ownership or host-side arbitration before hardware execution. Hardware emergency
stop and controller protections remain independent of this software subsystem.
Resume after a stop requires fresh state, a reassessed map, and a new motion
permit; an old queued trajectory must not restart automatically.

### Teammate executor integration

Use a ROS action or an equivalent request/feedback interface for guarded motion.
The action server should acknowledge requests quickly and validate asynchronously.
Only one client may hold active motion ownership at a time; a search request and
a pick/stack request cannot interleave robot commands. Bound pending work and
discard canceled, expired, or previous-session requests.

The teammate executor submits time-parameterized joint trajectories with an
expected starting state and task context. Approval is tied to the exact submitted
trajectory, attachment state, and relevant workspace constraints. The gateway
executes that trajectory and reports progress; changing it requires validation
again. An approval token must not authorize arbitrary later commands. Initially,
use finite trajectories; low-level streaming control requires a separately
defined bounded-horizon guard contract.

Picking and placing introduce intentional contact. Define narrowly scoped contact
rules for the selected object and appropriate gripper surfaces. Clients may
request a contact phase but cannot disable environment or workspace checks.
Update carried-object geometry when a grasp is verified and after verified
release. Reject motion if attachment state is unknown or inconsistent. The
teammate executor supplies grasp/release evidence through an agreed interface;
the guard does not treat a model prediction as verified attachment state.

If a client disconnects during execution, apply the defined controller stop
policy and confirm stopping. If the safeguard itself dies or communication to
the controller is lost, a controller/host watchdog must enforce the agreed
timeout behavior; a dead monitor cannot issue its own stop. Restart invalidates
old ownership, requests, and permits. Establish fresh robot and attachment state
before accepting another trajectory.

### Performance boundaries

Keep image conversion, point-cloud reconstruction, map rebuilding, model
inference, planning, and rendering outside the monitor/gateway process. Pass
compact robot state, conservative collision geometry, and map updates through
bounded interfaces. Separate processes alone do not guarantee deadlines under
CPU saturation; measure monitor scheduling and stop latency with camera and
teammate workloads active, and reserve resources where measurements require it.

Do not invalidate every trajectory for every observation revision. Reassess when
changes affect its permitted corridor, obstacles, attachment state, calibration,
or freshness. A relevant new obstruction must revoke approval promptly. Keep
accumulated search coverage distinct from the fresh geometric evidence required
to authorize the currently executing motion.

## Bridge compatibility and failure handling

Use the existing bridge command interface behind the gateway, with deployment
configuration ensuring that task apps cannot independently command the robot.
The safeguard does not make arbitrary publishers trustworthy merely by running.
Establish command ownership through the deployment or existing host arbitration;
do not add project geometry rules to the bridge.

Confirm which stop, timeout, acknowledgement, and watchdog capabilities the
native bridge and robot controller already support. Use those general-purpose
capabilities without changing their semantics for this project. If a required
stop or watchdog capability is absent, record that as a hardware-execution blocker
and resolve it with the bridge owner; do not emulate a stop by withholding messages.
The generic bridge might be extended separately for a missing general capability,
but that is outside this app separation and needs its own design.

## Implementation milestones

| Phase | Deliverable | Acceptance criteria |
| --- | --- | --- |
| 1. Contracts and installation profile | Shared interfaces, measured geometry/limits, bridge capability inventory | Clients agree on finite trajectory and feedback semantics; frames and revisions are consistent; stop and watchdog prerequisites are identified |
| 2. Offline validation | Robot/attachment geometry, workspace containment, whole-path collision strategy | Intermediate-path, lateral-boundary, height, joint-limit, attachment, and contact-rule violations are rejected |
| 3. Gateway and monitor | Independent processes, exclusive client ownership, permit expiry, stop handling | Only approved exact trajectories reach the bridge; competing clients and stale requests are rejected; feedback deadlines and stop acknowledgements are checked |
| 4. Failure and load validation | Client/safeguard restart behavior, watchdog integration, stressed scheduling | Client loss and safeguard failure follow the defined stopping policy; old requests never resume; monitor deadlines and stopping clearance hold under camera/model load |
| 5. Task integration | Junh search and teammate pick/stack clients | Both use the same safeguard; grasp/release attachment transitions are verified; relevant workspace changes revoke or reassess permits |

Use synthetic geometry and recorded state first, followed by simulator integration.
Enable physical search/pick/stack only after bridge capabilities and measured
stopping behavior meet the installation's acceptance criteria.

Required cases include overhead ceiling approach, table edges and gaps, unsafe
intermediate trajectory segments, self-collision, camera/attachment collision,
invalid starting state, intentional contact limited to the selected object,
unknown attachment state, stale feedback/transforms/maps, map contraction during
execution, irrelevant map revisions, competing clients, expired permits, client
loss, safeguard death, bridge communication loss, stop timeout, and restart.

Measure validation latency, monitor scheduling delay, queue age, feedback age,
stop-request-to-acknowledgement latency, stopping distance, and CPU/memory use.
Select deadlines and clearance allowances from measurements at intended speeds.

## Decisions and measurements still needed

1. Robot and attachment geometry accuracy, initial search envelope, and measured
   overhead obstruction geometry; the 0.89 m height is only nominal.
2. Existing bridge command, stop, acknowledgement, and watchdog behavior.
3. Deployment command ownership and competing-publisher prevention.
4. Common ROS interface package location and inclusion in selected app builds.
5. Monitor execution budget, resource isolation, permitted speeds, and stopping
   allowances under simultaneous discovery and teammate workloads.
6. Accepted grasp/release evidence and attachment/contact transition protocol.
7. Freshness rules, relevant map-change detection, and permit expiry semantics.

## Proposed app layout

```text
apps/safeguard/
├── README.md
├── docs/implementation-plan.md
├── config/                    # Future measured project profiles
├── start.sh                   # Future independent launch entry point
└── safeguard_app/              # Future ROS package
    ├── package.xml
    ├── launch/
    ├── safeguard_app/
    │   ├── guard.py
    │   ├── monitor.py
    │   ├── gateway.py
    │   └── bridge_adapter.py
    └── test/
```

Only README and planning documentation are created at this stage. Runtime code,
launch scripts, and configuration profiles follow the milestones above.
