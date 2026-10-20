# Project safeguard app

This planned app provides environment-specific motion validation, runtime
monitoring, and a command gateway for wrist search, picking, and stacking.
It forwards approved commands to the existing general-purpose ROS 2 bridge.
Project workspace and overhead-clearance rules belong here; the bridge remains
unchanged by this app separation.

Read the [implementation plan](docs/implementation-plan.md) for architecture,
shared interfaces, process boundaries, failure handling, and validation phases.
The workspace publisher and search client are planned in
[Junh's implementation plan](../junh/docs/implementation-plan.md).

This directory currently contains documentation only. A safeguard runtime,
ROS package, and `start.sh` have not been implemented.
