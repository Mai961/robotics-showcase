# nt-bridge: NetworkTables 4 ↔ ROS 2 mirror

See the project site: https://mai961.github.io/

## What it does

It is a ROS 2 node that connects to the robot program's NetworkTables 4 (NT4) server as a client.
NT4 is WPILib's own publish/subscribe system, used by dashboards. The node copies chosen ROS
topics into NT, so dashboards and the robot side can see them, and copies chosen NT entries back
into ROS topics. Geometry goes out as WPILib structs (`Pose3d` and similar), so robot code and
dashboards decode it natively. A second part, not shown here, mirrors the ROS parameters of chosen
nodes into NT and applies edits made from the robot side.

## Why it exists

The ROS side needed a way to show its state on the tools the robot side already uses, and to
receive a few values back. It is included here because it is the second half of the transport
story; `udp-bridge` is the first half.

## Why two bridges

This repository has two ROS bridges: this one, and [`udp-bridge`](../udp-bridge/) for typed UDP.
The source documents divide the work between them as follows:

- **UDP carries what the robot program acts on**: the fused field pose, the whole-body arm
  references and measurements, and operator commands. The SystemCore workspace README says it
  plainly: "The robot reads **UDP only**." The UDP bridge's package description calls it "the
  successor to talos_bridge's NT4 backend", using typed WPILib-struct datagrams.
- **NT4 is a mirror for people and tools.** The coprocessor runbook lists this bridge as
  "ROS → NT4 (`/Talos/Topics/...`) for the RIO/dashboard, and the Limelight botposes back in". The
  SystemCore config calls the NT copy of the pose "a debugging/logging convenience only", because
  "a RIO-side probe found zero NT readers of this key". Its parameter bridge also lets ROS
  parameters be edited from the robot side; the tuning bench uses that.

The sources do not compare the two bridges' latency, and neither does this README.

## How it works

Read `nt_ros_bridge.cpp` top-down:

1. `connect_to_robot` starts an NT4 client. On the robot, it finds the server by team number; in
   simulation, it connects to a local server.
2. `discover_topics_to_mirror` reads each configured topic's message type from the ROS graph once a
   second, and mirrors it to `<nt_topic_root>/<topic>`. Topics that do not exist yet are retried.
3. The `PoseStamped` converter publishes a WPILib `Pose3d` struct. The ROS header is dropped,
   because NT stamps each value itself when it is set.
4. The `JointState` converter publishes a subtable of parallel arrays (`names`, `position`, ...),
   because NT has no single type for it.
5. `mirror_double_array` shows the NT-to-ROS direction. A 10 ms timer drains the NT queue and
   republishes only the newest value.

## Interfaces

| Direction | Example | Mapping |
|---|---|---|
| ROS → NT | `/odin_tree/field_base` (`PoseStamped`) | `/Talos/Topics/odin_tree/field_base` as `struct:Pose3d` |
| ROS → NT | `std_msgs` scalars and arrays | `boolean`, `integer`, `double`, `string`, `double[]`, `raw` |
| ROS → NT | `JointState` | subtable `<key>/{names,position,velocity,effort}` |
| NT → ROS | a configured NT key | `Bool`, `Int64`, `Float64`, `String`, `Float64MultiArray`, `ByteMultiArray`, ... |

## How it was run

The bridge ran as a ROS 2 node on the coprocessor, connected to the robot program's NT4 server. It
mirrored the localization topics for dashboards and logging, brought camera pose estimates from the
robot side into ROS, and carried the tuning bench's gains.

## Replay

No recording is included for this module. Its example ROS topic, `/odin_tree/field_base`, is in
the [`state-estimation`](../state-estimation/) replay.

## Status

**Status: infrastructure excerpt.** The file is simplified from the source and will not compile
or run on its own.

## Credits

Written by Zimeng Chai (GitHub `KeseterG`). It is included here because it is the second half of
the transport story. Zile Liao's part is negligible: one packaging commit (a version bump and
rebuild) out of 2 non-merge commits on the package
(`git log --no-merges --format=%an -- src/talos_bridge` in the source repository).

The original package is licensed BSD-3-Clause, and its excerpt here is reproduced under that
license.
