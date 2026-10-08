# llm-tuning-agent: PID tuning on a test bench, steered by a coding agent

Shown in motion on my project site: https://mai961.github.io/#auto-tuning

## What it does

It tunes the position controller (PID plus feedforward gains) of one robot arm joint on a test
bench. A ROS 2 node on the bench computer runs the experiments: it sets gains, drives the joint
through a test trajectory, scores the result, and aborts if anything looks unsafe. The decisions
in between, such as which gain to sweep next and over what range, and when to stop, are made by a
coding agent (Claude Code) following a written procedure.

There is no language-model API client in this code. The agent is a coding assistant running a
documented procedure against the bench node's parameter interface: it sends one directive at a
time and reads back a result file.

## Why it exists

A blind grid or greedy search over gains tends to chase noisy minima into extreme values. In the
procedure's own words: "you don't replace the search, you **point it**". The deterministic sweep
stays in code that enforces the limits. The reasoning about traces, such as recognizing that
overshoot at the stop calls for acceleration feedforward rather than more damping, is done by the
agent, and every move has a stated reason.

## How it works

The bench node runs gain experiments inside hard limits: every gain has a bounded range, and an
episode that leaves the joint's safe envelope is aborted and can never win a sweep. A coding agent
chooses the next sweep and when to stop, by reading each result and the traces behind it. Safety
stays in code: whatever the agent sends, the node enforces the bounds and the abort guards. When the
agent is done, the node writes the tuned gains to a file, and a human moves them into the robot's
configuration.

## Interfaces

The agent talks to the bench node through one ROS parameter and a result file. The bench node
reaches the joint on the roboRIO through NetworkTables 4 (via `nt-bridge`): it sends gains and a
command, and receives the joint's telemetry.

## How it was run

The node ran on a bench computer wired to a roboRIO that drove one arm joint at a time (shoulder
or forearm). A Claude Code session drove the loop over SSH, following the procedure. The procedure
records a provisional result for the shoulder from a session on 2026-06-30 (loaded arm, unstable
mount), with a note to re-validate it on a rigid mount.

## Replay

No recording is included for this module. The project site shows a clip of a tuning session.

## Status

**Status: clip only (website).** The bench node's and the procedure's source are not in this
repository. Tuning quality compared with manual tuning: not measured.

## Credits

Written by Zile Liao: all commits in the source repository are his
(`git log --no-merges --format=%an`). The procedure is written for, and was run by, an AI coding
assistant, as described above.
