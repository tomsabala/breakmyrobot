# CLAUDE.md — Agent Working Instructions

You are working on **Resilient Autonomy Lab**, a robotics-first interactive simulation project.

Read these documents before proposing changes or implementing features:

1. [`README.md`](README.md) — Product vision, MVP boundaries, and success criteria.
2. [`ENGINEERING.md`](ENGINEERING.md) — Binding engineering principles and acceptance standards.

If an implementation shortcut conflicts with `ENGINEERING.md`, **preserve the principle**. Do not weaken it silently to finish a feature or make the demo look better. Record the tradeoff, limitation, or blocker clearly.

## Project mission

Build a platform where a user selects a **realistically modeled robot type** (initially a Roomba-style differential-drive robot or a car-like steering robot), configures a compatible environment and capabilities, gives it a destination, watches it navigate, and injects environmental, sensor, capability, or communication disturbances while it is operating.

The project's value lies in **robot integration, navigation, localization, sensing, control, communication, fault handling, and inspectable behavior**. The dashboard enables those workflows; it is not the engineering centerpiece.

The user selects **one robot per experiment**. “Multiple robots” means support for multiple selectable types, not simultaneous fleet coordination.

## Non-negotiable implementation behavior

- **Never fake robotic behavior for a convincing demonstration.** Robot motion, sensing, planning, control, and reactions must result from operational simulated subsystems.
- **Never fake fault recovery.** Show a recovery only when it occurs; safe stops, aborts, degraded operation, and failures are valid results.
- **Never equate different robots with different visual models.** Their genuine motion constraints, sensors, control modes, and valid configurations must influence execution.
- **Never claim a fault is implemented when it only modifies dashboard state.** Prove that it affects an actual modeled world property, physical capability, sensor stream, or communication path.
- **Never use simulator ground truth as the robot's localization estimate without labeling that shortcut.** The dashboard can display ground truth separately for evaluation.
- **Never fabricate sensor confidence, telemetry, internal decisions, status transitions, or event logs.** Use verified sources and explicit “unsupported/unavailable” states.
- **Never silently substitute canned paths, teleportation, scripted recovery, or cosmetic effects for navigation/control.**
- **Do not force one robotics protocol or control model across incompatible robots.** Platform-specific integration is acceptable and expected.
- **Do not add implementation breadth at the expense of a working, measurable robotics vertical slice.**

## How to work on a task

1. **Establish the current truth.** Inspect the repository and identify which robot, simulation, navigation, localization, control, and telemetry pieces actually exist. Never assume a feature is present because an interface suggests it.
2. **Tie the task to an MVP outcome.** Specify the operator action, expected robotic effect, and what evidence would confirm it.
3. **Identify compatibility and boundaries.** Which robot(s) and configurations support this? Which input, actuator, simulated physical property, or communication channel will be affected?
4. **Implement the smallest honest vertical slice.** Prefer working end-to-end behavior over UI placeholders or speculative abstraction layers.
5. **Verify the causal chain.** Check real input/output, robot behavior, and observable results—both nominal operation and at least one failure or degraded case where relevant.
6. **Report facts clearly.** State what was implemented, which scenarios were exercised, what passed/failed, and which effects remain unverified or unsupported.
7. **Update project documentation** when a capability, limitation, test scenario, or user-facing behavior changes.

## Project-specific decisions and priorities

- Keep implementation and architecture decisions open until justified by working requirements. Do not invent a large architecture from the high-level specification.
- Established robotics software is encouraged when it is appropriate; understand and validate its behavior rather than treating it as an opaque black box.
- Treat integration, coordinate frames, state estimation, control semantics, timestamps, and telemetry provenance as serious engineering concerns.
- Make faults available in the UI **only when they have a real implemented effect** on the selected robot. Clearly mark any planned or unsupported features.
- Show truth versus estimate when both are available; clearly distinguish operator-issued commands from robot/system-confirmed outcomes.
- The initial implementation may validate one robot end-to-end, but the defined MVP requires **two genuinely different robot types**.
- Avoid overbuilding the dashboard. Expose the diagnostics required to understand robotic behavior first; optimize appearance second.
- A mission that cannot be completed safely must be allowed to fail visibly and accurately.

## When presenting a plan or change

Include concise answers to these questions:

- **Robotics objective:** Which real robotic behavior or integration does this add?
- **Supported platforms:** Which robot types and configurations does it apply to?
- **Observed mechanism:** Which real subsystem produces the behavior?
- **Proof:** How will we validate it beyond a successful-looking UI?
- **Limitations:** What is not modeled, supported, or yet demonstrated?

If a request encourages misleading realism, unsupported cross-robot generalization, or fabricated results, explain the problem and propose an honest narrower implementation instead.

## Definition of done for a feature

A feature is done when it functions through the real simulated robotics path, is valid for its supported robot configurations, is observable and testable, has meaningful failure behavior, and has truthful documentation. A working UI control alone is **not** enough.

**Engineering integrity is a feature.**
