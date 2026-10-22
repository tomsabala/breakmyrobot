# Engineering Principles & Acceptance Standards

This document defines **constraints that must survive implementation pressure**. The project should be technically honest, physically grounded, and valuable as hands-on robotics engineering work. These principles take precedence over convenience, UI polish, or a demo that merely looks convincing.

## 1. The fundamental rule: no fake robotics

**Every demonstrated behavior must emerge from the simulated robot, its configured sensors, actuators, navigation/control software, and environment—not from a dashboard animation or a hardcoded success narrative.**

Allowed: simplifying the world, choosing a small set of modeled faults, using established robotics algorithms, using documented abstractions, and offering clearly identified diagnostic overlays.

Not allowed: teleporting a robot along a calculated line; drawing a path that the controller never follows; changing a UI health icon without changing a sensor stream; fabricating localization uncertainty; inventing recovery events; silently overriding collisions; or calling a scripted reaction “autonomous recovery.”

Simulation is a model of reality, not reality itself. **State model assumptions and limitations openly**; never claim field-level fidelity without validation.

## 2. Robots must remain physically distinct

Robot selection changes more than appearance.

- Respect the platform's geometry, collision volume, motion constraints, actuation, and achievable maneuvers.
- A differential-drive robot may rotate in place; an ordinary Ackermann-steered car may not. Do not give the car differential-drive behavior for convenience.
- Robot-specific sensors and control modes must match what is actually modeled or supported.
- Robot-specific configuration, valid destinations, failure options, and relevant telemetry should be driven by actual capabilities, not identical forms with relabeled names.
- Reject incompatible configurations explicitly; do not silently invent missing capabilities.
- Reuse concepts between robots where appropriate, but preserve genuine differences in planning, localization, and control.

**Selecting one platform for a run is required; commanding multiple robots simultaneously is not.**

## 3. Navigation must be an actual closed loop

A navigation task must produce robot motion through actual command execution in simulation.

- The system must continually use observed/estimated state, not just precompute an ideal trajectory and animate it.
- Obstacles must participate in collision or environment modeling and in the sensing/planning pipeline as appropriate.
- Replanning, avoidance, slowing, stopping, failure, and mission completion must reflect actual subsystem outcomes.
- Respect collision geometry, robot footprint, steering/turning limits, acceleration/velocity bounds, and other constraints that the chosen simulated platform models.
- Do not call a position trace a “path planner” if it cannot produce executable robot behavior.
- Distinguish **goal accepted**, **navigation active**, **goal reached**, **unreachable**, **aborted**, and **unsafe to continue** whenever the underlying system can establish those states.

A blocked path does not guarantee a new feasible path. A valid outcome is to stop and report failure.

## 4. Localization must be honest about what the robot knows

The robot's state estimate is not the simulator's ground truth.

- Maintain a clear distinction between **true simulated state** (for evaluation) and **estimated state** (available to navigation).
- Don't leak ground-truth pose directly into the operational navigation path while claiming sensor-based localization. If a temporary shortcut is used for development, label it explicitly and do not count it as satisfying localization requirements.
- Sensor availability, drift, noise, timing, and loss must affect the relevant estimation input if those failure modes are claimed to be implemented.
- A “confidence” or “uncertainty” display must come from a defined estimator metric; don't invent a percentage as a decorative health score.
- Log when pose estimates become stale, diverge, or cannot support safe operation, where detectability exists.
- Do not claim the robot detected a localization problem if only an omniscient simulator-side checker discovered it.

For fault experiments, show both **what actually happened** and **what the robot had enough information to know**.

## 5. Fault injection must act at the correct boundary

There are distinct fault categories. Treat them as different engineering problems:

| Category | Examples | Meaningful effect |
| --- | --- | --- |
| **Environment** | New wall, moved obstacle, blocked passage | Changes collision/sensor environment and may invalidate routes. |
| **Sensing** | Missing readings, delay, drift, bias, noise | Changes data delivered to the real consumer of that sensor. |
| **Actuation / mobility** | Reduced torque/velocity capability, steering fault, wheel slip | Changes physical response or achievable motion in the simulator. |
| **Communication** | Dropouts, latency, message loss | Changes actual command/telemetry transfer across a modeled link. |
| **Mission conditions** | New destination, cancel request | Changes actual command or mission state, not just dashboard text. |

For each implemented fault, document:

1. **Injection point:** What modeled component, channel, or physical property is altered?
2. **Affected consumer:** Which estimator, planner, controller, or communication endpoint receives the altered behavior?
3. **Expected observable consequences:** What should change in messages, estimates, commands, motion, or state?
4. **Recovery/removal:** How is the fault cleared, and what might persist afterward?
5. **Limitations:** What aspects of the real-world failure are *not* represented?

A control cannot be displayed as supported merely because the UI can toggle it. If the robot does not have GPS or does not use GPS, a “GPS failure” control is invalid or has to be explicitly demonstrated as a no-impact experiment—not promoted as a meaningful recovery test.

Faults may be introduced and removed **during an active mission**. If several faults overlap, their real combined effects must be preserved; never reset the robot behind the scenes to produce a clean demo.

## 6. Recovery must follow real system behavior

**Recovery is an observed capability, not a promised animation or a hardcoded message.**

- Responses must derive from actual failure detection, estimated state, and supported robot capabilities.
- Clearly distinguish fault injected, fault observable, fault detected by the robot, action attempted, and recovery verified.
- A fault may be invisible to the robot, or visible without being recoverable. Both are legitimate results.
- Never equate reaching the goal with safe behavior; a safe stop, abort, or operator handoff may be correct.
- If recovery policy is explicitly programmed, describe it as a deterministic policy—not general reasoning or intelligence.
- If autonomy depends on communications or external control, state that dependency rather than falsely claiming independent operation.

Example: disabling GPS should change only a GPS signal that is actually used. If another localization method remains viable, the robot might continue. If not, it may drift, become lost, or stop. **No predetermined recovery outcome is acceptable without corresponding real behavior.**

## 7. Communication and integration must be authentic

The project should teach practical robotics integration, not manufacture protocol complexity.

- Use platform-appropriate command, feedback, telemetry, and health interfaces.
- Preserve meaningful semantics such as frame conventions, units, message timestamps, command acknowledgment, timeouts, and stale data detection where applicable.
- Do not mislabel application-level dashboard communication as a robotics protocol implementation.
- MAVLink is relevant for compatible autopilot-equipped robots; it is not a universal protocol requirement for a vacuum-style robot or car.
- Different robots may need different software interfaces. Interoperability should not erase essential platform behavior.
- Simulated disconnection has to occur on an actual modeled communication path to claim link-loss resilience.
- A dashboard reporting “disconnected” without changing operational communication is only a cosmetic demonstration.

## 8. Observability is part of the product

The dashboard must support an engineer's investigation. It must not merely replay a story.

At minimum, aim to distinguish and timestamp:

- **Mission intent:** destination, mission state, cancellation/abort outcomes.
- **Simulation truth:** physical robot pose, collisions, environment modifications.
- **Robot knowledge:** estimated pose, sensor outputs and status, navigation state.
- **Robot actions:** commanded versus actual movement, when available.
- **Fault history:** requested fault, confirmed activation, affected source, clearance.
- **System behavior:** real detected errors, replanning/recovery attempts, timeouts, safety transitions.

Engineering dashboards must clearly label provenance: simulator ground truth, robot-estimated value, actual measured value, operator command, or derived metric. Do not display inferred internal reasoning as fact. An event such as “path replanned” requires real evidence from the navigation system.

Metrics must have definitions. Examples: mission time, path length, collisions, goal error, replanning count (if directly measurable), estimation error (using truth for evaluation), and delay from injection to observable response.

Use timestamps and stable experiment identifiers so different streams and experiments can be related. Do not claim precision the data sources cannot support.

## 9. Safety, validity, and failure are first-class results

This is a simulator; any observed behavior must not be interpreted as proof of safety on physical hardware.

- Preserve the distinction between simulation validity and real-world safety certification.
- Enforce the robot's modeled physical constraints and collisions instead of silently correcting unsafe motion.
- Surface impossible commands, unsupported sensor combinations, unreachable goals, and loss of localization as explicit states.
- Make unsuccessful experiments inspectable and reproducible.
- Avoid hidden resets, automatic teleports, and implicit retries that conceal failure unless they are clearly exposed as intentional recovery mechanisms.
- Keep safety-relevant stops attributable to actual control/supervision logic, not dashboard-side after-the-fact declarations.

**An honest failure with complete evidence is better than a fabricated success.**

## 10. Test the behavior, not just the interface

For each core scenario, capture enough evidence to establish a causal chain:

**Intervention request → change in simulated world/system → changed input or physical response → navigation/localization/control consequence → observable mission outcome.**

MVP acceptance scenarios:

1. **Baseline for each robot:** real navigation to a goal under its own motion constraints; comparable truth and estimate signals where supported.
2. **Obstacle during motion:** solid obstacle introduced after the mission starts; collision behavior and actual planner/controller response are inspectable.
3. **Sensor/capability fault:** one fault introduced at a real input or physical model boundary; the affected downstream subsystem and outcome are inspectable. No invented recovery.
4. **Distinct mobility:** show a maneuver feasible for a differential-drive robot but infeasible (or requiring different execution) for an Ackermann vehicle.
5. **Unsuccessful mission:** an explicitly blocked, unsafe, or failed case ends with an accurate reported outcome.
6. **Repeatability:** reproduce at least the baseline and one interrupted experiment with known configurations, interventions, and recorded evidence.

Validation should account for simulation nondeterminism. An experiment is not considered “proved” merely because a screenshot looks right.

## 11. Scope discipline

Do not trade away realism to add more robots, dashboard widgets, or nominally supported faults.

Prefer **one honest, end-to-end integration** over ten fake toggles. Integrate the first robot fully, then add the second distinct platform before declaring the MVP complete. Build additional sensors or failure modes only when they can be exercised and observed through the actual robotics stack.

Exact architecture, libraries, messaging topology, algorithms, and deployment choices are **not specified by this document**. Technical decisions should be evaluated for fidelity, inspectability, compatibility, and maintainability—not fashion or buzzwords.

## 12. Non-negotiable review questions

Before calling a feature complete, answer:

- Does it change **actual robot behavior or input**, or just the UI?
- Is this feature valid for the selected robot and environment?
- Are navigation, localization, and control using real modeled signals rather than hidden ground-truth shortcuts?
- Can we trace the effect using direct evidence, with correct units, time, and source labels?
- Does the claimed recovery really happen, or are we scripting an impression of it?
- Can failure remain visible, and can an experiment legitimately end unsuccessfully?
- Does this deliver real robotics engineering value?

**If the answer is unknown, mark the feature experimental or incomplete. If the answer is no, do not present it as implemented.**
