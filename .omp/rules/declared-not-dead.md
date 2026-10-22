---
name: declared-not-dead
description: Never delete code, config or dependencies whose comment reserves them for a later phase — a declaration is a decision, not a leftover
condition: (?i)(dead[- ]code|unused|cleanup|clean up|simplif|refactor|prune|tidy|trim|remove unused|not needed|unnecessary)
scope: always
---

# A comment naming a later phase is a decision, not a leftover

You are about to decide that something is unused. Before deleting it, read what the code
says about itself.

If a comment, docstring, manifest entry or plan reserves it — "unused in Phase 1, present
for Phase 4", "needed when X lands", "kept so the SDF is self-describing" — then it is
**declared**, and a dead-code sweep does not get to overturn a declaration. Removing it is
a scope change that needs the author's agreement, not a judgement call you make while
tidying.

This happened in this repo: a simplify sweep deleted `gz-sim-user-commands-system` and
`gz-sim-apply-link-wrench-system` from `worlds/ral_apartment.sdf`, whose comment said in
plain words that Phase 4's obstacle spawn/remove and external-push faults need them. The
same sweep deleted an inert `<topic>` that its comment said was kept deliberately. Both
had to be restored.

## The distinction

| Class | Example | Action |
| --- | --- | --- |
| **Dead** — nothing declares it, nothing references it | A function parameter no caller passes; a duplicated `except` branch; a cache directory | Delete, and say what proved it unreachable |
| **Declared** — a comment, manifest or plan reserves it | `<plugin>` kept for a named later phase; an inert element the comment calls deliberate; a dependency documented as the runtime's rosdep key | Keep. If you believe it is wrong, say so and ask — do not delete |
| **Undeclared but load-bearing** | An empty `resource/<pkg>` marker; `SceneBroadcaster`, which serves `/world/<w>/stats` | Keep. Find the consumer before concluding there is none |

## Before deleting anything in a sweep

1. Read the comment attached to it and the comments that mention it by name. `grep` the
   repo for the symbol, file or topic: a cross-file contract is usually written down
   somewhere other than the line you are deleting.
   In the case above, `config/bridge_roomba_ral_apartment.yaml` states that Phase 4 will
   bridge `/world/<world>/create`, `/remove` and `/set_pose` — the three services
   `UserCommands` serves. Deleting the plugin would have made a tracked comment false.
2. State the evidence that it is unreferenced, not the impression. "No caller passes this
   parameter" is evidence; "this looks speculative" is not.
3. If the thing is declared and you still think it should go, raise it as a finding with
   the tradeoff and let the author decide. A sweep that removes declarations is no longer
   a sweep.

Deleting code nobody declared is cleanup. Deleting code someone declared is a decision
you were not asked to make.
