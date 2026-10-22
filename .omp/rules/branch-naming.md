---
name: branch-naming
description: Name branches for the change they carry, never for a plan's phase/sprint/step numbering
condition: (?i)git\s+(checkout\s+-b|switch\s+-c|branch\s+(-m\s+\S+)?)\s+["']?(phase|sprint|step|stage|milestone|iteration|part|week|day|task)[-_/]?\d
scope: tool:bash
---

# Branch names describe the change, not its position in a plan

You are about to create or rename a branch whose name starts with a plan coordinate —
`phase0/`, `sprint-3/`, `step2-`, `milestone1/`. Drop the coordinate and name the work.

This repo organises work as Phases 0–6 in `RESILIENT_AUTONOMY_LAB_MVP_PLAN.md`, which
makes `phaseN/` the obvious and wrong branch prefix.

The numbering is the weakest part of the name:

- It is meaningful only while the plan is open. Six months later `phase0/` tells a reader
  nothing about what is inside; `toolchain-and-gate` still does.
- It is unstable. Plans get re-cut, phases merge and split, and the branch keeps a number
  that no longer matches the document it referenced.
- It leaks past the branch. `git merge` writes the branch name into the merge commit
  subject, so renaming the ref afterwards does not fix the history — only a rewrite does,
  and that is not worth a force-push.

Name it after the change, in the shortest form a stranger would understand:

| Instead of | Use |
| --- | --- |
| `phase0/toolchain-and-gate` | `toolchain-and-gate` |
| `sprint2/user-auth` | `user-auth` or `auth/oidc-login` |
| `step3-fix-flaky-tests` | `fix-flaky-scheduler-tests` |

Topic prefixes that describe *kind* rather than *schedule* are fine and often useful:
`fix/`, `feat/`, `docs/`, `spike/`, or a tracker id (`RAL-42-kidnap-fault`) when the repo
already uses one. Match whatever `git branch -a` and the existing history show; a repo's
own convention beats this table.

The plan phase still belongs in the work — in the commit body, the PR description, or the
doc the branch updates. Just not in the ref.
