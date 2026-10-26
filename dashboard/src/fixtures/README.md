# Mock fixtures

These three files are **recordings, not inventions**. Each was captured verbatim from a
running gateway on 2026-10-26 with:

```
curl -s localhost:8771/api/reference      > reference.json
curl -s localhost:8771/api/runs           > (filtered to 16 entries) runs.json
curl -s localhost:8771/api/runs/<id>      > (4 entries) runDetails.json
```

They exist so `VITE_RAL_SOURCE=mock npm run dev` can exercise the console's layout and
interactions with no container, no gz and no stack. Two deliberate edits were made to the
captured run details:

- `has_plot` is forced to `false`, because `plot.png` is served from `runs/` by the
  gateway and the mock has no gateway to serve it;
- `log_tail` and `bag_record_log_tail` are truncated to their last 800 characters, to keep
  the bundle small.

Nothing else was changed. Every metric, reason, provenance hash and manager string in here
is the one a real run produced.

**Mock mode is labelled in the UI.** The top bar reads `MOCK` instead of `LIVE`, and the
replay controls — which only a scripted fixture can honour — appear only in that mode. A
reader can always tell which they are looking at.

The fixtures go stale the moment the catalogue, a robot profile, the map or the scenario
set changes. They are not a contract and no test asserts against them: if the mock looks
wrong, re-capture it with the commands above.
