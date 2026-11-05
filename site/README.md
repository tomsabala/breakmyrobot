# The public static site

`site/dist/` is the published face of this repository: the project overview, the
architecture write-up, the measured evidence, and a **replay** of the operator
console. It is a directory of plain files — there is no server, no API and no
simulator behind it.

Nothing on the site is written twice. Every page is rendered from the
repository's own markdown (`README.md`, `docs/*.md`) at build time, so a claim
can only change on the site by changing in the repo. The single piece of
hand-written body copy is the hero in `site/content/hero.html`, and even its
title and one-sentence description are substituted from `README.md`'s own first
heading and first paragraph.

## Build

```bash
scripts/build_site.sh          # -> site/dist/
```

Runs on the **host**, not in the `ral:dev` container: it needs Node ≥ 20, npm
and a Chrome/Chromium binary, not ROS. It never starts a stack, a gateway or a
simulator. Re-running it rebuilds everything.

What it does, in order:

1. `VITE_RAL_SOURCE=replay npm run build` in `dashboard/`, then copies
   `dashboard/dist` to `site/dist/console/` and makes sure the replay fixtures
   from `dashboard/public/replay/` came along. `dashboard/vite.config.ts` sets
   `base: './'` so the console's assets are relative — Vite's default writes
   `/assets/…`, which 404s under any path prefix.
2. `node site/build.mjs`: copies `site/assets/style.css` and `docs/media/*`,
   pre-renders the mermaid diagrams, and renders the markdown into
   `index.html`, `architecture.html` and `evidence/*.html`, rewriting
   `docs/foo.md` → `evidence/foo.html` and `docs/media/x.png` → `media/x.png`.
   Repository files the site does not publish (`ENGINEERING.md`, `scripts/…`,
   `ros2_ws/…`) become links to the file on GitHub rather than dead links.

It fails loudly instead of publishing something misleading: a missing source
document, a missing replay fixture, a dashboard bundle with root-absolute asset
URLs, and a mermaid diagram it cannot render are all errors or loud warnings,
never silent omissions.

| Variable | Effect |
| --- | --- |
| `RAL_SITE_CONSOLE=skip` | Do not build `dashboard/`; write a placeholder `console/index.html` that says the console is not in this build. |
| `RAL_SITE_ALLOW_NO_REPLAY=1` | Build the documents even though no replay fixture exists. The console then has nothing to play. |
| `RAL_SITE_MERMAID=source` | Publish mermaid blocks as labelled source text instead of SVG, for a build host with no Chrome. |
| `RAL_SITE_CHROME=<path>` | The Chrome/Chromium binary used to pre-render the diagrams. |

### Why the diagrams are pre-rendered

`README.md` and `docs/architecture.md` contain mermaid blocks. The generator
renders them to inline SVG in a headless Chrome **at build time**, rather than
vendoring `mermaid.min.js` into `assets/`. That bundle is 3.5 MB of JavaScript
to draw diagrams that cannot change after the build, and without it the diagrams
are missing. Pre-rendering ships **no JavaScript bundle at all** outside
`console/`, and the diagrams are in the HTML — in both palettes, so the theme
toggle swaps between two pre-rendered SVGs rather than re-laying-out a graph in
the reader's browser. The price is a Chrome binary on
the build machine; `RAL_SITE_MERMAID=source` is the escape hatch. (A CDN
`<script src="https://…">` is not an option — this project bundles its assets,
including the console's fonts.)

## Where it is published

**<https://apps.tom-sabala.dev/#breakmyrobot>**, as a container on that
gateway's own terms rather than as files copied into it. The pattern is the
one every other app there follows:

1. `.github/workflows/apps-image.yml` builds `site/dist` on a runner (which
   has the Chrome the diagrams need) and pushes
   `ghcr.io/tomsabala/breakmyrobot:apps-mount-<sha>` — one immutable tag per
   commit, because the gateway's `deploy.sh` refuses a moving one.
2. `docker/app/Dockerfile` puts that build behind Caddy on port 8080, serving
   it at `/a/breakmyrobot/` — the gateway does **not** strip the service
   prefix, so the container has to answer underneath it.
3. The gateway's `frontend/src/apps/apps.json` pins the tag as a
   `kind: "service"`, `mode: "shared"` entry: one warm container for everyone,
   because this site stores nothing per visitor.
4. On the VPS, `gateway/deploy.sh` pulls that tag and the broker starts it on
   the first request.

Measured on the built image, started the way the broker starts one
(`--cap-drop ALL --security-opt no-new-privileges:true --memory 64m
--read-only`): healthy, **12 MiB of RSS**, 56 MB on disk, `/healthz` 200,
every document and the console's replay fixture served under the prefix.

## Deploy it anywhere else

`site/dist/` is static and **position-independent**: every URL it emits is
relative to the document. Both layouts were crawled over HTTP during
development — 17 URLs, all 200, no root-absolute reference — at a bucket root
(`http://host/`) and under a prefix (`http://host/breakmyrobot/`). The document
pages also open straight off disk over `file://`; the console does not, because
a browser blocks its fixture fetch there, so give it an HTTP origin.

```bash
python3 -m http.server -d site/dist 8080      # local check
```

- **GitHub Pages** — publish `site/dist` (project pages mount at
  `/<repo>/`; relative URLs handle that).
- **Vercel / Netlify** — build command `scripts/build_site.sh`, output
  directory `site/dist`. The build image needs Node and a Chrome binary, or set
  `RAL_SITE_MERMAID=source`.
- **nginx / S3 / any object store** — copy the tree. No rewrites, no redirects,
  no server-side logic. Nothing requires directory-index behaviour either: every
  link names an `index.html` explicitly.

## What is in the bundle, and what is not

In:

- `index.html`, `architecture.html`, `evidence/*.html` — rendered markdown.
- `assets/style.css` — one hand-written stylesheet, two themes (dark, light,
  `prefers-color-scheme` by default). No webfont, no framework. System font
  stack. The only JavaScript on a document page is two inline blocks totalling
  ~25 lines: one in `<head>` that applies a stored theme before first paint, one
  at the end that handles the toggle and stores the choice in `localStorage`.
- `media/*.png` — the plots and screenshots from `docs/media/`.
- `console/` — the operator dashboard's own Vite build (with its bundled IBM
  Plex fonts), compiled with `VITE_RAL_SOURCE=replay`, plus
  `console/replay/*.json`: recordings exported from real runs' rosbags.

Not in, and deliberately so:

- **No backend.** The console's gateway (`ral_eval/dashboard_api.py`), ROS 2,
  Gazebo and ArduPilot SITL exist only in the container. On this site the
  console reads a recorded fixture off disk; no stack can be started, no goal
  sent, no fault injected.
- **No run artifacts.** `runs/` and `logs/` are not committed and not published.
  The exported replay fixtures are the only run data on the site.
- **No CDN, no analytics, no third-party request of any kind.** A visitor
  downloads only files from this origin.
- **No `site/dist/` in git.** It is generated; `site/build.mjs`,
  `site/template.mjs`, `site/content/` and `site/assets/` are the tracked
  sources.

The dependencies in `site/package.json` (`marked`, `mermaid`, `puppeteer-core`)
are build-time only — none of them is served to a visitor.
