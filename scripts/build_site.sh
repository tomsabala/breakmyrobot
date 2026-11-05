#!/usr/bin/env bash
# Build the public static site into site/dist/.
#
#   scripts/build_site.sh
#
# The result is a directory of plain files with NO server behind it: HTML, one
# stylesheet, three PNGs, the dashboard bundle and the replay fixtures it plays.
# Serve it from anything — `python3 -m http.server`, GitHub Pages, a bucket,
# nginx — at the root or under any prefix. Every URL it emits is relative to the
# document, which is also why dashboard/vite.config.ts sets `base: './'`: Vite's
# default writes root-absolute `/assets/...` into the console's index.html, and
# that 404s under a prefix such as GitHub Pages' /<repo>/.
#
# This runs on the HOST, not in the ral:dev container: it needs Node and npm,
# not ROS. Nothing here starts a stack, a gateway or a simulator.
#
# Re-runnable: it rebuilds the console and re-renders every page each time.
#
# Environment:
#   RAL_SITE_CONSOLE=skip       do not build dashboard/; leave a placeholder
#                               console/index.html that says so. For working on
#                               the documents while someone else edits the
#                               dashboard sources.
#   RAL_SITE_ALLOW_NO_REPLAY=1  proceed when dashboard/public/replay/index.json
#                               is missing (the console then has nothing to
#                               play; it is exported by ral_eval's replay
#                               exporter, not by this script).
#   RAL_SITE_MERMAID=source     publish mermaid blocks as labelled source text
#                               instead of pre-rendered SVG, for a build host
#                               with no Chrome/Chromium binary.
#   RAL_SITE_CHROME=<path>      the Chrome/Chromium binary used to pre-render
#                               the mermaid diagrams at build time.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

die() { echo "build_site: $*" >&2; exit 1; }
step() { echo; echo "==> $*"; }

command -v node >/dev/null 2>&1 || die "node is required and was not found on PATH"
command -v npm  >/dev/null 2>&1 || die "npm is required and was not found on PATH"

NODE_MAJOR="$(node -p 'process.versions.node.split(".")[0]')"
[ "$NODE_MAJOR" -ge 20 ] || die "node >= 20 required, found $(node --version)"

[ -f site/build.mjs ]          || die "site/build.mjs is missing"
[ -f site/assets/style.css ]   || die "site/assets/style.css is missing"
[ -f dashboard/package.json ]  || die "dashboard/package.json is missing"
[ -d docs/media ]              || die "docs/media/ is missing; the pages reference its images"

REPLAY_SRC="dashboard/public/replay"
if [ ! -f "$REPLAY_SRC/index.json" ]; then
  if [ "${RAL_SITE_ALLOW_NO_REPLAY:-0}" = "1" ]; then
    echo "build_site: WARNING: $REPLAY_SRC/index.json absent; the console will have" \
         "no replay to play (RAL_SITE_ALLOW_NO_REPLAY=1)" >&2
  else
    die "$REPLAY_SRC/index.json is missing. The console on this site plays an exported
       recording of a real run; without a fixture it would come up empty, which the
       site must not pretend otherwise. Export one, or set
       RAL_SITE_ALLOW_NO_REPLAY=1 to build the documents anyway."
  fi
fi

# --------------------------------------------------------------- 1. console

step "console"
if [ "${RAL_SITE_CONSOLE:-}" = "skip" ]; then
  echo "RAL_SITE_CONSOLE=skip — writing a placeholder instead of building dashboard/"
  rm -rf site/dist/console
  mkdir -p site/dist/console
  cat > site/dist/console/index.html <<'HTML'
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Replay console — not built</title>
<link rel="stylesheet" href="../assets/style.css">
</head>
<body>
<header class="topbar"><a class="brand" href="../index.html">Resilient Autonomy Lab</a></header>
<main>
<h1>The console is not in this build</h1>
<p>This directory is a placeholder. <code>scripts/build_site.sh</code> was run with
<code>RAL_SITE_CONSOLE=skip</code>, so the operator dashboard was not compiled and no
replay is available here. Nothing is being withheld and nothing is simulated behind
this page — the bundle simply is not present.</p>
<p>Re-run <code>scripts/build_site.sh</code> without that variable to build it.</p>
<p><a href="../index.html">Back to the overview</a></p>
</main>
</body>
</html>
HTML
else
  [ -d dashboard/node_modules ] || {
    echo "dashboard/node_modules absent — npm install"
    (cd dashboard && npm install --no-audit --no-fund)
  }
  echo "building dashboard in replay mode (VITE_RAL_SOURCE=replay)"
  (cd dashboard && VITE_RAL_SOURCE=replay npm run build)
  [ -f dashboard/dist/index.html ] || die "dashboard build produced no dist/index.html"

  # A root-absolute asset URL here would 404 under any prefix; catch it at build
  # time rather than in someone's browser.
  if grep -Eq '(src|href)="/[^/]' dashboard/dist/index.html; then
    die "dashboard/dist/index.html contains root-absolute asset URLs; vite.config.ts
       must set base: './' for the console to serve from site/dist/console/"
  fi

  rm -rf site/dist/console
  mkdir -p site/dist
  cp -R dashboard/dist site/dist/console

  # Vite copies dashboard/public/** into dist, but that is its behaviour, not a
  # contract this script can assume: verify, and copy the fixtures ourselves if
  # they did not come along.
  if [ -f "$REPLAY_SRC/index.json" ] && [ ! -f site/dist/console/replay/index.json ]; then
    echo "vite did not carry the replay fixtures; copying $REPLAY_SRC -> console/replay"
    mkdir -p site/dist/console/replay
    cp -R "$REPLAY_SRC/." site/dist/console/replay/
  fi
fi

# ------------------------------------------------------ 2. render the pages

step "pages"
if [ ! -d site/node_modules ]; then
  echo "site/node_modules absent — npm install"
  (cd site && npm install --no-audit --no-fund)
fi
node site/build.mjs

# ------------------------------------------------------------ 3. report out

step "result"
du -sh site/dist
find site/dist -maxdepth 1 -mindepth 1 | sort
echo
echo "serve it:  python3 -m http.server -d site/dist 8080"
