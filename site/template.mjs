// The page shell every generated page shares: <head>, the header nav, and the
// footer. Kept apart from build.mjs so the markup can be read without the
// rendering machinery around it.
//
// Every URL this file emits is RELATIVE and prefixed with `up` (the number of
// directories the page sits below site/dist/). A root-absolute `/assets/...`
// would 404 on GitHub Pages project sites, which mount at `/<repo>/`.

export const REPO_URL = 'https://github.com/tomsabala/breakmyrobot';

/** The five entries of the header nav, in order. `href` is relative to site/dist/. */
const NAV = [
  ['overview', 'Overview', 'index.html'],
  ['architecture', 'Architecture', 'architecture.html'],
  ['evidence', 'Evidence', 'evidence/index.html'],
  ['console', 'Replay console', 'console/index.html'],
  ['github', 'GitHub', REPO_URL],
];

export function escapeHtml(s) {
  return String(s)
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;');
}

/**
 * @param {object} o
 * @param {string} o.title      <title> and the og:title
 * @param {string} o.nav        which NAV id is current
 * @param {number} o.depth      directories below site/dist/ (0 for index.html)
 * @param {string} o.body       rendered HTML
 * @param {string} o.provenance one line naming the commit this was built from
 * @param {string} [o.description] meta description
 */
export function page({ title, nav, depth, body, provenance, description }) {
  const up = '../'.repeat(depth);
  const links = NAV.map(([id, label, href]) => {
    const url = href.startsWith('http') ? href : up + href;
    const ext = href.startsWith('http')
      ? ' rel="noreferrer noopener"'
      : '';
    const current = id === nav ? ' aria-current="page"' : '';
    return `      <a href="${escapeHtml(url)}"${current}${ext}>${escapeHtml(label)}</a>`;
  }).join('\n');

  return `<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>${escapeHtml(title)}</title>
${description ? `<meta name="description" content="${escapeHtml(description)}">\n` : ''}\
<!-- Inline, because a browser otherwise asks for a ROOT-absolute /favicon.ico,
     which 404s under a path prefix. A data URI needs no file and no request. -->
<link rel="icon" href="data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 16 16'%3E%3Crect width='16' height='16' rx='3' fill='%231c1c1c'/%3E%3Ccircle cx='8' cy='8' r='3.2' fill='none' stroke='%238fd0ff' stroke-width='1.6'/%3E%3Ccircle cx='8' cy='8' r='1' fill='%23FF0000'/%3E%3C/svg%3E">
<link rel="stylesheet" href="${up}assets/style.css">
</head>
<body>
<header class="topbar">
  <a class="brand" href="${up}index.html">Resilient Autonomy Lab</a>
  <nav aria-label="Primary">
${links}
  </nav>
</header>
<main>
${body}
</main>
<footer>
  <p><strong>The console on this site is a replay, not a live system.</strong>
  It is the operator dashboard's own build serving a recording exported from one
  real run's rosbag: static files, no gateway, no ROS 2, no simulator, no
  Gazebo. Nothing on this site can be commanded, and no fault can be injected —
  the runs already happened. To drive a stack live, clone the repository and run
  it; everything that produced the recordings is committed.</p>
  <p class="meta">${escapeHtml(provenance)} &middot;
  <a href="${REPO_URL}" rel="noreferrer noopener">${escapeHtml(REPO_URL.replace('https://', ''))}</a></p>
</footer>
</body>
</html>
`;
}
