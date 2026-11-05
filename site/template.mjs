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
<link rel="icon" href="data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 16 16'%3E%3Crect width='16' height='16' rx='3' fill='%239b8cff'/%3E%3Ccircle cx='8' cy='8' r='3.4' fill='none' stroke='%230f1115' stroke-width='1.7'/%3E%3Ccircle cx='8' cy='8' r='1.1' fill='%230f1115'/%3E%3C/svg%3E">
<link rel="stylesheet" href="${up}assets/style.css">
<!-- The theme, before first paint. A stored choice must be applied in <head>
     or the page paints in the other theme and flashes. Three states: with no
     stored value the attribute is left off and the stylesheet follows the
     operating system through prefers-color-scheme, so this runs only for a
     reader who has chosen. Eleven lines, no dependency; apart from this and
     the toggle's handler the site still ships no JavaScript outside console/. -->
<script>
(function () {
  try {
    var t = localStorage.getItem('ral-theme');
    if (t === 'light' || t === 'dark') document.documentElement.dataset.theme = t;
  } catch (e) { /* storage blocked: fall through to the OS preference */ }
})();
</script>
</head>
<body>
<header class="topbar">
  <a class="brand" href="${up}index.html">
    <svg class="mark" width="18" height="18" viewBox="0 0 16 16" aria-hidden="true">
      <circle cx="8" cy="8" r="6.4" fill="none" stroke="currentColor" stroke-width="1.3" opacity="0.45"/>
      <circle cx="8" cy="8" r="3.1" fill="none" stroke="currentColor" stroke-width="1.5"/>
      <circle cx="8" cy="8" r="1.1" fill="currentColor"/>
    </svg>
    <span>Resilient Autonomy Lab</span>
  </a>
  <nav aria-label="Primary">
${links}
  </nav>
  <button class="theme-toggle" type="button" id="theme-toggle"
          title="Switch between the light and dark theme"
          aria-label="Switch between the light and dark theme">
    <svg class="icon-moon" width="16" height="16" viewBox="0 0 24 24" fill="none"
         stroke="currentColor" stroke-width="1.8" stroke-linecap="round"
         stroke-linejoin="round" aria-hidden="true">
      <path d="M21 12.8A9 9 0 1 1 11.2 3a7 7 0 0 0 9.8 9.8z"/>
    </svg>
    <svg class="icon-sun" width="16" height="16" viewBox="0 0 24 24" fill="none"
         stroke="currentColor" stroke-width="1.8" stroke-linecap="round"
         stroke-linejoin="round" aria-hidden="true">
      <circle cx="12" cy="12" r="4.2"/>
      <path d="M12 2.5v2M12 19.5v2M2.5 12h2M19.5 12h2M5.2 5.2l1.4 1.4M17.4 17.4l1.4 1.4M18.8 5.2l-1.4 1.4M6.6 17.4l-1.4 1.4"/>
    </svg>
  </button>
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

<!-- The toggle's handler. It writes the RESOLVED theme, never "system": a
     reader who presses this has stated a preference, and the stylesheet's
     prefers-color-scheme default stays in force only until they do. -->
<script>
(function () {
  var root = document.documentElement;
  var button = document.getElementById('theme-toggle');
  if (!button) return;
  var media = window.matchMedia('(prefers-color-scheme: light)');
  button.addEventListener('click', function () {
    var current = root.dataset.theme || (media.matches ? 'light' : 'dark');
    var next = current === 'light' ? 'dark' : 'light';
    root.dataset.theme = next;
    try { localStorage.setItem('ral-theme', next); } catch (e) { /* storage blocked */ }
  });
})();
</script>
</body>
</html>
`;
}
