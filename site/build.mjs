// Static-site generator for the public Resilient Autonomy Lab site.
//
// It renders the repository's OWN markdown — README.md for the landing page and
// docs/*.md for everything else — so the published site cannot drift from the
// repo: there is no second copy of any claim to keep in sync. The only
// hand-written body copy is site/content/hero.html, and even that takes its
// title and one-sentence description from README.md.
//
// Mermaid is PRE-RENDERED to inline SVG here, at build time, by a headless
// Chrome that is already on the build machine. The alternative — vendoring
// mermaid.min.js into assets/ — costs 3.5 MB of JavaScript per visitor to draw
// two static diagrams that never change after the build, and makes the diagrams
// disappear without JS. Pre-rendering ships no JavaScript bundle on the site
// and the diagrams are in the HTML. The cost is a build-time dependency on a
// Chrome binary; `RAL_SITE_MERMAID=source` degrades to a labelled source block
// instead of failing, for a build host without one. Each diagram is rendered
// once per theme — mermaid bakes its colours into the SVG, and the reader
// picks the theme at runtime; see MERMAID_LIGHT/MERMAID_DARK below.
//
// Everything emitted is relative to the document: the site must serve from a
// bucket root, from GitHub Pages' /<repo>/ prefix, or from a file:// checkout.
//
// Usage: node site/build.mjs   (normally via scripts/build_site.sh)

import { execFileSync } from 'node:child_process';
import { existsSync } from 'node:fs';
import { copyFile, mkdir, readFile, readdir, rm, stat, writeFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

import { Marked, Renderer } from 'marked';
import puppeteer from 'puppeteer-core';

import { REPO_URL, escapeHtml, page } from './template.mjs';

const SITE_DIR = path.dirname(fileURLToPath(import.meta.url));
const REPO = path.resolve(SITE_DIR, '..');
const DIST = path.join(SITE_DIR, 'dist');

/** Evidence documents, in the order README.md's evidence index lists them. */
const EVIDENCE = [
  'nav-closed-loop.md',
  'car-closed-loop.md',
  'repeatability-baseline_roomba.md',
  'repeatability-baseline_car.md',
  'gate.md',
  'roomba-teleop-loop.md',
  'engineering-log.md',
];

/**
 * Every rendered page: which repository file it comes from, where it lands
 * under dist/, and which nav entry it belongs to.
 */
const PAGES = [
  { src: 'README.md', out: 'index.html', nav: 'overview', hero: true },
  { src: 'docs/architecture.md', out: 'architecture.html', nav: 'architecture' },
  ...EVIDENCE.map((f) => ({
    src: `docs/${f}`,
    out: `evidence/${f.replace(/\.md$/, '.html')}`,
    nav: 'evidence',
    evidence: true,
  })),
];

/** repo-relative markdown path -> dist-relative html path, for link rewriting. */
const SITE_PATHS = new Map(PAGES.map((p) => [p.src, p.out]));

/** Outputs this generator owns; cleaned on every run. dist/console/ is NOT ours. */
const OWNED = ['index.html', 'architecture.html', 'evidence', 'media', 'assets'];

const warnings = [];
const warn = (msg) => {
  warnings.push(msg);
  console.warn(`  warning: ${msg}`);
};

// ---------------------------------------------------------------- utilities

const esc = (s) => String(s)
  .replace(/&/g, '&amp;')
  .replace(/</g, '&lt;')
  .replace(/>/g, '&gt;')
  .replace(/"/g, '&quot;');

/** HTML inline markup -> plain text, for slugs and <title>. */
const plain = (html) => html
  .replace(/<[^>]*>/g, '')
  .replace(/&quot;/g, '"')
  .replace(/&#39;/g, "'")
  .replace(/&lt;/g, '<')
  .replace(/&gt;/g, '>')
  .replace(/&amp;/g, '&')
  .replace(/\s+/g, ' ')
  .trim();

/** GitHub's heading slug rule, so anchors written for GitHub keep working. */
const slugify = (text) => text
  .toLowerCase()
  .replace(/[^\p{L}\p{N} _-]/gu, '')
  .trim()
  .replace(/\s+/g, '-');

function gitProvenance() {
  const git = (args, fallback) => {
    try {
      return execFileSync('git', args, { cwd: REPO, encoding: 'utf8' }).trim();
    } catch {
      return fallback;
    }
  };
  const sha = git(['rev-parse', '--short', 'HEAD'], null);
  if (!sha) return 'Generated from this repository (no git metadata available)';
  const dirty = git(['status', '--porcelain'], '') !== '';
  const when = new Date().toISOString().slice(0, 10);
  return `Generated ${when} from commit ${sha}${dirty ? ' (working tree dirty)' : ''}`;
}

// ------------------------------------------------------------------ mermaid

const CHROME_CANDIDATES = [
  process.env.RAL_SITE_CHROME,
  process.env.PUPPETEER_EXECUTABLE_PATH,
  process.env.CHROME_PATH,
  '/usr/bin/google-chrome-stable',
  '/usr/bin/google-chrome',
  '/usr/bin/chromium',
  '/usr/bin/chromium-browser',
  '/snap/bin/chromium',
  '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',
];

function findChrome() {
  for (const c of CHROME_CANDIDATES) {
    if (c && existsSync(c)) return c;
  }
  return null;
}

/**
 * Both palettes are rendered, because the reader picks the theme at runtime.
 *
 * Mermaid bakes its colours into the SVG it emits: a `<style>` block scoped to
 * the diagram's own id, plus per-element `fill`/`stroke` attributes. There is
 * no stylesheet hook to re-colour that from outside, so a single SVG driven by
 * the site's CSS custom properties would mean rewriting mermaid's output —
 * mapping its generated selectors and inline attributes onto var(--...) — and
 * that rewrite breaks silently the first time mermaid changes a class name or
 * moves a colour from the style block to an attribute. The other way out,
 * re-rendering in the browser on theme change, puts mermaid's 3.5 MB bundle
 * back on every visitor, which is the whole thing this build avoids.
 *
 * So: render twice at build time and let CSS show one. It costs two SVG bodies
 * in the HTML (~40 KB gzipped across the site) and one extra mermaid layout
 * pass per diagram in a browser session that is already open, and it cannot
 * drift from what mermaid actually produces.
 */

/** Shared between the two palettes: layout, safety and typography are not themed. */
const MERMAID_BASE = {
  startOnLoad: false,
  securityLevel: 'strict',
  flowchart: { htmlLabels: false, curve: 'basis', padding: 10 },
};

const MERMAID_FONT =
  'ui-sans-serif, system-ui, "Segoe UI", Roboto, Helvetica, Arial, sans-serif';

/**
 * Mermaid's dark theme, re-pointed at dashboard/src/theme/tokens.ts's palette.
 *
 * Keep this variable set and MERMAID_LIGHT's identical: mermaid derives every
 * variable it is not given from the ones it is, so a key present in only one
 * palette is a colour that silently falls back to a mermaid default there.
 */
const MERMAID_DARK = {
  ...MERMAID_BASE,
  theme: 'dark',
  themeVariables: {
    darkMode: true,
    background: '#1c1c1c',
    fontFamily: MERMAID_FONT,
    fontSize: '14px',
    primaryColor: '#262626',
    primaryTextColor: '#e6e6e6',
    primaryBorderColor: '#5a5a5a',
    secondaryColor: '#232323',
    tertiaryColor: '#1e1e1e',
    mainBkg: '#262626',
    nodeBorder: '#5a5a5a',
    clusterBkg: '#1e1e1e',
    clusterBorder: '#444444',
    lineColor: '#a8a8a8',
    textColor: '#cfcfcf',
    titleColor: '#e6e6e6',
    edgeLabelBackground: '#1c1c1c',
    nodeTextColor: '#e6e6e6',
  },
};

/**
 * The same diagram for a near-white page. `base` is mermaid's themeable theme:
 * `default` hard-codes its own lavender node fills, which read as decoration
 * next to this site's greys. Fills stay just off the page background so a node
 * is a surface rather than a hole, borders stay dark enough to survive a
 * low-contrast laptop panel, and edges are darker than the borders so an arrow
 * is never the faintest thing in the picture.
 */
const MERMAID_LIGHT = {
  ...MERMAID_BASE,
  theme: 'base',
  themeVariables: {
    darkMode: false,
    background: '#ffffff',
    fontFamily: MERMAID_FONT,
    fontSize: '14px',
    primaryColor: '#f4f5f7',
    primaryTextColor: '#1b1d22',
    primaryBorderColor: '#c9ccd3',
    secondaryColor: '#e9ebef',
    tertiaryColor: '#eef0f3',
    mainBkg: '#f4f5f7',
    nodeBorder: '#c9ccd3',
    clusterBkg: '#eceef3',
    clusterBorder: '#cdd1d9',
    lineColor: '#6b7079',
    textColor: '#1b1d22',
    titleColor: '#1b1d22',
    edgeLabelBackground: '#ffffff',
    nodeTextColor: '#1b1d22',
  },
};

/**
 * Render every mermaid source twice — light and dark — in one browser session.
 * @param {string[]} sources
 * @returns {Promise<Map<string,string>>} source -> <figure> (or a labelled fallback)
 */
async function renderMermaid(sources) {
  const out = new Map();
  if (sources.length === 0) return out;

  const mode = process.env.RAL_SITE_MERMAID ?? 'prerender';
  if (mode === 'source') {
    warn(`RAL_SITE_MERMAID=source: ${sources.length} diagram(s) shipped as source text, not rendered`);
    for (const s of sources) out.set(s, mermaidFallback(s, 'not rendered (RAL_SITE_MERMAID=source)'));
    return out;
  }

  const exe = findChrome();
  if (!exe) {
    throw new Error(
      `mermaid pre-rendering needs a Chrome or Chromium binary and none was found.\n` +
      `  Looked at: ${CHROME_CANDIDATES.filter(Boolean).join(', ')}\n` +
      `  Point RAL_SITE_CHROME at one, or set RAL_SITE_MERMAID=source to publish the\n` +
      `  ${sources.length} diagram(s) as labelled source text instead of silently dropping them.`,
    );
  }

  const bundle = path.join(SITE_DIR, 'node_modules', 'mermaid', 'dist', 'mermaid.min.js');
  if (!existsSync(bundle)) {
    throw new Error(`missing ${path.relative(REPO, bundle)} — run \`npm install\` in site/`);
  }

  console.log(`  mermaid: pre-rendering ${sources.length} diagram(s) × 2 themes with ${exe}`);
  const browser = await puppeteer.launch({
    executablePath: exe,
    headless: true,
    args: ['--no-sandbox', '--disable-dev-shm-usage', '--disable-gpu'],
  });
  try {
    const tab = await browser.newPage();
    await tab.setContent('<!doctype html><html><body></body></html>');
    await tab.addScriptTag({ path: bundle });

    for (const [i, src] of sources.entries()) {
      const result = await tab.evaluate(
        async (text, jobs) => {
          const svgs = {};
          for (const job of jobs) {
            try {
              // initialize() rebuilds mermaid's site config from its defaults
              // every call, so switching palette mid-session does not leave the
              // previous one's variables behind.
              window.mermaid.initialize(job.config);
              const { svg } = await window.mermaid.render(job.id, text);
              svgs[job.theme] = svg;
            } catch (e) {
              return { error: `${job.theme}: ${String((e && e.message) || e)}` };
            }
          }
          return { svgs };
        },
        src,
        // Distinct ids per theme: mermaid scopes its <style> block and its
        // marker/node ids to the id it is handed, and both copies land in the
        // same document. Share the id and the first copy's style block restyles
        // the second one — which is exactly the theme bleed this is avoiding.
        [
          { theme: 'light', id: `ral-diagram-${i}-light`, config: MERMAID_LIGHT },
          { theme: 'dark', id: `ral-diagram-${i}-dark`, config: MERMAID_DARK },
        ],
      );
      if (result.error) {
        // A diagram the repo's own markdown cannot render is a repo bug, not a
        // site bug: say so and keep the source visible rather than dropping it.
        // One <pre>, not one per theme: the source text has no palette.
        warn(`mermaid diagram ${i} failed to render: ${result.error}`);
        out.set(src, mermaidFallback(src, `failed to render: ${result.error}`));
      } else {
        checkDiagramIsolation(result.svgs, i);
        out.set(src, diagramFigure(result.svgs.light, result.svgs.dark));
      }
    }
  } finally {
    await browser.close();
  }
  return out;
}

/** Add a class to an SVG's root element, keeping whatever mermaid put there. */
function addSvgClass(svg, cls) {
  const end = svg.indexOf('>');
  const open = svg.slice(0, end);
  if (/\sclass="/.test(open)) {
    return open.replace(/\sclass="([^"]*)"/, ` class="$1 ${cls}"`) + svg.slice(end);
  }
  return `<svg class="${cls}"${svg.slice(4)}`;
}

/**
 * Nothing here renames mermaid's ids, because mermaid already scopes all of
 * them: every `id=` it emits — the root `<svg>`, the arrowhead `<marker>`s,
 * the drop-shadow filters, the gradient, each node, each edge path — starts
 * with the id handed to `mermaid.render()`, every `url(#…)` reference points
 * inside the same prefix, and every selector in its `<style>` block is
 * `#<that-id> …`. Giving the two copies different render ids is therefore the
 * whole isolation story: the light copy's style block cannot select into the
 * dark one, and the dark copy's arrows cannot resolve to the light copy's
 * markers through document-order id lookup.
 *
 * That is a property of mermaid's output, not a promise in its API, so the
 * build checks it on every run instead of trusting it. The failure it guards
 * against is silent — a diagram that quietly wears the other theme's colours —
 * and a mermaid upgrade is exactly when it would appear.
 *
 * @param {{light: string, dark: string}} svgs the two raw renders
 * @param {number} i the diagram's index, for the warning text
 */
function checkDiagramIsolation(svgs, i) {
  const ids = {};
  for (const [theme, svg] of Object.entries(svgs)) {
    const root = /^<svg\b[^>]*?\sid="([^"]+)"/.exec(svg)?.[1];
    if (!root) {
      warn(`mermaid diagram ${i} (${theme}): the rendered <svg> carries no id to scope it`);
      continue;
    }
    ids[theme] = new Set([...svg.matchAll(/(?<![-\w])id="([^"]+)"/g)].map((m) => m[1]));
    const loose = [...ids[theme]].filter((id) => !id.startsWith(root));
    if (loose.length) {
      warn(`mermaid diagram ${i} (${theme}): ${loose.length} id(s) outside #${root}, ` +
        `so the two themes can collide: ${loose.slice(0, 3).join(', ')}`);
    }
    // Selectors, split crudely on rule boundaries; at-rules (@keyframes and
    // the `from`/`to`/`50%` inside them) are global by construction and carry
    // no colour, so they are not a bleed path.
    const style = /<style>([\s\S]*?)<\/style>/.exec(svg)?.[1] ?? '';
    const unscoped = style.split('}')
      .map((rule) => rule.split('{')[0])
      .flatMap((sel) => sel.split(','))
      .map((sel) => sel.trim())
      .filter((sel) => sel && !sel.startsWith(`#${root}`) && !sel.startsWith('@')
        && !/^(from|to|[\d.]+%)$/.test(sel));
    if (unscoped.length) {
      warn(`mermaid diagram ${i} (${theme}): ${unscoped.length} style rule(s) not scoped to ` +
        `#${root}, so one theme can restyle the other: ${unscoped.slice(0, 3).join(', ')}`);
    }
  }
  const shared = ids.light && ids.dark ? [...ids.light].filter((id) => ids.dark.has(id)) : [];
  if (shared.length) {
    warn(`mermaid diagram ${i}: light and dark share ${shared.length} id(s) in one document: ` +
      `${shared.slice(0, 3).join(', ')}`);
  }
}

/**
 * Mermaid emits `width="100%"` plus an inline `max-width: <natural>px`, which
 * makes a 20-node graph shrink to whatever column it lands in — at this site's
 * 75ch measure the system map came out at 681 px wide and its labels were
 * unreadable. Pin the SVG to the natural width its own layout produced and let
 * the figure scroll instead; shrinking a diagram is not a way to show it.
 * Past the width the stylesheet can give the figure, say that it scrolls,
 * because a horizontal scrollbar inside a page is easy to miss.
 *
 * Both copies get the same treatment at their own measured width: the two
 * renders differ only in colour, so the widths agree, but pinning each to what
 * its own layout produced means a future palette that changes metrics (a
 * different font, say) still gets a correctly sized diagram rather than a
 * scaled one. The scroll hint uses the wider of the two.
 */
function diagramFigure(lightSvg, darkSvg) {
  const prepare = (svg, cls) => {
    const vb = /viewBox="0 0 ([\d.]+) ([\d.]+)"/.exec(svg);
    const width = vb ? Math.ceil(parseFloat(vb[1])) : 0;
    const sized = width
      ? svg.replace(/\bwidth="100%"/, `width="${width}"`)
        .replace(/\bstyle="max-width:[^"]*"/, 'style="max-width:none"')
      : svg;
    return { svg: addSvgClass(sized, cls), width };
  };
  const light = prepare(lightSvg, 'diagram-svg diagram-light');
  const dark = prepare(darkSvg, 'diagram-svg diagram-dark');
  const width = Math.max(light.width, dark.width);
  // 1500 px is the stylesheet's cap on .diagram; below it nothing scrolls on a
  // desktop, and the phone breakpoint scrolls everything anyway.
  const hint = width > 1400
    ? `<figcaption>${width} px wide — scroll the diagram sideways to read it.</figcaption>\n`
    : '';
  // Light first, dark second; the stylesheet shows exactly one of the pair.
  return `<figure class="diagram" role="img">\n${light.svg}\n${dark.svg}\n${hint}</figure>\n`;
}

function mermaidFallback(src, reason) {
  return `<figure class="diagram diagram-fallback">\n` +
    `<figcaption>Diagram source — ${esc(reason)}</figcaption>\n` +
    `<pre><code class="language-mermaid">${esc(src)}</code></pre>\n</figure>\n`;
}

// ------------------------------------------------------------------ marked

/**
 * A marked instance bound to one page: it knows where its source lives (to
 * resolve relative links) and where its output lands (to make them relative
 * again), and it substitutes pre-rendered mermaid SVG for mermaid code blocks.
 */
function markedFor(srcRel, outRel, diagrams) {
  const up = '../'.repeat(outRel.split('/').length - 1);
  const srcDir = path.posix.dirname(srcRel);
  const seen = new Map();

  /** Repo-relative markdown link -> a URL that works from this page. */
  const rewrite = (href) => {
    if (!href) return href;
    if (/^[a-z][a-z0-9+.\-]*:/i.test(href) || href.startsWith('//')) return href; // external
    if (href.startsWith('#')) return href;                                        // same page

    const hashAt = href.indexOf('#');
    const hash = hashAt === -1 ? '' : href.slice(hashAt);
    const raw = hashAt === -1 ? href : href.slice(0, hashAt);
    if (!raw) return href;

    const target = path.posix.normalize(
      raw.startsWith('/') ? raw.slice(1) : path.posix.join(srcDir, raw),
    );

    if (SITE_PATHS.has(target)) return up + SITE_PATHS.get(target) + hash;
    if (target.startsWith('docs/media/')) return `${up}media/${path.posix.basename(target)}${hash}`;
    if (existsSync(path.join(REPO, target))) {
      // Tracked but not published here (ENGINEERING.md, scripts/, ros2_ws/):
      // send the reader to the file itself rather than to a dead link.
      return `${REPO_URL}/blob/main/${target}${hash}`;
    }
    warn(`${srcRel}: link target "${href}" does not exist in the repository; left as written`);
    return href;
  };

  const renderer = new Renderer();

  renderer.heading = function heading(token) {
    const html = this.parser.parseInline(token.tokens);
    const base = slugify(plain(html));
    const n = (seen.get(base) ?? 0) + 1;
    seen.set(base, n);
    const id = n === 1 ? base : `${base}-${n - 1}`;
    return `<h${token.depth} id="${esc(id)}">${html}</h${token.depth}>\n`;
  };

  renderer.code = function code(token) {
    if ((token.lang || '').trim().toLowerCase() === 'mermaid') {
      return diagrams.get(token.text) ?? mermaidFallback(token.text, 'not rendered');
    }
    const cls = token.lang ? ` class="language-${esc(token.lang.split(/\s+/)[0])}"` : '';
    return `<pre><code${cls}>${esc(token.text)}\n</code></pre>\n`;
  };

  renderer.link = function link(token) {
    const href = rewrite(token.href);
    const ext = /^https?:/i.test(href) ? ' rel="noreferrer noopener"' : '';
    const title = token.title ? ` title="${esc(token.title)}"` : '';
    return `<a href="${esc(href)}"${title}${ext}>${this.parser.parseInline(token.tokens)}</a>`;
  };

  renderer.image = function image(token) {
    const href = rewrite(token.href);
    const title = token.title ? ` title="${esc(token.title)}"` : '';
    // The README's images carry long descriptive alt text that doubles as the
    // caption; render it as one so the description is visible, not only to
    // screen readers.
    return `<figure class="shot"><img src="${esc(href)}" alt="${esc(token.text)}"${title}` +
      ` loading="lazy">\n<figcaption>${esc(token.text)}</figcaption></figure>`;
  };

  renderer.table = function table(token) {
    // Wide measurement tables must scroll instead of overflowing a phone.
    return `<div class="table-wrap">\n${Renderer.prototype.table.call(this, token)}</div>\n`;
  };

  return new Marked({ renderer, gfm: true });
}

// ------------------------------------------------------------------- build

async function copyDir(from, to) {
  await mkdir(to, { recursive: true });
  let n = 0;
  for (const entry of await readdir(from, { withFileTypes: true })) {
    const src = path.join(from, entry.name);
    const dst = path.join(to, entry.name);
    if (entry.isDirectory()) n += await copyDir(src, dst);
    else { await copyFile(src, dst); n += 1; }
  }
  return n;
}

async function dirSize(dir) {
  let bytes = 0;
  for (const entry of await readdir(dir, { withFileTypes: true })) {
    const p = path.join(dir, entry.name);
    if (entry.isDirectory()) bytes += await dirSize(p);
    else bytes += (await stat(p)).size;
  }
  return bytes;
}

async function main() {
  // Prerequisites, all of them, before anything is written or removed.
  const missing = PAGES.map((p) => p.src).filter((s) => !existsSync(path.join(REPO, s)));
  if (missing.length) {
    throw new Error(
      `missing source document(s): ${missing.join(', ')}\n` +
      `  Every page of the site is rendered from the repository's own markdown; there is\n` +
      `  no placeholder copy to fall back on.`,
    );
  }
  for (const f of ['content/hero.html', 'assets/style.css']) {
    if (!existsSync(path.join(SITE_DIR, f))) throw new Error(`missing site/${f}`);
  }

  const provenance = gitProvenance();

  // 1. Lex every page once, collecting the mermaid sources.
  const lexer = new Marked({ gfm: true });
  const docs = [];
  const mermaidSources = new Set();
  for (const p of PAGES) {
    const md = await readFile(path.join(REPO, p.src), 'utf8');
    const tokens = lexer.lexer(md);
    for (const t of tokens) {
      if (t.type === 'code' && (t.lang || '').trim().toLowerCase() === 'mermaid') {
        mermaidSources.add(t.text);
      }
    }
    docs.push({ ...p, tokens });
  }

  // 2. Render the diagrams (one browser for all of them).
  const diagrams = await renderMermaid([...mermaidSources]);

  // 3. Clean only what this generator owns — dist/console/ belongs to the
  //    dashboard build and survives a re-render.
  for (const name of OWNED) await rm(path.join(DIST, name), { recursive: true, force: true });
  await mkdir(DIST, { recursive: true });

  // 4. Static files.
  await mkdir(path.join(DIST, 'assets'), { recursive: true });
  await copyFile(path.join(SITE_DIR, 'assets', 'style.css'), path.join(DIST, 'assets', 'style.css'));
  const mediaSrc = path.join(REPO, 'docs', 'media');
  const images = existsSync(mediaSrc) ? await copyDir(mediaSrc, path.join(DIST, 'media')) : 0;
  console.log(`  media: ${images} file(s) from docs/media/`);

  // 5. Pages.
  // The template's leading comment explains the placeholders to whoever edits
  // it; it is not for the visitor, and it would otherwise be substituted into.
  const heroTemplate = (await readFile(path.join(SITE_DIR, 'content', 'hero.html'), 'utf8'))
    .replace(/<!--[\s\S]*?-->\s*/g, '');
  const evidenceIndex = [];

  for (const doc of docs) {
    const md = markedFor(doc.src, doc.out, diagrams);
    let tokens = doc.tokens;

    // The H1 is the page title everywhere.
    const h1 = tokens.find((t) => t.type === 'heading' && t.depth === 1);
    const title = h1 ? plain(md.parseInline(h1.text)) : doc.src;

    let hero = '';
    let taglineText = '';
    if (doc.hero) {
      // The hero's sentence IS the README's first paragraph, so it cannot drift.
      const tagline = tokens.find((t) => t.type === 'paragraph');
      taglineText = tagline ? plain(md.parseInline(tagline.text)) : '';
      hero = heroTemplate
        .replace(/\{\{title\}\}/g, esc(title))
        .replace(/\{\{tagline\}\}/g, esc(taglineText))
        .replace(/\{\{up\}\}/g, '')
        .replace(/\{\{repo\}\}/g, REPO_URL);
      tokens = tokens.filter((t) => t !== h1 && t !== tagline);
    }

    const body = hero + md.parser(tokens);

    if (doc.evidence) {
      // The doc's own opening paragraph names the harness that wrote it; that
      // is the only description the index needs, and it is the document's.
      const first = doc.tokens.find((t) => t.type === 'paragraph');
      evidenceIndex.push({
        href: path.posix.basename(doc.out),
        src: doc.src,
        title,
        summary: first ? md.parseInline(first.text) : '',
      });
    }

    const html = page({
      title: doc.out === 'index.html' ? title : `${title} — Resilient Autonomy Lab`,
      nav: doc.nav,
      depth: doc.out.split('/').length - 1,
      body,
      provenance,
      description: taglineText || undefined,
    });
    const outPath = path.join(DIST, doc.out);
    await mkdir(path.dirname(outPath), { recursive: true });
    await writeFile(outPath, html);
    console.log(`  page: ${doc.out}  <- ${doc.src}`);
  }

  // 6. The evidence index, assembled from the documents themselves.
  const list = evidenceIndex.map((e) => `  <li>
    <a class="doc-title" href="${esc(e.href)}">${esc(e.title)}</a>
    <p class="doc-summary">${e.summary}</p>
    <p class="doc-path"><code>${esc(e.src)}</code></p>
  </li>`).join('\n');
  const evidenceBody = `<h1 id="measured-evidence">Measured evidence</h1>
<p>Each document below is written by the harness that produced it: re-running the
harness rewrites the document. The opening line of every one names the script,
the UTC time it ran, and the clock its durations come from. None of it is
summarised or re-worded here.</p>
<p>The artifacts themselves — <code>runs/</code> and <code>logs/</code>, bags
included — are not committed, so the run identifiers name artifacts on the
machine that produced them. Everything that produces them is committed,
<code>scripts/checks/</code> included.</p>
<ul class="doc-list">
${list}
</ul>
`;
  await writeFile(path.join(DIST, 'evidence', 'index.html'), page({
    title: 'Measured evidence — Resilient Autonomy Lab',
    nav: 'evidence',
    depth: 1,
    body: evidenceBody,
    provenance,
    description: 'Every measurement this project publishes, with the harness that produced it.',
  }));
  console.log('  page: evidence/index.html  <- docs/*.md');

  // 7. The console is built and copied by scripts/build_site.sh; say plainly
  //    whether it is there, because the nav links to it either way.
  const consoleIndex = path.join(DIST, 'console', 'index.html');
  if (!existsSync(consoleIndex)) {
    warn('dist/console/index.html is absent — the "Replay console" nav link is dead. ' +
      'Run scripts/build_site.sh, which builds dashboard/ in replay mode first.');
  } else {
    const fixtures = path.join(DIST, 'console', 'replay', 'index.json');
    if (!existsSync(fixtures)) {
      warn('dist/console/replay/index.json is absent — the console will load with no replay to play.');
    }
  }

  const bytes = await dirSize(DIST);
  console.log(`\nsite/dist: ${(bytes / 1024 / 1024).toFixed(2)} MiB`);
  if (warnings.length) console.log(`${warnings.length} warning(s) above.`);
}

main().catch((e) => {
  console.error(`\nsite build FAILED: ${e.message}`);
  process.exit(1);
});
