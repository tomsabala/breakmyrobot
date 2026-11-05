import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';

// Bundled, not fetched from a CDN: the gateway serves this build from inside
// the container on a workstation that may have no internet, and a console
// whose type silently falls back to a system font is a different console from
// the one the design was measured in.
//
// LATIN ONLY, and only the three weights this UI uses. The unqualified
// `400.css` entry points pull every subset @fontsource ships - latin,
// latin-ext, cyrillic, cyrillic-ext, vietnamese, greek - in both woff2 and a
// legacy woff copy, which is 892 kB of fonts in a bundle whose own JavaScript
// is 439 kB. Nothing in this console renders Cyrillic, Greek or Vietnamese:
// every string it draws is a topic name, a unit, a metric or an English
// sentence this repository wrote. The `latin-` entry points are the same
// faces with the other subsets dropped.
import '@fontsource/ibm-plex-sans/latin-400.css';
import '@fontsource/ibm-plex-sans/latin-500.css';
import '@fontsource/ibm-plex-sans/latin-600.css';
import '@fontsource/ibm-plex-mono/latin-400.css';
import '@fontsource/ibm-plex-mono/latin-500.css';
import '@fontsource/ibm-plex-mono/latin-600.css';

import { App } from './App';
import './index.css';

const host = document.getElementById('root');
if (!host) throw new Error('index.html has no #root');
createRoot(host).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
