import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';

// Bundled, not fetched from a CDN: the gateway serves this build from inside
// the container on a workstation that may have no internet, and a console
// whose type silently falls back to a system font is a different console from
// the one the design was measured in.
import '@fontsource/ibm-plex-sans/400.css';
import '@fontsource/ibm-plex-sans/500.css';
import '@fontsource/ibm-plex-sans/600.css';
import '@fontsource/ibm-plex-mono/400.css';
import '@fontsource/ibm-plex-mono/500.css';
import '@fontsource/ibm-plex-mono/600.css';

import { App } from './App';
import './index.css';

const host = document.getElementById('root');
if (!host) throw new Error('index.html has no #root');
createRoot(host).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
