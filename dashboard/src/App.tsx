import { useEffect, useState } from 'react';
import { Console } from './console/Console';
import { Inspector } from './inspector/Inspector';
import { useRal } from './client/useRal';
import { C, F } from './theme/tokens';

export type Screen = 'console' | 'runs' | 'compare' | 'reference';

const SCREENS: Screen[] = ['console', 'runs', 'compare', 'reference'];

function screenFromHash(): Screen {
  const value = (window.location.hash || '').slice(1);
  return (SCREENS as string[]).includes(value) ? (value as Screen) : 'console';
}

export function App() {
  const [screen, setScreen] = useState<Screen>(screenFromHash);
  const ral = useRal();

  useEffect(() => {
    const onHash = () => setScreen(screenFromHash());
    window.addEventListener('hashchange', onHash);
    return () => window.removeEventListener('hashchange', onHash);
  }, []);

  const go = (next: Screen) => {
    window.location.hash = next;
    setScreen(next);
  };

  if (ral.error) {
    return (
      <div style={{
        padding: 24, font: `12.5px/1.6 ${F.mono}`, color: C.bad, whiteSpace: 'pre-wrap',
      }}
      >
        {`The dashboard could not read /api/reference, so it has no catalogue, no robot
profiles and no map, and will not guess any of them.

${ral.error}

Start the gateway with scripts/dashboard.sh, or run with ?source=mock for the
design fixture.`}
      </div>
    );
  }

  if (!ral.reference || !ral.state) {
    return (
      <div style={{ padding: 24, font: `12.5px ${F.mono}`, color: C.text4 }}>
        loading /api/reference…
      </div>
    );
  }

  if (!ral.reference.scenarios || !ral.reference.world_geometry) {
    return (
      <div style={{
        padding: 24, font: `12.5px/1.6 ${F.mono}`, color: C.bad, whiteSpace: 'pre-wrap',
      }}
      >
        {`/api/reference answered without \`scenarios\` or \`world_geometry\`.

That is an older gateway than this bundle: the console reads the world's doorways and the
committed scenario poses from it and will not substitute its own. Restart
scripts/dashboard.sh against the current ros2_ws/install.`}
      </div>
    );
  }

  if (screen === 'console') return <Console ral={ral} go={go} />;
  return <Inspector ral={ral} screen={screen} go={go} />;
}
