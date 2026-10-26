import { useCallback, useEffect, useRef, useState } from 'react';
import type { CSSProperties, ReactNode } from 'react';
import { C, F } from '../theme/tokens';

/** The `?` affordance. Its text is always a reason, never a restatement. */
export function Tip({ text }: { text: string }) {
  return (
    <span
      data-tip={text}
      tabIndex={0}
      style={{
        display: 'inline-flex', alignItems: 'center', justifyContent: 'center',
        width: 15, height: 15, borderRadius: '50%', border: `1px solid ${C.borderHi}`,
        color: C.text3, font: `600 9.5px ${F.sans}`, cursor: 'help', flex: 'none',
      }}
    >
      ?
    </span>
  );
}

interface TipState { text: string; x: number; y: number; up: boolean; ax: number }

/** The one tooltip layer, driven by `data-tip` anywhere beneath the root. */
export function useTips() {
  const [tip, setTip] = useState<TipState | null>(null);
  const current = useRef<Element | null>(null);

  const onTip = useCallback((event: React.SyntheticEvent) => {
    const target = event.target;
    const element = target instanceof Element ? target.closest('[data-tip]') : null;
    if (!element) {
      if (current.current && event.type !== 'focus'
        && document.activeElement !== current.current) {
        current.current = null;
        setTip(null);
      }
      return;
    }
    if (current.current === element) return;
    current.current = element;
    const box = element.getBoundingClientRect();
    const centre = box.left + box.width / 2;
    const up = box.bottom > window.innerHeight - 170;
    const x = Math.max(8, Math.min(centre - 22, window.innerWidth - 290));
    setTip({
      text: element.getAttribute('data-tip') ?? '',
      x, y: up ? box.top - 9 : box.bottom + 9, up,
      ax: Math.max(8, Math.min(centre - x - 5, 270)),
    });
  }, []);

  const hideTip = useCallback(() => {
    current.current = null;
    setTip(null);
  }, []);

  const layer = tip && tip.text ? (
    <div
      style={{
        position: 'fixed', zIndex: 100, left: tip.x, top: tip.y,
        transform: tip.up ? 'translateY(-100%)' : 'none',
        width: 'max-content', maxWidth: 280, pointerEvents: 'none',
      }}
    >
      <div
        style={{
          position: 'absolute', left: tip.ax, top: tip.up ? 'calc(100% - 5px)' : -5,
          width: 9, height: 9, background: C.tip,
          borderLeft: `1px solid ${C.borderAlt}`, borderTop: `1px solid ${C.borderAlt}`,
          transform: tip.up ? 'rotate(225deg)' : 'rotate(45deg)',
        }}
      />
      <div
        style={{
          background: C.tip, border: `1px solid ${C.borderAlt}`, color: C.text,
          padding: '9px 11px', font: `11.5px/1.5 ${F.sans}`,
          boxShadow: '0 8px 24px rgba(0,0,0,.55)',
        }}
      >
        {tip.text}
      </div>
    </div>
  ) : null;

  return { onTip, hideTip, layer };
}

/** Copy to the clipboard, saying so for 1.2 s. */
export function CopyButton({ text, label = 'Copy' }: { text: string; label?: string }) {
  const [copied, setCopied] = useState(false);
  useEffect(() => {
    if (!copied) return undefined;
    const handle = window.setTimeout(() => setCopied(false), 1200);
    return () => window.clearTimeout(handle);
  }, [copied]);
  return (
    <button
      type="button"
      onClick={() => {
        void navigator.clipboard?.writeText(text);
        setCopied(true);
      }}
      style={{
        background: 'transparent', border: `1px solid ${C.lineBright}`, color: C.text2,
        font: `10.5px ${F.sans}`, padding: '1px 8px', cursor: 'pointer',
      }}
    >
      {copied ? 'Copied' : label}
    </button>
  );
}

export function Chevron({ open }: { open: boolean }) {
  return <span style={{ font: `11px ${F.mono}`, color: C.text4 }}>{open ? '▾' : '▸'}</span>;
}

export function SectionHead({ children }: { children: ReactNode }) {
  return (
    <span style={{
      font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text4,
    }}
    >
      {children}
    </span>
  );
}

/**
 * A value the robot did not report, with the reason it did not.
 *
 * Used everywhere a number would otherwise be invented. The reason is always
 * the one the profile, the manager or the metrics file gave, never a generic
 * "n/a".
 */
export function Unavailable({ short, reason, style }: {
  short: string; reason: string; style?: CSSProperties;
}) {
  return (
    <div style={{
      display: 'flex', gap: 6, alignItems: 'center',
      border: `1px dashed ${C.border}`, padding: '5px 8px', ...style,
    }}
    >
      <span style={{ font: `500 10.5px ${F.mono}`, color: C.text, flex: 1 }}>{short}</span>
      <Tip text={reason} />
    </div>
  );
}

export const buttonStyle = (active = false): CSSProperties => ({
  background: active ? C.text : 'transparent',
  border: `1px solid ${C.lineBright}`,
  color: active ? C.ink : C.text,
  font: `500 11.5px ${F.sans}`,
  padding: '4px 9px',
  cursor: 'pointer',
});

export const primaryButtonStyle = (disabled: boolean): CSSProperties => ({
  background: C.text, color: C.ink, border: `1px solid ${C.text}`,
  font: `600 11.5px ${F.sans}`, padding: '5px 11px',
  cursor: disabled ? 'default' : 'pointer', opacity: disabled ? 0.45 : 1,
});

export function f2(value: number): string { return value.toFixed(2); }
export function f3(value: number): string { return value.toFixed(3); }
