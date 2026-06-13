// teleop-app.jsx — WALL-E Drive interactive teleop screen (shared behavior, themed by CSS scope)
// Exports: TeleopScreen({ variant: 'deck'|'avionics', density: 'compact'|'roomy', simState })
// Behavior mirrors pi_app/web/teleop.py _DRIVE_HTML contracts:
//   · ARM = press-and-hold 520ms with radial progress ring; tap to disarm
//   · E-STOP latches on pointerdown; two-tap clear (bench/no-RC flow)
//   · Joystick: pointer capture, ±38% knob travel, spring return on release
//   · Arcade mix: thr=-y, L=clamp(thr+0.8x), R=clamp(thr-0.8x)

const { useState, useEffect, useRef } = React;

const STEER_GAIN = 0.8;
const CAPS = { SLOW: 30, NORMAL: 60, FAST: 100 };
const clamp1 = (v) => Math.max(-1, Math.min(1, v));

function TeleopScreen({ variant = 'deck', density = 'compact', simState = 'nominal' }) {
  const [speed, setSpeed] = useState('SLOW');
  const [estop, setEstop] = useState('idle');          // idle | latched | dim1
  const [armState, setArmState] = useState('idle');    // idle | holding | armed
  const [holdP, setHoldP] = useState(0);
  const [camOn, setCamOn] = useState(false);
  const [rcInHand, setRcInHand] = useState(true);
  const [stick, setStick] = useState({ x: 0, y: 0 });
  const [dragging, setDragging] = useState(false);
  const [tele, setTele] = useState({ rtt: 38, batt: 51.1, hb: 42, fps: 24 });
  const [ovDismissed, setOvDismissed] = useState(false);

  const padRef = useRef(null);
  const rafRef = useRef(null);
  const dimRef = useRef(null);

  const disarm = () => { setArmState('idle'); setHoldP(0); };

  // ----- simulated telemetry -----
  useEffect(() => {
    const id = setInterval(() => {
      setTele({
        rtt: Math.round(24 + Math.random() * 26),
        batt: Math.round((51.2 - Math.random() * 0.4) * 10) / 10,
        hb: Math.round(28 + Math.random() * 38),
        fps: Math.round(22 + Math.random() * 4),
      });
    }, 1300);
    return () => clearInterval(id);
  }, []);

  // ----- simState from Tweaks -----
  useEffect(() => {
    setOvDismissed(false);
    if (simState === 'estop-latched') { setEstop('latched'); disarm(); }
    else if (simState === 'disconnected') { disarm(); }
    else if (simState === 'nominal') { setEstop((s) => (s === 'latched' || s === 'dim1') ? s : 'idle'); }
  }, [simState]);

  useEffect(() => () => { cancelAnimationFrame(rafRef.current); clearTimeout(dimRef.current); }, []);

  const linkUp = simState !== 'disconnected';
  const battVal = simState === 'low-battery' ? 19.8 : tele.batt;
  const battWarn = battVal < 21;
  const capPct = CAPS[speed];

  // ----- ARM -----
  const armDisabled = estop !== 'idle' || !linkUp;
  function armDown(e) {
    if (armDisabled) return;
    if (armState === 'armed') { disarm(); return; }
    e.currentTarget.setPointerCapture && e.currentTarget.setPointerCapture(e.pointerId);
    setArmState('holding');
    const t0 = performance.now();
    const step = (t) => {
      const p = Math.min(1, (t - t0) / 520);
      setHoldP(p);
      if (p >= 1) { setArmState('armed'); return; }
      rafRef.current = requestAnimationFrame(step);
    };
    rafRef.current = requestAnimationFrame(step);
  }
  function armUp() {
    cancelAnimationFrame(rafRef.current);
    setArmState((s) => {
      if (s === 'holding') { setHoldP(0); return 'idle'; }
      return s;
    });
  }
  const armLabel = armDisabled
    ? (estop !== 'idle' ? 'CLEAR E-STOP FIRST' : 'LINK DOWN')
    : armState === 'armed' ? 'ARMED · TAP TO DISARM'
    : armState === 'holding' ? 'HOLD…'
    : 'HOLD TO ARM';
  const ringOffset = armState === 'armed' ? 0 : 94.25 * (1 - holdP);

  // ----- E-STOP -----
  function estopDown() {
    if (estop === 'idle') { setEstop('latched'); disarm(); }
    else if (estop === 'latched') {
      setEstop('dim1');
      clearTimeout(dimRef.current);
      dimRef.current = setTimeout(() => setEstop((s) => (s === 'dim1' ? 'latched' : s)), 3000);
    } else {
      clearTimeout(dimRef.current);
      setEstop('idle');
    }
  }
  const estopLabel = estop === 'latched' ? 'E-STOP LATCHED'
    : estop === 'dim1' ? 'TAP AGAIN TO CLEAR'
    : 'E — STOP';

  // ----- joystick -----
  function calcStick(e) {
    const r = padRef.current.getBoundingClientRect();
    const rad = r.width * 0.38;
    setStick({
      x: clamp1((e.clientX - (r.left + r.width / 2)) / rad),
      y: clamp1((e.clientY - (r.top + r.height / 2)) / rad),
    });
  }
  function padDown(e) {
    e.currentTarget.setPointerCapture && e.currentTarget.setPointerCapture(e.pointerId);
    setDragging(true);
    calcStick(e);
  }
  function padMove(e) { if (dragging) calcStick(e); }
  function padUp() { setDragging(false); setStick({ x: 0, y: 0 }); }

  const thr = -stick.y;
  const trackL = clamp1(thr + STEER_GAIN * stick.x);
  const trackR = clamp1(thr - STEER_GAIN * stick.x);
  const padDir = stick.y < -0.05 ? 'fwd' : stick.y > 0.05 ? 'rev' : '';
  const knobStyle = {
    left: (50 + stick.x * 38) + '%',
    top: (50 + stick.y * 38) + '%',
  };

  // ----- HUD values -----
  const rttStr = linkUp ? tele.rtt + ' ms' : '—';
  const hbStr = linkUp && armState === 'armed' ? tele.hb + ' ms' : '—';
  const battStr = linkUp ? battVal.toFixed(1) + ' V' : '—';
  const stateChip = estop !== 'idle'
    ? { cls: 'bad', label: 'ESTOP' }
    : armState === 'armed' ? { cls: 'ok', label: 'ARMED' } : { cls: '', label: 'DISARMED' };
  const rcChip = linkUp ? { cls: 'ok', label: 'RC ARMED' } : { cls: '', label: 'RC —' };

  const gaugeLive = armState === 'armed' && estop === 'idle' && linkUp;
  const gL = gaugeLive ? trackL : 0;
  const gR = gaugeLive ? trackR : 0;

  const overlayVisible = simState === 'disconnected' && !ovDismissed;

  const cells = [
    { lab: 'RTT', val: rttStr, cls: '' },
    { lab: 'STATE', val: stateChip.label, cls: stateChip.cls },
    { lab: 'BATT', val: battStr, cls: battWarn && linkUp ? 'warn' : '' },
    { lab: 'CAP', val: capPct + '%', cls: '' },
    { lab: 'DEADMAN', val: hbStr, cls: '' },
    { lab: 'RC', val: rcChip.label.replace('RC ', '') || '—', cls: rcChip.cls },
  ];

  return (
    <div className={'tscreen tv-' + variant + ' d-' + density}>

      {/* HUD */}
      <div className="t-hud">
        <div className="t-linkrow">
          <div className={'t-dot' + (linkUp ? ' live' : ' dead')}></div>
          <span className={'t-lnk' + (linkUp ? ' live' : ' dead')}>{linkUp ? 'LINK UP' : 'LINK DOWN'}</span>
          <div className="t-sp"></div>
          <button className={'t-cambtn' + (camOn ? ' on' : '')} onClick={() => setCamOn(!camOn)}>
            <span className="cd"></span>
            <span>{camOn ? 'CAM ON' : 'CAM OFF'}</span>
          </button>
        </div>

        {density === 'compact' ? (
          <React.Fragment>
            <div className="t-chips">
              <span className="t-chip num">RTT {rttStr}</span>
              <span className={'t-chip ' + stateChip.cls}>{stateChip.label}</span>
              <span className={'t-chip num' + (battWarn && linkUp ? ' warn' : '')}>{battStr}</span>
            </div>
            <div className="t-chips">
              <span className="t-chip num">CAP {capPct}%</span>
              <span className="t-chip num">HB {hbStr}</span>
              <span className={'t-chip ' + rcChip.cls}>{rcChip.label}</span>
            </div>
          </React.Fragment>
        ) : (
          <div className="t-cells">
            {cells.map((c) => (
              <div key={c.lab} className={'t-cell ' + c.cls}>
                <span className="lab">{c.lab}</span>
                <span className="val num">{c.val}</span>
              </div>
            ))}
          </div>
        )}
      </div>

      {/* camera panel */}
      {camOn && (
        <div className="t-cam">
          <div className="ph">
            <span>[ live camera feed ]</span>
            <span>4:3 JPEG over WebSocket</span>
          </div>
          <div className="fps num">{linkUp ? tele.fps + ' fps' : 'stale'}</div>
        </div>
      )}

      {/* drive zone — joystick bottom-anchored */}
      <div className="t-drive">
        <div className="t-padrow">
          <div className="t-gauge">
            <div className="t-gbar">
              <div className={'t-gfill ' + (gL >= 0 ? 'pos' : 'neg')} style={{ height: Math.abs(gL) * 50 + '%' }}></div>
            </div>
            <span className="t-glab">L</span>
          </div>

          <div
            className={'t-pad ' + padDir}
            ref={padRef}
            onPointerDown={padDown}
            onPointerMove={padMove}
            onPointerUp={padUp}
            onPointerCancel={padUp}
          >
            <span className="t-c tl"></span>
            <span className="t-c tr"></span>
            <span className="t-c bl"></span>
            <span className="t-c br"></span>
            <div className="t-guide"></div>
            <div className={'t-knob' + (dragging ? '' : ' spring')} style={knobStyle}></div>
          </div>

          <div className="t-gauge">
            <div className="t-gbar">
              <div className={'t-gfill ' + (gR >= 0 ? 'pos' : 'neg')} style={{ height: Math.abs(gR) * 50 + '%' }}></div>
            </div>
            <span className="t-glab">R</span>
          </div>
        </div>
        <div className="t-slbl">↑ FWD / REV ↓ &nbsp;&nbsp; ← STEER →</div>
      </div>

      {/* speed segmented */}
      <div className="t-seg">
        {['SLOW', 'NORMAL', 'FAST'].map((s) => (
          <button key={s} className={speed === s ? 'on' : ''} onClick={() => setSpeed(s)}>{s}</button>
        ))}
      </div>

      {/* ARM row */}
      <div className="t-armrow">
        <div className="t-armouter">
          <button
            className={'t-arm' + (armState === 'holding' ? ' arming' : '') + (armState === 'armed' ? ' armed' : '')}
            disabled={armDisabled}
            onPointerDown={armDown}
            onPointerUp={armUp}
            onPointerCancel={armUp}
          >{armLabel}</button>
          <svg className="t-ring" viewBox="0 0 32 32" aria-hidden="true">
            <circle className="r-bg" cx="16" cy="16" r="15"></circle>
            <circle className="r-fg" cx="16" cy="16" r="15" transform="rotate(-90 16 16)" style={{ strokeDashoffset: ringOffset }}></circle>
          </svg>
        </div>
        <label className="t-rc">
          <input type="checkbox" checked={rcInHand} onChange={(e) => setRcInHand(e.target.checked)} />
          <span>RC in hand</span>
        </label>
      </div>

      {/* E-STOP — bottom-anchored, nearest the thumb */}
      <div className="t-estopwrap">
        <button className={'t-estop ' + (estop === 'idle' ? '' : estop)} onPointerDown={estopDown}>{estopLabel}</button>
      </div>

      {/* disconnect overlay */}
      {overlayVisible && (
        <div className="t-ov">
          <div className="ic">⚠</div>
          <div className="ti">DISCONNECTED</div>
          <div className="su">Robot stopped · reconnecting…</div>
          <button onClick={() => setOvDismissed(true)}>DISMISS</button>
        </div>
      )}
    </div>
  );
}

Object.assign(window, { TeleopScreen });
