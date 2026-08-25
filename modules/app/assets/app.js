'use strict';

// The operator console.
//
// Nothing here knows what a channel carries. It subscribes by name, renders what
// arrives, and reads the few fields it displays. Adding a node to the graph makes
// its channel appear in the diagnostics list with no change here — the reason the
// bridge is generic over channels instead of carrying an enum of message types.

const WANTED = ['base.odom', 'base.status', 'base.ranges', 'base.imu',
                'phone_imu.imu', 'camera.frame', 'ar.frame', 'ar.pose',
                'fusion.pose'];

// Full-stick and full-key speeds. A P3-DX will do far more; this is an indoor pace.
const MAX_SPEED = 0.5;      // m/s
const MAX_TURN = 1.2;       // rad/s
const SLOW_FACTOR = 0.35;   // held shift
// Commands repeat while a control is held, so the base driver's silence timeout
// stops the robot if this page freezes or the link drops mid-drive.
const DRIVE_PERIOD_MS = 100;
const STALE_MS = 2000;
const RANGE_M = 3.0;

const el = (id) => document.getElementById(id);
const latest = {};
const seen = {};
let ws = null;
let motorsOn = false;
let videoAlive = false;

// ── connection ─────────────────────────────────────────────────────────────

function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);

  ws.onopen = () => {
    el('dot').classList.add('on');
    ws.send(JSON.stringify({
      type: 'subscribe',
      payload: { channels: WANTED, maxHz: 10 },
    }));
  };

  ws.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    if (msg.type === 'msg') accept(msg.payload);
    else if (msg.type === 'snapshot') msg.payload.channels.forEach(accept);
    else if (msg.type === 'channels') renderChannels(msg.payload.channels);
    else if (msg.type === 'error') console.warn('bridge', msg.payload);
  };

  ws.onclose = () => {
    el('dot').classList.remove('on');
    // Reconnect rather than leaving a dead page: the phone restarts and the
    // Wi-Fi drops, and an operator should not have to know to reload.
    setTimeout(connect, 1500);
  };
  ws.onerror = () => ws.close();
}

const accept = (m) => {
  latest[m.channel] = m.value;
  seen[m.channel] = Date.now();
};

const send = (obj) => {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
};

// ── rendering ──────────────────────────────────────────────────────────────

const fmt = (v, d, u) =>
  v === undefined || v === null || Number.isNaN(v) ? '–' : `${Number(v).toFixed(d)}${u || ''}`;

function render() {
  const status = latest['base.status'];
  const odom = latest['base.odom'];
  const ranges = latest['base.ranges'];
  const imu = latest['phone_imu.imu'];
  const video = latest['ar.frame'] || latest['camera.frame'];

  if (status) {
    el('model').textContent = status.model;
    const v = status.batteryVolts;
    const battery = el('battery');
    battery.textContent = fmt(v, 1, 'V');
    // A 12 V pack is flat near 11; say so before the robot stops mid-room.
    battery.className = v < 11.2 ? 'bad' : v < 11.8 ? 'warn' : 'good';

    el('motors').textContent = status.motorsEnabled ? 'on' : 'off';
    el('motors').className = status.motorsEnabled ? 'good' : 'dim';
    el('estop').textContent = status.emergencyStopped ? 'PRESSED' : 'clear';
    el('estop').className = status.emergencyStopped ? 'bad' : 'good';

    if (motorsOn !== status.motorsEnabled) {
      // Follow the robot, not our intent: it drops the motors on an e-stop.
      motorsOn = status.motorsEnabled;
      el('motorsBtn').classList.toggle('on', motorsOn);
    }
  }

  if (odom) {
    el('speed').textContent = fmt(odom.velocity.linearX, 2, ' m/s');
    el('turn').textContent = fmt(odom.velocity.angular * 180 / Math.PI, 0, '°/s');
    el('theta').textContent = fmt(odom.pose.theta * 180 / Math.PI, 0, '°');
    el('d-x').textContent = fmt(odom.pose.x, 3, ' m');
    el('d-y').textContent = fmt(odom.pose.y, 3, ' m');
    el('d-th').textContent = fmt(odom.pose.theta * 180 / Math.PI, 1, '°');
    el('d-v').textContent = fmt(odom.velocity.linearX, 3, ' m/s');
    el('d-w').textContent = fmt(odom.velocity.angular, 3, ' rad/s');
    trail.push({ x: odom.pose.x, y: odom.pose.y });
    if (trail.length > 400) trail.shift();
  }

  const fused = latest['fusion.pose'];
  if (fused) {
    const near = el('theta');
    el('theta').textContent = fmt(fused.pose.theta * 180 / Math.PI, 0, '°');
    const src = el('poseSrc');
    if (src) {
      src.textContent = fused.source === 'TRACKED'
        ? 'tracked'
        : `dead reckoning ${fused.secondsSinceFix.toFixed(0)}s`;
      src.className = fused.source === 'TRACKED' ? 'good' : 'warn';
    }
  }

  if (ranges) {
    const c = ranges.closest;
    const near = el('nearest');
    near.textContent = typeof c === 'number' ? fmt(c, 2, 'm') : '–';
    near.className = typeof c === 'number' ? (c < 0.4 ? 'bad' : c < 0.8 ? 'warn' : '') : '';
  }

  if (imu) {
    el('d-gz').textContent = fmt(imu.angularVelocity.z, 3, ' rad/s');
    el('d-az').textContent = fmt(imu.linearAcceleration.z, 2, ' m/s²');
    el('d-ih').textContent = fmt(imu.heading * 180 / Math.PI, 1, '°');
  }

  if (video) {
    el('d-vsize').textContent = `${video.width}×${video.height}`;
    el('d-vidx').textContent = video.index;
    el('d-vbytes').textContent = `${Math.round(video.sizeBytes / 1024)} KB`;
    // The metadata channel is the honest liveness signal: the img element keeps
    // showing the last frame long after the stream has died.
    const fresh = Date.now() -
      Math.max(seen['ar.frame'] || 0, seen['camera.frame'] || 0) < STALE_MS;
    if (fresh !== videoAlive) {
      videoAlive = fresh;
      el('noVideo').style.display = fresh ? 'none' : 'flex';
    }
  }

  el('maxSpeed').textContent = fmt(slow ? MAX_SPEED * SLOW_FACTOR : MAX_SPEED, 2, ' m/s');
  drawRadar();
}

function renderChannels(channels) {
  el('channels').innerHTML = channels.map((c) => {
    const age = seen[c.name] ? Date.now() - seen[c.name] : null;
    const stale = age === null || age > STALE_MS;
    return `<tr><td class="chname">${c.name}</td>` +
           `<td class="num${stale ? ' stale' : ''}">${c.type}</td>` +
           `<td class="num">${c.published}</td></tr>`;
  }).join('');
}

// ── sonar minimap ──────────────────────────────────────────────────────────

const trail = [];

function fit(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  if (canvas.width !== Math.round(w * ratio) || canvas.height !== Math.round(h * ratio)) {
    canvas.width = Math.round(w * ratio);
    canvas.height = Math.round(h * ratio);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  return { ctx, w, h };
}

function drawRadar() {
  const { ctx, w, h } = fit(el('radar'));
  const cx = w / 2;
  const cy = h / 2;
  const scale = (Math.min(cx, cy) - 6) / RANGE_M;

  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = 'rgba(120,140,170,.2)';
  ctx.lineWidth = 1;
  for (let r = 1; r <= 3; r++) {
    ctx.beginPath();
    ctx.arc(cx, cy, r * scale, 0, Math.PI * 2);
    ctx.stroke();
  }

  const odom = latest['base.odom'];
  if (odom && trail.length > 1) {
    // The trail is in the odometry frame; rotate so the robot always points up.
    const c = Math.cos(-odom.pose.theta + Math.PI / 2);
    const s = Math.sin(-odom.pose.theta + Math.PI / 2);
    ctx.strokeStyle = 'rgba(77,163,255,.5)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    trail.forEach((p, i) => {
      const dx = p.x - odom.pose.x;
      const dy = p.y - odom.pose.y;
      const px = cx + (dx * c - dy * s) * scale;
      const py = cy - (dx * s + dy * c) * scale;
      i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
    });
    ctx.stroke();
  }

  const ranges = latest['base.ranges'];
  if (ranges) {
    for (let i = 0; i < ranges.size; i++) {
      const valid = ranges.valid[i];
      const a = ranges.bearings[i] - Math.PI / 2;   // ahead is up
      const r = valid ? Math.min(ranges.ranges[i], RANGE_M) : RANGE_M;
      const px = cx + Math.cos(a) * r * scale;
      const py = cy + Math.sin(a) * r * scale;

      ctx.strokeStyle = valid ? 'rgba(63,185,80,.45)' : 'rgba(120,140,170,.12)';
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(px, py);
      ctx.stroke();

      if (valid) {
        // Only a real echo is marked. A beam at maximum range heard nothing,
        // which is not the same as clear space.
        ctx.fillStyle = ranges.ranges[i] < 0.6 ? '#f0616d' : '#ffa657';
        ctx.beginPath();
        ctx.arc(px, py, 3, 0, Math.PI * 2);
        ctx.fill();
      }
    }
  }

  ctx.fillStyle = '#4da3ff';
  ctx.beginPath();
  ctx.moveTo(cx, cy - 8);
  ctx.lineTo(cx - 6, cy + 6);
  ctx.lineTo(cx + 6, cy + 6);
  ctx.closePath();
  ctx.fill();
}

// ── command sources: stick and keyboard ────────────────────────────────────

let stickX = 0, stickY = 0, held = false;
const keys = new Set();
let slow = false;

function drawStick() {
  const { ctx, w, h } = fit(el('stick'));
  const cx = w / 2, cy = h / 2;
  const radius = Math.min(cx, cy) - 4;

  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = 'rgba(255,255,255,.04)';
  ctx.beginPath(); ctx.arc(cx, cy, radius, 0, Math.PI * 2); ctx.fill();
  ctx.strokeStyle = 'rgba(120,140,170,.25)'; ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(cx - radius, cy); ctx.lineTo(cx + radius, cy);
  ctx.moveTo(cx, cy - radius); ctx.lineTo(cx, cy + radius);
  ctx.strokeStyle = 'rgba(120,140,170,.13)'; ctx.stroke();

  // Show the commanded vector whatever produced it, so the keyboard moves the
  // knob too and there is one place to see what the robot was told.
  const [fwd, turn] = commanded();
  const kx = held ? stickX : -turn / MAX_TURN;
  const ky = held ? stickY : -fwd / MAX_SPEED;
  const active = held || keys.size > 0;

  ctx.fillStyle = active ? '#6cb6ff' : 'rgba(77,163,255,.55)';
  ctx.beginPath();
  ctx.arc(cx + kx * radius * 0.72, cy + ky * radius * 0.72, radius * 0.26, 0, Math.PI * 2);
  ctx.fill();
}

/** The velocity currently being commanded, from whichever control is active. */
function commanded() {
  const scale = slow ? SLOW_FACTOR : 1;
  if (held) {
    return [-stickY * MAX_SPEED * scale, -stickX * MAX_TURN * scale];
  }
  let fwd = 0, turn = 0;
  if (keys.has('up')) fwd += 1;
  if (keys.has('down')) fwd -= 1;
  if (keys.has('left')) turn += 1;
  if (keys.has('right')) turn -= 1;
  return [fwd * MAX_SPEED * scale, turn * MAX_TURN * scale];
}

function stickAt(event) {
  const rect = el('stick').getBoundingClientRect();
  const point = event.touches ? event.touches[0] : event;
  const radius = Math.min(rect.width, rect.height) / 2 - 4;
  let dx = (point.clientX - rect.left - rect.width / 2) / radius;
  let dy = (point.clientY - rect.top - rect.height / 2) / radius;
  const mag = Math.hypot(dx, dy);
  if (mag > 1) { dx /= mag; dy /= mag; }
  stickX = dx; stickY = dy;
  drawStick();
}

function release() {
  // Letting go must command zero, not hold the last value.
  held = false; stickX = 0; stickY = 0;
  keys.clear();
  paintKeys();
  drawStick();
  send({ type: 'drive', payload: { linearX: 0, angular: 0 } });
}

function wireStick() {
  const canvas = el('stick');
  const grab = (e) => { held = true; stickAt(e); e.preventDefault(); };
  const move = (e) => { if (held) { stickAt(e); e.preventDefault(); } };
  canvas.addEventListener('mousedown', grab);
  canvas.addEventListener('mousemove', move);
  window.addEventListener('mouseup', () => { if (held) release(); });
  canvas.addEventListener('touchstart', grab, { passive: false });
  canvas.addEventListener('touchmove', move, { passive: false });
  canvas.addEventListener('touchend', release);
  canvas.addEventListener('touchcancel', release);
}

const KEY_MAP = {
  ArrowUp: 'up', ArrowDown: 'down', ArrowLeft: 'left', ArrowRight: 'right',
  w: 'up', s: 'down', a: 'left', d: 'right',
  W: 'up', S: 'down', A: 'left', D: 'right',
};

function paintKeys() {
  ['up', 'down', 'left', 'right'].forEach((k) =>
    el(`k-${k}`).classList.toggle('hit', keys.has(k)));
  el('k-space').classList.toggle('hit', false);
}

function wireKeyboard() {
  window.addEventListener('keydown', (e) => {
    if (e.repeat) return;
    const dir = KEY_MAP[e.key];
    if (dir) {
      // Arrows scroll the page by default, and a page that scrolls while you
      // drive is unusable.
      e.preventDefault();
      keys.add(dir);
      paintKeys();
      drawStick();
      return;
    }
    if (e.key === 'Shift') { slow = true; return; }
    if (e.key === ' ') { e.preventDefault(); release(); return; }
    if (e.key === 'm' || e.key === 'M') { toggleMotors(); return; }
    if (e.key === 'i' || e.key === 'I') {
      showDrawer(!el('drawer').classList.contains('open'));
      return;
    }
    if (e.key === 'Escape') { showDrawer(false); return; }
    if (e.key === 'e' || e.key === 'E') { emergencyStop(); }
  });

  window.addEventListener('keyup', (e) => {
    const dir = KEY_MAP[e.key];
    if (dir) {
      keys.delete(dir);
      paintKeys();
      drawStick();
      if (keys.size === 0) send({ type: 'drive', payload: { linearX: 0, angular: 0 } });
      return;
    }
    if (e.key === 'Shift') slow = false;
  });

  // A key held while the window loses focus never fires keyup, and the robot
  // would keep the command until its own timeout. Let go on the way out.
  window.addEventListener('blur', release);
}

// ── controls ───────────────────────────────────────────────────────────────

function toggleMotors() {
  motorsOn = !motorsOn;
  el('motorsBtn').classList.toggle('on', motorsOn);
  fetch(`/api/motors?on=${motorsOn ? 1 : 0}`);
}

function emergencyStop() {
  release();
  send({ type: 'estop', payload: {} });
}

function showDrawer(open) {
  el('drawer').classList.toggle('open', open);
}

function wireButtons() {
  el('motorsBtn').onclick = toggleMotors;
  el('estopBtn').onclick = emergencyStop;
  el('infoBtn').onclick = () => showDrawer(true);
  el('closeBtn').onclick = () => showDrawer(false);

  // Clicking the backdrop closes, but only the backdrop: a click that started on
  // a panel must not dismiss it, or selecting a value becomes a fight.
  el('drawer').addEventListener('mousedown', (e) => {
    if (e.target === el('drawer')) showDrawer(false);
  });
}

// ── video ──────────────────────────────────────────────────────────────────

function startVideo() {
  const img = el('video');
  img.onerror = () => { videoAlive = false; setTimeout(startVideo, 2000); };
  // Cache-busted so a reconnect starts a new multipart stream rather than
  // resuming a dead one.
  img.src = `/video.mjpeg?t=${Date.now()}`;
}

// Leaving the page must not leave the robot driving.
document.addEventListener('visibilitychange', () => { if (document.hidden) release(); });
window.addEventListener('pagehide', release);
window.addEventListener('resize', () => { drawRadar(); drawStick(); });

wireStick();
wireKeyboard();
wireButtons();
startVideo();
connect();

// One timer drives both the repeat of held commands and the redraw. Repeating
// matters: a single dropped message must not leave the base holding a velocity.
setInterval(() => {
  if (held || keys.size > 0) {
    const [linearX, angular] = commanded();
    send({ type: 'drive', payload: { linearX, angular } });
  }
}, DRIVE_PERIOD_MS);

setInterval(render, 200);
render();
drawStick();
