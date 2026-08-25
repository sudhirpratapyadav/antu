'use strict';

// The operator console.
//
// Nothing here knows what a channel carries. It subscribes by name, renders
// whatever arrives, and picks out the few fields it wants to show. Adding a node
// to the graph makes its channel appear in the list below with no change here,
// which is the whole reason the bridge is generic over channels rather than
// carrying an enum of message types.

const WANTED = ['base.odom', 'base.status', 'base.ranges', 'base.imu',
                'phone_imu.imu', 'camera.frame'];

// Full-stick speeds. A P3-DX will do far more; this is a sane indoor pace.
const MAX_SPEED = 0.5;          // m/s
const MAX_TURN = 1.2;           // rad/s
// Commands repeat while the stick is held: the robot's own timeout stops it if
// this page freezes or the link drops mid-drive.
const DRIVE_PERIOD_MS = 100;
// Beyond this a reading is stale enough to say so rather than show quietly.
const STALE_MS = 2000;

const el = (id) => document.getElementById(id);
const latest = {};              // channel name -> last value
const seen = {};                // channel name -> {seq, at}
let ws = null;
let motorsOn = false;

// ---------- connection ----------

function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);

  ws.onopen = () => {
    el('dot').classList.add('on');
    el('link').textContent = location.host;
    ws.send(JSON.stringify({
      type: 'subscribe',
      payload: { channels: WANTED, maxHz: 10 },
    }));
  };

  ws.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    if (msg.type === 'msg') {
      accept(msg.payload);
    } else if (msg.type === 'snapshot') {
      msg.payload.channels.forEach(accept);
    } else if (msg.type === 'channels') {
      renderChannels(msg.payload.channels);
    } else if (msg.type === 'error') {
      console.warn('bridge error', msg.payload);
    }
    render();
  };

  ws.onclose = () => {
    el('dot').classList.remove('on');
    // Reconnect rather than leaving a dead page: the phone restarts, the Wi-Fi
    // drops, and an operator should not have to know to reload.
    setTimeout(connect, 1500);
  };

  ws.onerror = () => ws.close();
}

function accept(m) {
  latest[m.channel] = m.value;
  seen[m.channel] = { seq: m.seq, at: Date.now() };
}

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  }
}

// ---------- rendering ----------

const fmt = (v, digits, unit) =>
  v === undefined || v === null ? '–' : `${Number(v).toFixed(digits)}${unit || ''}`;

function render() {
  const status = latest['base.status'];
  const odom = latest['base.odom'];
  const ranges = latest['base.ranges'];

  if (status) {
    el('model').textContent = status.model;
    el('battery').textContent = fmt(status.batteryVolts, 1, ' V');
    el('battery').className = 'num' + (status.batteryVolts < 11.5 ? ' stale' : '');
    el('motors').textContent = status.motorsEnabled ? 'on' : 'off';
    el('estop').textContent = status.emergencyStopped ? 'PRESSED' : 'clear';
    el('estop').className = 'num' + (status.emergencyStopped ? ' stale' : '');
    if (motorsOn !== status.motorsEnabled) {
      // Follow the robot, not our intent: it drops the motors on an e-stop.
      motorsOn = status.motorsEnabled;
      el('motorsBtn').classList.toggle('on', motorsOn);
    }
  }

  if (odom) {
    el('x').textContent = fmt(odom.pose.x, 2, ' m');
    el('y').textContent = fmt(odom.pose.y, 2, ' m');
    el('theta').textContent = fmt(odom.pose.theta * 180 / Math.PI, 1, '°');
    el('speed').textContent = fmt(odom.velocity.linearX, 2, ' m/s');
    el('turn').textContent = fmt(odom.velocity.angular * 180 / Math.PI, 1, '°/s');
    trail.push({ x: odom.pose.x, y: odom.pose.y, th: odom.pose.theta });
    if (trail.length > 400) trail.shift();
  }

  const video = latest['camera.frame'];
  if (video) {
    el('videoInfo').textContent =
      `${video.width}x${video.height}  ${Math.round(video.sizeBytes / 1024)} KB  #${video.index}`;
  }

  if (ranges) {
    const closest = ranges.closest;
    el('nearest').textContent =
      typeof closest === 'number' ? fmt(closest, 2, ' m') : 'no return';
  }

  drawRadar();
}

function renderChannels(channels) {
  el('channels').innerHTML = channels.map((c) => {
    const s = seen[c.name];
    const stale = s && Date.now() - s.at > STALE_MS;
    return `<tr><td>${c.name}</td>` +
           `<td class="num${stale ? ' stale' : ''}">${c.type}</td></tr>`;
  }).join('');
}

// ---------- radar ----------

const trail = [];
const RANGE_M = 3.0;

function drawRadar() {
  const canvas = el('radar');
  const ctx = fit(canvas);
  const w = canvas.clientWidth;
  const h = canvas.height;
  const cx = w / 2;
  const cy = h / 2;
  const scale = (Math.min(cx, cy) - 12) / RANGE_M;

  ctx.clearRect(0, 0, w, h);

  ctx.strokeStyle = '#263041';
  ctx.fillStyle = '#55606f';
  ctx.font = '9px monospace';
  for (let r = 1; r <= 3; r++) {
    ctx.beginPath();
    ctx.arc(cx, cy, r * scale, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillText(`${r}m`, cx + 3, cy - r * scale + 11);
  }

  const odom = latest['base.odom'];
  if (odom && trail.length > 1) {
    // The trail lives in the odometry frame; rotate it so the robot points up.
    const th = odom.pose.theta;
    const c = Math.cos(-th + Math.PI / 2);
    const s = Math.sin(-th + Math.PI / 2);
    ctx.strokeStyle = '#3a5a7a';
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
      // Straight ahead is up, so a bearing is measured from -90 on screen.
      const a = ranges.bearings[i] - Math.PI / 2;
      const r = valid ? Math.min(ranges.ranges[i], RANGE_M) : RANGE_M;
      const px = cx + Math.cos(a) * r * scale;
      const py = cy + Math.sin(a) * r * scale;

      ctx.strokeStyle = valid ? '#2f6f4a' : '#22282f';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(px, py);
      ctx.stroke();

      if (valid) {
        // Only a real echo gets a mark. A beam at maximum range heard nothing,
        // which is not the same as clear space.
        ctx.fillStyle = '#ff9f43';
        ctx.beginPath();
        ctx.arc(px, py, 4, 0, Math.PI * 2);
        ctx.fill();
      }
    }
  }

  ctx.fillStyle = '#4da3ff';
  ctx.beginPath();
  ctx.moveTo(cx, cy - 11);
  ctx.lineTo(cx - 8, cy + 8);
  ctx.lineTo(cx + 8, cy + 8);
  ctx.closePath();
  ctx.fill();
}

function fit(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const w = canvas.clientWidth;
  if (canvas.width !== w * ratio) {
    canvas.width = w * ratio;
    canvas.height = canvas.height;
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  return ctx;
}

// ---------- joystick ----------

let stickX = 0;
let stickY = 0;
let held = false;

function drawStick() {
  const canvas = el('stick');
  const ctx = fit(canvas);
  const w = canvas.clientWidth;
  const h = canvas.height;
  const cx = w / 2;
  const cy = h / 2;
  const radius = Math.min(cx, cy) - 8;

  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#1b1f27';
  ctx.beginPath();
  ctx.arc(cx, cy, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = '#39424f';
  ctx.stroke();

  ctx.strokeStyle = '#2a3240';
  ctx.beginPath();
  ctx.moveTo(cx - radius, cy); ctx.lineTo(cx + radius, cy);
  ctx.moveTo(cx, cy - radius); ctx.lineTo(cx, cy + radius);
  ctx.stroke();

  ctx.fillStyle = held ? '#6cb6ff' : '#4da3ff';
  ctx.beginPath();
  ctx.arc(cx + stickX * radius, cy + stickY * radius, 26, 0, Math.PI * 2);
  ctx.fill();
}

function stickAt(event) {
  const canvas = el('stick');
  const rect = canvas.getBoundingClientRect();
  const point = event.touches ? event.touches[0] : event;
  const radius = Math.min(rect.width, rect.height) / 2 - 8;
  let dx = (point.clientX - rect.left - rect.width / 2) / radius;
  let dy = (point.clientY - rect.top - rect.height / 2) / radius;
  const mag = Math.hypot(dx, dy);
  if (mag > 1) { dx /= mag; dy /= mag; }
  stickX = dx;
  stickY = dy;
  drawStick();
}

function release() {
  // Lifting a finger must command zero, not hold the last value.
  held = false;
  stickX = 0;
  stickY = 0;
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

  setInterval(() => {
    if (!held) return;
    send({
      type: 'drive',
      payload: {
        linearX: -stickY * MAX_SPEED,     // screen y grows downward
        angular: -stickX * MAX_TURN,
      },
    });
  }, DRIVE_PERIOD_MS);
}

// ---------- controls ----------

function wireButtons() {
  el('motorsBtn').onclick = () => {
    motorsOn = !motorsOn;
    fetch(`/api/motors?on=${motorsOn ? 1 : 0}`);
  };
  el('resetBtn').onclick = () => {
    trail.length = 0;
    fetch('/api/reset');
  };
  el('estopBtn').onclick = () => {
    release();
    send({ type: 'estop', payload: {} });
  };
}

// Leaving the page must not leave the robot driving.
document.addEventListener('visibilitychange', () => {
  if (document.hidden) release();
});
window.addEventListener('pagehide', release);

// The MJPEG stream is a plain img src, so the browser handles decoding and
// reconnection. Only the metadata comes over the telemetry socket; sending
// frames as base64 there would swamp it.
function startVideo() {
  const img = el('video');
  img.onerror = () => setTimeout(startVideo, 2000);
  img.src = `/video.mjpeg?t=${Date.now()}`;
}

wireStick();
wireButtons();
startVideo();
drawStick();
drawRadar();
connect();
setInterval(render, 250);
