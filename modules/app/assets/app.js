'use strict';

// The operator console.
//
// Nothing here knows what a channel carries. It subscribes by name, renders what
// arrives, and reads the few fields it displays. Adding a node to the graph makes
// its channel appear in the diagnostics list with no change here — the reason the
// bridge is generic over channels instead of carrying an enum of message types.

const WANTED = ['base.odom', 'base.status', 'base.ranges', 'base.imu',
                'phone_imu.imu', 'camera.frame', 'ar.frame', 'ar.pose',
                'fusion.pose', 'mapper.map'];

// Full-stick and full-key speeds. A P3-DX will do far more; this is an indoor pace.
const MAX_SPEED = 0.5;      // m/s
const MAX_TURN = 1.2;       // rad/s
const SLOW_FACTOR = 0.35;   // held shift
// Commands repeat while a control is held, so the base driver's silence timeout
// stops the robot if this page freezes or the link drops mid-drive.
const DRIVE_PERIOD_MS = 100;
const STALE_MS = 2000;
const RANGE_M = 3.0;      // sonar rings
let mapRange = 8.0;       // how much of the map to show, metres

const el = (id) => document.getElementById(id);
const latest = {};
const seen = {};
let ws = null;
let motorsOn = false;
let videoAlive = false;
// Set once quit has been accepted. Everything that retries checks it: after a
// deliberate shutdown the socket closing and the video stalling are the correct
// outcome, and a console that spends the next hour reconnecting to a robot the
// operator just switched off is reporting a fault that is not one.
let stopped = false;

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
    if (stopped) return;
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
    odomTrail.push({ x: odom.pose.x, y: odom.pose.y });
    if (odomTrail.length > 400) odomTrail.shift();
  }

  const fused = latest['fusion.pose'];
  if (fused) {
    el('theta').textContent = fmt(fused.pose.theta * 180 / Math.PI, 0, '°');
    trail.push({ x: fused.pose.x, y: fused.pose.y });
    if (trail.length > 400) trail.shift();

    const src = el('poseSrc');
    src.textContent = fused.source === 'TRACKED'
      ? 'tracked'
      : `dead ${fused.secondsSinceFix.toFixed(0)}s`;
    src.className = fused.source === 'TRACKED' ? 'good' : 'warn';

    el('d-src').textContent = fused.source;
    el('d-fix').textContent = fmt(fused.secondsSinceFix, 1, ' s');
    el('d-corr').textContent = fmt(fused.lastCorrection, 3, ' m');
    // How far the wheels have wandered from the anchored estimate. This number
    // is the whole argument for having a visual tracker at all.
    if (odom) {
      const dx = fused.pose.x - odom.pose.x;
      const dy = fused.pose.y - odom.pose.y;
      el('d-drift').textContent = fmt(Math.hypot(dx, dy), 3, ' m');
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

  // Keep the robot marker tracking the pose. updateMarker is a no-op within a
  // centimetre, so a parked robot costs nothing here.
  const hadMarker = markerPose;
  updateMarker();
  if (markerPose !== hadMarker) cloudDraw();
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

// Two trails, deliberately. The wheels drift and the visual tracker does not, so
// drawing both makes the difference between them visible instead of theoretical:
// drive a loop and the odometry trail walks away from the fused one.
const trail = [];        // fused: anchored to the room
const odomTrail = [];    // wheels: continuous, drifting

/**
 * The occupied cells, seen from above with the robot always pointing up.
 *
 * Cells arrive as world coordinates in metres, so nothing here needs to know how
 * the grid is laid out on the robot. They are drawn relative to the robot's
 * current pose, which is what makes the view egocentric without the map itself
 * being egocentric — the map is anchored to the room and this is a window onto it.
 */
function drawMap(ctx, cx, cy, scale) {
  const m = latest['mapper.map'];
  const fused = latest['fusion.pose'];
  if (!m || !fused || !m.cells || !m.cells.length) return;

  const c = Math.cos(-fused.pose.theta + Math.PI / 2);
  const s = Math.sin(-fused.pose.theta + Math.PI / 2);
  const cell = Math.max(1.5, m.resolution * scale);

  ctx.fillStyle = 'rgba(120,200,255,.75)';
  for (const [wx, wy] of m.cells) {
    const dx = wx - fused.pose.x;
    const dy = wy - fused.pose.y;
    if (Math.abs(dx) > mapRange || Math.abs(dy) > mapRange) continue;
    const px = cx + (dx * c - dy * s) * scale;
    const py = cy - (dx * s + dy * c) * scale;
    ctx.fillRect(px - cell / 2, py - cell / 2, cell, cell);
  }

  el('d-mapcells').textContent = `${m.occupied} / ${m.observed}`;
}

function drawTrail(ctx, cx, cy, scale, points, here, colour, width) {
  // Rotate into a robot-up view, so the trail shows where the robot came from
  // regardless of which way it is now facing.
  const c = Math.cos(-here.theta + Math.PI / 2);
  const s = Math.sin(-here.theta + Math.PI / 2);
  ctx.strokeStyle = colour;
  ctx.lineWidth = width;
  ctx.beginPath();
  points.forEach((p, i) => {
    const dx = p.x - here.x;
    const dy = p.y - here.y;
    const px = cx + (dx * c - dy * s) * scale;
    const py = cy - (dx * s + dy * c) * scale;
    i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
  });
  ctx.stroke();
}

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
  const mapScale = (Math.min(cx, cy) - 6) / mapRange;
  const scale = (Math.min(cx, cy) - 6) / RANGE_M;

  ctx.clearRect(0, 0, w, h);

  // The map first, underneath everything: it is the thing the panel is for now,
  // and the sonar rings and trails are annotations on top of it.
  drawMap(ctx, cx, cy, mapScale);

  ctx.strokeStyle = 'rgba(120,140,170,.14)';
  ctx.lineWidth = 1;
  for (let r = 1; r <= 3; r++) {
    ctx.beginPath();
    ctx.arc(cx, cy, r * scale, 0, Math.PI * 2);
    ctx.stroke();
  }

  // Both trails are drawn relative to the robot's current position in their own
  // frame, so each shows where that estimator thinks it has been.
  const fused = latest['fusion.pose'];
  const odom = latest['base.odom'];
  if (fused && trail.length > 1) {
    drawTrail(ctx, cx, cy, mapScale, trail, fused.pose, 'rgba(77,163,255,.85)', 2);
  }
  if (odom && odomTrail.length > 1) {
    // Dimmer: this is the one that drifts, shown for comparison rather than use.
    drawTrail(ctx, cx, cy, mapScale, odomTrail, odom.pose, 'rgba(255,159,67,.45)', 1.2);
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

// ── view switching: hero + picture-in-picture ──────────────────────────────

// Three live surfaces — camera, 2D map, 3D map — one large, two thumbnails.
// Switching swaps class names only. Nothing is reparented and nothing stops
// rendering: the thumbnails are the same elements drawing at a smaller size,
// which is what makes them honest previews rather than stale snapshots.
const VIEW_SURFACE = { camera: 'surfCam', map2d: 'surfMap', map3d: 'surfCloud' };
const VIEW_LABEL = { camera: 'camera', map2d: '2D map', map3d: '3D map' };
let heroView = 'camera';

function applyViews() {
  const pips = Object.keys(VIEW_SURFACE).filter((v) => v !== heroView);
  el(VIEW_SURFACE[heroView]).className = 'surface hero';
  el(VIEW_SURFACE[pips[0]]).className = 'surface pip0';
  el(VIEW_SURFACE[pips[1]]).className = 'surface pip1';
  el('heroTag').textContent = VIEW_LABEL[heroView];
  document.body.dataset.view = heroView;
  // Both canvases just changed size; redraw at the new one rather than letting
  // the browser scale a stale frame.
  drawRadar();
  cloudDraw();
}

function setHero(view) {
  if (view === heroView) return;
  heroView = view;
  // Remembered so a reload mid-drive comes back showing the same thing.
  try { localStorage.setItem('antu.hero', view); } catch (e) { /* private mode */ }
  applyViews();
}

function wireViews() {
  try {
    const saved = localStorage.getItem('antu.hero');
    if (saved && VIEW_SURFACE[saved]) heroView = saved;
  } catch (e) { /* private mode */ }
  // #view=map3d opens on that view — a linkable way into a particular panel.
  const asked = new URLSearchParams(location.hash.slice(1)).get('view');
  if (asked && VIEW_SURFACE[asked]) heroView = asked;
  for (const [view, id] of Object.entries(VIEW_SURFACE)) {
    // Only a thumbnail switches; a click on the hero belongs to the hero.
    el(id).addEventListener('click', () => { if (heroView !== view) setHero(view); });
  }
  applyViews();
}

// ── 3D cloud viewer ────────────────────────────────────────────────────────
// The jarvis scan renderer, previously a separate /cloud.html page, now a
// surface in the console. Points arrive in the tracker's world frame, which is
// y-up, so the camera uses y as up and nothing needs converting.

let cgl = null, cloudProg, cloudBuf, cloudPts = 0, cloudXyz = null;
const cloudCam = { px: 0, py: 0, pz: 0, yaw: 0.7, pitch: 0.5, dist: 6, fovy: 1.0 };
let cloudLive = true;
// The robot's marker: a floor arrow rebuilt whenever fusion.pose moves.
let markerBuf = null, markerVerts = 0, markerPose = null;
// Where the floor sits in the cloud's y-up frame, estimated from the points
// themselves at each load. ARCore's origin is wherever the phone woke up, so
// nothing else knows the floor's height.
let cloudFloorY = 0;

const CLOUD_VS = `
attribute vec3 pos; attribute vec3 col;
uniform mat4 mvp; uniform float size;
varying vec3 vcol;
void main() {
  gl_Position = mvp * vec4(pos, 1.0);
  // Nearer points draw larger, which reads as depth without any shading.
  gl_PointSize = max(1.0, size / max(gl_Position.w, 0.15));
  vcol = col;
}`;
const CLOUD_FS = `
precision mediump float;
varying vec3 vcol;
void main() { gl_FragColor = vec4(vcol, 1.0); }`;

function cloudGlInit() {
  cgl = el('gl3d').getContext('webgl', { antialias: true });
  if (!cgl) { el('noCloud').textContent = 'WebGL unavailable in this browser'; return false; }
  const sh = (t, src) => {
    const o = cgl.createShader(t);
    cgl.shaderSource(o, src); cgl.compileShader(o);
    if (!cgl.getShaderParameter(o, cgl.COMPILE_STATUS)) throw cgl.getShaderInfoLog(o);
    return o;
  };
  cloudProg = cgl.createProgram();
  cgl.attachShader(cloudProg, sh(cgl.VERTEX_SHADER, CLOUD_VS));
  cgl.attachShader(cloudProg, sh(cgl.FRAGMENT_SHADER, CLOUD_FS));
  cgl.linkProgram(cloudProg);
  cloudBuf = cgl.createBuffer();
  cgl.enable(cgl.DEPTH_TEST);
  return true;
}

function m4Persp(out, fovy, asp, near, far) {
  const f = 1 / Math.tan(fovy / 2), nf = 1 / (near - far);
  out.fill(0);
  out[0] = f / asp; out[5] = f;
  out[10] = (far + near) * nf; out[11] = -1; out[14] = 2 * far * near * nf;
  return out;
}
function m4LookAt(out, eye, ctr, up) {
  let zx = eye[0]-ctr[0], zy = eye[1]-ctr[1], zz = eye[2]-ctr[2];
  let l = Math.hypot(zx, zy, zz) || 1; zx/=l; zy/=l; zz/=l;
  let xx = up[1]*zz - up[2]*zy, xy = up[2]*zx - up[0]*zz, xz = up[0]*zy - up[1]*zx;
  l = Math.hypot(xx, xy, xz) || 1; xx/=l; xy/=l; xz/=l;
  const yx = zy*xz - zz*xy, yy = zz*xx - zx*xz, yz = zx*xy - zy*xx;
  out[0]=xx; out[1]=yx; out[2]=zx; out[3]=0;
  out[4]=xy; out[5]=yy; out[6]=zy; out[7]=0;
  out[8]=xz; out[9]=yz; out[10]=zz; out[11]=0;
  out[12]=-(xx*eye[0]+xy*eye[1]+xz*eye[2]);
  out[13]=-(yx*eye[0]+yy*eye[1]+yz*eye[2]);
  out[14]=-(zx*eye[0]+zy*eye[1]+zz*eye[2]);
  out[15]=1;
  return out;
}
function m4Mul(out, a, b) {
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      let v = 0;
      for (let k = 0; k < 4; k++) v += a[k*4+j] * b[i*4+k];
      out[i*4+j] = v;
    }
  }
  return out;
}
function cloudEye() {
  const cp = Math.cos(cloudCam.pitch);
  return [cloudCam.px + cloudCam.dist * cp * Math.sin(cloudCam.yaw),
          cloudCam.py + cloudCam.dist * Math.sin(cloudCam.pitch),
          cloudCam.pz + cloudCam.dist * cp * Math.cos(cloudCam.yaw)];
}
function cloudMvp() {
  const cv = el('gl3d');
  const P = m4Persp(new Float32Array(16), cloudCam.fovy, (cv.width / cv.height) || 1, 0.02, 400);
  const V = m4LookAt(new Float32Array(16), cloudEye(),
                     [cloudCam.px, cloudCam.py, cloudCam.pz], [0, 1, 0]);
  return m4Mul(new Float32Array(16), P, V);
}

/** Count, then positions, then colours — packed, because JSON of this is huge. */
async function cloudLoad() {
  try {
    const b = await (await fetch('/cloud.bin')).arrayBuffer();
    if (b.byteLength < 4) return;
    const n = new Int32Array(b, 0, 1)[0];
    cloudPts = n;
    el('cloudN').textContent = n.toLocaleString();
    if (!n) return;
    el('noCloud').style.display = 'none';

    const pos = new Float32Array(b, 4, n * 3);
    const col = new Uint8Array(b, 4 + n * 12, n * 3);
    const first = !cloudXyz;
    cloudXyz = pos;

    // A near-minimum rather than the minimum: one depth artefact below the
    // floor would otherwise sink the robot marker with it.
    const ys = [];
    for (let i = 0; i < n; i += Math.max(1, Math.floor(n / 500))) ys.push(pos[i*3+1]);
    ys.sort((a, b2) => a - b2);
    cloudFloorY = ys.length ? ys[Math.floor(ys.length * 0.05)] : 0;

    if (!cgl && !cloudGlInit()) return;
    const arr = new Float32Array(n * 6);
    for (let i = 0; i < n; i++) {
      arr[i*6] = pos[i*3]; arr[i*6+1] = pos[i*3+1]; arr[i*6+2] = pos[i*3+2];
      arr[i*6+3] = col[i*3]/255; arr[i*6+4] = col[i*3+1]/255; arr[i*6+5] = col[i*3+2]/255;
    }
    cgl.bindBuffer(cgl.ARRAY_BUFFER, cloudBuf);
    cgl.bufferData(cgl.ARRAY_BUFFER, arr, cgl.STATIC_DRAW);
    // Frame it once on arrival; after that leave the camera where it was put,
    // or it would snap away every few seconds as the cloud grows.
    if (first) cloudFit(); else cloudDraw();
  } catch (e) {
    // The robot may be restarting; the next poll will find it.
  }
}

function cloudBounds() {
  let x0=1e9,y0=1e9,z0=1e9,x1=-1e9,y1=-1e9,z1=-1e9;
  for (let i = 0; i < cloudPts; i++) {
    const x=cloudXyz[i*3], y=cloudXyz[i*3+1], z=cloudXyz[i*3+2];
    if(x<x0)x0=x; if(y<y0)y0=y; if(z<z0)z0=z;
    if(x>x1)x1=x; if(y>y1)y1=y; if(z>z1)z1=z;
  }
  return [x0,y0,z0,x1,y1,z1];
}

function cloudFit() {
  if (!cloudXyz || !cloudPts) return;
  const [x0,y0,z0,x1,y1,z1] = cloudBounds();
  cloudCam.px=(x0+x1)/2; cloudCam.py=(y0+y1)/2; cloudCam.pz=(z0+z1)/2;
  cloudCam.dist = Math.max(0.6, Math.hypot(x1-x0, y1-y0, z1-z0) * 0.9);
  cloudDraw();
}

/** Straight down, which is the floor plan the occupancy grid also sees. */
// Not `top`: that is window.top, a non-configurable global, and a script-level
// declaration by that name is a SyntaxError that stops the whole script.
function cloudTopView() {
  if (!cloudXyz || !cloudPts) return;
  const [x0,y0,z0,x1,y1,z1] = cloudBounds();
  cloudCam.px=(x0+x1)/2; cloudCam.py=(y0+y1)/2; cloudCam.pz=(z0+z1)/2;
  cloudCam.pitch = 1.5; cloudCam.yaw = 0;
  cloudCam.dist = Math.max(0.6, Math.hypot(x1-x0, z1-z0) * 1.1);
  cloudDraw();
}

/**
 * Rebuilds the robot marker from the fused pose: an arrow on the floor plus a
 * post, so it reads from above and from the side alike.
 *
 * The fused pose is in the robot's z-up world and the cloud is in ARCore's
 * y-up world. PoseFusion maps camera to robot as x = -z_ar, y = -x_ar, so the
 * inverse used here is x_ar = -y, z_ar = -x; a heading of zero faces down
 * ARCore's -z, which is where the camera looked at startup.
 */
function updateMarker() {
  const f = latest['fusion.pose'];
  if (!f || !cgl) return;
  const p = f.pose;
  if (markerPose && Math.abs(markerPose.x - p.x) < 0.01 &&
      Math.abs(markerPose.y - p.y) < 0.01 &&
      Math.abs(markerPose.theta - p.theta) < 0.01) return;
  markerPose = { x: p.x, y: p.y, theta: p.theta };

  const ax = -p.y, az = -p.x, ay = cloudFloorY + 0.03;
  const fx = -Math.sin(p.theta), fz = -Math.cos(p.theta);   // forward, y-up frame
  const rx = -fz, rz = fx;                                  // right = forward × up

  // A solid arrow, not lines: WebGL line width is 1 px almost everywhere, and
  // a hairline marker disappears into a 70,000-point cloud. Roughly the
  // robot's own footprint, so it also communicates scale.
  const apex = [ax + fx * 0.45, ay, az + fz * 0.45];
  const left = [ax - fx * 0.18 + rx * 0.20, ay, az - fz * 0.18 + rz * 0.20];
  const right = [ax - fx * 0.18 - rx * 0.20, ay, az - fz * 0.18 - rz * 0.20];
  const peak = [ax, ay + 0.45, az];                         // a low pyramid, visible side-on

  const C = [0.30, 0.64, 1.0];                              // the console accent
  const W = [0.85, 0.93, 1.0];                              // apex, near-white
  const v = (pt, c) => [pt[0], pt[1], pt[2], ...c];
  const arr = new Float32Array([
    ...v(apex, W), ...v(left, C), ...v(right, C),           // floor arrow
    ...v(apex, W), ...v(peak, C), ...v(left, C),            // pyramid faces
    ...v(apex, W), ...v(right, C), ...v(peak, C),
    ...v(left, C), ...v(peak, C), ...v(right, C),
  ]);
  markerVerts = arr.length / 6;
  if (!markerBuf) markerBuf = cgl.createBuffer();
  cgl.bindBuffer(cgl.ARRAY_BUFFER, markerBuf);
  cgl.bufferData(cgl.ARRAY_BUFFER, arr, cgl.DYNAMIC_DRAW);
}

function cloudDraw() {
  if (!cgl || !cloudPts) return;
  const cv = el('gl3d');
  const ratio = window.devicePixelRatio || 1;
  const w = Math.round(cv.clientWidth * ratio), h = Math.round(cv.clientHeight * ratio);
  if (!w || !h) return;
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }

  cgl.viewport(0, 0, cv.width, cv.height);
  cgl.clearColor(0.02, 0.028, 0.04, 1);
  cgl.clear(cgl.COLOR_BUFFER_BIT | cgl.DEPTH_BUFFER_BIT);
  cgl.useProgram(cloudProg);

  const pos = cgl.getAttribLocation(cloudProg, 'pos');
  const col = cgl.getAttribLocation(cloudProg, 'col');
  cgl.bindBuffer(cgl.ARRAY_BUFFER, cloudBuf);
  cgl.enableVertexAttribArray(pos);
  cgl.vertexAttribPointer(pos, 3, cgl.FLOAT, false, 24, 0);
  cgl.enableVertexAttribArray(col);
  cgl.vertexAttribPointer(col, 3, cgl.FLOAT, false, 24, 12);

  cgl.uniformMatrix4fv(cgl.getUniformLocation(cloudProg, 'mvp'), false, cloudMvp());
  cgl.uniform1f(cgl.getUniformLocation(cloudProg, 'size'),
                3.5 * ratio * (heroView === 'map3d' ? 1 : 0.5));
  cgl.drawArrays(cgl.POINTS, 0, cloudPts);

  // The robot, on top of the cloud it is standing in. Same shader, same
  // attribute layout; triangles just ignore gl_PointSize.
  if (markerBuf && markerVerts) {
    cgl.bindBuffer(cgl.ARRAY_BUFFER, markerBuf);
    cgl.vertexAttribPointer(pos, 3, cgl.FLOAT, false, 24, 0);
    cgl.vertexAttribPointer(col, 3, cgl.FLOAT, false, 24, 12);
    cgl.drawArrays(cgl.TRIANGLES, 0, markerVerts);
  }
}

function cloudPan(dx, dy) {
  const e = cloudEye();
  let fx = cloudCam.px-e[0], fy = cloudCam.py-e[1], fz = cloudCam.pz-e[2];
  const fl = Math.hypot(fx,fy,fz)||1; fx/=fl; fy/=fl; fz/=fl;
  let rx = -fz, rz = fx;
  const rl = Math.hypot(rx,rz)||1; rx/=rl; rz/=rl;
  const ux = fy*rz, uy = fz*rx - fx*rz, uz = -fy*rx;
  const s = cloudCam.dist * 0.0022;
  cloudCam.px += (-rx*dx + ux*dy) * s;
  cloudCam.py += (uy*dy) * s;
  cloudCam.pz += (-rz*dx + uz*dy) * s;
}

function wireCloud() {
  // Gestures: drag rotates, two fingers or shift pans, pinch or wheel zooms.
  // Bound to the canvas, which only receives input while the 3D map is the
  // hero — as a thumbnail its children have pointer-events: none.
  const ptrs = new Map();
  let pinch0 = 0, lastPt = null;
  const cv = el('gl3d');

  cv.addEventListener('pointerdown', (e) => {
    ptrs.set(e.pointerId, {x:e.clientX, y:e.clientY});
    lastPt = {x:e.clientX, y:e.clientY};
    cv.setPointerCapture(e.pointerId);
  });
  cv.addEventListener('pointerup', (e) => { ptrs.delete(e.pointerId); pinch0 = 0; });
  cv.addEventListener('pointercancel', (e) => { ptrs.delete(e.pointerId); pinch0 = 0; });
  cv.addEventListener('pointermove', (e) => {
    if (!ptrs.has(e.pointerId) || !lastPt) return;
    ptrs.set(e.pointerId, {x:e.clientX, y:e.clientY});
    if (ptrs.size >= 2) {
      const p = [...ptrs.values()];
      const d = Math.hypot(p[0].x-p[1].x, p[0].y-p[1].y);
      if (pinch0) cloudCam.dist = Math.max(0.1, Math.min(120, cloudCam.dist * (pinch0/d)));
      pinch0 = d;
      const mx=(p[0].x+p[1].x)/2, my=(p[0].y+p[1].y)/2;
      cloudPan(mx-lastPt.x, my-lastPt.y);
      lastPt = {x:mx, y:my};
      cloudDraw(); e.preventDefault(); return;
    }
    const dx = e.clientX-lastPt.x, dy = e.clientY-lastPt.y;
    lastPt = {x:e.clientX, y:e.clientY};
    if (e.shiftKey || e.buttons === 2 || e.buttons === 4) cloudPan(dx, dy);
    else {
      cloudCam.yaw -= dx*0.006;
      // Stop just short of the poles, where the up vector degenerates and the
      // view flips over.
      cloudCam.pitch = Math.max(-1.5, Math.min(1.5, cloudCam.pitch + dy*0.006));
    }
    cloudDraw(); e.preventDefault();
  });
  cv.addEventListener('wheel', (e) => {
    cloudCam.dist = Math.max(0.1, Math.min(120, cloudCam.dist * (e.deltaY > 0 ? 1.1 : 0.9)));
    cloudDraw(); e.preventDefault();
  }, { passive: false });
  cv.addEventListener('contextmenu', (e) => e.preventDefault());

  el('fitBtn').onclick = cloudFit;
  el('topBtn').onclick = cloudTopView;
  el('liveBtn').onclick = () => {
    cloudLive = !cloudLive;
    el('liveBtn').classList.toggle('on', cloudLive);
  };

  cloudLoad();
  // Depth produces a frame every few seconds, so polling faster would mostly
  // re-fetch the same cloud. A hidden tab skips the fetch entirely.
  setInterval(() => { if (cloudLive && !stopped && !document.hidden) cloudLoad(); }, 3000);
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
  if (!open) disarmQuit();
}

// ── quit ───────────────────────────────────────────────────────────────────

// Two taps, not a confirm() dialog. confirm() blocks the event loop, which stops
// the drive repeat and the socket pump along with it — the last thing wanted
// while a robot is moving — and on a phone it is a system modal that can land
// under the notification shade. Arming in place asks the same question without
// leaving the page.
let quitArmTimer = null;

function disarmQuit() {
  clearTimeout(quitArmTimer);
  quitArmTimer = null;
  const b = el('quitBtn');
  if (b) { b.classList.remove('armed'); b.textContent = 'quit app'; }
}

function quitApp() {
  const b = el('quitBtn');
  if (!quitArmTimer) {
    b.classList.add('armed');
    b.textContent = 'tap again to quit';
    // Long enough to read and act on, short enough that a stray tap does not
    // leave the console armed indefinitely.
    quitArmTimer = setTimeout(disarmQuit, 4000);
    return;
  }
  clearTimeout(quitArmTimer);
  quitArmTimer = null;
  release();
  b.classList.remove('armed');
  b.textContent = 'stopping…';
  // The reply arrives before the teardown starts; the socket dying afterwards is
  // the expected end, not a failure, so the banner says so rather than letting
  // the reconnect logic report a lost robot.
  fetch('/api/shutdown', { method: 'POST' })
    .then(() => { stopped = true; el('quitBtn').textContent = 'stopped'; })
    .catch(() => { b.textContent = 'quit failed'; });
}

function wireButtons() {
  el('motorsBtn').onclick = toggleMotors;
  el('estopBtn').onclick = emergencyStop;
  el('infoBtn').onclick = () => showDrawer(true);
  el('closeBtn').onclick = () => showDrawer(false);
  el('quitBtn').onclick = quitApp;

  // Clicking the backdrop closes, but only the backdrop: a click that started on
  // a panel must not dismiss it, or selecting a value becomes a fight.
  el('drawer').addEventListener('mousedown', (e) => {
    if (e.target === el('drawer')) showDrawer(false);
  });
}

// ── video ──────────────────────────────────────────────────────────────────

function startVideo() {
  const img = el('video');
  img.onerror = () => {
    videoAlive = false;
    if (!stopped) setTimeout(startVideo, 2000);
  };
  // Cache-busted so a reconnect starts a new multipart stream rather than
  // resuming a dead one.
  img.src = `/video.mjpeg?t=${Date.now()}`;
}

// Leaving the page must not leave the robot driving.
document.addEventListener('visibilitychange', () => { if (document.hidden) release(); });
window.addEventListener('pagehide', release);
window.addEventListener('resize', () => { drawRadar(); drawStick(); cloudDraw(); });

wireStick();
wireKeyboard();
wireButtons();
wireViews();
wireCloud();
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
