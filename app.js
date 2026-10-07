(function () {
'use strict';

// ---------- Config (tune with real data) ----------
const CFG = {
  hit: 0.04,            // m, radius to touch start/end spheres
  pauseSpeed: 0.03,     // m/s, below this = hesitating
  pauseMin: 0.3,        // s, minimum length to count as a pause
  maxTime: 30,          // s, attempt timeout
  // Shoulder estimate relative to head, in the head's yaw-only frame (x right, y up, z back)
  shoulder: { x: 0.18, y: -0.22, z: 0.05 }
};
// Sample columns
const C = { T:0, WX:1, WY:2, WZ:3, HX:4, HY:5, HZ:6, QX:7, QY:8, QZ:9, QW:10, YAW:11, RX:12, RY:13, RZ:14, ELB:15, REACH:16, CLAMP:17 };
const HEADER = 't,wx,wy,wz,hx,hy,hz,hqx,hqy,hqz,hqw,yaw_deg,rx,ry,rz,elbow_deg,reach_frac,sh_clamped';

const $ = id => document.getElementById(id);
const Y = new THREE.Vector3(0, 1, 0);
const view = $('view');

// ---------- Scene ----------
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setClearColor(0x0f1114);
renderer.xr.enabled = true;
view.appendChild(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(60, 1, 0.01, 20);
camera.position.set(0, 1.2, 0.35);
camera.lookAt(0, 1.2, -0.45);

const pts = [];
for (let i = 0; i <= 8; i++) { const t = i / 8; pts.push(new THREE.Vector3(-0.25 + 0.5 * t, 1.2 + 0.08 * Math.sin(2 * Math.PI * t), -0.45)); }
const curve = new THREE.CatmullRomCurve3(pts);
const dense = curve.getPoints(200);
const A = dense[0].clone(), B = dense[200].clone();
const mk = (g, c) => new THREE.Mesh(g, new THREE.MeshBasicMaterial({ color: c }));
scene.add(mk(new THREE.TubeGeometry(curve, 80, 0.006, 8, false), 0x7aa7e0));
const startM = mk(new THREE.SphereGeometry(0.025, 20, 20), 0x43a047); startM.position.copy(A); scene.add(startM);
const endM = mk(new THREE.SphereGeometry(0.025, 20, 20), 0xe53935); endM.position.copy(B); scene.add(endM);
const cursor = mk(new THREE.SphereGeometry(0.012, 16, 16), 0x00e5ff); scene.add(cursor);

// ---------- Inputs ----------
const hands = [], ctrls = [];
for (let i = 0; i < 2; i++) {
  const h = renderer.xr.getHand(i), c = renderer.xr.getController(i);
  scene.add(h); scene.add(c);
  h.addEventListener('connected', e => { h.userData.h = e.data.handedness; h.userData.on = !!e.data.hand; });
  h.addEventListener('disconnected', () => { h.userData.on = false; });
  c.addEventListener('connected', e => { c.userData.h = e.data.handedness; });
  hands.push(h); ctrls.push(c);
}
const ray = new THREE.Raycaster(), plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0.45), ndc = new THREE.Vector2();
let mouse = null;
renderer.domElement.addEventListener('pointermove', e => {
  const r = renderer.domElement.getBoundingClientRect();
  ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
  ray.setFromCamera(ndc, camera);
  const v = new THREE.Vector3();
  mouse = ray.ray.intersectPlane(plane, v) ? v : null;
});
renderer.domElement.addEventListener('pointerleave', () => { mouse = null; });

const tmp = new THREE.Vector3(), hp = new THREE.Vector3(), hq = new THREE.Quaternion();
function getPoint() {
  if (renderer.xr.isPresenting) {
    const side = $('side').value;
    for (const h of hands) {
      const j = h.joints && h.joints['wrist'];
      if (h.userData.on && h.userData.h === side && j && j.visible) { j.getWorldPosition(tmp); return tmp.clone(); }
    }
    for (const c of ctrls) if (c.userData.h === side) { c.getWorldPosition(tmp); return tmp.clone(); }
    return null;
  }
  return mouse;
}
function getHead() {
  const cam = renderer.xr.isPresenting ? renderer.xr.getCamera(camera) : camera;
  cam.getWorldPosition(hp); cam.getWorldQuaternion(hq);
}

// ---------- Capture ----------
let state = 'idle', samples = [], t0 = 0, coolUntil = 0, yaw0 = 0, lastCSV = '', lastFeatures = null;
const setStatus = s => { $('status').textContent = s; };
const setCol = (m, c) => m.material.color.setHex(c);
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

function record(p, t) {
  getHead();
  const f = new THREE.Vector3(0, 0, -1).applyQuaternion(hq);
  const yaw = Math.atan2(-f.x, -f.z);
  if (!samples.length) yaw0 = yaw;
  const dyaw = ((yaw - yaw0 + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
  const rel = p.clone().sub(hp).applyAxisAngle(Y, -yaw);          // wrist in head yaw frame
  const sign = $('side').value === 'right' ? 1 : -1;
  const sh = new THREE.Vector3(sign * CFG.shoulder.x, CFG.shoulder.y, CFG.shoulder.z).applyAxisAngle(Y, yaw).add(hp);
  const a = +$('upper').value / 100, fa = +$('fore').value / 100;
  const d = p.distanceTo(sh);
  const dc = clamp(d, Math.abs(a - fa) + 1e-3, a + fa);
  const elbow = Math.acos(clamp((a * a + fa * fa - dc * dc) / (2 * a * fa), -1, 1)) * 180 / Math.PI;
  samples.push([t - t0, p.x, p.y, p.z, hp.x, hp.y, hp.z, hq.x, hq.y, hq.z, hq.w,
    dyaw * 180 / Math.PI, rel.x, rel.y, rel.z, elbow, Math.min(1, d / (a + fa)), d > a + fa ? 1 : 0]);
}

function step(p, t) {
  if (state === 'idle' && p.distanceTo(A) < CFG.hit) {
    state = 'rec'; samples = []; t0 = t; setCol(startM, 0xffd54f); setStatus('Recording… trace to the red sphere.');
  }
  if (state === 'rec') {
    record(p, t);
    const dur = t - t0;
    if (dur > 0.5 && p.distanceTo(B) < CFG.hit && samples.length > 15) finish(t);
    else if (dur > CFG.maxTime) { state = 'idle'; setCol(startM, 0x43a047); setStatus('Timed out. Return to the green sphere to retry.'); }
  }
  if (state === 'cool' && t > coolUntil) {
    state = 'idle'; setCol(startM, 0x43a047); setCol(endM, 0xe53935);
    setStatus('Ready. Move to the green sphere to start again.');
  }
}

// ---------- Metrics ----------
const col = (s, i) => s.map(r => r[i]);
const mean = a => a.reduce((x, y) => x + y, 0) / a.length;
const avgAt = (s, idx, from, to) => [C[idx[0]], C[idx[1]], C[idx[2]]].map(c => mean(s.slice(from, to).map(r => r[c])));
function pathLen(arr) {
  let L = 0;
  for (let i = 1; i < arr.length; i++) { const d = Math.hypot(arr[i][0] - arr[i-1][0], arr[i][1] - arr[i-1][1], arr[i][2] - arr[i-1][2]); if (d > 0.001) L += d; }
  return L;
}

function metrics(s) {
  const n = s.length, T = s[n-1][C.T] - s[0][C.T];
  // Wrist kinematics (smoothed with a 5-sample moving average)
  const P = s.map((r, i) => {
    let x = 0, y = 0, z = 0, k0 = Math.max(0, i - 2), k1 = Math.min(n - 1, i + 2);
    for (let k = k0; k <= k1; k++) { x += s[k][C.WX]; y += s[k][C.WY]; z += s[k][C.WZ]; }
    const c = k1 - k0 + 1; return [x / c, y / c, z / c];
  });
  let L = 0; const sp = [];
  for (let i = 1; i < n; i++) {
    const d = Math.hypot(P[i][0]-P[i-1][0], P[i][1]-P[i-1][1], P[i][2]-P[i-1][2]);
    L += d; sp.push(d / Math.max(1e-3, s[i][C.T] - s[i-1][C.T]));
  }
  const straight = Math.hypot(P[n-1][0]-P[0][0], P[n-1][1]-P[0][1], P[n-1][2]-P[0][2]);
  let J = 0;
  for (let i = 3; i < n; i++) {
    const dt = (s[i][C.T] - s[i-3][C.T]) / 3; if (dt <= 0) continue;
    for (let a = 0; a < 3; a++) { const j = (P[i][a] - 3*P[i-1][a] + 3*P[i-2][a] - P[i-3][a]) / (dt*dt*dt); J += j*j*dt; }
  }
  let slow = 0, pauses = 0, run = 0;
  for (let i = 1; i < n; i++) {
    const dt = s[i][C.T] - s[i-1][C.T];
    if (sp[i-1] < CFG.pauseSpeed) { slow += dt; run += dt; } else { if (run > CFG.pauseMin) pauses++; run = 0; }
  }
  if (run > CFG.pauseMin) pauses++;
  let eS = 0, eM = 0;
  for (const r of s) {
    let m = 1e9;
    for (const q of dense) { const d = Math.hypot(r[C.WX]-q.x, r[C.WY]-q.y, r[C.WZ]-q.z); if (d < m) m = d; }
    eS += m; if (m > eM) eM = m;
  }

  // Compensation features
  const w0 = avgAt(s, ['WX','WY','WZ'], 0, 3), w1 = avgAt(s, ['WX','WY','WZ'], n - 3, n);
  const h0 = avgAt(s, ['HX','HY','HZ'], 0, 3), h1 = avgAt(s, ['HX','HY','HZ'], n - 3, n);
  const W = w1.map((v, i) => v - w0[i]), Bv = h1.map((v, i) => v - h0[i]);
  const Wn = Math.hypot(...W) || 1e-6;
  const trunkNet = (Bv[0]*W[0] + Bv[1]*W[1] + Bv[2]*W[2]) / (Wn * Wn);  // share of net wrist displacement due to head movement
  const headPts = s.map(r => [r[C.HX], r[C.HY], r[C.HZ]]);
  const relPts = s.map(r => [r[C.WX]-r[C.HX], r[C.WY]-r[C.HY], r[C.WZ]-r[C.HZ]]);
  const Lh = pathLen(headPts), Lr = pathLen(relPts);
  let headRange = 0, yawRange = 0;
  for (const r of s) {
    headRange = Math.max(headRange, Math.hypot(r[C.HX]-s[0][C.HX], r[C.HY]-s[0][C.HY], r[C.HZ]-s[0][C.HZ]));
    yawRange = Math.max(yawRange, Math.abs(r[C.YAW]));
  }
  const elb = col(s, C.ELB), reach = col(s, C.REACH), k = Math.min(5, n);
  return {
    dur: T, len: L, eff: L > 0 ? straight / L : 0, mean: L / T, peak: Math.max(...sp),
    jerk: Math.log10(Math.sqrt(0.5 * J * Math.pow(T, 5) / (L * L))),
    hes: slow / T, pauses, eMean: eS / n, eMax: eM,
    leanShare: Lh / ((Lh + Lr) || 1e-6), trunkNet, headRange, yawRange,
    elbowStart: mean(elb.slice(0, k)), elbowEnd: mean(elb.slice(-k)),
    elbowRange: Math.max(...elb) - Math.min(...elb),
    reachMean: mean(reach), reachEnd: mean(reach.slice(-k)),
    clampPct: mean(col(s, C.CLAMP))
  };
}

// ---------- Storage + display ----------
let hist = [];
try { hist = JSON.parse(localStorage.getItem('mq_hist_v2') || '[]'); } catch (e) { hist = []; }
const f = (v, d) => v.toFixed(d);
const dl = (a, b, d, u) => b == null ? '' : ((a - b >= 0 ? '+' : '') + f(a - b, d) + u);

function finish(t) {
  state = 'cool'; coolUntil = t + 2.5; setCol(startM, 0x43a047); setCol(endM, 0x43a047);
  const m = metrics(samples);
  m.id = (hist.length ? hist[hist.length - 1].id : 0) + 1;
  m.side = $('side').value; m.limb = $('limb').value; m.when = new Date().toISOString();
  m.upperCm = +$('upper').value; m.foreCm = +$('fore').value;
  const prev = hist.length ? hist[hist.length - 1] : null;
  hist.push(m); if (hist.length > 50) hist.shift();
  try { localStorage.setItem('mq_hist_v2', JSON.stringify(hist)); } catch (e) {}
  lastCSV = HEADER + '\n' + samples.map(r => r.map((v, i) => v.toFixed(i === 0 ? 3 : 4)).join(',')).join('\n');
  lastFeatures = m;
  show(m, prev);
  setStatus('Attempt ' + m.id + ' complete (' + f(m.dur, 1) + ' s). Resets in a moment.');
  document.dispatchEvent(new CustomEvent('mq:attempt', { detail: { samples: samples.slice(), metrics: m, csv: lastCSV } }));
}

function show(m, p) {
  const g = t => ['__grp', t];
  const rows = [
    g('Task performance'),
    ['Completion time', f(m.dur, 2) + ' s', p && dl(m.dur, p.dur, 2, ' s')],
    ['Path efficiency', f(m.eff * 100, 0) + ' %', p && dl(m.eff * 100, p.eff * 100, 0, ' %')],
    ['Deviation from path (mean / max)', f(m.eMean * 100, 1) + ' / ' + f(m.eMax * 100, 1) + ' cm', p && dl(m.eMean * 100, p.eMean * 100, 1, ' cm')],
    ['Mean / peak speed', f(m.mean * 100, 1) + ' / ' + f(m.peak * 100, 1) + ' cm/s', ''],
    g('Smoothness and hesitation'),
    ['Log jerk (lower = smoother)', f(m.jerk, 2), p && dl(m.jerk, p.jerk, 2, '')],
    ['Time hesitating', f(m.hes * 100, 0) + ' % (' + m.pauses + ' pauses)', p && dl(m.hes * 100, p.hes * 100, 0, ' %')],
    g('Arm use (estimated)'),
    ['Elbow angle start → end (180° = straight)', f(m.elbowStart, 0) + '° → ' + f(m.elbowEnd, 0) + '°', ''],
    ['Elbow extension range', f(m.elbowRange, 0) + '°', p && dl(m.elbowRange, p.elbowRange, 0, '°')],
    ['Reach (fraction of arm length), mean / end', f(m.reachMean, 2) + ' / ' + f(m.reachEnd, 2), ''],
    g('Body movement'),
    ['Lean share of movement (head path vs wrist-to-head path)', f(m.leanShare * 100, 0) + ' %', p && dl(m.leanShare * 100, p.leanShare * 100, 0, ' %')],
    ['Head contribution to net wrist displacement', f(m.trunkNet * 100, 0) + ' %', p && dl(m.trunkNet * 100, p.trunkNet * 100, 0, ' %')],
    ['Head displacement (max) / head turn (max)', f(m.headRange * 100, 1) + ' cm / ' + f(m.yawRange, 0) + '°', '']
  ];
  $('res').innerHTML = '<tr><th>Metric</th><th>Value</th><th>vs previous</th></tr>' + rows.map(r =>
    r[0] === '__grp' ? '<tr class="grp"><td colspan="3">' + r[1] + '</td></tr>'
      : '<tr><td>' + r[0] + '</td><td>' + r[1] + '</td><td>' + (r[2] || '') + '</td></tr>').join('');
  $('warn').textContent = m.clampPct > 0.2
    ? 'Warning: in ' + f(m.clampPct * 100, 0) + '% of frames the wrist was beyond the estimated arm length. The shoulder estimate or arm lengths are probably off, so treat elbow and reach values with caution.' : '';
  $('hist').innerHTML = '<tr><th>#</th><th>Side</th><th>Limb</th><th>Time</th><th>Dev.</th><th>Elbow range</th><th>Lean</th></tr>' +
    hist.slice(-8).reverse().map(h => '<tr><td>' + h.id + '</td><td>' + h.side + '</td><td>' + h.limb + '</td><td>' + f(h.dur, 1) + ' s</td><td>' +
      f(h.eMean * 100, 1) + ' cm</td><td>' + f(h.elbowRange, 0) + '°</td><td>' + f(h.leanShare * 100, 0) + '%</td></tr>').join('');
}
if (hist.length) show(hist[hist.length - 1], hist.length > 1 ? hist[hist.length - 2] : null);

// ---------- UI ----------
function download(name, text, type) {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], { type }));
  a.download = name; document.body.appendChild(a); a.click(); a.remove();
}
$('dlcsv').onclick = () => lastCSV ? download('attempt_' + lastFeatures.id + '_raw.csv', lastCSV, 'text/csv') : setStatus('Nothing to export yet.');
$('dljson').onclick = () => hist.length ? download('features.json', JSON.stringify(hist, null, 2), 'application/json') : setStatus('Nothing to export yet.');
$('clr').onclick = () => {
  hist = []; try { localStorage.removeItem('mq_hist_v2'); } catch (e) {}
  $('res').innerHTML = $('hist').innerHTML = '<tr><td>No attempts yet.</td></tr>'; $('warn').textContent = '';
};
$('vr').onclick = async () => {
  if (!navigator.xr) { setStatus('WebXR not available. Use the Quest Browser over HTTPS or localhost.'); return; }
  try {
    if (!(await navigator.xr.isSessionSupported('immersive-vr'))) { setStatus('Immersive VR not supported on this device/browser.'); return; }
    const s = await navigator.xr.requestSession('immersive-vr', { optionalFeatures: ['local-floor', 'hand-tracking'] });
    renderer.xr.setReferenceSpaceType('local-floor');
    await renderer.xr.setSession(s);
    state = 'idle'; setStatus('In VR. Touch the green sphere with your wrist to start.');
    s.addEventListener('end', () => { setStatus('Exited VR. Results are shown below.'); resize(); });
  } catch (e) { setStatus('Could not start VR: ' + e.message); }
};
function resize() {
  if (renderer.xr.isPresenting) return;
  const w = view.clientWidth, h = view.clientHeight;
  renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix();
}
addEventListener('resize', resize); resize();

renderer.setAnimationLoop(time => {
  const p = getPoint();
  if (p) { cursor.visible = true; cursor.position.copy(p); step(p, time / 1000); } else cursor.visible = false;
  renderer.render(scene, camera);
});
})();
