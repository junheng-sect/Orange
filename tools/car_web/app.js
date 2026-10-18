'use strict';
// 小车网页控制台，阶段 1：只读显示。
// 浏览器直接连车端 rosbridge（Melodic，rosbridge_server 0.11.16），只订阅话题、调只读服务，不发任何指令。
// 默认走 Xiaomi 网的 192.168.31.231；连车热点时用 ?ws=ws://10.7.5.88:9090。

const WS_URL = new URLSearchParams(location.search).get('ws') || 'ws://192.168.31.231:9090';
// base_footprint→scan 只有 z=0.2（2026-10-18 实测 /tf），平面上和车体原点重合。
// /tracked_pose 就是 map→base_footprint，激光直接按它投到地图上。
const POSE_STALE_MS = 2000;

// ---------- rosbridge 客户端 ----------
class Rosbridge {
  constructor(url, onState) {
    this.url = url;
    this.onState = onState;
    this.subs = new Map();     // topic → {type, cb, throttle}
    this.pending = new Map();  // id → {resolve, reject, timer}
    this.seq = 0;
    this.connect();
  }

  connect() {
    this.onState('connecting');
    const ws = new WebSocket(this.url);
    this.ws = ws;
    ws.onopen = () => {
      this.onState('open');
      for (const [topic, s] of this.subs) this.sendSub(topic, s);
    };
    ws.onclose = () => {
      this.onState('closed');
      for (const p of this.pending.values()) {
        clearTimeout(p.timer);
        p.reject(new Error('连接断开'));
      }
      this.pending.clear();
      setTimeout(() => this.connect(), 2000);
    };
    ws.onmessage = (ev) => this.onMessage(parseJson(ev.data));
  }

  onMessage(m) {
    if (m.op === 'publish') {
      const s = this.subs.get(m.topic);
      if (s) s.cb(m.msg);
    } else if (m.op === 'service_response') {
      const p = this.pending.get(m.id);
      if (!p) return;
      this.pending.delete(m.id);
      clearTimeout(p.timer);
      if (m.result === false) p.reject(new Error(String(m.values)));
      else p.resolve(m.values);
    }
  }

  send(obj) {
    if (this.ws.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(obj));
  }

  sendSub(topic, s) {
    this.send({ op: 'subscribe', id: 'sub:' + topic, topic, type: s.type,
                throttle_rate: s.throttle, queue_length: 1 });
  }

  // 每个话题只订一次：rosbridge 对同一客户端同一话题只推一份消息。
  subscribe(topic, type, cb, throttle = 0) {
    const s = { type, cb, throttle };
    this.subs.set(topic, s);
    this.sendSub(topic, s);
  }

  unsubscribe(topic) {
    if (!this.subs.delete(topic)) return;
    this.send({ op: 'unsubscribe', id: 'sub:' + topic, topic });
  }

  call(service, args = {}, timeoutMs = 5000) {
    return new Promise((resolve, reject) => {
      if (this.ws.readyState !== WebSocket.OPEN) return reject(new Error('未连接'));
      const id = 'call:' + (++this.seq);
      const timer = setTimeout(() => {
        this.pending.delete(id);
        reject(new Error(service + ' 超时'));
      }, timeoutMs);
      this.pending.set(id, { resolve, reject, timer });
      this.send({ op: 'call_service', id, service, args });
    });
  }
}

// rosbridge 0.11 用 Python json 序列化，激光里的 inf/nan 会变成 JSON 不认的 Infinity/NaN。
function parseJson(text) {
  try {
    return JSON.parse(text);
  } catch (e) {
    return JSON.parse(text.replace(/-?\bInfinity\b|\bNaN\b/g, 'null'));
  }
}

// ---------- 状态 ----------
const st = {
  map: null,        // {info, canvas, key}
  costmap: null,
  pose: null,       // {x, y, yaw, t}
  scan: null,
  path: [],         // [[x, y]]
  footprint: [],    // [{x, y, r}] 或 [{poly: [[x, y]]}]
  points: [],       // [{name, x, y, sys}]
  trail: [],        // [[x, y]]
  nav: null,        // {x, y, status}
};
const view = { scale: 80, rot: 0, ox: 0, oy: 0, follow: false, fitted: false };

const LAYERS = {
  scan:      { label: '激光', def: true },
  path:      { label: '规划路径', def: true },
  footprint: { label: '车身轮廓', def: true },
  trail:     { label: '轨迹', def: true },
  points:    { label: '导航点', def: true },
  costmap:   { label: '局部代价图', def: false },
  score:     { label: '定位分数', note: '860 Hz 话题，车端开销大', def: false },
};
const layers = loadPrefs('car_web.layers', Object.fromEntries(Object.entries(LAYERS).map(([k, v]) => [k, v.def])));
view.rot = loadPrefs('car_web.rot', 0);

function loadPrefs(key, def) {
  try {
    const v = JSON.parse(localStorage.getItem(key));
    return v === null ? def : (typeof def === 'object' ? { ...def, ...v } : v);
  } catch (e) {
    return def;
  }
}

function savePrefs(key, v) {
  try { localStorage.setItem(key, JSON.stringify(v)); } catch (e) { /* 隐私模式等情况下存不了，忽略 */ }
}

// ---------- 订阅 ----------
const ros = new Rosbridge(WS_URL, onConnState);

ros.subscribe('/map', 'nav_msgs/OccupancyGrid', onMap, 1000);
ros.subscribe('/tracked_pose', 'geometry_msgs/PoseStamped', onPose);
ros.subscribe('/clean_pnc/global_planner/plan', 'nav_msgs/Path', onPath, 500);
ros.subscribe('/clean_pnc/cost_map/footprint', 'visualization_msgs/MarkerArray', onFootprint, 500);
ros.subscribe('/navigation_status', 'map_msgs/NavigationStatusResult', onNav);
ros.subscribe('/power_report', 'std_msgs/Int16', onPower);
ros.subscribe('/location_status', 'std_msgs/Int16', onLoc);
ros.subscribe('/raw_odom', 'nav_msgs/Odometry', onOdom, 200);
applyLayerSubs();

// 激光、代价图、定位分数数据量大，只在图层打开时订阅。
function applyLayerSubs() {
  toggleSub(layers.scan, '/scan', 'sensor_msgs/LaserScan', onScan, 200);
  toggleSub(layers.costmap, '/clean_pnc/cost_map/local_costmap', 'nav_msgs/OccupancyGrid', onCostmap, 500);
  toggleSub(layers.score, '/local_global_score', 'std_msgs/Float64MultiArray', onScore, 500);
  if (!layers.scan) st.scan = null;
  if (!layers.costmap) st.costmap = null;
  $('c-score').hidden = !layers.score;
}

function toggleSub(on, topic, type, cb, throttle) {
  if (!on) ros.unsubscribe(topic);
  else if (!ros.subs.has(topic)) ros.subscribe(topic, type, cb, throttle);
}

// ---------- 消息处理 ----------
function yawOf(q) {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

function onConnState(s) {
  if (s === 'open') setChip('c-conn', '已连接', 'ok');
  else if (s === 'connecting') setChip('c-conn', '连接中', 'warn');
  else setChip('c-conn', '断开，重连中', 'bad');
  if (s === 'open') loadPoints();
}

function onMap(m) {
  const i = m.info;
  const key = [i.width, i.height, i.resolution, i.origin.position.x, i.origin.position.y].join(',');
  const changed = !st.map || st.map.key !== key;
  st.map = { ...gridToImage(m, mapColor), key };
  if (changed) {
    loadPoints();
    fitView();
  }
  redraw();
}

function onCostmap(m) {
  st.costmap = gridToImage(m, costColor);
  redraw();
}

function onPose(m) {
  const p = m.pose;
  const pose = { x: p.position.x, y: p.position.y, yaw: yawOf(p.orientation), t: performance.now() };
  const last = st.trail[st.trail.length - 1];
  if (!last || Math.hypot(pose.x - last[0], pose.y - last[1]) > 0.03) {
    st.trail.push([pose.x, pose.y]);
    if (st.trail.length > 3000) st.trail.shift();
  }
  st.pose = pose;
  if (view.follow) centerOn(pose.x, pose.y);
  updatePoseChip();
  redraw();
}

function onScan(m) {
  st.scan = m;
  redraw();
}

function onPath(m) {
  st.path = m.poses.map((p) => [p.pose.position.x, p.pose.position.y]);
  redraw();
}

function onFootprint(m) {
  st.footprint = m.markers.filter((k) => k.action !== 2).map((k) => {
    if (k.points && k.points.length) return { poly: k.points.map((q) => [q.x, q.y]) };
    return { x: k.pose.position.x, y: k.pose.position.y, r: k.scale.x / 2 };
  });
  redraw();
}

// status：1 进行中，2 到达。强停后会一直停在 1，没有「取消」状态。
function onNav(m) {
  st.nav = m;
  const text = { 1: '进行中', 2: '已到达' }[m.status] || '状态 ' + m.status;
  setChip('c-nav', text, m.status === 1 ? 'warn' : m.status === 2 ? 'ok' : '');
  redraw();
}

function onPower(m) {
  const v = m.data;
  setChip('c-batt', v + '%', v > 30 ? 'ok' : v > 15 ? 'warn' : 'bad');
}

// 3/13 正常，1 丢失。重启后旧位姿错了也会报正常，真假要看定位分数。
function onLoc(m) {
  const v = m.data;
  if (v === 3 || v === 13) setChip('c-loc', '正常', 'ok');
  else if (v === 1) setChip('c-loc', '丢失', 'bad');
  else setChip('c-loc', '状态 ' + v, 'warn');
}

// /local_global_score[1]：定位对时约 0.86，错时约 0.29。
function onScore(m) {
  const v = m.data[1];
  if (v === undefined || v === null) return;
  setChip('c-score', v.toFixed(2), v >= 0.6 ? 'ok' : v >= 0.4 ? 'warn' : 'bad');
}

function onOdom(m) {
  const t = m.twist.twist;
  setChip('c-vel', `${t.linear.x.toFixed(2)} m/s ${(t.angular.z * 180 / Math.PI).toFixed(0)}°/s`);
}

// 车端每次调用都会 check_map 重读地图 json，所以只在连上和地图变化时调用。
async function loadPoints() {
  try {
    const r = await ros.call('/list_navi_points');
    const conv = (sys) => (p) => ({ name: p.point_name, x: p.x, y: p.y, sys });
    st.points = [...(r.list_navi_points || []).map(conv(false)),
                 ...(r.list_system_points || []).map(conv(true))];
    redraw();
  } catch (e) {
    toast('导航点读取失败：' + e.message);
  }
}

// ---------- 栅格 ----------
function gridToImage(m, color) {
  const { width: w, height: h } = m.info;
  const cv = document.createElement('canvas');
  cv.width = w;
  cv.height = h;
  const g = cv.getContext('2d');
  const img = g.createImageData(w, h);
  for (let i = 0; i < w * h; i++) color(m.data[i], img.data, i * 4);
  g.putImageData(img, 0, 0);
  return { info: m.info, canvas: cv };
}

function mapColor(v, d, k) {
  if (v < 0) {
    d[k] = 112; d[k + 1] = 120; d[k + 2] = 130;
  } else {
    const g = 245 - Math.round(v * 2.15);
    d[k] = d[k + 1] = d[k + 2] = g;
  }
  d[k + 3] = 255;
}

// 代价图是 int8：0 空闲，-1 未知（255），其余按障碍着色。
function costColor(v, d, k) {
  d[k] = 230; d[k + 1] = 40; d[k + 2] = 200;
  d[k + 3] = v === 0 || v === -1 ? 0 : v > 0 ? 20 + v : 140;
}

// ---------- 视图变换 ----------
// 地图 x 朝右、y 朝上，再整体旋转 view.rot。
function w2s(x, y) {
  const c = Math.cos(view.rot), s = Math.sin(view.rot);
  return [view.ox + view.scale * (x * c - y * s), view.oy - view.scale * (x * s + y * c)];
}

function s2w(px, py) {
  const c = Math.cos(view.rot), s = Math.sin(view.rot);
  const xr = (px - view.ox) / view.scale, yr = (view.oy - py) / view.scale;
  return [xr * c + yr * s, -xr * s + yr * c];
}

// 平移视图，让世界点 (wx, wy) 落到屏幕 (px, py)。
function anchor(wx, wy, px, py) {
  const [sx, sy] = w2s(wx, wy);
  view.ox += px - sx;
  view.oy += py - sy;
}

function areaTop() {
  return $('bar').offsetHeight;
}

function centerOn(x, y) {
  const top = areaTop();
  anchor(x, y, W / 2, top + (H - top) / 2);
}

function fitView() {
  if (!st.map) {
    centerOn(st.pose ? st.pose.x : 0, st.pose ? st.pose.y : 0);
    return redraw();
  }
  const i = st.map.info, o = i.origin.position;
  const w = i.width * i.resolution, h = i.height * i.resolution;
  const c = Math.abs(Math.cos(view.rot)), s = Math.abs(Math.sin(view.rot));
  const top = areaTop();
  view.scale = Math.min((W - 40) / (w * c + h * s), (H - top - 40) / (w * s + h * c));
  centerOn(o.x + w / 2, o.y + h / 2);
  view.fitted = true;
  redraw();
}

function clampScale(k) {
  return Math.min(1000, Math.max(5, k));
}

// ---------- 绘制 ----------
const cv = $('map');
const ctx = cv.getContext('2d');
let dpr = 1, W = 0, H = 0, dirty = false;

function resize() {
  dpr = window.devicePixelRatio || 1;
  W = innerWidth;
  H = innerHeight;
  cv.width = W * dpr;
  cv.height = H * dpr;
  cv.style.width = W + 'px';
  cv.style.height = H + 'px';
  if (!view.fitted) fitView();
  redraw();
}

function redraw() {
  if (dirty) return;
  dirty = true;
  requestAnimationFrame(draw);
}

function draw() {
  dirty = false;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = '#1e2227';
  ctx.fillRect(0, 0, W, H);
  if (st.map) drawGrid(st.map);
  if (layers.costmap && st.costmap) drawGrid(st.costmap);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  drawMeterGrid();
  drawAxes();
  if (layers.trail) drawPolyline(st.trail, 'rgba(88,166,255,0.55)', 2, 0.5);
  if (layers.path) drawPolyline(st.path, '#3fb950', 2.5);
  if (layers.points) drawPoints();
  drawGoal();
  if (layers.scan) drawScan();
  if (layers.footprint) drawFootprint();
  drawRobot();
  drawScaleBar();
}

function drawGrid(g) {
  const c = Math.cos(view.rot), s = Math.sin(view.rot), k = view.scale * dpr;
  const o = g.info.origin;
  ctx.setTransform(k * c, -k * s, -k * s, -k * c, dpr * view.ox, dpr * view.oy);
  ctx.translate(o.position.x, o.position.y);
  ctx.rotate(yawOf(o.orientation));
  ctx.scale(g.info.resolution, g.info.resolution);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(g.canvas, 0, 0);
}

// 1 m 网格，范围取屏幕四角对应的世界坐标。
function drawMeterGrid() {
  const corners = [s2w(0, 0), s2w(W, 0), s2w(0, H), s2w(W, H)];
  const xs = corners.map((p) => p[0]), ys = corners.map((p) => p[1]);
  const x0 = Math.floor(Math.min(...xs)), x1 = Math.ceil(Math.max(...xs));
  const y0 = Math.floor(Math.min(...ys)), y1 = Math.ceil(Math.max(...ys));
  if (x1 - x0 > 200 || y1 - y0 > 200) return;
  ctx.strokeStyle = 'rgba(88,166,255,0.18)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let x = x0; x <= x1; x++) line(x, y0, x, y1);
  for (let y = y0; y <= y1; y++) line(x0, y, x1, y);
  ctx.stroke();
}

function line(xa, ya, xb, yb) {
  const [ax, ay] = w2s(xa, ya), [bx, by] = w2s(xb, yb);
  ctx.moveTo(ax, ay);
  ctx.lineTo(bx, by);
}

function drawAxes() {
  ctx.lineWidth = 2;
  for (const [dx, dy, color] of [[0.5, 0, '#f85149'], [0, 0.5, '#3fb950']]) {
    ctx.strokeStyle = color;
    ctx.beginPath();
    line(0, 0, dx, dy);
    ctx.stroke();
  }
}

// breakDist：相邻两点距离超过它就断开（重定位后位姿会跳）。
function drawPolyline(pts, color, width, breakDist = Infinity) {
  if (pts.length < 2) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.lineJoin = 'round';
  ctx.beginPath();
  for (let i = 0; i < pts.length; i++) {
    const [sx, sy] = w2s(pts[i][0], pts[i][1]);
    const jump = i > 0 && Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]) > breakDist;
    if (i === 0 || jump) ctx.moveTo(sx, sy);
    else ctx.lineTo(sx, sy);
  }
  ctx.stroke();
}

function label(text, sx, sy, color) {
  ctx.font = '12px system-ui, "Noto Sans CJK SC", sans-serif';
  ctx.lineWidth = 3;
  ctx.strokeStyle = 'rgba(0,0,0,0.75)';
  ctx.strokeText(text, sx, sy);
  ctx.fillStyle = color;
  ctx.fillText(text, sx, sy);
}

function drawPoints() {
  for (const p of st.points) {
    const [sx, sy] = w2s(p.x, p.y);
    const color = p.sys ? '#58a6ff' : '#f0883e';
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(sx, sy - 6);
    ctx.lineTo(sx + 6, sy);
    ctx.lineTo(sx, sy + 6);
    ctx.lineTo(sx - 6, sy);
    ctx.closePath();
    ctx.fill();
    label(p.name, sx + 8, sy + 4, color);
  }
}

function drawGoal() {
  if (!st.nav || st.nav.status !== 1) return;
  const [sx, sy] = w2s(st.nav.x, st.nav.y);
  ctx.strokeStyle = '#f85149';
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.arc(sx, sy, 9, 0, 2 * Math.PI);
  ctx.stroke();
  label('目标', sx + 11, sy - 8, '#f85149');
}

function drawScan() {
  const sc = st.scan, p = st.pose;
  if (!sc || !p) return;
  ctx.fillStyle = '#ff4d4f';
  for (let i = 0; i < sc.ranges.length; i++) {
    const r = sc.ranges[i];
    if (!(r >= sc.range_min && r <= sc.range_max)) continue;
    const a = p.yaw + sc.angle_min + i * sc.angle_increment;
    const [sx, sy] = w2s(p.x + r * Math.cos(a), p.y + r * Math.sin(a));
    ctx.fillRect(sx - 1, sy - 1, 2, 2);
  }
}

function drawFootprint() {
  ctx.fillStyle = 'rgba(47,129,247,0.18)';
  ctx.strokeStyle = 'rgba(47,129,247,0.75)';
  ctx.lineWidth = 1.5;
  for (const f of st.footprint) {
    ctx.beginPath();
    if (f.poly) {
      f.poly.forEach(([x, y], i) => {
        const [sx, sy] = w2s(x, y);
        if (i === 0) ctx.moveTo(sx, sy);
        else ctx.lineTo(sx, sy);
      });
      ctx.closePath();
    } else {
      const [sx, sy] = w2s(f.x, f.y);
      ctx.arc(sx, sy, Math.max(1, f.r * view.scale), 0, 2 * Math.PI);
    }
    ctx.fill();
    ctx.stroke();
  }
}

function drawRobot() {
  const p = st.pose;
  if (!p) return;
  const stale = performance.now() - p.t > POSE_STALE_MS;
  const c = Math.cos(p.yaw), s = Math.sin(p.yaw);
  const body = [[0.3, 0], [-0.15, 0.15], [-0.15, -0.15]];
  ctx.fillStyle = stale ? '#6e7681' : '#2f81f7';
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  body.forEach(([bx, by], i) => {
    const [sx, sy] = w2s(p.x + bx * c - by * s, p.y + bx * s + by * c);
    if (i === 0) ctx.moveTo(sx, sy);
    else ctx.lineTo(sx, sy);
  });
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
}

function drawScaleBar() {
  const len = [0.1, 0.2, 0.5, 1, 2, 5, 10].find((l) => l * view.scale >= 60) || 10;
  const px = len * view.scale, x = 12, y = H - 16;
  ctx.strokeStyle = '#e6e6e6';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(x, y - 5);
  ctx.lineTo(x, y);
  ctx.lineTo(x + px, y);
  ctx.lineTo(x + px, y - 5);
  ctx.stroke();
  label(len >= 1 ? len + ' m' : len * 100 + ' cm', x + px + 6, y + 4, '#e6e6e6');
}

// ---------- 交互 ----------
const ptrs = new Map();
let pinch = null;

cv.addEventListener('pointerdown', (e) => {
  cv.setPointerCapture(e.pointerId);
  ptrs.set(e.pointerId, [e.clientX, e.clientY]);
  if (ptrs.size === 2) startPinch();
});

cv.addEventListener('pointermove', (e) => {
  showCursor(e.clientX, e.clientY);
  const prev = ptrs.get(e.pointerId);
  if (!prev) return;
  ptrs.set(e.pointerId, [e.clientX, e.clientY]);
  if (ptrs.size === 1) {
    view.ox += e.clientX - prev[0];
    view.oy += e.clientY - prev[1];
    setFollow(false);
    redraw();
  } else if (ptrs.size === 2 && pinch) {
    const [a, b] = [...ptrs.values()];
    const mx = (a[0] + b[0]) / 2, my = (a[1] + b[1]) / 2;
    view.scale = clampScale(pinch.scale * Math.hypot(a[0] - b[0], a[1] - b[1]) / pinch.dist);
    anchor(pinch.wx, pinch.wy, mx, my);
    setFollow(false);
    redraw();
  }
});

for (const type of ['pointerup', 'pointercancel']) {
  cv.addEventListener(type, (e) => {
    ptrs.delete(e.pointerId);
    pinch = null;
    if (ptrs.size === 2) startPinch();
  });
}

function startPinch() {
  const [a, b] = [...ptrs.values()];
  const [wx, wy] = s2w((a[0] + b[0]) / 2, (a[1] + b[1]) / 2);
  pinch = { wx, wy, scale: view.scale, dist: Math.max(1, Math.hypot(a[0] - b[0], a[1] - b[1])) };
}

cv.addEventListener('wheel', (e) => {
  e.preventDefault();
  const [wx, wy] = s2w(e.clientX, e.clientY);
  view.scale = clampScale(view.scale * Math.exp(-e.deltaY * 0.0015));
  anchor(wx, wy, e.clientX, e.clientY);
  if (view.follow && st.pose) centerOn(st.pose.x, st.pose.y);
  redraw();
}, { passive: false });

function showCursor(px, py) {
  const [x, y] = s2w(px, py);
  setChip('c-cur', `${x.toFixed(2)}, ${y.toFixed(2)}`);
}

function setFollow(on) {
  view.follow = on;
  $('b-follow').classList.toggle('on', on);
  if (on && st.pose) {
    centerOn(st.pose.x, st.pose.y);
    redraw();
  }
}

$('b-follow').onclick = () => setFollow(!view.follow);
$('b-fit').onclick = () => {
  setFollow(false);
  fitView();
};
$('b-rot').onclick = () => {
  const top = areaTop();
  const [cx, cy] = s2w(W / 2, top + (H - top) / 2);
  view.rot = (view.rot + Math.PI / 2) % (2 * Math.PI);
  savePrefs('car_web.rot', view.rot);
  centerOn(cx, cy);
  redraw();
};
$('b-layers').onclick = () => $('layers').classList.toggle('open');
$('b-trail').onclick = () => {
  st.trail = [];
  redraw();
};
$('b-points').onclick = loadPoints;

for (const [k, v] of Object.entries(LAYERS)) {
  const lab = document.createElement('label');
  const box = document.createElement('input');
  box.type = 'checkbox';
  box.checked = layers[k];
  box.onchange = () => {
    layers[k] = box.checked;
    savePrefs('car_web.layers', layers);
    applyLayerSubs();
    redraw();
  };
  lab.append(box, ' ' + v.label);
  if (v.note) {
    const note = document.createElement('small');
    note.textContent = '（' + v.note + '）';
    lab.append(note);
  }
  $('layer-list').append(lab);
}

// ---------- 状态栏 ----------
function $(id) {
  return document.getElementById(id);
}

function setChip(id, text, level = '') {
  const el = $(id);
  el.lastChild.textContent = text;
  el.className = 'chip' + (level ? ' ' + level : '');
}

function updatePoseChip() {
  const p = st.pose;
  if (!p) return;
  if (performance.now() - p.t > POSE_STALE_MS) return setChip('c-pose', '位姿超时', 'bad');
  setChip('c-pose', `${p.x.toFixed(2)}, ${p.y.toFixed(2)}, ${(p.yaw * 180 / Math.PI).toFixed(0)}°`, 'ok');
}

let toastTimer = 0;
function toast(text) {
  $('toast').textContent = text;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { $('toast').textContent = ''; }, 6000);
}

// 位姿超时要靠定时检查：车端重定位后约 2.3 s 不发 /tracked_pose。
setInterval(() => {
  updatePoseChip();
  if (st.pose && performance.now() - st.pose.t > POSE_STALE_MS) redraw();
}, 500);

addEventListener('resize', resize);
resize();
