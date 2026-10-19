'use strict';
// 小车网页控制台。浏览器直接连车端 rosbridge（Melodic，rosbridge_server 0.11.16），没有后端。
// 阶段 1：只读显示。阶段 2：停止、暂停/继续、点地图设目标、按点位导航、摇杆；
// 发目标前先用 /get_plan_movebase 规划（不动车），再按车身轮廓做碰撞检查，确认后才发。
// 默认走 Xiaomi 网的 192.168.31.231；连车热点时用 ?ws=ws://10.7.5.88:9090。

const WS_URL = new URLSearchParams(location.search).get('ws') || 'ws://192.168.31.231:9090';
// base_footprint→scan 只有 z=0.2（2026-10-18 实测 /tf），平面上和车体原点重合。
// /tracked_pose 就是 map→base_footprint，激光直接按它投到地图上。
const POSE_STALE_MS = 2000;
// 车身按规格书 650×550 mm 取圆角矩形，车体原点在中心，圆角半径按外观照片估。显示和碰撞检查都用它。
// 车端 /clean_pnc/cost_map/footprint 是 6 个 r=0.1375 的圆（10.18 实测，包络 0.675×0.55 m），不再用。
const BODY_L = 0.65, BODY_W = 0.55, BODY_R = 0.08;

// ---------- rosbridge 客户端 ----------
class Rosbridge {
  constructor(url, onState) {
    this.url = url;
    this.onState = onState;
    this.subs = new Map();     // topic → {type, cb, throttle}
    this.pending = new Map();  // id → {resolve, reject, timer}
    this.advs = new Map();     // topic → type
    this.seq = 0;
    this.connect();
  }

  connect() {
    this.onState('connecting');
    const ws = new WebSocket(this.url);
    this.ws = ws;
    ws.onopen = () => {
      this.onState('open');
      for (const [topic, type] of this.advs) this.send({ op: 'advertise', topic, type });
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

  // 连上就先 advertise：车端新建的发布者要过一会儿才连上订阅者，用到时才 advertise 会丢第一条。
  advertise(topic, type) {
    this.advs.set(topic, type);
    this.send({ op: 'advertise', topic, type });
  }

  // 返回是否真的发出去了。
  publish(topic, msg) {
    if (this.ws.readyState !== WebSocket.OPEN) return false;
    this.send({ op: 'publish', topic, msg });
    return true;
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
  gcost: null,      // 全局代价图：规划禁行区，设目标时叠加显示
  pose: null,       // {x, y, yaw, t}
  scan: null,
  path: [],         // [[x, y]]
  points: [],       // [{name, x, y, sys}]
  trail: [],        // [[x, y]]
  nav: null,        // {x, y, status, t}
  floor: null,      // {floorId, mapId}，按点位导航要用
  stopped: false,   // 本页叫停过：强停后 /navigation_status 会一直停在 1，没有「取消」状态
  active: null,     // 本页发出、还在执行的目标（带规划路径）
};
const view = { scale: 80, rot: 0, ox: 0, oy: 0, follow: false, fitted: false };

const LAYERS = {
  scan:      { label: '激光', def: true },
  path:      { label: '规划路径', def: true },
  trail:     { label: '轨迹', def: true },
  points:    { label: '导航点', def: true },
  costmap:   { label: '局部代价图', def: false },
  gcost:     { label: '全局代价图', note: '设目标时自动显示', def: false },
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
ros.subscribe('/navigation_status', 'map_msgs/NavigationStatusResult', onNav);
ros.subscribe('/power_report', 'std_msgs/Int16', onPower);
ros.subscribe('/location_status', 'std_msgs/Int16', onLoc);
ros.subscribe('/raw_odom', 'nav_msgs/Odometry', onOdom, 200);
ros.subscribe('/clean_pnc/cost_map/global_costmap', 'nav_msgs/OccupancyGrid', (m) => {
  st.gcost = { ...gridToImage(m, costColor), data: m.data };
  redraw();
}, 2000);
ros.subscribe('/current_floormap', 'map_msgs/RobotFloorMap', (m) => {
  st.floor = { floorId: m.floorId, mapId: m.mapId };
});
ros.advertise('/navi_stop', 'std_msgs/Int16');
ros.advertise('/task_op', 'std_msgs/Int16');
ros.advertise('/move_base_simple/goal', 'geometry_msgs/PoseStamped');
ros.advertise('/cmd_vel', 'geometry_msgs/Twist');
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
  st.map = { ...gridToImage(m, mapColor), key, data: m.data };
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

// status：1 进行中，2 到达。强停后会一直停在 1，没有「取消」状态。
function onNav(m) {
  st.nav = { ...m, t: performance.now() };
  if (m.status === 2) {
    st.stopped = false;
    st.active = null;
  }
  updateNavChip();
  redraw();
}

function updateNavChip() {
  if (st.stopped) return setChip('c-nav', '已叫停', 'bad');
  if (!st.nav) return;
  const s = st.nav.status;
  setChip('c-nav', { 1: '进行中', 2: '已到达' }[s] || '状态 ' + s, s === 1 ? 'warn' : s === 2 ? 'ok' : '');
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
    // z 是朝向，单位是度。
    const conv = (sys) => (p) => ({ name: p.point_name, id: p.point_id, x: p.x, y: p.y,
                                     yaw: p.z * Math.PI / 180, sys });
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
  if (st.gcost && (layers.gcost || goalMode || pending)) drawGrid(st.gcost);
  if (layers.costmap && st.costmap) drawGrid(st.costmap);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  drawMeterGrid();
  drawAxes();
  if (layers.trail) drawPolyline(st.trail, 'rgba(88,166,255,0.55)', 2, 0.5);
  if (layers.path) drawPolyline(st.path, '#3fb950', 2.5);
  if (st.active) drawPlan(st.active, 'rgba(240,136,62,0.55)');
  if (pending) drawPlan(pending, '#f0883e');
  if (picking) drawPick();
  if (layers.points) drawPoints();
  drawGoal();
  if (layers.scan) drawScan();
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
  if (!st.nav || st.nav.status !== 1 || st.stopped) return;
  const [sx, sy] = w2s(st.nav.x, st.nav.y);
  ctx.strokeStyle = '#f85149';
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.arc(sx, sy, 9, 0, 2 * Math.PI);
  ctx.stroke();
  label('目标', sx + 11, sy - 8, '#f85149');
}

function forEachScanPoint(fn) {
  const sc = st.scan, p = st.pose;
  if (!sc || !p) return;
  for (let i = 0; i < sc.ranges.length; i++) {
    const r = sc.ranges[i];
    if (!(r >= sc.range_min && r <= sc.range_max)) continue;
    const a = p.yaw + sc.angle_min + i * sc.angle_increment;
    fn(p.x + r * Math.cos(a), p.y + r * Math.sin(a));
  }
}

function drawScan() {
  ctx.fillStyle = '#ff4d4f';
  forEachScanPoint((x, y) => {
    const [sx, sy] = w2s(x, y);
    ctx.fillRect(sx - 1, sy - 1, 2, 2);
  });
}

function drawPlan(t, color) {
  if (t.pts) {
    ctx.setLineDash([6, 4]);
    drawPolyline(t.pts, color, 2.5);
    ctx.setLineDash([]);
  }
  drawGhost(t.x, t.y, t.yaw, color);
  const c = t.check;
  if (c && c.worstPath && c.minPath < CLEAR_WARN) drawGhost(c.worstPath.x, c.worstPath.y, c.worstPath.yaw, '#f85149');
  if (c && c.worstStart && c.minStart < CLEAR_WARN) drawGhost(c.worstStart.x, c.worstStart.y, c.worstStart.yaw, '#f85149');
}

// 车体系下的车身轮廓点，每个圆角用 6 段折线。
const BODY_OUTLINE = (() => {
  const ax = BODY_L / 2 - BODY_R, ay = BODY_W / 2 - BODY_R, out = [];
  for (const [cx, cy, a0] of [[ax, ay, 0], [-ax, ay, 0.5], [-ax, -ay, 1], [ax, -ay, 1.5]]) {
    for (let k = 0; k <= 6; k++) {
      const a = (a0 + k / 12) * Math.PI;
      out.push([cx + BODY_R * Math.cos(a), cy + BODY_R * Math.sin(a)]);
    }
  }
  return out;
})();

// 把车体系下的多边形摆到 (x, y, yaw) 处建成路径，由调用方 fill / stroke。
function bodyPoly(pts, x, y, yaw) {
  const c = Math.cos(yaw), s = Math.sin(yaw);
  ctx.beginPath();
  pts.forEach(([bx, by], i) => {
    const [sx, sy] = w2s(x + bx * c - by * s, y + bx * s + by * c);
    if (i === 0) ctx.moveTo(sx, sy);
    else ctx.lineTo(sx, sy);
  });
  ctx.closePath();
}

// 在 (x, y, yaw) 处画车身轮廓和朝向：目标位姿、碰撞检查里最危险的位姿。
function drawGhost(x, y, yaw, color) {
  const c = Math.cos(yaw), s = Math.sin(yaw);
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  bodyPoly(BODY_OUTLINE, x, y, yaw);
  ctx.stroke();
  const [ax, ay] = w2s(x, y), [bx, by] = w2s(x + 0.3 * c, y + 0.3 * s);
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.moveTo(ax, ay);
  ctx.lineTo(bx, by);
  ctx.stroke();
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(ax, ay, 4, 0, 2 * Math.PI);
  ctx.fill();
}

function drawPick() {
  drawGhost(picking.x, picking.y, pickYaw(picking), '#f0883e');
}

function drawRobot() {
  const p = st.pose;
  if (!p) return;
  const stale = performance.now() - p.t > POSE_STALE_MS;
  ctx.fillStyle = stale ? 'rgba(110,118,129,0.75)' : 'rgba(47,129,247,0.75)';
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 1.5;
  bodyPoly(BODY_OUTLINE, p.x, p.y, p.yaw);
  ctx.fill();
  ctx.stroke();
  // 车头方向：车身前半部的白色三角。
  ctx.fillStyle = '#ffffff';
  bodyPoly([[0.24, 0], [0.02, 0.13], [0.02, -0.13]], p.x, p.y, p.yaw);
  ctx.fill();
}

function drawScaleBar() {
  const len = [0.1, 0.2, 0.5, 1, 2, 5, 10].find((l) => l * view.scale >= 60) || 10;
  const px = len * view.scale, x = 12, y = H - $('ctl').offsetHeight - 16;
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
  if (ptrs.size === 2) {
    picking = null;
    startPinch();
  } else if (ptrs.size === 1 && goalMode) {
    const [x, y] = s2w(e.clientX, e.clientY);
    picking = { x, y, yaw: null, sx: e.clientX, sy: e.clientY };
    redraw();
  }
});

cv.addEventListener('pointermove', (e) => {
  showCursor(e.clientX, e.clientY);
  const prev = ptrs.get(e.pointerId);
  if (!prev) return;
  ptrs.set(e.pointerId, [e.clientX, e.clientY]);
  if (picking && ptrs.size === 1) {
    if (Math.hypot(e.clientX - picking.sx, e.clientY - picking.sy) > 12) {
      const [x, y] = s2w(e.clientX, e.clientY);
      picking.yaw = Math.atan2(y - picking.y, x - picking.x);
    }
    return redraw();
  }
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
    if (picking && ptrs.size === 0) {
      if (type === 'pointerup') finishPick();
      else picking = null;
    }
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

// ---------- 操作（阶段 2） ----------
// 接口 2026-10-16～18 都实测过：/navi_stop 5 最快（约 0.7 s 停住，滑行约 5 cm），驱动也会清掉目标；
// /task_op 1/2 暂停/继续要等驱动 0.5 s 看门狗，约 1.2 s 停住；/cmd_vel 停发约 0.8 s 自停。
const CLEAR_BAD = 0.03;   // 最小间隙低于它算可能碰撞
const CLEAR_WARN = 0.08;  // 低于它提示间隙小
const SEARCH = 0.2;       // 碰撞检查在车身外这么远内搜障碍，更远的间隙显示成「> x cm」
const START_SEG = 0.3;    // 离起点这么近的位姿算起步段（停在充电桩上时本来就贴着桩）
const JOY_SPEEDS = [{ label: '慢 0.2 m/s', v: 0.2, w: 0.4 }, { label: '快 0.4 m/s', v: 0.4, w: 0.6 }];

let goalMode = false;
let picking = null;   // 设目标时正在拖的 {x, y, yaw, sx, sy}
let pending = null;   // 等确认的目标 {kind, title, x, y, yaw, point, start, pts, check, danger, armed}
let joySpeed = 0, joyVec = null, joyTimer = 0;

function sendOrWarn(topic, msg) {
  if (ros.publish(topic, msg)) return true;
  toast('未连接，指令没发出去');
  return false;
}

function stopAll() {
  joyRelease();
  if (!sendOrWarn('/navi_stop', { data: 5 })) return;
  // 停止要可靠：再补发两次，重复的 5 没有副作用。
  setTimeout(() => ros.publish('/navi_stop', { data: 5 }), 100);
  setTimeout(() => ros.publish('/navi_stop', { data: 5 }), 200);
  st.stopped = true;
  st.active = null;
  closeConfirm();
  setGoalMode(false);
  updateNavChip();
  toast('已发停止', true);
  redraw();
}

function navBusy() {
  if (st.stopped) return false;
  if (st.active) return true;
  return !!st.nav && st.nav.status === 1 && performance.now() - st.nav.t < 3000;
}

function deg(r) {
  return (r * 180 / Math.PI).toFixed(0);
}

function wrapAngle(a) {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

function poseMsg(x, y, yaw) {
  return { header: { frame_id: 'map' },
           pose: { position: { x, y, z: 0 }, orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) } } };
}

function twist(v, w) {
  return { linear: { x: v, y: 0, z: 0 }, angular: { x: 0, y: 0, z: w } };
}

// ----- 设目标 -----
function setGoalMode(on) {
  goalMode = on;
  picking = null;
  $('b-goal').classList.toggle('on', on);
  cv.style.cursor = on ? 'crosshair' : '';
  if (on) toast('在地图上按下选位置，拖动定朝向', true);
}

// 只点不拖时，朝向取车指向目标的方向。
function pickYaw(t) {
  if (t.yaw !== null) return t.yaw;
  const p = st.pose;
  return p ? Math.atan2(t.y - p.y, t.x - p.x) : 0;
}

function finishPick() {
  const t = picking;
  const yaw = pickYaw(t);
  setGoalMode(false);
  preview({ kind: 'goal', x: t.x, y: t.y, yaw,
            title: `去 (${t.x.toFixed(2)}, ${t.y.toFixed(2)})，朝向 ${deg(yaw)}°` });
}

// ----- 规划预览和碰撞检查 -----
async function preview(t) {
  const p = st.pose;
  if (!p) return toast('还没有车的位姿');
  pending = t;
  t.start = { x: p.x, y: p.y, yaw: p.yaw };
  $('cf-title').textContent = t.title;
  setConfirmInfo('规划中…');
  setGoButton(true, '出发', false);
  $('confirm').classList.add('open');
  redraw();
  let r;
  try {
    r = await ros.call('/get_plan_movebase', { start: poseMsg(p.x, p.y, p.yaw), goal: poseMsg(t.x, t.y, t.yaw) }, 8000);
  } catch (e) {
    if (pending === t) setConfirmInfo('规划失败：' + e.message, 'bad');
    return;
  }
  if (pending !== t) return;
  t.pts = (r.path || []).map((q) => [q.pose.position.x, q.pose.position.y]);
  if (t.pts.length < 2) {
    const v = gridValue(st.gcost, t.x, t.y);
    return setConfirmInfo(v !== 0 && v !== -1 && v !== undefined
      ? '规划失败：目标离障碍太近，在禁行区里（紫色区域）' : '规划失败：没有路径', 'bad');
  }
  t.check = checkPlan(t.pts, t.yaw);

  const parts = [`路径 ${pathLength(t.pts).toFixed(2)} m`];
  let level = 'ok';
  const c = t.check;
  const cm = (v) => (v > c.limit ? `> ${Math.floor(c.limit * 100)}` : (v * 100).toFixed(0)) + ' cm';
  if (!c) {
    parts.push('还没收到地图，没做碰撞检查');
    level = 'warn';
  } else {
    parts.push(`间隙：路上 ${cm(c.minPath)}，起步 ${cm(c.minStart)}（现在 ${cm(c.now)}）`);
    // 起步段只要不比现在更贴近障碍（容差半个 5 cm 栅格）就不算碰撞：停在充电桩上时车尾本来就贴着桩。
    const startBad = c.minStart < CLEAR_BAD && c.minStart < c.now - 0.025;
    if (c.minPath < CLEAR_BAD || startBad) level = 'bad';
    else if (c.minPath < CLEAR_WARN || c.minStart < CLEAR_WARN) level = 'warn';
    if (c.now < CLEAR_BAD) parts.push('车现在就贴着障碍（可能在充电桩上）');
    if (!c.withScan) parts.push('激光图层关着，只查了地图');
  }
  const end = t.pts[t.pts.length - 1];
  const off = Math.hypot(end[0] - t.x, end[1] - t.y);
  if (off > 0.1) {
    parts.push(`路径终点离目标 ${off.toFixed(2)} m`);
    if (level === 'ok') level = 'warn';
  }
  setConfirmInfo(parts.join('，'), level);
  t.danger = level !== 'ok';
  setGoButton(false, level === 'bad' ? '可能碰撞，仍要出发' : t.danger ? '仍要出发' : '出发', t.danger);
  redraw();
}

function gridValue(g, x, y) {
  if (!g) return undefined;
  const i = g.info, c = Math.floor((x - i.origin.position.x) / i.resolution);
  const r = Math.floor((y - i.origin.position.y) / i.resolution);
  return c < 0 || r < 0 || c >= i.width || r >= i.height ? undefined : g.data[r * i.width + c];
}

function pathLength(pts) {
  let len = 0;
  for (let i = 1; i < pts.length; i++) len += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
  return len;
}

// 沿路径取车的位姿：路径点去重后用前后 3 个点算切向（规划结果不带朝向），
// 起点和终点的原地转向按最短方向每 10° 取一个。
function poseSamples(pts, p, goalYaw) {
  // 规划器把起点吸附到 5 cm 栅格，开头几个点可能比车的真实位置更靠近障碍，换成真实位姿。
  let k = 0;
  while (k < pts.length - 1 && Math.hypot(pts[k][0] - p.x, pts[k][1] - p.y) <= 0.05) k++;
  const raw = [[p.x, p.y], ...pts.slice(k)];
  const q = raw.filter((a, i) => i === 0 || a[0] !== raw[i - 1][0] || a[1] !== raw[i - 1][1]);
  const yawAt = (i) => {
    const a = q[Math.max(0, i - 3)], b = q[Math.min(q.length - 1, i + 3)];
    return Math.atan2(b[1] - a[1], b[0] - a[0]);
  };
  const out = [];
  const turn = (x, y, from, to) => {
    const d = wrapAngle(to - from), n = Math.max(1, Math.ceil(Math.abs(d) / (Math.PI / 18)));
    for (let k = 0; k <= n; k++) out.push({ x, y, yaw: from + d * k / n });
  };
  if (q.length < 2) return [{ x: p.x, y: p.y, yaw: p.yaw }];
  turn(p.x, p.y, p.yaw, yawAt(0));
  for (let i = 0; i < q.length; i++) out.push({ x: q[i][0], y: q[i][1], yaw: yawAt(i) });
  const last = q[q.length - 1];
  turn(last[0], last[1], yawAt(q.length - 1), goalYaw);
  return out;
}

// 车体系点 (bx, by) 到车身圆角矩形边缘的距离，在车身内为负。
function bodyDist(bx, by) {
  const qx = Math.abs(bx) - (BODY_L / 2 - BODY_R), qy = Math.abs(by) - (BODY_W / 2 - BODY_R);
  return Math.hypot(Math.max(qx, 0), Math.max(qy, 0)) + Math.min(Math.max(qx, qy), 0) - BODY_R;
}

// 车身取圆角矩形（BODY_*），障碍取地图占据栅格（≥50）加当前激光端点，
// 每个位姿算车身到最近障碍的间隙。返回现在、起步段、路上三个最小间隙和对应位姿，没有地图时返回 null。
function checkPlan(pts, goalYaw) {
  const g = st.map, p = st.pose;
  if (!g || !p) return null;
  const { width: w, height: h, resolution: res } = g.info;
  const ox = g.info.origin.position.x, oy = g.info.origin.position.y;
  const occ = new Uint8Array(w * h);
  for (let i = 0; i < w * h; i++) if (g.data[i] >= 50) occ[i] = 1;
  forEachScanPoint((x, y) => {
    const c = Math.floor((x - ox) / res), r = Math.floor((y - oy) / res);
    if (c >= 0 && r >= 0 && c < w && r < h) occ[r * w + c] = 1;
  });
  // 搜索窗口取车身外接圆再往外 SEARCH，窗口外的障碍离车身一定超过 SEARCH。
  const R = Math.ceil((Math.hypot(BODY_L / 2, BODY_W / 2) + SEARCH) / res);
  const clearAt = (s) => {
    let best = Infinity;
    const c = Math.cos(s.yaw), sn = Math.sin(s.yaw);
    const ci = Math.floor((s.x - ox) / res), ri = Math.floor((s.y - oy) / res);
    for (let dr = -R; dr <= R; dr++) {
      const rr = ri + dr;
      if (rr < 0 || rr >= h) continue;
      for (let dc = -R; dc <= R; dc++) {
        const cc = ci + dc;
        if (cc < 0 || cc >= w || !occ[rr * w + cc]) continue;
        const dx = ox + (cc + 0.5) * res - s.x, dy = oy + (rr + 0.5) * res - s.y;
        const d = bodyDist(dx * c + dy * sn, -dx * sn + dy * c) - res / 2;
        if (d < best) best = d;
      }
    }
    return best;
  };
  const r = { now: clearAt(p), minStart: Infinity, minPath: Infinity, worstStart: null, worstPath: null,
              limit: SEARCH - res / 2, withScan: !!st.scan };
  for (const s of poseSamples(pts, p, goalYaw)) {
    const d = clearAt(s);
    if (Math.hypot(s.x - p.x, s.y - p.y) < START_SEG) {
      if (d < r.minStart) [r.minStart, r.worstStart] = [d, s];
    } else if (d < r.minPath) {
      [r.minPath, r.worstPath] = [d, s];
    }
  }
  return r;
}

function setConfirmInfo(text, level = '') {
  $('cf-info').textContent = text;
  $('cf-info').className = level ? level + '-t' : '';
}

function setGoButton(disabled, text, danger) {
  const b = $('cf-go');
  b.disabled = disabled;
  b.textContent = text;
  b.className = 'go' + (danger ? ' danger' : '');
}

function closeConfirm() {
  pending = null;
  $('confirm').classList.remove('open');
  redraw();
}

function go() {
  const t = pending, p = st.pose;
  if (!t || !t.pts || !p) return;
  // 规划之后车动过，旧路径的碰撞检查不算数，重新规划。
  if (Math.hypot(p.x - t.start.x, p.y - t.start.y) > 0.1 || Math.abs(wrapAngle(p.yaw - t.start.yaw)) > 0.2) {
    toast('车动过了，重新规划', true);
    return preview({ ...t, pts: null, check: null, armed: false });
  }
  if (t.danger && !t.armed) {
    t.armed = true;
    setGoButton(false, '再点一次确认', true);
    return;
  }
  if (t.kind === 'goal') {
    if (!sendOrWarn('/move_base_simple/goal', poseMsg(t.x, t.y, t.yaw))) return;
  } else {
    if (!st.floor) return toast('还没收到 /current_floormap，不知道地图 ID');
    // 返回 status −1 不代表失败（10.16 实测），只把原始结果显示出来。
    ros.call('/navi_targegoalplan', { type: 0, floor_id: st.floor.floorId, map_id: st.floor.mapId,
                                      point_id: Number(t.point.id) }, 10000)
      .then((r) => toast(`按点位导航已下发（status ${r.status}${r.msg ? '，' + r.msg : ''}）`, true))
      .catch((e) => toast('按点位导航调用失败：' + e.message));
  }
  st.stopped = false;
  st.active = t;
  pending = null;
  $('confirm').classList.remove('open');
  updateNavChip();
  redraw();
}

// ----- 摇杆：按住才发，10 Hz；松手、切后台都会发零 -----
function joyMove(e) {
  const r = $('joy').getBoundingClientRect(), R = r.width / 2;
  let dx = (e.clientX - r.left - R) / R, dy = (e.clientY - r.top - R) / R;
  const n = Math.hypot(dx, dy);
  if (n > 1) {
    dx /= n;
    dy /= n;
  }
  joyVec = [dx, dy];
  $('knob').style.transform = `translate(${dx * R * 0.7}px, ${dy * R * 0.7}px)`;
}

function joySend() {
  if (!joyVec) return;
  const s = JOY_SPEEDS[joySpeed];
  const dz = (a) => (Math.abs(a) < 0.15 ? 0 : a);
  ros.publish('/cmd_vel', twist(-dz(joyVec[1]) * s.v, -dz(joyVec[0]) * s.w));
}

function joyRelease() {
  if (!joyTimer) return;
  clearInterval(joyTimer);
  joyTimer = 0;
  joyVec = null;
  $('knob').style.transform = '';
  ros.publish('/cmd_vel', twist(0, 0));
}

$('joy').addEventListener('pointerdown', (e) => {
  if (e.target.id === 'joy-speed' || joyTimer) return;
  if (navBusy()) return toast('导航进行中，先按停止再用摇杆');
  $('joy').setPointerCapture(e.pointerId);
  joyMove(e);
  joySend();
  joyTimer = setInterval(joySend, 100);
});
$('joy').addEventListener('pointermove', (e) => {
  if (joyTimer) joyMove(e);
});
for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) $('joy').addEventListener(type, joyRelease);
document.addEventListener('visibilitychange', () => {
  if (document.hidden) joyRelease();
});
$('joy-speed').textContent = JOY_SPEEDS[0].label;
$('joy-speed').onclick = () => {
  joySpeed = (joySpeed + 1) % JOY_SPEEDS.length;
  $('joy-speed').textContent = JOY_SPEEDS[joySpeed].label;
};

// ----- 按钮 -----
// 停止用 pointerdown，比 click 早一个松手的时间。
$('b-stop').addEventListener('pointerdown', stopAll);
$('b-pause').onclick = () => {
  if (sendOrWarn('/task_op', { data: 1 })) toast('已发暂停', true);
};
$('b-resume').onclick = () => {
  if (sendOrWarn('/task_op', { data: 2 })) toast('已发继续', true);
};
$('b-goal').onclick = () => {
  closeConfirm();
  $('points').classList.remove('open');
  setGoalMode(!goalMode);
};
$('b-nav').onclick = () => {
  closeConfirm();
  setGoalMode(false);
  const list = $('point-list');
  list.textContent = '';
  // 系统点（充电桩、初始点、桩位置）贴着充电桩，导航过去可能撞桩，不列出来。
  const pts = st.points.filter((p) => !p.sys);
  if (!pts.length) list.textContent = '没有点位';
  for (const p of pts) {
    const b = document.createElement('button');
    b.textContent = `${p.name}　(${p.x.toFixed(2)}, ${p.y.toFixed(2)}, ${deg(p.yaw)}°)`;
    b.onclick = () => {
      $('points').classList.remove('open');
      preview({ kind: 'point', point: p, x: p.x, y: p.y, yaw: p.yaw, title: `去点位 ${p.name}` });
    };
    list.append(b);
  }
  $('points').classList.toggle('open');
};
$('pt-close').onclick = () => $('points').classList.remove('open');
$('cf-cancel').onclick = closeConfirm;
$('cf-go').onclick = go;
$('b-joy').onclick = () => {
  const open = !$('joy').classList.contains('open');
  if (!open) joyRelease();
  $('joy').classList.toggle('open', open);
  $('b-joy').classList.toggle('on', open);
};
addEventListener('keydown', (e) => {
  if (e.key !== 'Escape') return;
  setGoalMode(false);
  closeConfirm();
  $('points').classList.remove('open');
});

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
function toast(text, info = false) {
  $('toast').textContent = text;
  $('toast').className = info ? 'info' : '';
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
