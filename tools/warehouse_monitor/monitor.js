import {
  BufferGeometry, Float32BufferAttribute, Group, PerspectiveCamera, Quaternion,
  Scene, Vector3, WebGLRenderer,
} from "three";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { CSS2DRenderer } from "three/addons/renderers/CSS2DRenderer.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { droneModel, makeCloud, wideLine } from "./scene_parts.js";
import { createInspection } from "./inspection.js";
import { createTasks } from "./tasks.js";
import { startDroneStatus } from "./drone-status.js";

const indoorMap = "20261114_101654_indoor_fly_raw_000001";
const previousIndoorMap = "20261021_110311_indoor";
const outdoorMap = "20261105_114605_outdoor";
const byId = id => document.getElementById(id);
const viewport = byId("viewport");
const detail = byId("detail");
const ctx = detail.getContext("2d");
const toggles = [...document.querySelectorAll("[data-layer]")];
const views = [...document.querySelectorAll("[data-view]")];
const pos = new Vector3();
const orient = new Quaternion();
const forward = new Vector3();
const styles = {
  floor: [0x648796, 2.5, .62],
  structure: [0x9baeba, 2.1, .66],
  walls: [0x708b9e, 2.2, .67],
  near: [0x9babb6, 2.1, .65],
  roof: [0x8c9fae, 2.4, .59],
  rear: [0x9baeba, 2.1, .66],
};
const colors = { trail: "#4fdfec", plan: "#ffd182", drone: "#f3f7fb" };

let rig;
let dirty = true;
let pose = null;
let trail = [];
let trailFlat = [];
let plan = new Float32Array(0);
let cloud = new Float32Array(0);
let view = "room";
let epoch = "";
let trailId = 0, cloudId = 0, planId = 0;
let wantedMap = null;
let loadedMap = null;
let retryMapAt = 0;
let loading = null;
let mapError = "";
let lastResult = null;
let online = false;
let lastResponse = 0;
let pending = false;
let inspection;
let tasks;
let catalog = [];
let manualMap = "";
let offline = false;
let telemetryTimer;
let telemetryRequest;

function watching() {
  return !offline && !document.hidden && (byId("tab-live").getAttribute("aria-pressed") === "true"
    || tasks?.watchingFlight());
}

function pauseTelemetry() {
  clearTimeout(telemetryTimer);
  telemetryRequest?.abort();
  telemetryRequest = null;
  online = false;
  paintStatus();
}

function syncTelemetry() {
  if (!watching()) {
    pauseTelemetry();
    return;
  }
  clearTimeout(telemetryTimer);
  void poll();
}

function addLayers(map) {
  if (rig) {
    for (const child of [...rig.layers.children]) {
      child.geometry.dispose();
      child.material.dispose();
      child.removeFromParent();
    }
    if (map) {
      for (const [name, points] of Object.entries(map.layers)) {
        const layer = makeCloud(points, ...styles[name]);
        layer.name = name;
        rig.layers.add(layer);
      }
    }
  }
  updateLayers();
}

function updateLayers() {
  let points = 0;
  for (const input of toggles) {
    const name = input.dataset.layer;
    const geometry = loadedMap?.layers[name];
    input.disabled = !geometry?.length;
    input.parentElement.hidden = input.disabled;
    if (input.checked) points += (geometry?.length ?? 0) / 3;
    const object = rig?.layers.getObjectByName(name);
    if (object) object.visible = input.checked;
  }
  byId("map-layers").hidden = !loadedMap;
  byId("structure-layer-name").textContent = loadedMap?.id === outdoorMap ? "室外结构"
    : loadedMap?.layers.walls ? "室内陈设" : "环境结构";
  byId("point-count").textContent = loadedMap ? `地图 ${points.toLocaleString("en-US")} 点` : "";
  applyVisibility();
  dirty = true;
}

function visible() {
  return Boolean(online && lastResult?.aligned && loadedMap?.key === lastResult.mapKey);
}

function applyVisibility() {
  if (rig) {
    const active = visible();
    rig.drone.visible = active && Boolean(pose);
    rig.trail.visible = active && trail.length > 1 && byId("traveled").checked;
    rig.plan.visible = active && plan.length >= 6 && byId("planned").checked;
    rig.cloud.visible = active && cloud.length > 0 && byId("live-cloud").checked;
  }
  drawDetail();
  dirty = true;
}

function clearMotion() {
  pose = null;
  trail = [];
  trailFlat = [];
  plan = cloud = new Float32Array(0);
  trailId = cloudId = planId = 0;
  applyVisibility();
}

function scene() {
  const renderer = new WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.setClearColor(0x111b25);
  viewport.prepend(renderer.domElement);
  const labels = new CSS2DRenderer();
  labels.domElement.className = "waypoint-overlay";
  viewport.append(labels.domElement);
  const world = new Scene();
  const camera = new PerspectiveCamera(40, viewport.clientWidth / Math.max(1, viewport.clientHeight), .025, 800);
  camera.up.set(0, 0, 1);
  camera.position.set(4, 6, 5);
  const controls = new OrbitControls(camera, renderer.domElement);
  // OrbitControls already pans on Shift + left drag. Avoid browser right-button gestures.
  controls.mouseButtons.RIGHT = null;
  controls.minDistance = .20;
  controls.maxDistance = 360;
  controls.addEventListener("change", () => { dirty = true; });

  const layers = new Group();
  const drone = droneModel();
  const trail = wideLine([0, 0, 0, .01, 0, 0], { color: colors.trail, linewidth: 3.2 });
  trail.geometry.setPositions(new Float32Array(4800 * 3));
  trail.frustumCulled = false;
  const plan = wideLine([0, 0, 0, .01, 0, 0], { color: colors.plan, linewidth: 2.7 });
  const cloud = makeCloud([], 0xb297ff, 2.6, .88);
  cloud.frustumCulled = false;
  cloud.renderOrder = 1;
  world.add(layers, drone, trail, plan, cloud);
  const observer = new ResizeObserver(() => {
    camera.aspect = viewport.clientWidth / Math.max(1, viewport.clientHeight);
    camera.updateProjectionMatrix();
    renderer.setSize(viewport.clientWidth, viewport.clientHeight, false);
    labels.setSize(viewport.clientWidth, viewport.clientHeight);
    dirty = true;
  });
  observer.observe(viewport);
  return { renderer, labels, world, camera, controls, layers, drone, trail, plan, cloud };
}

function setView(mode, fitMap = false) {
  view = mode;
  views.forEach(button => button.setAttribute("aria-pressed", String(button.dataset.view === mode)));
  if (!rig) return;
  const { camera, controls, drone } = rig;
  const bounds = loadedMap?.bounds;
  const target = bounds
    ? new Vector3(...bounds[0]).lerp(new Vector3(...bounds[1]), .5) : new Vector3();
  const extent = bounds ? new Vector3(...bounds[1]).sub(new Vector3(...bounds[0])) : new Vector3(5, 4, 3);
  const width = Math.max(2.5, extent.x, extent.y, extent.z);
  const narrow = Math.max(1, 1.15 / Math.max(.42, camera.aspect));
  const focusPose = !fitMap && pose && visible();
  const center = focusPose && mode !== "room" ? pos : target;
  let offset;
  if (mode === "room") {
    if (loadedMap && [indoorMap, previousIndoorMap].includes(loadedMap.id)) target.set(-.85, -.18, .65);
    offset = new Vector3(.79, 1.01, .69).multiplyScalar(width * narrow);
  } else if (mode === "close") {
    offset = new Vector3(1.25, 1.65, 1.25).multiplyScalar(narrow);
  } else {
    offset = new Vector3(0, -.002, focusPose ? 3.65 : width * 1.85).multiplyScalar(narrow);
  }
  drone.scale.setScalar(mode === "room" ? Math.max(1.35, width / 6) : 1);
  camera.position.copy(center).add(offset);
  controls.target.copy(center);
  controls.update();
  dirty = true;
}

function focusRoute(positions) {
  if (!rig) return;
  const { camera, controls } = rig;
  const lower = new Vector3(Infinity, Infinity, Infinity);
  const upper = new Vector3(-Infinity, -Infinity, -Infinity);
  for (const point of positions) {
    const p = new Vector3(...point);
    lower.min(p);
    upper.max(p);
  }
  const center = lower.clone().lerp(upper, .5);
  const extent = upper.sub(lower);
  const width = Math.max(1.4, extent.length());
  const scale = width * Math.max(1, .9 / camera.aspect);
  camera.position.copy(center).add(new Vector3(.75, 1.05, 1.1).multiplyScalar(scale));
  controls.target.copy(center);
  controls.update();
  views.forEach(button => button.setAttribute("aria-pressed", "false"));
  dirty = true;
}

function updateGeometry(trailChanged, planChanged, cloudChanged) {
  if (!rig) return;
  if (trailChanged) {
    const geometry = rig.trail.geometry;
    const segments = geometry.getAttribute("instanceStart").data;
    const count = Math.max(0, trail.length - 1);
    for (let i = 0; i < count; i++) {
      segments.array.set(trail[i], i * 6);
      segments.array.set(trail[i + 1], i * 6 + 3);
    }
    geometry.instanceCount = count;
    segments.needsUpdate = true;
  }
  if (planChanged && plan.length >= 6) {
    rig.plan.geometry.dispose();
    rig.plan.geometry = new LineGeometry().setPositions(plan);
    rig.plan.material.color.set(pending ? "#f1c78b" : colors.plan);
    rig.plan.material.opacity = pending ? .62 : 1;
    rig.plan.material.transparent = pending;
  }
  if (cloudChanged) {
    const geometry = new BufferGeometry();
    geometry.setAttribute("position", new Float32BufferAttribute(cloud, 3));
    rig.cloud.geometry.dispose();
    rig.cloud.geometry = geometry;
  }
  if (pose) {
    rig.drone.position.copy(pos);
    rig.drone.quaternion.copy(orient);
  }
}

function drawDetail() {
  const density = Math.min(devicePixelRatio, 2);
  const w = detail.clientWidth, h = detail.clientHeight;
  if (!w || !h) return;
  if (detail.width !== Math.round(w * density) || detail.height !== Math.round(h * density)) {
    detail.width = Math.round(w * density);
    detail.height = Math.round(h * density);
  }
  ctx.setTransform(density, 0, 0, density, 0, 0);
  ctx.clearRect(0, 0, w, h);
  if (!visible() || !pose) {
    ctx.fillStyle = "#a4b4c4";
    ctx.font = "12px sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("等待地图与定位对齐", w / 2, h / 2);
    return;
  }
  const radius = 1.25;
  const scale = Math.min(w, h) / (radius * 2);
  const sx = x => w / 2 + (x - pos.x) * scale;
  const sy = y => h / 2 - (y - pos.y) * scale;
  const grid = .5;
  ctx.strokeStyle = "#283847";
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let x = Math.ceil((pos.x - w / 2 / scale) / grid) * grid; sx(x) < w; x += grid) {
    ctx.moveTo(sx(x), 0);
    ctx.lineTo(sx(x), h);
  }
  for (let y = Math.ceil((pos.y - h / 2 / scale) / grid) * grid; sy(y) > 0; y += grid) {
    ctx.moveTo(0, sy(y));
    ctx.lineTo(w, sy(y));
  }
  ctx.stroke();
  function path(points, color, dashed = false) {
    if (points.length < 6) return;
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.setLineDash(dashed ? [4, 4] : []);
    ctx.beginPath();
    ctx.moveTo(sx(points[0]), sy(points[1]));
    for (let i = 3; i < points.length; i += 3) ctx.lineTo(sx(points[i]), sy(points[i + 1]));
    ctx.stroke();
    ctx.setLineDash([]);
  }
  if (byId("planned").checked) path(plan, colors.plan, pending);
  if (byId("traveled").checked) path(trailFlat, colors.trail);
  ctx.save();
  ctx.translate(w / 2, h / 2);
  ctx.rotate(-Math.atan2(forward.y, forward.x));
  ctx.fillStyle = colors.drone;
  ctx.strokeStyle = colors.drone;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(3, 0); ctx.lineTo(22, 0); ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(25, 0); ctx.lineTo(16, -5); ctx.lineTo(16, 5); ctx.closePath(); ctx.fill();
  ctx.beginPath(); ctx.arc(0, 0, 4, 0, Math.PI * 2); ctx.fill();
  ctx.restore();
  ctx.strokeStyle = "#9db1c3";
  ctx.beginPath(); ctx.moveTo(10, h - 12); ctx.lineTo(10 + .5 * scale, h - 12); ctx.stroke();
  ctx.fillStyle = "#afc0cd";
  ctx.textAlign = "left";
  ctx.font = "10px ui-monospace, monospace";
  ctx.fillText("0.5 m", 10, h - 17);
}

function asFloats(encoded) {
  const raw = atob(encoded);
  const buffer = Uint8Array.from(raw, ch => ch.charCodeAt(0));
  return new Float32Array(buffer.buffer);
}

function splitOutdoorStructure(points) {
  const isRear = index => points[index] < 0 && points[index + 2] > .30;
  let rearLength = 0;
  for (let i = 0; i < points.length; i += 3) {
    if (isRear(i)) rearLength += 3;
  }
  const rear = new Float32Array(rearLength);
  const structure = new Float32Array(points.length - rearLength);
  let rearIndex = 0;
  let structureIndex = 0;
  for (let i = 0; i < points.length; i += 3) {
    if (isRear(i)) {
      rear[rearIndex++] = points[i];
      rear[rearIndex++] = points[i + 1];
      rear[rearIndex++] = points[i + 2];
    } else {
      structure[structureIndex++] = points[i];
      structure[structureIndex++] = points[i + 1];
      structure[structureIndex++] = points[i + 2];
    }
  }
  return { structure, rear };
}

function decodeMap(buffer, id) {
  const len = new DataView(buffer).getUint32(0, true);
  const metadata = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 4, len)));
  const coordinates = new Float32Array(buffer.slice(4 + len));
  let offset = 0;
  const layers = {};
  for (const [key, count] of Object.entries(metadata.counts)) {
    layers[key] = coordinates.subarray(offset, offset + count * 3);
    offset += count * 3;
  }
  if (offset !== coordinates.length) throw new Error("地图点数不符");
  if (id === outdoorMap && layers.structure) {
    Object.assign(layers, splitOutdoorStructure(layers.structure));
  }
  return { ...metadata, layers };
}

async function loadMap(key, id) {
  const controller = new AbortController();
  loading = controller;
  const timer = setTimeout(() => controller.abort(), 20000);
  try {
    const response = await fetch("/map?key=" + encodeURIComponent(key), { signal: controller.signal });
    if (!response.ok) {
      const message = response.status === 404 && lastResult?.localMap
        ? `电脑未缓存当前地图版本，请在终端运行 warehouse-monitor --sync ${id}`
        : `地图读取失败（HTTP ${response.status}）`;
      throw new Error(message);
    }
    const decoded = decodeMap(await response.arrayBuffer(), id);
    if (wantedMap !== key || loading !== controller) return;
    loadedMap = { key, id, ...decoded };
    mapError = "";
    addLayers(loadedMap);
    inspection?.load(loadedMap);
    tasks?.setMap(loadedMap);
    setView("room");
  } catch (error) {
    if (wantedMap !== key || loading !== controller) return;
    mapError = error.name === "AbortError" ? "地图读取超时" : error.message;
    retryMapAt = performance.now() + 2500;
  } finally {
    clearTimeout(timer);
    if (loading === controller) loading = null;
    paintStatus();
  }
}

function defaultIndoorMap() {
  return catalog.find(map => map.id === indoorMap)
    ?? catalog.find(map => map.id === previousIndoorMap);
}

function syncMap() {
  const manual = catalog.find(map => map.key === manualMap);
  const localized = lastResult?.mapKey ? { key: lastResult.mapKey, id: lastResult.mapId } : null;
  const selected = manual ?? localized ?? defaultIndoorMap();
  const key = selected?.key ?? null;
  const id = selected?.id;
  if (key !== wantedMap) {
    loading?.abort();
    loading = null;
    wantedMap = key;
    loadedMap = null;
    mapError = "";
    retryMapAt = 0;
    inspection?.load(null);
    tasks?.setMap(null);
    addLayers(null);
  }
  if (wantedMap && !loadedMap && !loading && performance.now() >= retryMapAt) {
    void loadMap(wantedMap, id);
  }
}

async function loadCatalog() {
  try {
    const response = await fetch("/maps");
    if (!response.ok) return;
    catalog = await response.json();
    byId("display-map").parentElement.hidden = false;
    byId("tab-inspection").hidden = false;
    const selector = byId("display-map");
    const previous = selector.value;
    selector.length = 1;
    for (const map of catalog) {
      const date = new Date(Number(map.version) / 1e6);
      const label = map.id + " · " + date.toLocaleString("zh-CN", { hour12: false });
      selector.add(new Option(label, map.key));
    }
    selector.value = previous;
    if (catalog.some(map => map.id === indoorMap)) {
      selector.options[0].textContent = "跟随定位（默认室内飞扫图）";
    }
    if (offline && !selector.value && catalog.length) {
      selector.value = (defaultIndoorMap() ?? catalog[0]).key;
      manualMap = selector.value;
      byId("tab-inspection").click();
    }
    syncMap();
    paintStatus();
  } catch {
    // Board-only fallback still follows the map supplied by telemetry.
  }
}

function paintStatus() {
  const result = lastResult;
  const connection = byId("connection");
  connection.dataset.online = String(online);
  connection.textContent = offline ? "本机离线预览" : online ? "Orange Pi 已连接" : watching() ? "等待 Orange Pi · 自动重连" : "实时监看已暂停";
  byId("map-id").textContent = result?.mapId ?? "—";
  byId("loc-status").textContent = offline ? "定位 · 未连接" : !online ? "定位 · 数据中断"
    : result?.aligned ? "定位 · 已对齐"
      : result?.relocation === "LOCALIZED" ? "定位 · 等待地图 TF"
        : `定位 · ${result?.relocation === "WAITING" ? "等待" : result?.relocation || "等待"}`;
  byId("cloud-status").textContent = online && cloudId
    ? `点云 · 实时 (${result.cloudCount.toLocaleString("en-US")} 点)` : "点云 · 等待";
  byId("plan-status").textContent = online && planId
    ? `规划 · ${pending ? "机头对齐候选" : "执行器当前轨迹"}`
    : `规划 · ${online && result?.executor ? result.executor : "等待"}`;
  const usingCache = loadedMap && (offline || result?.localMap || catalog.some(map => map.key === loadedMap.key));
  byId("map-status").textContent = result?.mapError || (loadedMap ? (usingCache ? "地图 · 本机缓存" : "地图 · Orange Pi") : mapError);
  const notice = byId("notice");
  notice.textContent = (offline || manualMap || !watching() || !result?.mapKey) && loadedMap ? ""
    : (offline || manualMap || !result?.mapKey) && wantedMap ? mapError || "正在加载地图"
    : !online && watching() ? "等待 Orange Pi"
    : !result?.mapKey ? result?.mapError || "等待定位地图"
      : !loadedMap ? mapError || "正在加载地图"
        : !result.aligned ? "等待重定位对齐"
          : !pose ? "等待位姿" : "";
  notice.hidden = !notice.textContent;
  const showPose = visible() && pose;
  byId("phase").textContent = !showPose ? "等待地图与定位对齐"
    : result.poseSource === "navigation" ? "/drone_nav/odom" : "/fastlio/odom_world";
  const heading = Math.atan2(forward.y, forward.x) * 180 / Math.PI;
  [pos.x, pos.y, pos.z, heading].forEach((value, i) => {
    byId(["px", "py", "pz", "yaw"][i]).textContent = showPose ? value.toFixed(i === 3 ? 1 : 2) : "—";
  });
  applyVisibility();
  inspection?.poseChanged();
}

function consume(result) {
  lastResult = result;
  lastResponse = performance.now();
  online = true;
  syncMap();
  if (epoch !== result.epoch) {
    epoch = result.epoch;
    clearMotion();
  }
  const trailChanged = trailId !== result.trailId;
  const planChanged = planId !== result.planId;
  const cloudChanged = cloudId !== result.cloudId;
  if (trailChanged) {
    if (result.trailReset) trail = [];
    trail.push(...result.trail);
    if (trail.length > 4800) trail.splice(0, trail.length - 4800);
    trailFlat = trail.flat();
  }
  if (planChanged) plan = result.plan ? asFloats(result.plan) : new Float32Array(0);
  if (cloudChanged) cloud = result.cloud ? asFloats(result.cloud) : new Float32Array(0);
  trailId = result.trailId;
  planId = result.planId;
  cloudId = result.cloudId;
  pending = result.pending;
  pose = result.pose;
  if (pose) {
    pos.fromArray(pose, 1);
    orient.fromArray(pose, 4).normalize();
    forward.set(1, 0, 0).applyQuaternion(orient);
  }
  updateGeometry(trailChanged, planChanged, cloudChanged);
  paintStatus();
}

async function poll() {
  if (!watching() || telemetryRequest) return;
  const controller = new AbortController();
  telemetryRequest = controller;
  const timer = setTimeout(() => controller.abort(), 950);
  try {
    const params = new URLSearchParams({ epoch, trail: trailId, plan: planId, cloud: cloudId });
    const response = await fetch("/telemetry?" + params, { signal: controller.signal });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const result = await response.json();
    if (telemetryRequest !== controller) return;
    if (result.offline) {
      offline = true;
      syncMap();
      paintStatus();
      void loadCatalog();
      return;
    }
    consume(result);
  } catch {
    if (telemetryRequest === controller && (performance.now() - lastResponse > 1050 || !lastResponse)) {
      online = false;
      paintStatus();
    }
  } finally {
    clearTimeout(timer);
    if (telemetryRequest === controller) {
      telemetryRequest = null;
      if (watching()) telemetryTimer = setTimeout(poll, online ? 100 : 800);
    }
  }
}

function draw() {
  if (dirty && rig) {
    rig.renderer.render(rig.world, rig.camera);
    rig.labels.render(rig.world, rig.camera);
    dirty = false;
  }
  requestAnimationFrame(draw);
}

try {
  rig = scene();
} catch {
  const error = byId("render-error");
  error.hidden = false;
  error.textContent = "三维视图需要 WebGL 2。右侧俯视和状态仍可使用。";
}
inspection = createInspection({
  rig, viewport, focusRoute, redraw: () => { dirty = true; }, setView: mode => setView(mode, true),
  currentPose: () => visible() && pose ? { position: pos.toArray(), yaw: Math.atan2(forward.y, forward.x) * 180 / Math.PI } : null,
});
tasks = createTasks({ rig, focusRoute, redraw: () => { dirty = true; }, syncTelemetry,
  pinMap: () => {
    manualMap = loadedMap?.key ?? "";
    byId("display-map").value = manualMap;
  },
});
byId("display-map").addEventListener("change", event => {
  manualMap = event.target.value;
  syncMap();
  paintStatus();
});
setView("room");
updateLayers();
void loadCatalog();
new ResizeObserver(drawDetail).observe(detail);
views.forEach(button => button.addEventListener("click", () => setView(button.dataset.view)));
toggles.forEach(input => input.addEventListener("change", updateLayers));
for (const id of ["live-cloud", "planned", "traveled"]) {
  byId(id).addEventListener("change", applyVisibility);
}
requestAnimationFrame(draw);
startDroneStatus();
for (const id of ["tab-live", "tab-inspection"]) {
  byId(id).addEventListener("click", syncTelemetry);
}
document.addEventListener("visibilitychange", syncTelemetry);
window.addEventListener("pagehide", pauseTelemetry);
window.addEventListener("pageshow", syncTelemetry);
syncTelemetry();
