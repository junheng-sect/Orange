import {
  ArrowHelper, Group, Mesh, MeshBasicMaterial, Plane, Raycaster, SphereGeometry,
  Vector2, Vector3,
} from "three";
import { CSS2DObject } from "three/addons/renderers/CSS2DRenderer.js";
import { wideLine } from "./scene_parts.js";

const byId = id => document.getElementById(id);
const green = "#8ee1ad";

async function api(path, body) {
  const options = body && {
    method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body),
  };
  const response = await fetch(path, options);
  if (response.ok) return response.json();
  const detail = await response.json().catch(() => null);
  throw new Error(detail?.error || `HTTP ${response.status}`);
}

export function createInspection({ rig, viewport, currentPose, redraw, setView, focusRoute }) {
  const panel = byId("inspection-panel");
  const form = byId("point-editor");
  const fields = ["x", "y", "z", "yaw", "dwell"].map(key => byId("point-" + key));
  const list = byId("point-list");
  const status = byId("inspection-status");
  const cruise = byId("cruise-z");
  const markers = new Group();
  rig?.world.add(markers);
  const ray = new Raycaster();
  let map = null;
  let record = null;
  let editId = null;
  let picking = false;
  let busy = false;
  let tab = "live";
  let loadSequence = 0;
  let startClick = null;

  function activePreview() {
    return record?.preview?.cruise === cruise.valueAsNumber ? record.preview : null;
  }

  function stopPicking() {
    picking = false;
    viewport.dataset.picking = "false";
    byId("pick-help").hidden = true;
    byId("pick-point").setAttribute("aria-pressed", "false");
  }

  function cancelEdit() {
    editId = null;
    form.hidden = true;
    stopPicking();
  }

  function showTab(next) {
    tab = next;
    panel.hidden = tab !== "inspection";
    byId("live-panel").hidden = !panel.hidden;
    byId("tab-live").setAttribute("aria-pressed", String(panel.hidden));
    byId("tab-inspection").setAttribute("aria-pressed", String(!panel.hidden));
    if (panel.hidden) stopPicking();
    paintScene();
  }

  function paintScene() {
    for (const group of [...markers.children]) {
      group.traverse(object => {
        object.geometry?.dispose();
        object.material?.dispose();
        if (object.isCSS2DObject) object.element.remove();
      });
      group.removeFromParent();
    }
    const show = tab === "inspection" && byId("manual-preview").open && record;
    const result = activePreview();
    document.querySelector(".preview-key").hidden = !(show && result?.legs?.length);
    if (show) {
      const extent = map.bounds[1][0] - map.bounds[0][0];
      const size = Math.max(.045, Math.min(.25, extent / 130));
      for (const [index, point] of record.points.entries()) {
        const color = point.selected ? green : "#a3b9c9";
        const symbol = new Group();
        symbol.position.fromArray(point.position);
        const dot = new Mesh(new SphereGeometry(size, 12, 8), new MeshBasicMaterial({ color }));
        const angle = point.yaw * Math.PI / 180;
        const arrow = new ArrowHelper(new Vector3(Math.cos(angle), Math.sin(angle), 0),
                                      new Vector3(0, 0, .02), size * 4, color, size, size * .7);
        // ArrowHelper shares its geometry across instances.
        arrow.traverse(object => {
          if (object.geometry) object.geometry = object.geometry.clone();
        });
        const label = document.createElement("span");
        label.className = "waypoint-label";
        label.textContent = `${index + 1}. ${point.name}`;
        const annotation = new CSS2DObject(label);
        annotation.position.set(0, 0, size * 2.4);
        symbol.add(dot, arrow, annotation);
        markers.add(symbol);
      }
      for (const leg of result?.legs || []) {
        if (leg.positions.length >= 2) {
          markers.add(wideLine(leg.positions.flat(), { color: green, linewidth: 3 }));
        }
      }
    }
    redraw();
  }

  function pointButton(label, title, action, disabled = false) {
    const button = document.createElement("button");
    button.type = "button";
    button.textContent = label;
    button.title = title;
    button.setAttribute("aria-label", title);
    button.disabled = busy || disabled;
    button.addEventListener("click", action);
    return button;
  }

  function paint() {
    const points = record?.points || [];
    const chosen = points.filter(point => point.selected).length;
    list.replaceChildren();
    byId("new-point").disabled = !record || busy;
    byId("generate-preview").disabled = !record || busy || chosen < 2;
    byId("focus-preview").disabled = !activePreview()?.legs.length;
    byId("inspection-empty").hidden = points.length > 0;
    byId("inspection-empty").textContent = !map ? "选择一张缓存地图开始。"
      : record ? "暂无点位。新增后可勾选并调整顺序。" : "正在读取巡检点…";
    for (const [i, point] of points.entries()) {
      const row = document.createElement("li");
      const label = document.createElement("label");
      label.className = "point-choice";
      const check = document.createElement("input");
      check.type = "checkbox";
      check.checked = point.selected;
      check.disabled = busy;
      check.addEventListener("change", () => void persist(points.map((p, index) =>
        index === i ? { ...p, selected: check.checked } : p)));
      label.append(check, document.createTextNode(`${i + 1}. ${point.name}`));
      row.append(label, pointButton("编辑", "编辑 " + point.name, () => edit(point)),
                 pointButton("↑", "上移 " + point.name, () => move(i, -1), i === 0),
                 pointButton("↓", "下移 " + point.name, () => move(i, 1), i === points.length - 1));
      list.append(row);
    }
    byId("point-list").hidden = !points.length;
    byId("point-list").setAttribute("aria-label", "按巡检顺序排列的点位");
    byId("pick-point").disabled = !rig || busy;
    cruise.disabled = busy;
    poseChanged();
    for (const input of form.querySelectorAll("input, button[type=submit], #delete-point")) input.disabled = busy;
    paintScene();
  }

  function describe() {
    const result = activePreview();
    const errors = byId("preview-errors");
    errors.replaceChildren();
    if (!result) {
      status.textContent = record?.preview ? "巡航高度已修改，请重新生成路线。" : "";
      return;
    }
    for (const message of result.errors) {
      const item = document.createElement("li");
      item.textContent = message;
      errors.append(item);
    }
    status.textContent = result.complete
      ? `路线已生成 · ${result.legs.length} 段 · ${result.length.toFixed(2)} m · 停留 ${result.dwell} s`
      : `无完整路线 · ${result.legs.length} 段可预览`;
  }

  async function load(next) {
    const seq = ++loadSequence;
    map = next;
    record = null;
    busy = false;
    cancelEdit();
    describe();
    cruise.value = "";
    paint();
    if (!next) return;
    try {
      const result = await api("/inspection?key=" + encodeURIComponent(next.key));
      if (seq !== loadSequence) return;
      record = result;
      const midpoint = (map.bounds[0][2] + map.bounds[1][2]) / 2;
      cruise.value = result.preview?.cruise ?? result.points[0]?.position[2] ?? midpoint.toFixed(2);
      describe();
      paint();
    } catch (error) {
      if (seq === loadSequence) {
        byId("inspection-empty").textContent = "巡检点读取失败";
        status.textContent = error.message;
      }
    }
  }

  async function mutate(path, data, after) {
    if (!record || busy) return;
    const seq = loadSequence;
    busy = true;
    paint();
    try {
      const result = await api(path, { key: map.key, revision: record.revision, ...data });
      if (seq !== loadSequence) return;
      record = result;
      describe();
      after?.();
    } catch (error) {
      if (seq === loadSequence) {
        status.textContent = error.message;
        if (error.message.includes("重新加载")) {
          await load(map);
          status.textContent = error.message;
        }
      }
    } finally {
      if (seq === loadSequence) {
        busy = false;
        paint();
      }
    }
  }

  function persist(points, after) {
    return mutate("/inspection", { points }, after);
  }

  function move(i, delta) {
    const points = [...record.points];
    [points[i], points[i + delta]] = [points[i + delta], points[i]];
    void persist(points);
  }

  function edit(point) {
    editId = point?.id ?? null;
    byId("point-name").value = point?.name ?? "";
    const z = cruise.valueAsNumber;
    const values = [...(point?.position || ["", "", Number.isFinite(z) ? z : ""]), point?.yaw ?? 0, point?.dwell ?? 2];
    fields.forEach((input, index) => { input.value = values[index]; });
    byId("delete-point").hidden = !editId;
    form.hidden = false;
    byId("point-name").focus();
  }

  form.addEventListener("submit", event => {
    event.preventDefault();
    if (!record || !form.reportValidity()) return;
    const [x, y, z, yaw, dwell] = fields.map(input => input.valueAsNumber);
    const name = byId("point-name").value.trim();
    const existing = record.points.find(p => p.id === editId);
    const point = { id: editId || crypto.randomUUID(), name, position: [x, y, z], yaw, dwell, selected: existing?.selected ?? true };
    const points = editId ? record.points.map(p => p.id === editId ? point : p) : [...record.points, point];
    void persist(points, cancelEdit);
  });
  byId("new-point").addEventListener("click", () => edit());
  byId("cancel-point").addEventListener("click", cancelEdit);
  byId("delete-point").addEventListener("click", () => void persist(record.points.filter(p => p.id !== editId), cancelEdit));
  byId("manual-preview").addEventListener("toggle", event => {
    if (!event.target.open) stopPicking();
    paintScene();
  });
  byId("tab-live").addEventListener("click", () => showTab("live"));
  byId("tab-inspection").addEventListener("click", () => showTab("inspection"));
  function focus() {
    const positions = activePreview()?.legs.flatMap(leg => leg.positions);
    if (positions?.length) focusRoute(positions);
  }

  cruise.addEventListener("input", () => { describe(); paint(); });
  byId("focus-preview").addEventListener("click", focus);
  byId("generate-preview").addEventListener("click", () => {
    if (!Number.isFinite(cruise.valueAsNumber)) {
      status.textContent = "请输入巡航 Z";
      return;
    }
    status.textContent = "正在检查路线…";
    void mutate("/preview", { cruise: cruise.valueAsNumber }, focus);
  });
  byId("use-position").addEventListener("click", () => {
    const pose = currentPose();
    if (pose) {
      [...pose.position, pose.yaw].forEach((value, index) => { fields[index].value = value.toFixed(3); });
    }
  });
  byId("pick-point").addEventListener("click", () => {
    if (picking) return stopPicking();
    if (!Number.isFinite(fields[2].valueAsNumber)) {
      status.textContent = "先填写点位 Z，再从地图点选 XY";
      return;
    }
    picking = true;
    viewport.dataset.picking = "true";
    byId("pick-help").hidden = false;
    byId("pick-point").setAttribute("aria-pressed", "true");
    setView("top");
  });
  viewport.addEventListener("pointerdown", event => {
    if (picking && event.button === 0) startClick = [event.clientX, event.clientY];
  });
  viewport.addEventListener("pointerup", event => {
    const start = startClick;
    startClick = null;
    if (!picking || !start || Math.hypot(start[0] - event.clientX, start[1] - event.clientY) > 5) return;
    const rect = rig.renderer.domElement.getBoundingClientRect();
    const pointer = new Vector2(2 * (event.clientX - rect.left) / rect.width - 1,
                                1 - 2 * (event.clientY - rect.top) / rect.height);
    ray.setFromCamera(pointer, rig.camera);
    const plane = new Plane(new Vector3(0, 0, 1), -fields[2].valueAsNumber);
    const point = ray.ray.intersectPlane(plane, new Vector3());
    if (!point) return;
    fields[0].value = point.x.toFixed(3);
    fields[1].value = point.y.toFixed(3);
    stopPicking();
  });
  document.addEventListener("keydown", event => { if (event.key === "Escape") stopPicking(); });

  function poseChanged() {
    byId("use-position").disabled = busy || !currentPose();
  }

  paint();
  return { load, poseChanged };
}
