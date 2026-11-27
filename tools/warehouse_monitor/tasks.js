import { ConeGeometry, Group, Mesh, MeshBasicMaterial, SphereGeometry, Vector3 } from "three";
import { CSS2DObject } from "three/addons/renderers/CSS2DRenderer.js";
import { wideLine } from "./scene_parts.js";

const byId = id => document.getElementById(id);
const terminal = new Set(["done", "failed", "cancelled", "expired"]);
const labels = {
  queued: "等待上电", waiting_fcu: "等待飞控连接", preparing: "定位与预检",
  wait_pilot: "等待遥控起飞", wait_offboard: "等待遥控切 OFFBOARD",
  wait_arm: "等待遥控解锁", running: "执行中", action_done: "动作结束，等待收尾",
  stopping: "正在收尾", done: "已收尾", failed: "任务失败", cancelled: "已取消",
  expired: "任务已过期", delivered: "已送达板端",
};
const purple = "#dabdff";
const up = new Vector3(0, 1, 0);

function addDirectionMarkers(overlay, points) {
  for (let index = 1; index < points.length; index++) {
    const start = new Vector3(...points[index - 1]);
    const direction = new Vector3(...points[index]).sub(start);
    const length = direction.length();
    if (length < 1e-6) continue;
    const height = Math.min(.56, length * .26);
    const arrow = new Mesh(new ConeGeometry(height * .28, height, 8),
                           new MeshBasicMaterial({ color: "#f2dbff" }));
    arrow.position.copy(start).addScaledVector(direction, .62);
    arrow.quaternion.setFromUnitVectors(up, direction.normalize());
    overlay.add(arrow);
  }
}

function closesRoute(points) {
  const end = points[points.length - 1];
  return points.length > 1 && points[0].every((value, axis) => value === end[axis]);
}

function stopLabels(task, closes) {
  let number = 0;
  return new Map([...task.stops].sort((a, b) => a.index - b.index).map(stop => {
    let badge = "";
    if (task.fixed) {
      if (closes && stop.index === task.points.length - 1) badge = "返航";
      else if (stop.index === 0 && !stop.name) badge = "起点";
      else badge = String(++number);
    }
    const name = stop.name ?? `P${String(stop.index).padStart(2, "0")}`;
    return [stop.index, {
      badge,
      description: `${badge ? `${badge} · ` : ""}${name} · ${stop.seconds} s`,
    }];
  }));
}

function drawRelative(canvas, task) {
  const side = task.points.every(p => p[1] === 0) && task.points.some(p => p[2] !== 0);
  const points = task.points.map(p => [p[0], p[side ? 2 : 1]]);
  const width = 360, height = 125;
  canvas.width = width;
  canvas.height = height;
  const ctx = canvas.getContext("2d");
  const xs = points.map(p => p[0]), ys = points.map(p => p[1]);
  const spanX = Math.max(...xs) - Math.min(...xs), spanY = Math.max(...ys) - Math.min(...ys);
  const centerX = (Math.max(...xs) + Math.min(...xs)) / 2;
  const centerY = (Math.max(...ys) + Math.min(...ys)) / 2;
  const scale = Math.min(260 / Math.max(.5, spanX), 70 / Math.max(.5, spanY));
  const projected = points.map(p => [width / 2 + (p[0] - centerX) * scale, 55 - (p[1] - centerY) * scale]);
  ctx.strokeStyle = purple;
  ctx.lineWidth = 2;
  ctx.beginPath();
  projected.forEach(([x, y], i) => i ? ctx.lineTo(x, y) : ctx.moveTo(x, y));
  ctx.stroke();
  projected.forEach(([x, y], i) => {
    ctx.fillStyle = i === 0 ? "#fff" : purple;
    ctx.beginPath();
    ctx.arc(x, y, i === 0 ? 5 : 3.5, 0, 2 * Math.PI);
    ctx.fill();
  });
  ctx.font = '13px sans-serif';
  ctx.fillStyle = "#b7c9d7";
  ctx.fillText(side ? "侧视 · 向右为前方 · 向上为升高" : "俯视 · 向右为机头前方", 12, 114);
}

export function createTasks({ rig, focusRoute, redraw, syncTelemetry }) {
  const selector = byId("task-choice");
  const dialog = byId("task-auto-dialog");
  const overlay = new Group();
  const scanOverlay = new Group();
  rig?.world.add(overlay, scanOverlay);
  let scanKey = "";
  let data = { maps: {}, tasks: [] };
  let map = null;
  let status = null;
  let busy = false;
  let confirmation = null;
  let nextId = null;
  let requestSerial = 0;
  let timer;

  const selected = () => data.tasks.find(task => task.id === selector.value);
  const panelVisible = () => !byId("inspection-panel").hidden;

  function report(message = "") {
    const info = byId("task-error");
    info.hidden = !message;
    info.textContent = message;
  }

  function paintRoute(task, closes, stops) {
    for (const child of [...overlay.children]) {
      child.geometry?.dispose();
      child.material?.dispose();
      if (child.isCSS2DObject) child.element.remove();
      overlay.remove(child);
    }
    const shown = Boolean(rig && task?.fixed && panelVisible() && data.maps[map?.id]?.key === map?.key);
    document.querySelector(".task-key").hidden = !shown;
    if (shown) {
      overlay.add(wideLine(task.points.flat(), { color: purple, linewidth: 3 }));
      addDirectionMarkers(overlay, task.points);
      for (const [index, point] of task.points.entries()) {
        const last = index === task.points.length - 1;
        if (closes && last) continue;
        const stop = stops.get(index) ?? (index === 0 && closes ? stops.get(task.points.length - 1) : undefined);
        const marker = new Mesh(new SphereGeometry(stop ? .07 : .04, 10, 8),
                                new MeshBasicMaterial({ color: stop ? "#fff0bc" : purple }));
        marker.position.fromArray(point);
        overlay.add(marker);
        const badge = index === 0 && closes ? "起/返"
          : stop?.badge || (index === 0 ? "起" : last ? "终" : "");
        if (badge) {
          const span = document.createElement("span");
          span.className = "task-waypoint";
          span.textContent = badge;
          const label = new CSS2DObject(span);
          label.center.set(.5, 1.3);
          label.position.fromArray(point);
          overlay.add(label);
        }
      }
    }
    redraw();
  }

  function paintDetails() {
    const task = selected();
    const closes = task ? closesRoute(task.points) : false;
    const stops = task ? stopLabels(task, closes) : new Map();
    byId("task-details").hidden = !task;
    byId("task-damper-row").hidden = task?.id !== "test_indoor3_5";
    if (task) {
      byId("task-trigger").textContent = task.manual ? "全程遥控手飞"
        : task.automatic
        ? "地面遥控解锁后自动切 OFFBOARD 并起飞"
        : "遥控定点起飞，悬停后手动切 OFFBOARD";
      byId("task-route").textContent = task.route;
      byId("task-finish").textContent = task.finish;
      byId("task-focus").hidden = !task.fixed || !rig || data.maps[map?.id]?.key !== map?.key;
      const diagram = byId("task-diagram");
      diagram.hidden = task.fixed || !task.points.length;
      if (!diagram.hidden) drawRelative(diagram, task);
      const stopList = byId("task-stops");
      stopList.hidden = !stops.size;
      stopList.replaceChildren(...Array.from(stops.values(), stop => {
        const li = document.createElement("li");
        li.textContent = stop.description;
        return li;
      }));
    }
    paintRoute(task, closes, stops);
    paintStatus();
  }

  function populate() {
    const previous = selector.value;
    selector.length = 1;
    for (const task of data.tasks.filter(task => task.mapId === map?.id)) {
      selector.add(new Option(`${task.label} · ${task.title}`, task.id));
    }
    selector.value = previous;
    if (selector.selectedIndex < 0) selector.selectedIndex = 0;
    const empty = byId("task-empty");
    const stale = data.maps[map?.id] && data.maps[map.id].key !== map.key;
    empty.hidden = selector.length > 1 && !stale;
    empty.textContent = stale ? "任务绑定了这张地图的另一版本，请同步对应地图。"
      : map ? "这张地图没有预设任务。" : "先选择并加载地图。";
    selector.disabled = selector.length < 2;
    paintDetails();
  }

  function paintScan(active) {
    const scan = active?.scan;
    const route = panelVisible() && data.maps[map?.id]?.key === map?.key &&
      active?.mapKey === map?.key ? scan?.route : null;
    const key = JSON.stringify(route || null);
    if (key === scanKey) return;
    scanKey = key;
    for (const item of [...scanOverlay.children]) {
      item.geometry?.dispose();
      item.material?.dispose();
      if (item.isCSS2DObject) item.element.remove();
      scanOverlay.remove(item);
    }
    if (route?.ids?.length && route.points.length === route.ids.length + 2) {
      scanOverlay.add(wideLine(route.points.flat(), { color: "#6ee7d0", linewidth: 3 }));
      for (const [i, id] of route.ids.entries()) {
        const point = route.points[i + 1];
        const marker = new Mesh(new SphereGeometry(.045, 10, 8),
                                new MeshBasicMaterial({ color: "#6ee7d0" }));
        marker.position.fromArray(point);
        scanOverlay.add(marker);
        const span = document.createElement("span");
        span.className = "task-waypoint";
        span.textContent = `${i + 1} · Tag ${id}`;
        const label = new CSS2DObject(span);
        label.center.set(.5, 1.5);
        label.position.fromArray(point);
        scanOverlay.add(label);
      }
    }
    redraw();
  }

  function paintStatus() {
    const request = status?.request;
    const active = status?.board?.active;
    const current = active ?? request;
    paintScan(active);
    const outstanding = Boolean(active || (request && !terminal.has(request.state)));
    byId("task-confirm").disabled = busy || !status?.enabled || !selected()
      || data.maps[map?.id]?.key !== map?.key || outstanding;
    byId("task-confirm").textContent = status?.enabled === false
      ? "离线预览" : "确认任务，等待上电";
    byId("task-current").hidden = !current;
    if (!current) return;
    const task = data.tasks.find(task => task.id === current.taskId);
    byId("task-active-label").textContent = task
      ? `${task.id.startsWith("test_indoor") ? "室内" : "室外"} ${task.label} · ${task.title}` : "";
    byId("task-phase").textContent = (labels[current.state] ?? current.state)
      + (active && !status.online ? " · 板端连接中断，显示上次状态" : "");
    const mode = status.online && active?.mode ? ` · 飞控 ${active.mode}` : "";
    byId("task-message").textContent = ((request?.state === "queued" && active?.id !== request.id)
      ? request.message : status?.board?.error ?? active?.message ?? request?.message ?? "") + mode;
    const scan = active?.scan;
    if (scan && current.state === "running") {
      const tag = scan.tagId != null ? ` · Tag ${scan.tagId}` : "";
      const order = scan.route?.ids?.length ? ` · 顺序 ${scan.route.ids.join(" → ")}` : "";
      byId("task-message").textContent =
        `${scan.observation} · ${scan.phase}${tag} · 发现 ${scan.discovered} 张 · 成功 ${scan.success} · 失败 ${scan.failed}${order}${mode}`;
    }
    byId("task-cancel").hidden = !(active ? active.canCancel : request?.state === "queued");
    const stop = byId("task-stop");
    const actionDone = active?.state === "action_done";
    stop.hidden = !actionDone && (!active?.canStop || !status.online);
    stop.disabled = busy || !status.online || !active?.canStop;
    byId("task-cancel").disabled = busy;
    const finishHint = byId("task-finish-hint");
    finishHint.hidden = !actionDone;
    if (actionDone) {
      finishHint.textContent = !status.online
        ? "等待 Orange Pi 重连后再收尾。"
        : !active.canStop && active.mode === "OFFBOARD"
          ? "着地上锁后，用遥控器切回定点模式（POSCTL），关通道 6，再点「结束任务并收尾」。"
          : !active.canStop
            ? "等待飞控确认着地上锁；关通道 6 后点「结束任务并收尾」。"
            : "关通道 6 后点「结束任务并收尾」。显示「已收尾」后再断电。";
    }
  }

  function accept(snapshot) {
    const before = watchingFlight();
    status = snapshot;
    paintStatus();
    if (before !== watchingFlight()) syncTelemetry();
  }

  async function poll() {
    clearTimeout(timer);
    if (document.hidden || !panelVisible()) return;
    const serial = ++requestSerial;
    try {
      const response = await fetch("/task");
      if (!response.ok) throw Error("读取任务状态失败");
      const snapshot = await response.json();
      if (serial === requestSerial) accept(snapshot);
    } catch (error) {
      if (serial === requestSerial) {
        if (status) accept({ ...status, online: false });
        else report(error.message);
      }
    }
    if (serial === requestSerial) timer = setTimeout(poll, 1500);
  }

  async function action(verb, body) {
    if (busy) return;
    busy = true;
    requestSerial++;
    clearTimeout(timer);
    report();
    paintStatus();
    try {
      const response = await fetch(`/task/${verb}`, {
        method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body),
      });
      const result = await response.json();
      if (!response.ok) throw Error(result.error ?? "任务请求失败");
      accept(result);
      if (verb === "submit") {
        nextId = null;
        byId("task-damper").value = "";
      }
    } catch (error) {
      report(error.message);
    } finally {
      busy = false;
      paintStatus();
      void poll();
    }
  }

  function randomId() {
    return Array.from(crypto.getRandomValues(new Uint8Array(16)),
                      value => value.toString(16).padStart(2, "0")).join("");
  }

  function payload(task, confirmAuto) {
    nextId ??= randomId();
    const body = { id: nextId, taskId: task.id, mapKey: map.key, confirmAuto };
    const input = byId("task-damper");
    if (task.id === "test_indoor3_5" && input.value !== "") body.damperDeg = input.valueAsNumber;
    return body;
  }

  function watchingFlight() {
    return panelVisible() && Boolean(status?.board?.active)
      && status.board.active.state !== "waiting_fcu";
  }

  byId("task-damper").addEventListener("input", () => { nextId = null; });
  selector.addEventListener("change", () => {
    byId("task-damper").value = "";
    nextId = null;
    report();
    paintDetails();
  });
  byId("task-confirm").addEventListener("click", () => {
    const task = selected();
    if (!task) return;
    if (task.id === "test_indoor3_5" && !byId("task-damper").reportValidity()) return;
    const body = payload(task, task.automatic);
    if (task.automatic) {
      confirmation = body;
      dialog.returnValue = "cancel";
      dialog.showModal();
    } else {
      void action("submit", body);
    }
  });
  dialog.addEventListener("close", () => {
    if (dialog.returnValue === "confirm" && confirmation && confirmation.mapKey === map?.key
        && confirmation.taskId === selected()?.id) {
      void action("submit", confirmation);
    }
    confirmation = null;
  });
  byId("task-cancel").addEventListener("click", () => {
    const id = status?.board?.active?.id ?? status?.request?.id;
    if (id) void action("cancel", { id });
  });
  byId("task-stop").addEventListener("click", () => {
    const id = status?.board?.active?.id;
    if (id) void action("stop", { id });
  });
  byId("task-focus").addEventListener("click", () => {
    const task = selected();
    if (task?.fixed) focusRoute(task.points);
  });
  for (const id of ["tab-live", "tab-inspection"]) {
    byId(id).addEventListener("click", () => {
      paintDetails();
      void poll();
    });
  }
  document.addEventListener("visibilitychange", () => void poll());
  fetch("/tasks").then(response => {
    if (!response.ok) throw Error("任务目录不可用");
    return response.json();
  }).then(value => {
    data = value;
    populate();
    void poll();
  }).catch(error => report(error.message));

  return {
    setMap(value) {
      if (map?.key === value?.key) return;
      map = value;
      selector.value = "";
      nextId = null;
      report();
      populate();
    },
    watchingFlight,
  };
}
