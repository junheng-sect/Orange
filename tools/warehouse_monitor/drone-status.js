const byId = id => document.getElementById(id);
const LEVEL = { ok: "ok", bad: "bad", unknown: "warn" };

function renderChecks(entries) {
  const list = byId("flight-checks");
  for (const [index, [text, level, detail = ""]] of entries.entries()) {
    const chip = list.children[index] || list.appendChild(document.createElement("span"));
    chip.className = "flight-item";
    chip.dataset.level = level;
    chip.dataset.detail = detail || "";
    chip.tabIndex = detail ? 0 : -1;
    chip.title = detail ? text + " · " + detail : text;
    if (chip.textContent !== text) chip.textContent = text;
  }
  while (list.children.length > entries.length) list.lastElementChild.remove();
}

function batteryText(battery) {
  if (battery.connected == null) return "无数据";
  if (!battery.connected) return "未接电池";
  const values = [];
  if (Number.isFinite(battery.percent)) values.push(battery.percent + "%");
  if (Number.isFinite(battery.voltage)) values.push(battery.voltage.toFixed(2) + " V");
  if (Number.isFinite(battery.current)) values.push(battery.current.toFixed(1) + " A");
  return values.join(" · ") || "无数据";
}

function render(status) {
  const { lamp, battery, flight, positioning, checks, master } = status;
  const badge = byId("flight-lamp");
  badge.textContent = lamp.text;
  badge.dataset.level = LEVEL[lamp.level] || "warn";
  const batteryLabel = byId("flight-battery");
  batteryLabel.textContent = "电池 " + batteryText(battery);
  batteryLabel.dataset.level = battery.connected === false ? "warn" : "";
  const mode = flight.connected ? [flight.mode, flight.armed ? "已解锁" : "未解锁", flight.failsafe && "失效保护"]
    .filter(Boolean).join(" · ") : "—";
  const entries = [["模式 " + mode, flight.failsafe ? "bad" : "", flight.landed]];
  for (const check of checks) {
    const [summary, level] = check.ok === true ? ["通过", "ok"] : check.required === false
      ? ["参考", "muted"] : check.ok === false ? ["异常", "bad"] : ["未知", "warn"];
    entries.push([check.name + " " + summary, level, check.text]);
  }
  const hz = positioning?.vision_hz;
  entries.push(["外部视觉 " + (Number.isFinite(hz) ? hz + " Hz" : "—"), ""]);
  entries.push([master.ok ? "板端 已连接" : "板端 未连接", master.ok ? "ok" : "bad", master.error]);
  if (status.unhealthy?.length) entries.push(["传感器异常", "bad", status.unhealthy.join("、")]);
  renderChecks(entries);
}

export function startDroneStatus() {
  let timer, request;
  async function poll() {
    if (document.hidden || request) return;
    const controller = new AbortController();
    request = controller;
    const deadline = setTimeout(() => controller.abort(), 2000);
    try {
      const response = await fetch("/drone-status", { signal: controller.signal });
      if (!response.ok) throw new Error("状态桥不可用");
      const status = await response.json();
      if (request === controller) render(status);
    } catch {
      if (request === controller) {
        byId("flight-lamp").textContent = "状态桥不可用";
        byId("flight-lamp").dataset.level = "warn";
        byId("flight-battery").textContent = "";
        renderChecks([]);
      }
    } finally {
      clearTimeout(deadline);
      if (request === controller) {
        request = null;
        if (!document.hidden) timer = setTimeout(poll, 1000);
      }
    }
  }
  function sync() {
    clearTimeout(timer);
    request?.abort();
    request = null;
    if (!document.hidden) void poll();
  }
  document.addEventListener("visibilitychange", sync);
  window.addEventListener("pagehide", () => {
    clearTimeout(timer);
    request?.abort();
    request = null;
  });
  window.addEventListener("pageshow", sync);
  sync();
}
