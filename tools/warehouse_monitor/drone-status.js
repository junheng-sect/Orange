const byId = id => document.getElementById(id);
const LEVEL = { ok: "ok", bad: "bad", unknown: "warn" };

function row(list, label, text, level = "") {
  const item = document.createElement("div");
  item.dataset.level = level;
  const title = document.createElement("b");
  title.textContent = label;
  item.append(title, document.createTextNode(" " + text));
  list.append(item);
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
  const batteryLabel = batteryText(battery);
  badge.textContent = lamp.text;
  badge.dataset.level = LEVEL[lamp.level] || "warn";
  byId("flight-battery").textContent = "电池 " + batteryLabel;
  const list = byId("flight-checks");
  list.replaceChildren();
  row(list, "飞控", flight.connected
    ? [flight.armed ? "已解锁" : "未解锁", flight.mode, flight.landed,
       flight.failsafe && "失效保护"].filter(Boolean).join(" · ")
    : "未连接", flight.failsafe ? "bad" : "");
  row(list, "电池", batteryLabel);
  for (const check of checks) {
    const [mark, level] = check.ok === true ? ["✓", "ok"] : check.required === false
      ? ["–", "muted"] : check.ok === false ? ["✗", "bad"] : ["?", "warn"];
    row(list, mark + " " + check.name, check.text, level);
  }
  if (Array.isArray(status.unhealthy) && status.unhealthy.length) {
    row(list, "传感器异常", status.unhealthy.join("、"), "bad");
  }
  if (Number.isFinite(positioning?.vision_hz)) {
    row(list, "外部视觉", positioning.vision_hz + " Hz");
  }
  row(list, "板端", master.ok ? "已连接" : "连不上");
}

export function startDroneStatus() {
  const details = byId("flight-status");
  if (!details) return;
  let timer, request;
  async function poll() {
    if (!enabled() || request) return;
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
        byId("flight-checks").replaceChildren();
        row(byId("flight-checks"), "状态桥", "检查电脑上的 drone-status.service");
      }
    } finally {
      clearTimeout(deadline);
      if (request === controller) {
        request = null;
        if (enabled()) timer = setTimeout(poll, 1000);
      }
    }
  }
  function enabled() {
    return !document.hidden && byId("tab-live").getAttribute("aria-pressed") === "true";
  }
  function sync() {
    clearTimeout(timer);
    request?.abort();
    request = null;
    if (enabled()) void poll();
  }
  document.addEventListener("visibilitychange", sync);
  window.addEventListener("pagehide", () => {
    clearTimeout(timer);
    request?.abort();
    request = null;
  });
  window.addEventListener("pageshow", sync);
  byId("tab-live").addEventListener("click", sync);
  byId("tab-inspection").addEventListener("click", sync);
  sync();
}
