(() => {
  const byId = id => document.getElementById(id);
  const picker = byId("scan-run");
  const list = byId("scan-rows");
  const statusNames = {
    SUCCEEDED: "已完成", ABORTED: "任务中止", INTERRUPTED: "任务中断",
    RUNNING: "进行中", WAITING: "等待任务记录", NO_RECORD: "已结束 · 无扫码记录",
  };
  const rowNames = {
    failed: "失败", running: "读取中", aborted: "中断", skipped: "跳过", unattempted: "未尝试",
  };
  const clock = new Intl.DateTimeFormat("zh-CN", {timeZone: "Asia/Shanghai", hour12: false,
    month: "2-digit", day: "2-digit", hour: "2-digit", minute: "2-digit", second: "2-digit"});
  let ids = [], timer, request, shown = "", pinned = "";
  try { pinned = sessionStorage.getItem("scan-run") || ""; } catch {}
  const cache = new Map();

  function title(id) {
    const [date, time] = id.split("_");
    const name = id.slice(16).split("-")[0].replace(/^test_indoor/, "test").replace(/^mission_/, "");
    return date.slice(0, 4) + "-" + date.slice(4, 6) + "-" + date.slice(6) + " " +
      time.slice(0, 2) + ":" + time.slice(2, 4) + ":" + time.slice(4) + " · " + name;
  }

  function cell(text, className) {
    const el = document.createElement("span");
    el.className = className;
    el.textContent = text;
    return el;
  }

  function render(id, result, error = "") {
    const data = result?.data;
    const info = byId("scan-info");
    const last = result?.updatedAt ? " · 最近更新 " + clock.format(result.updatedAt * 1000) : "";
    const message = [data?.reason, error && error + last].filter(Boolean).join(" · ");
    info.textContent = message;
    info.title = message;
    info.dataset.level = ["ABORTED", "INTERRUPTED"].includes(data?.state) ? "bad" : error ? "warn" : "";
    const key = JSON.stringify([id, data]);
    if (shown === key) return;
    shown = key;
    byId("scan-identity").textContent = id ? title(id) + "（北京时间）" : "";
    const state = byId("scan-state");
    state.textContent = data ? statusNames[data.state] || data.state : "";
    state.dataset.state = data?.state || "";
    for (const metric of ["discovered", "success", "failed"]) {
      byId("scan-" + metric).textContent = data?.[metric] ?? "—";
    }
    byId("scan-failed").dataset.nonzero = String(data?.failed > 0);
    const rows = (data?.rows || []).map(row => {
      const el = document.createElement("li");
      const label = row.status === "success" ? (row.frame === 1 ? "首帧读出" :
        row.frame ? `第 ${row.frame} 帧读出` : "成功 · 帧号 —") : rowNames[row.status] || row.status;
      const state = cell(label, "scan-result");
      state.dataset.status = row.status;
      el.append(cell(`${row.observation} · Tag ${row.tag}`, "scan-tag"), cell(row.qr || "—", "scan-code"), state);
      if (row.reason) el.append(cell(row.reason, "scan-reason"));
      return el;
    });
    list.replaceChildren(...rows.reverse());
  }

  async function read(path, signal) {
    const response = await fetch(path, {signal});
    if (!response.ok) throw new Error("读码记录暂不可用");
    return response.json();
  }

  async function poll() {
    if (document.hidden || request) return;
    const current = new AbortController();
    request = current;
    let id = pinned || ids[0] || "";
    try {
      const catalog = await read("/scan-runs", current.signal);
      if (current.signal.aborted) return;
      if (catalog.data && (ids.join() !== catalog.data.join() || picker.value !== pinned)) {
        ids = catalog.data;
        if (pinned && !ids.includes(pinned) && catalog.online) pinned = "";
        picker.replaceChildren(new Option("跟随最新", ""), ...ids.map(key => new Option(title(key), key)));
        picker.value = pinned;
      }
      id = pinned || ids[0] || "";
      if (!id) {
        render("", null, catalog.error || "暂无室内扫码实验");
      } else {
        if (!cache.has(id)) render(id, null, "正在读取…");
        const detail = await read("/scan-runs/" + encodeURIComponent(id), current.signal);
        if (current.signal.aborted) return;
        if (detail.data) cache.set(id, detail);
        render(id, cache.get(id), detail.error || catalog.error);
      }
    } catch (error) {
      if (!current.signal.aborted) render(id, cache.get(id), "读取中断 · 正在重连");
    } finally {
      if (request === current) {
        request = null;
        if (!document.hidden) timer = setTimeout(poll, 2000);
      }
    }
  }

  function pause() {
    clearTimeout(timer);
    request?.abort();
    request = null;
  }
  function resume() {
    pause();
    if (!document.hidden) void poll();
  }
  picker.addEventListener("change", () => {
    pinned = picker.value;
    try { sessionStorage.setItem("scan-run", pinned); } catch {}
    const id = pinned || ids[0] || "";
    render(id, cache.get(id), cache.has(id) ? "" : "正在读取…");
    resume();
  });
  document.addEventListener("visibilitychange", resume);
  window.addEventListener("pageshow", resume);
  window.addEventListener("pagehide", pause);
  void poll();
})();
