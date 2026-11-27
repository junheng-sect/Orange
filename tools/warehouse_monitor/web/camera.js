(() => {
  const image = document.querySelector("#camera-image");
  const status = document.querySelector("#camera-status");
  const select = document.querySelector("#camera-mode");
  const events = document.querySelector("#camera-events");
  const client = crypto.randomUUID();
  let active = false, generation = 0, timer, lastSrc = "";
  const names = {raw: "原始画面", task: "任务标注", local: "本地识别"};
  function request(mode, keepalive = false) {
    return fetch("/camera/watch", {method: "POST", keepalive,
      headers: {"Content-Type": "application/json"}, body: JSON.stringify({client, mode})});
  }
  async function tick(epoch) {
    if (!active || epoch !== generation) return;
    try {
      const response = await request(select.value);
      const data = await response.json();
      if (!active || epoch !== generation) return;
      if (!response.ok) throw new Error(data.error || "连接失败");
      status.textContent = data.error || (data.live ?
        [data.source, names[data.mode], data.fps === null ? "正在测帧率" : data.fps + " fps"].join(" · ") : "正在连接相机…");
      if (data.live && !lastSrc) {
        lastSrc = "/camera/stream?client=" + client + "&v=" + epoch;
        image.src = lastSrc;
      }
      image.hidden = !data.live;
      const items = data.events.map(({stamp, text}) => {
        const label = stamp === "0" ? "无采集时间" : new Date(Number(BigInt(stamp) / 1000000n)).toLocaleTimeString();
        return label + "　" + text;
      });
      events.textContent = items.join("\n");
    } catch (e) {
      if (active && epoch === generation) { status.textContent = e.message; image.hidden = true; }
    } finally {
      if (active && epoch === generation) timer = setTimeout(() => tick(epoch), 1000);
    }
  }
  function stop() {
    if (!active) return;
    active = false; generation++; clearTimeout(timer);
    image.removeAttribute("src"); lastSrc = ""; image.hidden = true;
    events.textContent = ""; status.textContent = "";
    request("close", true).catch(() => {});
  }
  function update() {
    const wanted = !document.hidden;
    if (!wanted) return stop();
    if (!active) { active = true; tick(++generation); }
  }
  image.addEventListener("error", () => { lastSrc = ""; });
  select.addEventListener("change", () => {
    if (active) { clearTimeout(timer); tick(++generation); }
  });
  document.addEventListener("visibilitychange", update);
  window.addEventListener("pagehide", stop);
  window.addEventListener("pageshow", update);
  update();
})();
