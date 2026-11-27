(() => {
  const image = document.querySelector("#camera-image");
  const status = document.querySelector("#camera-status");
  const placeholder = document.querySelector("#camera-placeholder");
  const select = document.querySelector("#camera-mode");
  const events = document.querySelector("#camera-events");
  const client = crypto.randomUUID();
  let active = false, generation = 0, timer, receivedFrame = false;
  const names = {raw: "原始画面", task: "任务标注", local: "本地识别"};
  function request(mode, keepalive = false) {
    return fetch("/camera/watch", {method: "POST", keepalive,
      headers: {"Content-Type": "application/json"}, body: JSON.stringify({client, mode})});
  }
  function emptyFrame(text) {
    image.hidden = true;
    image.removeAttribute("src");
    placeholder.hidden = false;
    placeholder.textContent = text;
  }
  function showFrame() {
    if (active && image.hasAttribute("src") && image.naturalWidth > 0) {
      receivedFrame = true;
      image.hidden = false;
      placeholder.hidden = true;
    }
  }
  async function tick(epoch) {
    if (!active || epoch !== generation) return;
    try {
      const response = await request(select.value);
      const data = await response.json();
      if (!active || epoch !== generation) return;
      if (!response.ok) throw new Error(data.error || "连接失败");
      status.textContent = data.error || (data.live ?
        [data.source, names[data.mode], data.fps === null ? "正在测帧率" : data.fps + " fps"].join(" · ") : "");
      status.title = status.textContent;
      if (!data.live) {
        emptyFrame(receivedFrame || data.error ? "未接收到视频流" : "正在连接相机…");
      } else if (!image.hasAttribute("src")) {
        placeholder.textContent = "等待视频首帧…";
        image.src = "/camera/stream?client=" + client + "&v=" + epoch;
      } else {
        showFrame();
      }
      const items = data.events.map(({stamp, text}) => {
        const label = stamp === "0" ? "无采集时间" : new Date(Number(BigInt(stamp) / 1000000n)).toLocaleTimeString();
        return label + "　" + text;
      });
      events.textContent = items.join("\n");
    } catch (e) {
      if (active && epoch === generation) {
        status.textContent = status.title = e.message;
        emptyFrame("未接收到视频流");
      }
    } finally {
      if (active && epoch === generation) timer = setTimeout(() => tick(epoch), 1000);
    }
  }
  function stop() {
    if (!active) return;
    active = false; generation++; clearTimeout(timer);
    emptyFrame("正在连接相机…");
    receivedFrame = false;
    events.textContent = ""; status.textContent = "";
    request("close", true).catch(() => {});
  }
  function update() {
    const wanted = !document.hidden;
    if (!wanted) return stop();
    if (!active) { active = true; tick(++generation); }
  }
  image.addEventListener("load", showFrame);
  image.addEventListener("error", () => {
    if (image.hasAttribute("src")) emptyFrame("未接收到视频流");
  });
  select.addEventListener("change", () => {
    if (active) { clearTimeout(timer); void tick(++generation); }
  });
  document.addEventListener("visibilitychange", update);
  window.addEventListener("pagehide", stop);
  window.addEventListener("pageshow", update);
  update();
})();
