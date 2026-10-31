'use strict';
// 无人机状态面板（只读）：每秒从本机状态桥拉一份 JSON。状态桥在 mid360s_ws 的 docker/ros-noetic-rviz 里
// （local_view/drone_status.py，起法 ./scripts/run_drone_status.sh），在本机 docker 里订阅板端常驻 MAVROS 的低频话题，
// 板端不加进程。和小车的 rosbridge 连接互不相干：桥没开只影响这一块。
// 默认 http://<打开本页的主机>:8091/status；?drone=http://<地址>:8091 手动指定。
// 页面只显示、不发任何指令；“可不可以起飞”由状态桥统一判断，这里只照着画。
(() => {
  const BASE = (new URLSearchParams(location.search).get('drone') || `http://${location.hostname || '127.0.0.1'}:8091`)
    .replace(/\/$/, '');
  const POLL_MS = 1000, TIMEOUT_MS = 2000, MSG_SHOW = 6;
  const SEVERITY = ['紧急', '警报', '严重', '错误', '警告', '注意', '信息', '调试'];
  const LEVEL = { ok: 'ok', bad: 'bad', unknown: 'warn' };
  const panel = $('drone');
  let open = false, snap = null, err = '';

  $('c-drone').addEventListener('click', () => {
    open = !open;
    panel.classList.toggle('open', open);
    place();
    render();
  });
  // 顶栏在手机上会折行，面板按它的实际高度往下放
  function place() {
    panel.style.top = $('bar').offsetHeight + 8 + 'px';
  }
  window.addEventListener('resize', place);

  async function poll() {
    const ctl = new AbortController();
    const timer = setTimeout(() => ctl.abort(), TIMEOUT_MS);
    try {
      const r = await fetch(BASE + '/status', { cache: 'no-store', signal: ctl.signal });
      if (!r.ok) throw new Error('HTTP ' + r.status);
      snap = await r.json();
      err = '';
    } catch (e) {
      snap = null;
      err = e.name === 'AbortError' ? '超时' : e.message;
    } finally {
      clearTimeout(timer);
      render();
      setTimeout(poll, POLL_MS);
    }
  }

  function esc(s) {
    return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
  }

  function render() {
    if (!snap) {
      setChip('c-drone', '本机状态桥连不上', 'warn');
      setChip('c-dbat', '—');
      if (open) {
        panel.innerHTML = `<div class="lamp warn-t">本机状态桥连不上（${esc(err)}）</div>
          <small>${esc(BASE)}/status<br>本机用户服务 drone-status：systemctl --user status drone-status<br>
          没装就在本机 mid360s_ws/docker/ros-noetic-rviz 下运行 ./scripts/run_drone_status.sh --install</small>`;
      }
      return;
    }
    const lamp = snap.lamp;
    setChip('c-drone', lamp.text, LEVEL[lamp.level]);
    const b = snap.battery;
    if (b.connected === null) setChip('c-dbat', '—');
    else if (!b.connected) setChip('c-dbat', '未接电池', 'warn');
    else {
      const pct = b.percent;
      const level = pct === null ? '' : pct >= 40 ? 'ok' : pct >= 20 ? 'warn' : 'bad';
      setChip('c-dbat', `${pct === null ? '?' : pct + '%'} ${b.voltage.toFixed(1)}V`, level);
    }
    if (open) panel.innerHTML = detail(snap);
  }

  function detail(d) {
    const out = [`<div class="lamp ${LEVEL[d.lamp.level]}-t">${esc(d.lamp.text)}</div>`];
    for (const c of d.checks) {
      // required === false：当前模式用不上这一项（如 ALTCTL 下的定位），只作参考、不算进总灯
      const [mark, cls] = c.ok === true ? ['✓', 'ok-t'] : c.required === false ? ['–', 'mute-t']
        : c.ok === false ? ['✗', 'bad-t'] : ['?', 'warn-t'];
      out.push(`<div class="row"><span class="${cls}">${mark} ${esc(c.name)}</span> ${esc(c.text)}</div>`);
    }
    const f = d.flight;
    if (f.connected) {
      out.push('<h4>飞行</h4>');
      out.push(`<div class="row">${f.armed ? '<span class="warn-t">已解锁</span>' : '未解锁'} · ${esc(f.mode || '?')}`
        + ` · ${esc(f.landed || '?')}${f.failsafe ? ' · <span class="bad-t">失效保护</span>' : ''}</div>`);
    }
    const b = d.battery;
    out.push('<h4>电池</h4>');
    if (b.connected === null) out.push('<div class="row">没有数据</div>');
    else if (!b.connected) out.push('<div class="row">未接电池（飞控读到电压 0）</div>');
    else {
      out.push(`<div class="row">${b.percent === null ? '?' : b.percent + '%'} · ${b.voltage.toFixed(2)} V`
        + `${b.current === null ? '' : ' · ' + b.current.toFixed(1) + ' A'}</div>`);
    }
    out.push('<h4>最近飞控消息</h4>');
    const msgs = d.messages.slice(-MSG_SHOW).reverse();
    if (!msgs.length) out.push('<div class="row"><small>本机状态桥起来后还没有（飞控只在检查结果变化时发）</small></div>');
    for (const m of msgs) {
      const cls = m.severity <= 3 ? 'bad-t' : m.severity === 4 ? 'warn-t' : '';
      const ago = Math.max(0, Math.round(d.t - m.t));
      out.push(`<div class="row"><small>${ago} s 前</small> <span class="${cls}">${SEVERITY[m.severity] || m.severity}</span>`
        + ` ${esc(m.text)}</div>`);
    }
    out.push(`<small>数据：${esc(BASE)}/status · 板端 ${d.master.ok ? '已连上' : '连不上'}</small>`);
    return out.join('');
  }

  poll();
})();
