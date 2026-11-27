# 小车网页控制台（car_web）

浏览器直连车端 rosbridge 的纯前端控制台，无后端依赖。

## 启动方式

```bash
tools/car_web/serve.sh [端口]   # 默认 8080
```

也可直接用浏览器打开 `index.html`。

## rosbridge 端点配置

连接地址按以下优先级决定（支持直接打开 HTML）：

1. URL 查询参数 `?ws=ws://<host>:9090`（单次覆盖）。
2. 同目录私有文件 `config.local.js`（已由 `.gitignore` 忽略），可配置多个候选端点同时探测：
   ```js
   window.CAR_ROSBRIDGE_URLS = ["ws://car-a.local:9090", "ws://car-b.local:9090"];
   ```
   注意：其中的地址对访问页面的浏览器可见。
3. 未配置时默认使用 `ws://spirit.local:9090`。
