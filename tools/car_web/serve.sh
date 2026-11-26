#!/usr/bin/env bash
# 小车网页控制台的静态服务。日常在台式机上运行（板端是无人机机载电脑，经常关机），
# 手机打开 http://<台式机 IP>:8080；台式机自己也可以直接用浏览器打开 index.html。
# 页面在浏览器里直接连车端 rosbridge（车的几个地址同时试，见 app.js 开头），这里只提供静态文件，
# 放在哪台机器上都不影响数据路径。
# 用法：tools/car_web/serve.sh [端口]
set -e
PORT=${1:-8080}
# 关掉浏览器缓存，改完页面刷新就生效。用 Python 解析脚本真实目录，兼容 macOS/Linux 及符号链接调用。
exec python3 - "$0" "$PORT" <<'EOF'
import http.server
import os
from pathlib import Path
import sys

os.chdir(Path(sys.argv[1]).resolve(strict=True).parent)


class Handler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Cache-Control', 'no-cache')
        super().end_headers()


http.server.ThreadingHTTPServer(('0.0.0.0', int(sys.argv[2])), Handler).serve_forever()
EOF
