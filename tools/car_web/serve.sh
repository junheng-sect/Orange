#!/usr/bin/env bash
# 小车网页控制台的静态服务。日常在台式机上运行（板端是无人机机载电脑，经常关机），
# 手机打开 http://<台式机 IP>:8080；台式机自己也可以直接用浏览器打开 index.html。
# 页面在浏览器里直接连车端 rosbridge（默认 ws://192.168.31.231:9090），这里只提供静态文件，
# 放在哪台机器上都不影响数据路径。
# 用法：tools/car_web/serve.sh [端口]
set -e
cd "$(dirname "$0")"
PORT=${1:-8080}
# 关掉浏览器缓存，改完页面刷新就生效。
exec python3 - "$PORT" <<'EOF'
import http.server
import sys


class Handler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Cache-Control', 'no-cache')
        super().end_headers()


http.server.ThreadingHTTPServer(('0.0.0.0', int(sys.argv[1])), Handler).serve_forever()
EOF
