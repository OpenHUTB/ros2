#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""web_remote_car 前端静态页面服务。

由 launch/web_bridge.launch 自动拉起，因此只要执行
`roslaunch web_remote_car start_all.launch`，浏览器就能直接打开遥控面板，
不再需要单独再开一个终端跑 `python3 -m http.server 8000`。

参数（可在 launch 里覆盖或者命令行传参）：
  ~port     监听端口，默认 8000
  ~address  监听地址，默认 0.0.0.0（局域网其他电脑可以直接访问）
"""

import functools
import os
import socket
import threading

import rospy
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer


class _NoCacheHandler(SimpleHTTPRequestHandler):
    """禁止浏览器缓存页面。

    SimpleHTTPRequestHandler 默认允许浏览器缓存 index.html，
    改了页面后用户刷新还是旧版本（表现为“明明改了还是连不上”）。
    """

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        SimpleHTTPRequestHandler.end_headers(self)


def _local_ips():
    """尽力猜出本机对外的 IP，用来在日志里直接打印可访问地址。"""
    ips = []
    try:
        for ip in socket.gethostbyname_ex(socket.gethostname())[2]:
            if not ip.startswith("127.") and ip not in ips:
                ips.append(ip)
    except Exception:
        pass
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("8.8.8.8", 80))
        ip = probe.getsockname()[0]
        probe.close()
        if not ip.startswith("127.") and ip not in ips:
            ips.append(ip)
    except Exception:
        pass
    return ips


def _resolve_web_dir():
    """返回前端页面目录。

    launch 里用 $(find web_remote_car)/www 直接把路径传进来，节点启动后立刻绑定端口；
    只有 rosrun 单独跑脚本、没传参数时，才退回去用 rospack 查包路径
    （机器上包特别多时 rospack 首次遍历很慢，会让端口迟迟起不来）。
    """
    web_dir = rospy.get_param("~web_root", "")
    if web_dir:
        return web_dir
    import rospkg
    return os.path.join(rospkg.RosPack().get_path("web_remote_car"), "www")


def main():
    rospy.init_node("frontend_http_server", anonymous=False, disable_signals=True)

    port = int(rospy.get_param("~port", 8000))
    address = rospy.get_param("~address", "0.0.0.0")

    web_dir = _resolve_web_dir()
    index_html = os.path.join(web_dir, "index.html")
    if not os.path.isfile(index_html):
        rospy.logfatal("找不到前端页面：%s", index_html)
        raise SystemExit(1)

    handler = functools.partial(_NoCacheHandler, directory=web_dir)
    try:
        httpd = ThreadingHTTPServer((address, port), handler)
    except OSError as exc:
        rospy.logfatal("前端页面端口 %d 绑定失败：%s", port, exc)
        rospy.logfatal("通常是端口被别的程序占用，可换端口启动："
                       "roslaunch web_remote_car start_all.launch frontend_port:=8001")
        raise SystemExit(1)

    rospy.loginfo("前端页面目录：%s", web_dir)
    rospy.loginfo("本机访问：http://127.0.0.1:%d/", port)
    for ip in _local_ips():
        rospy.loginfo("局域网访问：http://%s:%d/", ip, port)

    serve_thread = threading.Thread(target=httpd.serve_forever, name="frontend_http")
    serve_thread.daemon = True
    serve_thread.start()

    rospy.spin()
    httpd.shutdown()


if __name__ == "__main__":
    main()
