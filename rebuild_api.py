import base64, json, os, sys, urllib.request, urllib.error

TOKEN = os.environ["GH_TOKEN"]
REPO = "Enemy52/ros2"
BRANCH = "task3-slam-nav"
BASE = "21a3224"  # upstream master, will resolve full sha via API
LOCAL = "/home/user/Doubao/chats/38445783240432130/.tmp_squash/src/air/uav_slam_nav"

FILES = [
    "CMakeLists.txt", "README.md", "package.xml",
    "config/costmap.yaml", "launch/slam_nav.launch",
    "scripts/demo.py", "scripts/nav_relay.py",
    "scripts/points_to_scan.py", "scripts/simple_nav.py",
    "docs/slam_nav_demo.gif",
]

def api(method, path, payload=None):
    url = f"https://api.github.com/repos/{REPO}{path}"
    data = json.dumps(payload).encode() if payload is not None else None
    req = urllib.request.Request(url, data=data, method=method)
    req.add_header("Authorization", f"token {TOKEN}")
    req.add_header("Accept", "application/vnd.github+json")
    req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=120) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        body = e.read().decode()
        print(f"HTTP {e.code} on {method} {path}\n{body[:800]}", file=sys.stderr)
        raise

# 1. master commit -> tree
m = api("GET", f"/git/commits/{BASE}")
base_tree = m["tree"]["sha"]
base_sha = m["sha"]
print("base commit:", base_sha)
print("base tree:", base_tree)

# 2. blobs
entries = []
for rel in FILES:
    p = os.path.join(LOCAL, rel)
    raw = open(p, "rb").read()
    is_text = not rel.endswith(".gif")
    if is_text:
        payload = {"content": raw.decode("utf-8"), "encoding": "utf-8"}
    else:
        payload = {"content": base64.b64encode(raw).decode(), "encoding": "base64"}
    b = api("POST", "/git/blobs", payload)
    entries.append({"path": f"src/air/uav_slam_nav/{rel}", "mode": "100644", "type": "blob", "sha": b["sha"]})
    print("blob", rel, len(raw), "->", b["sha"][:8])

# 3. tree
tree = api("POST", "/git/trees", {"base_tree": base_tree, "tree": entries})
print("new tree:", tree["sha"])

# 4. commit
commit = api("POST", "/git/commits", {
    "message": "Task3: 无人机SLAM建图与避障导航 (uav_slam_nav)\n\n"
               "- slam_toolbox 异步建图，points_to_scan 把世界系点云切片成 base_link 下的 /scan\n"
               "- move_base 全局/局部 costmap，nav_relay 限幅转发 /uav/cmd_vel\n"
               "- simple_nav 在规划失败时从 /scan 直接避障兜底",
    "tree": tree["sha"],
    "parents": [base_sha],
})
print("new commit:", commit["sha"])

# 5. update ref
ref = api("PATCH", f"/git/refs/heads/{BRANCH}", {"sha": commit["sha"], "force": True})
print("ref updated ->", ref["object"]["sha"])
