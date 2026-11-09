#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""文档自检工具（tools/check_docs.py）的测试。

两种跑法结果一致：
    python3 -m pytest tests/test_check_docs_local.py -v     # 断言失败即用例失败
    python3 tests/test_check_docs_local.py                  # 自带 runner, 打印逐项结果

不需要 ROS、模拟器、网络，也不构建文档站：每个用例在临时目录里造一个最小的
mkdocs 工程，再检查工具报告的问题。覆盖：

    A. 正常工程：不报告问题，退出码 0
    B. 导航项指向不存在的文件（行号精确到该导航项）
    C. 导航项指向 docs 之外：报错误而不是崩溃
    D. 同一栏目重复列出同一页面（行号指向重复的那一行）
    E. 同一栏目内两个不同页面重名
    F. 同一页面出现在两个栏目：只给警告
    G. 存在但没有导航入口的页面
    H. mkdocs.yml 无效或缺失：返回 2 且只打印一行错误
    I. 围栏代码块、四空格缩进代码块、行内代码、HTML 注释里的链接不算
    J. 同一行 HTML 注释之后的真实链接要被查出来
    K. 列表续行（缩进段落）里的链接不能被漏掉
    L. 带括号和 %20 的文件名
    M. 远程链接、页内锚点、docs 之外的路径
    N. baseline：已知问题被接受、过期条目要提示
    O. --json 输出、--strict 退出码
    P. 自定义 docs_dir
"""
from __future__ import annotations

import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

PASS, FAIL = [], []


def check(cond, label):
    """断言并记录；失败时抛 AssertionError，pytest 与自带 runner 都会判失败."""
    if cond:
        PASS.append(label)
        print("  [PASS] %s" % label)
        return True
    FAIL.append(label)
    print("  [FAIL] %s" % label)
    raise AssertionError(label)


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import check_docs  # noqa: E402


@contextlib.contextmanager
def project(files):
    """把 {相对路径: 内容} 写到临时目录，作为一次检查的工程根目录."""
    root = tempfile.mkdtemp(prefix="check_docs_")
    try:
        for rel, text in files.items():
            path = os.path.join(root, rel.replace("/", os.sep))
            parent = os.path.dirname(path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            with open(path, "w", encoding="utf-8") as handle:
                handle.write(text)
        yield root
    finally:
        shutil.rmtree(root, ignore_errors=True)


def codes(issues, level=None):
    return sorted({i.code for i in issues if level is None or i.level == level})


def run_main(argv):
    """调用命令行入口，返回 (退出码, stdout, stderr)."""
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        code = check_docs.main(argv)
    return code, out.getvalue(), err.getvalue()


BASE_CONFIG = """site_name: 测试站
nav:
  - 入门:
      - 首页: 'index.md'
      - 指南: 'guide/page.md'
"""

BASE_FILES = {
    "mkdocs.yml": BASE_CONFIG,
    "docs/index.md": "# 首页\n\n[去指南](guide/page.md)\n",
    "docs/guide/page.md": "# 指南\n\n[回首页](../index.md)\n",
}


def test_clean_project():
    with project(BASE_FILES) as root:
        issues, stats = check_docs.collect(root)
        code, out, _err = run_main(["--root", root, "--no-baseline"])
    check(issues == [], "正常工程不报告任何问题")
    check(stats["nav"] == 2, "统计到 2 个导航项（实际 %d）" % stats["nav"])
    check(stats["pages"] == 2, "统计到 2 个文档页（实际 %d）" % stats["pages"])
    check(code == 0, "正常工程退出码为 0（实际 %d）" % code)
    check("未发现问题" in out, "报告里说明没有问题")


def test_nav_missing_line():
    config = BASE_CONFIG + "      - 不存在的页: 'guide/missing.md'\n"
    files = dict(BASE_FILES)
    files["mkdocs.yml"] = config
    with project(files) as root:
        issues, _ = check_docs.collect(root, skip_links=True, skip_orphans=True)
    missing = [i for i in issues if i.code == "nav-missing"]
    check(len(missing) == 1, "报告 1 个 nav-missing（实际 %d）" % len(missing))
    check(missing[0].level == "error", "导航指向缺失文件算错误")
    check("guide/missing.md" in missing[0].message, "错误信息带上出错的路径")
    check(missing[0].line == 6, "行号指向该导航项所在行（实际 %r）" % missing[0].line)


def test_nav_outside_docs():
    """导航项指向 docs 之外且文件存在：必须报错，不能抛 traceback."""
    files = {
        "mkdocs.yml": "site_name: 测试站\nnav:\n  - 首页: 'index.md'\n  - 外面: '../outside.md'\n",
        "docs/index.md": "# 首页\n",
        "outside.md": "# docs 之外的页面\n",
    }
    with project(files) as root:
        issues, _ = check_docs.collect(root, skip_links=True, skip_orphans=True)
        code, out, err = run_main(["--root", root, "--no-baseline"])
    outside = [i for i in issues if i.code == "nav-outside"]
    check(len(outside) == 1, "报告 1 个 nav-outside（实际 %d）" % len(outside))
    check(outside[0].level == "error", "导航指向 docs 之外算错误")
    check(outside[0].line == 4, "nav-outside 也带行号（实际 %r）" % outside[0].line)
    check(code == 1, "有错误时退出码为 1（实际 %d）" % code)
    check("Traceback" not in err and "Traceback" not in out, "不打印 Python traceback")


def test_nav_duplicate_line_points_to_duplicate():
    """同一页面先在别的栏目出现一次, 重复项要报在重复的那一行, 不是首次出现处."""
    config = """site_name: 测试站
nav:
  - 栏目一:
      - 页面: 'guide/page.md'
  - 栏目二:
      - 页面（跨栏目）: 'guide/page.md'
      - 页面: 'guide/page.md'
"""
    files = {"mkdocs.yml": config, "docs/guide/page.md": "# 页面\n",
             "docs/index.md": "# 首页\n"}
    with project(files) as root:
        issues, _ = check_docs.collect(root, skip_links=True, skip_orphans=True)
    duplicate = [i for i in issues if i.code == "nav-duplicate"]
    check(len(duplicate) == 1, "同一栏目重复列出同一页面报 1 次（实际 %d）" % len(duplicate))
    check(duplicate[0].line == 7, "行号指向重复的那一行（实际 %r）" % duplicate[0].line)
    check("栏目二" in duplicate[0].message, "错误信息带上出问题的栏目")


def test_nav_duplicate_label():
    config = """site_name: 测试站
nav:
  - 地面载具:
      - 键盘控制: 'ground/car_keyboard.md'
      - 键盘控制: 'ground/drone_keyboard.md'
"""
    files = {"mkdocs.yml": config, "docs/ground/car_keyboard.md": "# 小车\n",
             "docs/ground/drone_keyboard.md": "# 无人机\n"}
    with project(files) as root:
        issues, _ = check_docs.collect(root, skip_links=True)
    check(codes(issues, "error") == [], "不同页面重名不算错误")
    labels = [i for i in issues if i.code == "nav-label"]
    check(len(labels) == 1, "同名导航项报 1 次（实际 %d）" % len(labels))
    check(labels[0].level == "warning", "重名导航项算警告")
    check("键盘控制" in labels[0].message, "警告里带重复的导航名")


def test_nav_shared_across_sections():
    config = """site_name: 测试站
nav:
  - 地面载具:
      - 雨雾对比: 'ground/weather.md'
  - 空域载具:
      - 雨雾对比: 'ground/weather.md'
"""
    files = {"mkdocs.yml": config, "docs/ground/weather.md": "# 雨雾\n"}
    with project(files) as root:
        issues, _ = check_docs.collect(root, skip_links=True, skip_orphans=True)
    check(codes(issues, "error") == [], "跨栏目引用不产生错误（实际 %s）" % codes(issues, "error"))
    check("nav-shared" in codes(issues, "warning"), "跨栏目引用给出确认用的警告")


def test_orphan_page():
    files = dict(BASE_FILES)
    files["docs/guide/orphan.md"] = "# 没入口的页面\n"
    with project(files) as root:
        issues, stats = check_docs.collect(root, skip_links=True)
        without, _ = check_docs.collect(root, skip_links=True, skip_orphans=True)
    orphan = [i for i in issues if i.code == "docs-orphan"]
    check(len(orphan) == 1, "报告 1 个孤立页面（实际 %d）" % len(orphan))
    check(orphan[0].level == "warning", "孤立页面算警告")
    check("orphan.md" in orphan[0].message, "警告里带页面路径")
    check(stats["pages"] == 3, "孤立页面同样计入文档页数")
    check(codes(without) == [], "--skip-orphans 后不再报告孤立页面")


def test_invalid_yaml_exits_2():
    files = dict(BASE_FILES)
    files["mkdocs.yml"] = "site_name: [未闭合\nnav:\n  - 首页: 'index.md'\n"
    with project(files) as root:
        code, out, err = run_main(["--root", root, "--no-baseline"])
    check(code == 2, "无效 mkdocs.yml 返回 2（实际 %d）" % code)
    check("Traceback" not in err, "不打印 traceback")
    check(err.count("\n") == 1, "错误只有一行（实际 %d 行）" % err.count("\n"))
    check("解析失败" in err, "错误信息说明是解析失败：%r" % err)


def test_missing_config_exits_2():
    with project({"docs/index.md": "# 首页\n"}) as root:
        code, _out, err = run_main(["--root", root, "--no-baseline"])
    check(code == 2, "缺少 mkdocs.yml 返回 2（实际 %d）" % code)
    check("找不到 mkdocs.yml" in err, "错误信息说明缺哪个文件：%r" % err)


def test_markdown_code_blocks_are_skipped():
    page = """# 指南

围栏代码块：

```bash
curl -O https://example.com/[坏链](missing.md)
```

四空格缩进代码块（上一行是空行）：

    [坏链](missing.md)

行内代码：`[坏链](missing.md)`

<!-- 独占一行的注释：[坏链](missing.md) -->

正文结束了。
"""
    files = dict(BASE_FILES)
    files["docs/guide/page.md"] = page
    with project(files) as root:
        issues, _ = check_docs.collect(root, skip_orphans=True)
    broken = [i for i in issues if i.code == "link-missing"]
    check(broken == [], "代码块、行内代码、注释里的链接都不算（实际 %d 项）" % len(broken))


def test_markdown_inline_comment_keeps_following_link():
    page = "# 指南\n\n<!-- note --> [坏链](missing.md)\n"
    files = dict(BASE_FILES)
    files["docs/guide/page.md"] = page
    with project(files) as root:
        issues, _ = check_docs.collect(root, skip_orphans=True)
    broken = [i for i in issues if i.code == "link-missing"]
    check(len(broken) == 1, "同一行注释之后的真实断链要被查出来（实际 %d）" % len(broken))
    check("missing.md" in broken[0].message, "报出的就是那个断链")


def test_markdown_list_continuation_link_kept():
    """列表续行虽然缩进 4 空格, 但它是正文, 里面的链接不能被当成代码块跳过."""
    page = """# 指南

* 第一步：看这里

    补充说明见 [缺页](missing.md)。

* 第二步：结束
"""
    files = dict(BASE_FILES)
    files["docs/guide/page.md"] = page
    with project(files) as root:
        issues, _ = check_docs.collect(root, skip_orphans=True)
    broken = [i for i in issues if i.code == "link-missing"]
    check(len(broken) == 1, "列表续行里的断链要被查出来（实际 %d）" % len(broken))


def test_link_with_parentheses():
    page = (
        "# 指南\n\n"
        "![编码过的空格](../img/code/Topic%20(PublishSubscribe).png)\n\n"
        "![尖括号形式](<../img/code/Topic (PublishSubscribe).png>)\n"
    )
    files = dict(BASE_FILES)
    files["docs/guide/page.md"] = page
    files["docs/img/code/Topic (PublishSubscribe).png"] = ""
    with project(files) as root:
        issues, _ = check_docs.collect(root, skip_orphans=True)
    check(codes(issues, "error") == [], "带括号的文件名不算断链（实际 %s）"
          % codes(issues, "error"))

    files["docs/guide/page.md"] = "# 指南\n\n![裸空格](../img/code/Topic (PublishSubscribe).png)\n"
    with project(files) as root:
        issues, _ = check_docs.collect(root, skip_orphans=True)
    broken = [i for i in issues if i.code == "link-missing"]
    check(len(broken) == 1, "目标里裸露空格按构建行为报断链（实际 %d）" % len(broken))


def test_links_outside_and_remote():
    page = ("# 指南\n\n[源码](../../src/demo/main.py)\n"
            "[主页](https://github.com/OpenHUTB/ros2)\n[小节](#section)\n")
    files = dict(BASE_FILES)
    files["docs/guide/page.md"] = page
    with project(files) as root:
        issues, _ = check_docs.collect(root, skip_orphans=True)
    check(codes(issues, "error") == [], "docs 之外的链接与远程链接不算错误")
    check("link-outside" in codes(issues, "warning"), "指向 docs 之外给出警告")


def test_baseline_accepts_known_issue():
    config = BASE_CONFIG + "      - 不存在的页: 'guide/missing.md'\n"
    files = dict(BASE_FILES)
    files["mkdocs.yml"] = config
    files["tools/check_docs_baseline.json"] = json.dumps({
        "entries": [{"code": "nav-missing", "path": "mkdocs.yml",
                     "message_contains": "guide/missing.md"}]
    }, ensure_ascii=False)
    with project(files) as root:
        baseline = check_docs.load_baseline(os.path.join(root, "tools", "check_docs_baseline.json"))
        issues, _ = check_docs.collect(root, skip_links=True, skip_orphans=True,
                                       baseline=baseline)
        code, out, _err = run_main(["--root", root, "--skip-links", "--skip-orphans"])
    known = [i for i in issues if i.known]
    check(len(known) == 1, "baseline 接受了 1 条已知问题（实际 %d）" % len(known))
    check(code == 0, "已知问题不影响退出码（实际 %d）" % code)
    check("已应用 baseline" in out and "已知（baseline）" in out, "报告里标出已知问题")


def test_baseline_stale_entry_is_reported():
    files = dict(BASE_FILES)
    files["tools/check_docs_baseline.json"] = json.dumps({
        "entries": [{"code": "nav-missing", "path": "mkdocs.yml",
                     "message_contains": "已经修好的问题"}]
    }, ensure_ascii=False)
    with project(files) as root:
        code, out, _err = run_main(["--root", root, "--skip-links", "--skip-orphans"])
    check("baseline-stale" in out, "过期 baseline 条目会被提示")
    check(code == 0, "过期条目只是警告，不影响退出码（实际 %d）" % code)


def test_json_and_strict():
    files = dict(BASE_FILES)
    files["docs/guide/orphan.md"] = "# 只有警告\n"
    with project(files) as root:
        code, out, _err = run_main(["--root", root, "--json", "--skip-links", "--no-baseline"])
        strict, strict_out, _err2 = run_main(
            ["--root", root, "--strict", "--skip-links", "--no-baseline"])
    payload = json.loads(out)
    check(payload.get("pages") == 3, "--json 输出包含页数")
    check(len(payload.get("warnings") or []) == 1, "--json 输出包含 1 条警告")
    check(payload.get("errors") == [], "--json 输出里没有错误")
    check("known" in payload, "--json 输出包含 known 字段")
    check(code == 0, "只有警告时默认退出码为 0（实际 %d）" % code)
    check(strict == 1, "--strict 时警告返回 1（实际 %d）" % strict)
    check("--strict" in strict_out, "--strict 会提示警告也视为失败")


def test_custom_docs_dir():
    config = """site_name: 测试站
docs_dir: content
nav:
  - 首页: 'index.md'
"""
    files = {"mkdocs.yml": config, "content/index.md": "# 首页\n"}
    with project(files) as root:
        issues, stats = check_docs.collect(root)
    check(codes(issues) == [], "自定义 docs_dir 时正常通过（实际 %s）" % codes(issues))
    check(stats["pages"] == 1, "按 docs_dir 统计文档页")


TESTS = [
    ("A. 正常工程", test_clean_project),
    ("B. 导航文件不存在（行号）", test_nav_missing_line),
    ("C. 导航指向 docs 之外", test_nav_outside_docs),
    ("D. 同栏目重复（行号）", test_nav_duplicate_line_points_to_duplicate),
    ("E. 同栏目重名导航项", test_nav_duplicate_label),
    ("F. 跨栏目引用", test_nav_shared_across_sections),
    ("G. 没有导航入口的页面", test_orphan_page),
    ("H. mkdocs.yml 无效", test_invalid_yaml_exits_2),
    ("I. mkdocs.yml 缺失", test_missing_config_exits_2),
    ("J. 代码块里的链接跳过", test_markdown_code_blocks_are_skipped),
    ("K. 行内注释后的链接", test_markdown_inline_comment_keeps_following_link),
    ("L. 列表续行里的链接", test_markdown_list_continuation_link_kept),
    ("M. 带括号的文件名", test_link_with_parentheses),
    ("N. docs 外与远程链接", test_links_outside_and_remote),
    ("O. baseline 接受已知问题", test_baseline_accepts_known_issue),
    ("P. baseline 过期条目", test_baseline_stale_entry_is_reported),
    ("Q. JSON 与 --strict", test_json_and_strict),
    ("R. 自定义 docs_dir", test_custom_docs_dir),
]


def main():
    failures = 0
    for title, function in TESTS:
        print(title)
        try:
            function()
        except AssertionError:
            failures += 1
    print("\n==================== 结果 ====================")
    print("通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    if FAIL:
        for item in FAIL:
            print("  FAIL: %s" % item)
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
