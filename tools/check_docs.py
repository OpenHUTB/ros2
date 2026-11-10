#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""文档站自检：检查 mkdocs 导航与文档内链接是否一致。

仓库约定「新模块在本首页和 mkdocs.yml 增加入口」，但导航项写错路径、页面重名、
页面写了却没有入口这几类问题只能靠人工翻页面发现，本脚本把它们变成可重复执行的检查。

用法（仓库根目录）：
    python tools/check_docs.py                      # 报告全部问题，有错误时退出码为 1
    python tools/check_docs.py --strict             # 警告也当作失败（适合放进 CI）
    python tools/check_docs.py --json               # 输出 JSON，便于别的脚本消费
    python tools/check_docs.py --skip-links         # 只查导航，跳过文档内链接
    python tools/check_docs.py --skip-orphans
    python tools/check_docs.py --baseline FILE      # 接受已知问题（见下）
    python tools/check_docs.py --no-baseline        # 忽略默认的 tools/check_docs_baseline.json

默认会读取仓库根的 `tools/check_docs_baseline.json`（若存在），把里面登记的已知问题
标成「已知」，不计入退出码；每条 baseline 只能消化一条问题，消化不到会提示该条目已过期。

检查项：
    1. 导航目标文件是否存在（docs 目录相对路径）；是否指向 docs 之外。
    2. 同一栏目内是否重复列出同一个页面。
    3. 同一栏目内是否有两个不同页面取了同一个导航名。
    4. 是否存在没有任何导航入口的页面（警告）。
    5. 文档内的相对链接、图片、附件是否指向已存在的文件。
    6. 链接是否指向 docs 之外（警告：网页里没有这些文件）。

退出码：0 无错误；1 有错误（--strict 时含警告）；2 工具无法运行（配置读不到或解析失败）。
"""
import argparse
import json
import re
import sys
from pathlib import Path
from urllib.parse import unquote

try:
    import yaml
except ImportError:  # pragma: no cover - 环境问题，不作为检查结果
    sys.stderr.write(
        "缺少 pyyaml，无法解析 mkdocs.yml。\n"
        "安装：python -m pip install pyyaml（mkdocs 本身也依赖它）\n"
    )
    raise SystemExit(2)

ROOT = Path(__file__).resolve().parents[1]
BASELINE_NAME = 'tools/check_docs_baseline.json'

# 不算本地文件的链接前缀
REMOTE_PREFIXES = ("http://", "https://", "//", "mailto:", "tel:", "data:", "ftp://")

# 引用式定义：[id]: 目标
REF_DEF = re.compile(r"^\s{0,3}\[[^\]]+\]:\s*(\S+)")
FENCE = re.compile(r"^\s{0,3}(`{3,}|~{3,})")
LIST_ITEM = re.compile(r"^\s{0,3}(?:[-*+]|\d+[.)])\s+")


class ConfigError(Exception):
    """mkdocs.yml 读不到或解析失败：工具没法运行，按约定返回 2."""


class Issue(object):
    """一条检查结果。level 为 error 或 warning；known 表示已被 baseline 接受。"""

    __slots__ = ("level", "code", "message", "path", "line", "known")

    def __init__(self, level, code, message, path=None, line=None):
        self.level = level
        self.code = code
        self.message = message
        self.path = path
        self.line = line
        self.known = False

    def as_dict(self):
        return {
            "level": self.level,
            "code": self.code,
            "message": self.message,
            "path": None if self.path is None else str(self.path),
            "line": self.line,
            "known": self.known,
        }

    def __str__(self):
        where = ""
        if self.path:
            where = str(self.path) + (":%d" % self.line if self.line else "")
            where += " "
        return "%s%s" % (where, self.message)


class NavItem(object):
    """一个导航叶子项：label 为完整栏目路径，target 为原始目标，line 为源文件行号."""

    __slots__ = ("label", "target", "section", "line")

    def __init__(self, label, target, section, line=None):
        self.label = label
        self.target = target
        self.section = section
        self.line = line

    @property
    def leaf(self):
        return self.label[-1] if self.label else ""


# ---------------------------------------------------------------- 配置解析


def read_config(config_path):
    """读取 mkdocs.yml，返回 (docs_dir, 导航项列表)。

    用 yaml.compose 只取节点树：既能拿到每个导航项的真实行号，也不会因为
    `!!python/name:` 这类 mkdocs 自有标签而报错（不构造 Python 对象）。
    """
    try:
        text = config_path.read_text(encoding="utf-8-sig")
    except OSError as exc:
        raise ConfigError("读不到 %s：%s" % (config_path.name, exc.strerror or exc))
    except UnicodeDecodeError:
        raise ConfigError("%s 不是 UTF-8 编码" % config_path.name)
    try:
        root = yaml.compose(text, Loader=yaml.SafeLoader)
    except yaml.YAMLError as exc:
        raise ConfigError("%s 解析失败：%s" % (config_path.name, _one_line(exc)))
    if root is None:
        return "docs", []
    if not isinstance(root, yaml.MappingNode):
        raise ConfigError("%s 顶层不是映射结构" % config_path.name)

    docs_dir = "docs"
    nav_node = None
    for key_node, value_node in root.value:
        if not isinstance(key_node, yaml.ScalarNode):
            continue
        if key_node.value == "docs_dir" and isinstance(value_node, yaml.ScalarNode):
            docs_dir = value_node.value
        elif key_node.value == "nav":
            nav_node = value_node
    items = []
    if nav_node is not None:
        _walk_nav(nav_node, (), items)
    return docs_dir, items


def _one_line(exc):
    """把 PyYAML 的多行错误压成一行，便于在终端和日志里阅读。"""
    return " ".join(str(exc).split())


def _walk_nav(node, labels, items):
    """展开 nav 节点树，行号取目标值所在行（1-based）。"""
    if isinstance(node, yaml.SequenceNode):
        for child in node.value:
            _walk_nav(child, labels, items)
        return
    if isinstance(node, yaml.MappingNode):
        for key_node, value_node in node.value:
            if not isinstance(key_node, yaml.ScalarNode):
                continue
            name = str(key_node.value).strip()
            if isinstance(value_node, yaml.ScalarNode):
                items.append(NavItem(labels + (name,), value_node.value, " / ".join(labels),
                                     value_node.start_mark.line + 1))
            else:
                _walk_nav(value_node, labels + (name,), items)
        return
    if isinstance(node, yaml.ScalarNode):
        items.append(NavItem(labels, node.value, " / ".join(labels),
                             node.start_mark.line + 1))


# ---------------------------------------------------------------- 导航检查


def is_remote(target):
    return target.strip().lower().startswith(REMOTE_PREFIXES)


def resolve_target(docs_dir, target):
    """把导航里的目标转换为 docs 目录下的绝对路径；不是本地页面时返回 None."""
    raw = target.strip()
    if raw.startswith("<") and raw.endswith(">"):
        raw = raw[1:-1].strip()
    if not raw or raw.startswith("#") or is_remote(raw):
        return None
    raw = raw.split("#", 1)[0].split("?", 1)[0]
    if not raw:
        return None
    return (docs_dir / unquote(raw)).resolve()


def check_nav(docs_dir, items, issues, config_name):
    """导航目标是否存在、是否落在 docs 内、是否在同一栏目内重复."""
    docs_root = docs_dir.resolve()
    by_file = {}
    for item in items:
        path = resolve_target(docs_dir, item.target)
        if path is None:
            continue
        try:
            inside = path.relative_to(docs_root)
        except ValueError:
            issues.append(Issue(
                "error", "nav-outside",
                '导航项「%s」指向 docs 目录之外：%s' % (item.leaf, item.target),
                config_name, item.line))
            continue
        if not path.exists():
            issues.append(Issue(
                "error", "nav-missing",
                '导航项「%s」指向的文件不存在：%s' % (item.leaf, item.target),
                config_name, item.line))
            continue
        by_file.setdefault(inside.as_posix(), []).append(item)

    for relative, group in sorted(by_file.items()):
        # 同一栏目（父级相同）重复列出同一页面：编辑时的重复粘贴
        sections = {}
        for item in group:
            sections.setdefault(item.section, []).append(item)
        for section, dup in sections.items():
            if len(dup) > 1:
                issues.append(Issue(
                    "error", "nav-duplicate",
                    '栏目「%s」重复列出同一页面 %s（%s）'
                    % (section, relative, "、".join(i.leaf for i in dup)),
                    config_name, dup[-1].line))
        # 跨栏目引用可能是刻意安排（同一页面服务两个栏目），只提示确认
        if len(sections) > 1:
            issues.append(Issue(
                "warning", "nav-shared",
                "页面 %s 被 %d 个栏目同时引用（%s），确认是否有意为之"
                % (relative, len(sections), " / ".join(i.leaf for i in group)),
                config_name, group[-1].line))

    # 同一栏目里两个不同的页面用了同一个名字，读者分不清，改标题即可
    by_section = {}
    for item in items:
        by_section.setdefault(item.section, []).append(item)
    for section, group in by_section.items():
        names = {}
        for item in group:
            names.setdefault(item.leaf, []).append(item)
        for name, same_name in sorted(names.items()):
            if len(same_name) > 1 and len({i.target for i in same_name}) > 1:
                issues.append(Issue(
                    "warning", "nav-label",
                    '栏目「%s」有 %d 个导航项同名「%s」，指向不同页面'
                    % (section, len(same_name), name),
                    config_name, same_name[-1].line))


def check_orphans(docs_dir, items, issues):
    """找出存在但没有导航入口的页面."""
    referenced = set()
    for item in items:
        path = resolve_target(docs_dir, item.target)
        if path is not None:
            referenced.add(str(path))
    for path in sorted(docs_dir.rglob("*.md")):
        if str(path.resolve()) in referenced:
            continue
        issues.append(Issue(
            "warning", "docs-orphan",
            "页面 %s 没有出现在 mkdocs.yml 的导航中" % path.relative_to(docs_dir).as_posix(),
            path))


# ---------------------------------------------------------------- 链接检查


def parse_destination(line, start):
    """从 line[start]（左括号之后）开始解析链接目标，返回目标字符串或 None.

    不能简单用正则截到第一个右括号：仓库里有 `Topic%20(PublishSubscribe).png`
    这种带括号的文件名，按 Markdown 规则括号可以成对出现。
    """
    length = len(line)
    index = start
    while index < length and line[index] in " \t":
        index += 1
    if index < length and line[index] == "<":
        end = line.find(">", index + 1)
        return line[index + 1:end] if end != -1 else None
    depth = 0
    out = []
    while index < length:
        char = line[index]
        if char == "(":
            depth += 1
        elif char == ")":
            if depth == 0:
                break
            depth -= 1
        elif char in " \t":
            break  # 空格之后是链接标题
        out.append(char)
        index += 1
    return "".join(out)


def inline_targets(line):
    """取出本行所有行内链接/图片的目标."""
    targets = []
    index = line.find("](")
    while index != -1:
        target = parse_destination(line, index + 2)
        if target:
            targets.append(target)
        index = line.find("](", index + 2)
    return targets


def _strip_comments(line, in_comment):
    """去掉 HTML 注释，保留同一行里注释之后的正文；返回 (剩余文本, 是否仍在注释里)."""
    while True:
        if in_comment:
            end = line.find("-->")
            if end == -1:
                return "", True
            line = " " * (end + 3) + line[end + 3:]
            in_comment = False
            continue
        start = line.find("<!--")
        if start == -1:
            return line, False
        end = line.find("-->", start + 4)
        if end == -1:
            return line[:start], True
        line = line[:start] + " " * (end + 3 - start) + line[end + 3:]


def markdown_links(text):
    """产出 (行号, 目标)：跳过代码块、行内代码和 HTML 注释里的伪链接.

    代码块包含围栏块与四空格缩进块；缩进块按 CommonMark 的起止规则近似判断——
    只有「上一行为空且当前不在列表项里」的缩进行才算代码块，列表项的续行不算。
    """
    in_fence = None
    in_comment = False
    in_indented = False
    in_list = False
    previous_blank = True
    for lineno, line in enumerate(text.splitlines(), 1):
        fence = FENCE.match(line)
        if fence:
            marker = fence.group(1)[0]
            if in_fence is None:
                in_fence = marker
            elif in_fence == marker:
                in_fence = None
            in_indented = False
            previous_blank = False
            continue
        if in_fence is not None:
            continue
        if not line.strip():
            previous_blank = True
            continue
        indent = len(line) - len(line.lstrip(" "))
        if in_indented:
            if indent >= 4:
                continue
            in_indented = False
        if indent >= 4 and previous_blank and not in_list:
            in_indented = True
            previous_blank = False
            continue
        previous_blank = False
        if LIST_ITEM.match(line):
            in_list = True
        elif indent == 0:
            in_list = False

        line, in_comment = _strip_comments(line, in_comment)
        if not line:
            continue
        line = re.sub(r"`[^`]*`", "", line)
        targets = inline_targets(line)
        ref = REF_DEF.match(line)
        if ref:
            targets.append(ref.group(1))
        for target in targets:
            yield lineno, target


def check_links(docs_dir, issues, md_files):
    """检查文档里的相对链接、图片和附件是否存在."""
    root = docs_dir.resolve()
    for path in md_files:
        try:
            text = path.read_text(encoding="utf-8-sig")
        except UnicodeDecodeError:
            issues.append(Issue(
                "warning", "file-encoding", "文件不是 UTF-8 编码，跳过链接检查", path))
            continue
        for lineno, target in markdown_links(text):
            target = target.strip()
            if target.startswith("<") and target.endswith(">"):
                target = target[1:-1].strip()
            if not target or target.startswith("#") or is_remote(target):
                continue
            raw = target.split("#", 1)[0].split("?", 1)[0]
            if not raw:
                continue
            resolved = (path.parent / unquote(raw)).resolve()
            try:
                resolved.relative_to(root)
            except ValueError:
                issues.append(Issue(
                    "warning", "link-outside",
                    "链接指向 docs 之外的路径：%s" % target, path, lineno))
                continue
            if not resolved.exists():
                issues.append(Issue(
                    "error", "link-missing",
                    "链接目标不存在：%s" % target, path, lineno))


# ---------------------------------------------------------------- baseline


def load_baseline(path):
    """读取已知问题清单；文件缺失返回空清单，格式错误抛 ConfigError."""
    path = Path(path)
    if not path.exists():
        return []
    try:
        payload = json.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, ValueError) as exc:
        raise ConfigError("baseline %s 读取失败：%s" % (path.name, _one_line(exc)))
    entries = payload.get("entries", []) if isinstance(payload, dict) else []
    if not isinstance(entries, list):
        raise ConfigError("baseline %s 的 entries 必须是数组" % path.name)
    return [entry for entry in entries if isinstance(entry, dict)]


def apply_baseline(issues, entries):
    """把 baseline 条目对应的问题标成已知；没匹配上的条目会作为过期条目报告."""
    remaining = list(entries)
    for entry in list(remaining):
        code = entry.get("code")
        wanted_path = entry.get("path")
        needle = entry.get("message_contains", "")
        for issue in issues:
            if issue.known or issue.code != code:
                continue
            if wanted_path and str(issue.path) != wanted_path:
                continue
            if needle and needle not in issue.message:
                continue
            issue.known = True
            remaining.remove(entry)
            break
    for entry in remaining:
        issues.append(Issue(
            "warning", "baseline-stale",
            "baseline 条目已过期（找不到对应问题），可以删除：%s"
            % json.dumps(entry, ensure_ascii=False),
            BASELINE_NAME))
    return remaining


# ---------------------------------------------------------------- 汇总


def collect(root, skip_links=False, skip_orphans=False, baseline=None):
    """对仓库根目录执行全部检查，返回 (issues, 统计信息)。"""
    root = Path(root).resolve()
    config_path = root / "mkdocs.yml"
    if not config_path.exists():
        raise ConfigError("找不到 %s（在仓库根目录运行，或用 --root 指定）" % config_path.name)

    docs_dir_name, items = read_config(config_path)
    docs_dir = (root / docs_dir_name).resolve()
    config_name = config_path.name
    issues = []

    if not items:
        issues.append(Issue("error", "nav-empty",
                            "mkdocs.yml 的 nav 为空，文档站不会有任何页面入口", config_name))

    check_nav(docs_dir, items, issues, config_name)
    if not skip_orphans:
        check_orphans(docs_dir, items, issues)

    md_files = sorted(docs_dir.rglob("*.md")) if docs_dir.exists() else []
    if not skip_links:
        check_links(docs_dir, issues, md_files)

    # 先把路径统一成仓库相对路径，baseline 才能用稳定的路径匹配
    for issue in issues:
        if isinstance(issue.path, Path):
            try:
                issue.path = issue.path.relative_to(root).as_posix()
            except ValueError:
                issue.path = str(issue.path)

    if baseline:
        apply_baseline(issues, baseline)

    return issues, {"nav": len(items), "pages": len(md_files)}


def summarize(issues):
    errors = [i for i in issues if i.level == "error" and not i.known]
    warnings = [i for i in issues if i.level == "warning" and not i.known]
    known = [i for i in issues if i.known]
    return errors, warnings, known


def print_report(issues, stats, use_json, strict):
    errors, warnings, known = summarize(issues)
    if use_json:
        print(json.dumps({
            "nav_items": stats["nav"],
            "pages": stats["pages"],
            "errors": [i.as_dict() for i in errors],
            "warnings": [i.as_dict() for i in warnings],
            "known": [i.as_dict() for i in known],
        }, ensure_ascii=False, indent=2))
        return

    print("文档自检：导航项 %d 个，文档页 %d 个" % (stats["nav"], stats["pages"]))
    for title, group in (("错误", errors), ("警告", warnings), ("已知（baseline）", known)):
        if not group:
            continue
        print("\n%s（%d 项）" % (title, len(group)))
        for issue in group:
            print("  [%s] %s" % (issue.code, issue))
    if not issues:
        print("\n未发现问题。")
        return
    print("\n合计：%d 个错误，%d 个警告，%d 个已知" % (len(errors), len(warnings), len(known)))
    if strict and warnings and not errors:
        print("--strict：警告同样视为失败。")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="检查 mkdocs 导航与文档链接的一致性（不需要 ROS，也不需要构建页面）。")
    parser.add_argument("--root", default=str(ROOT), help="仓库根目录，默认为脚本所在仓库")
    parser.add_argument("--strict", action="store_true", help="警告同样返回非零退出码")
    parser.add_argument("--json", action="store_true", help="以 JSON 输出结果")
    parser.add_argument("--skip-links", action="store_true", help="跳过文档内链接检查")
    parser.add_argument("--skip-orphans", action="store_true",
                        help="跳过“页面没有导航入口”的检查")
    parser.add_argument("--baseline", default=None,
                        help="已知问题清单；缺省读 <root>/%s（存在时）" % BASELINE_NAME)
    parser.add_argument("--no-baseline", action="store_true", help="不使用已知问题清单")
    args = parser.parse_args(argv)

    root = Path(args.root).resolve()
    baseline = []
    baseline_path = None
    if not args.no_baseline:
        if args.baseline:
            given = Path(args.baseline)
            # 相对路径按 --root 解析，避免在不同工作目录下跑出不同结果
            baseline_path = given if given.is_absolute() else root / given
        else:
            baseline_path = root / BASELINE_NAME
        try:
            baseline = load_baseline(baseline_path)
        except ConfigError as exc:
            sys.stderr.write("错误：%s\n" % exc)
            return 2
        if baseline:
            try:
                shown = baseline_path.resolve().relative_to(root).as_posix()
            except ValueError:
                shown = str(baseline_path)
            print("已应用 baseline：%s（%d 条）" % (shown, len(baseline)))

    try:
        issues, stats = collect(root, skip_links=args.skip_links,
                                skip_orphans=args.skip_orphans, baseline=baseline)
    except ConfigError as exc:
        sys.stderr.write("错误：%s\n" % exc)
        return 2

    print_report(issues, stats, args.json, args.strict)
    errors, warnings, _known = summarize(issues)
    if errors or (args.strict and warnings):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
