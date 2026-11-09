# 文档导航与链接自检工具

`tools/check_docs.py` 是本仓库的文档站自检脚本：它直接读 `mkdocs.yml` 和 `docs/`，检查导航项、
文档页和站内链接是否对得上。不需要 ROS、模拟器或浏览器，也不需要先构建网页。

仓库约定「新模块在本首页和 mkdocs.yml 增加入口」，但这类约定只能靠人工翻页面发现偏离。
把这个检查写成脚本后，同一件事可以用一条命令重复执行，也能放进 CI。

## 1. 检查项

| 级别 | 检查内容 | 说明 |
| --- | --- | --- |
| 错误 | 导航目标不存在 | `mkdocs.yml` 里写的 `xxx.md` 在 `docs/` 下找不到，构建出的页面会是死链 |
| 错误 | 导航目标在 `docs/` 之外 | 写成 `../xxx.md` 这类路径，网页里根本没有这个文件 |
| 错误 | 同一栏目重复列出同一页面 | 编辑时重复粘贴，导航栏里会出现两个一样的条目 |
| 错误 | 文档内链接或图片不存在 | 相对路径写错，图片在网页上显示不出来 |
| 警告 | 页面没有导航入口 | 页面写了但没人能找到；按约定应补首页或导航 |
| 警告 | 同一页面出现在多个栏目 | 可能是刻意安排，需要确认 |
| 警告 | 同一栏目内有重名的导航项 | 读者分不清两项的差别 |
| 警告 | 链接指向 `docs/` 之外 | 构建站里没有这些文件，只在本仓库内有效 |

链接检查只处理相对链接，`http(s)://`、`mailto:`、页内锚点 `#xx` 直接跳过；以下内容里的
`[...](...)` 也跳过，避免把教程里的示例当成真链接：

- 围栏代码块（``` 或 ~~~）；
- 四空格缩进的代码块（按 CommonMark 的起止规则近似判断：只有「上一行为空且当前不在列表项里」
  的缩进行才算代码块，列表项的续行仍按正文检查）；
- 行内代码 `` `...` ``；
- HTML 注释 `<!-- ... -->`（只跳过注释本身，同一行注释之后的正文照常检查）。

这是轻量启发式判断，不是完整的 Markdown 解析器：极端嵌套（代码块里嵌列表、列表里嵌代码块）
可能判断不准，需要时可换成 `markdown` 之类的成熟解析器。

## 2. 用法

在仓库根目录执行（Python 3.8 及以上，只需要 `pyyaml`，`mkdocs` 本身就依赖它）：

```bash
python tools/check_docs.py                      # 报告全部问题，有错误时退出码为 1
python tools/check_docs.py --strict             # 警告也当作失败
python tools/check_docs.py --json               # 输出 JSON，便于其他脚本消费
python tools/check_docs.py --skip-links         # 只查导航，跳过站内链接
python tools/check_docs.py --skip-orphans       # 跳过“页面没有导航入口”的检查
python tools/check_docs.py --baseline FILE      # 指定已知问题清单
python tools/check_docs.py --no-baseline        # 不用已知问题清单
```

缺少 `pyyaml` 时先安装：

```bash
python -m pip install pyyaml
```

## 3. 输出示例

以 `master` @ `a6ee15a` 为例，工具统计到 224 个导航项、270 个文档页：

```text
$ python tools/check_docs.py --no-baseline
文档自检：导航项 224 个，文档页 270 个

错误（2 项）
  [nav-missing] mkdocs.yml:111 导航项「拮抗共激活分析（二连杆机械臂肌肉骨骼仿真）」指向的文件不存在：./muscle_arm_cocontraction_simulation.md
  [link-missing] docs/humanoid/humanoid_keyboard_control.md:222 链接目标不存在：../water/rov_mujoco.md

警告（63 项）
  [nav-shared] mkdocs.yml:260 页面 ground/weather_perception_benchmark.md 被 2 个栏目同时引用（恶劣雨雾天气下多传感器感知性能对比实验 / 恶劣雨雾天气下相机感知性能对比实验），确认是否有意为之

合计：2 个错误，63 个警告，0 个已知
```

这两条是仓库里已知的、还没修的问题，登记在 `tools/check_docs_baseline.json`，所以默认运行是
通过的（退出码 0）：

```text
$ python tools/check_docs.py
已应用 baseline：tools/check_docs_baseline.json（2 条）
文档自检：导航项 224 个，文档页 270 个

已知（baseline）（2 项）
  [nav-missing] mkdocs.yml:111 导航项「拮抗共激活分析（二连杆机械臂肌肉骨骼仿真）」指向的文件不存在：./muscle_arm_cocontraction_simulation.md
  [link-missing] docs/humanoid/humanoid_keyboard_control.md:222 链接目标不存在：../water/rov_mujoco.md

合计：0 个错误，63 个警告，2 个已知
```

`mkdocs.yml:111`、`docs/humanoid/humanoid_keyboard_control.md:222` 都可以直接跳到编辑器对应行。
`--json` 输出字段为 `nav_items`、`pages`、`errors`、`warnings`、`known`，每条结果带
`level`、`code`、`message`、`path`、`line`、`known`。

## 4. 退出码

| 退出码 | 含义 |
| --- | --- |
| 0 | 没有错误（有警告时仍为 0，加 `--strict` 则警告也算失败） |
| 1 | 存在错误，或者 `--strict` 下有警告 |
| 2 | 工具无法运行：读不到 / 解析不了 `mkdocs.yml`、baseline 文件格式错误、缺少 `pyyaml` |

配置出错时只打印一行错误，不打印 Python traceback。

## 5. 已知问题清单（baseline）

`tools/check_docs_baseline.json` 用来登记暂时不修的问题，让 CI 只拦「新增」错误：

```json
{
  "entries": [
    {
      "code": "nav-missing",
      "path": "mkdocs.yml",
      "message_contains": "./muscle_arm_cocontraction_simulation.md",
      "reason": "页面本身还没提交，导航项在 #1015 里处理"
    }
  ]
}
```

匹配规则：`code` 必须相同，`path` 与 `message_contains` 用于区分同类问题；**每条只能消化一条
问题**，多出来的同类问题仍会报错。问题修好后忘记删条目时，工具会给出 `baseline-stale` 警告，
提示该条目已过期。

## 6. 单元测试

两种跑法等价（`check()` 失败会抛 `AssertionError`，pytest 与自带 runner 都会判失败）：

```bash
python -m pytest tests/test_check_docs_local.py -v   # 18 个用例
python tests/test_check_docs_local.py                # 自带 runner，逐项打印
```

用例都在临时目录里现造最小 mkdocs 工程，覆盖：正常工程、导航缺文件（行号）、导航指向 `docs/`
之外（不能崩溃）、同栏目重复（行号指向重复那一行）、同栏目重名、跨栏目引用、孤立页面、
`mkdocs.yml` 无效或缺失（返回 2）、代码块/行内代码/注释里的伪链接、行内注释之后的真实链接、
列表续行里的链接、带括号与 `%20` 的文件名、远程链接与 `docs/` 之外的路径、baseline 接受与
过期、`--json` 与 `--strict`、自定义 `docs_dir`。

## 7. 放进 CI

`.github/workflows/tools_tests.yml` 在 `tools/**`、`tests/test_check_docs_local.py`、`mkdocs.yml`、
`docs/**` 变化时触发，依次执行：pytest 跑 18 个用例、自带 runner 再跑一遍、对**真实仓库**跑
`python tools/check_docs.py --baseline tools/check_docs_baseline.json`（新错误才失败，警告不拦）、
`py_compile`。

也就是说，只改文档或导航的 PR 也会跑这项检查；仓库当前已知的 2 个问题不会挡住别人，新增的
断链或错误导航会。

## 8. 首次运行发现的问题

第一次在 `master` @ `0474301` 上运行（未加 baseline），报告 11 个错误、63 个警告，其中 9 个是
笔误，已在本修改中修掉：

* 3 处「水域载具」栏目内重复粘贴的导航项；
* 6 处图片相对路径错误（少了 `../` 或漏了子目录），图片本身都在。

剩下 2 个就是 baseline 里的条目：一条导航项指向不存在的页面（另有修改在处理），一条链接指向
不存在的 `water/rov_mujoco.md`（该指哪一页要作者确认）。63 个警告多为页面缺少导航入口，属于
各模块作者需要补充的内容，工具只提示，不代为修改。
