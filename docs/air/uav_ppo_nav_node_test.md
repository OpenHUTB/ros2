# uav_ppo_nav 部署节点本地测试

`tests/test_ppo_nav_node_local.py` 在装桩（假的 `rospy` / 消息类型）后导入 `ppo_nav_node`，离线检查部署节点的主逻辑：目标点解析、点云转世界系、速度发布、安全层 fail-safe、`body_frame` 校验等，共 31 项。

## 问题

文件里的检查函数带有 `N`（装桩后导入的 `ppo_nav_node` 模块）和 `rospy`（桩模块）两个参数。pytest 收集时会把它们当成 fixture，报：

```text
fixture 'N' not found
```

7 个用例全部 `ERROR`，CI 里等于一条断言都没跑到。

## 修复

- 把带参数的检查函数重命名为 `_check_*`，不再被 pytest 误当成用例收集；
- 新增 `test_offline_regression()` 作为 pytest 入口，复用 `main()` 跑完全部检查并断言结果；
- 独立运行 `python3 tests/test_ppo_nav_node_local.py` 的行为不变。

## 运行命令

```bash
MUJOCO_GL=osmesa python3 -m pytest tests/test_ppo_nav_node_local.py -q
```

直接运行：

```bash
python3 tests/test_ppo_nav_node_local.py
```

## 修复前后

| 场景 | 结果 |
|---|---|
| 修复前 pytest | `7 errors`（`fixture 'N' not found`） |
| 修复后 pytest | `1 passed`（该入口跑完 31 项检查） |
| 直接运行 | 通过 31 项，失败 0 项 |

环境：Ubuntu 22.04 / Python 3.10.12 / pytest 6.2.5。
