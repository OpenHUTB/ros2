# uav_ppo_nav 本地测试（pytest）

`tests/test_ppo_nav_local.py` 是 `uav_ppo_nav` 的离线自检，分两部分：

- **A. 策略纯函数**：直方图、世界系坐标变换、22 维观测拼接、动作映射、MLP 推理；
- **B. `NavEnv` 训练环境**：reset / step / 观测维度 / 终止判定。

A 部分只依赖 numpy；B 部分需要 `gymnasium`。

## 运行命令

```bash
MUJOCO_GL=osmesa python3 -m pytest tests/test_ppo_nav_local.py -q
```

也可以直接运行（不经过 pytest）：

```bash
python3 tests/test_ppo_nav_local.py
```

## 缺 gymnasium 时的行为

模块顶层会尝试导入 `env.NavEnv`。缺少 `gymnasium` 时 `NavEnv` 被置为 `None` 并记录 `ENV_SKIP`，`main()` 会跳过 B 部分；但 pytest 直接收集 `test_env` 时仍会调用 `NavEnv(...)`，得到：

```text
TypeError: 'NoneType' object is not callable
```

修复后 `test_env` 开头判断 `NavEnv is None`，用 `pytest.skip` 跳过，pytest 结果是干净的 skip，而不是失败。

## 修复前后

| 场景 | 结果 |
|---|---|
| 修复前 pytest | `1 failed, 1 passed`（`TypeError: 'NoneType' object is not callable`） |
| 修复后 pytest | `1 passed, 1 skipped` |
| 直接运行 | 通过 14 项，失败 0 项 |

环境：Ubuntu 22.04 / Python 3.10.12 / pytest 6.2.5，未安装 `gymnasium`。
