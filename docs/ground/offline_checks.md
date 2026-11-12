# 离线测试接入 GitHub Actions

仓库根目录 `tests/` 下有一批 `*_local.py` 离线测试，只使用 Python 标准库（`argparse` / `math` / `unittest` 等），不依赖 numpy、MuJoCo 或 ROS。它们在合并后没有工作流运行；本页对应的 `offline_checks.yml` 把这 6 个测试接入 CI。

## 覆盖的测试

| 测试文件 | 检查内容 | 通过项数 |
|---|---|---|
| `tests/test_braking_distance_check_local.py` | 制动距离 / 跟车间距核算 | 208 |
| `tests/test_gear_shift_policy_local.py` | 换挡策略门限与滞回 | 270 |
| `tests/test_lane_change_feasibility_local.py` | 换道可行性几何核算 | 242 |
| `tests/test_no_fly_zone_check_local.py` | 禁飞区几何判定 | 41 |
| `tests/test_speed_profile_check_local.py` | 速度曲线与限速核算 | 271 |
| `tests/test_turning_sweep_check_local.py` | 转弯扫掠与内轮差核算 | 204 |

## 本地运行

在仓库根目录执行：

```bash
PYTHONIOENCODING=utf-8 python3 tests/test_braking_distance_check_local.py
PYTHONIOENCODING=utf-8 python3 tests/test_gear_shift_policy_local.py
PYTHONIOENCODING=utf-8 python3 tests/test_lane_change_feasibility_local.py
PYTHONIOENCODING=utf-8 python3 tests/test_no_fly_zone_check_local.py
PYTHONIOENCODING=utf-8 python3 tests/test_speed_profile_check_local.py
PYTHONIOENCODING=utf-8 python3 tests/test_turning_sweep_check_local.py
```

每个测试以「通过 N 项，失败 0 项 / 全部通过」结束，退出码 0。

## 工作流

`.github/workflows/offline_checks.yml`：

- 运行环境：`ubuntu-22.04` / Python 3.10，不安装任何 pip 依赖；
- 触发条件：上述测试、对应模块目录或工作流自身改动时；
- 每个测试单独一步，失败时能直接看到是哪一个。

环境：Ubuntu 22.04.5 / Python 3.10.12。
