# rov_ballast_trim

ROV 浮力-压载配平与初稳性核算。由外壳排水体积与部件质量表算中性浮力、重心/浮心偏心、
初稳心高 GM、横摇周期与允许扰动力矩，并反解压载质量与安装位置。

全潜状态按 `GM = z_B - z_G` 计算，**不含**水面船的 `BM = I/V` 项；理由与公式见
[文档页](../../../docs/water/rov_ballast_trim.md)。

## 依赖

- 运行：Python 3.8+，仅标准库（`math` / `json` / `argparse` / `dataclasses`）；
- 测试：无（`test/test_ballast_trim.py` 可直接运行；CI 里用 pytest 收集）；
- 配图：Pillow（只有 `tools/make_figures.py` 需要）。

## 运行

```bash
cd src/water/rov_ballast_trim
python3 main.py --output outputs
python3 main.py --target-gm 0.05 --output outputs
python3 main.py --beam 0.80 --tray-y 0.35 --output outputs
```

输出 `outputs/report.json`（已在 `.gitignore` 中忽略）。

## 测试

```bash
python3 test/test_ballast_trim.py      # 18 项算法断言，无 pytest 也能跑
python3 test/test_packaging.py         # 6 项打包一致性检查（package.xml / entry point / launch 参数）
python3 -m pytest test -v              # CI 用
```

## ROS 2 包装

```bash
colcon build --packages-select rov_ballast_trim
source install/setup.bash
ros2 launch rov_ballast_trim main.launch.py
ros2 launch rov_ballast_trim main.launch.py target_gm:=0.05 beam:=0.80
```

> 在仓库根目录直接 `colcon build` 会 **0 packages finished**：上游 `src/water/package.xml`
> 把 `src/water` 整体识别成一个 ament_python 包 `water`，colcon 不再向下递归。
> 需要显式限定发现范围（与上游 `ros2_rov_pipeline_follow` 的写法一致）：
>
> ```bash
> colcon build --base-paths src/water/rov_ballast_trim \
>   --packages-select rov_ballast_trim --symlink-install
> ```

`launch/main.launch.py` 用 `ExecuteProcess` 调 CLI，不需要 rclpy。`colcon build` 与 `ros2 launch`
由 CI 的 `ros2` 任务在 Ubuntu 22.04 + Humble 上执行（`ros-tooling/setup-ros@v0.7`），
结果见 PR 的 Actions 页；本地 Windows 只做了打包一致性检查。

## 参数

| 参数 | 缺省 | 说明 |
| --- | --- | --- |
| `--length` / `--beam` / `--height` | 0.90 / 0.70 / 0.50 m | 长方体外壳 |
| `--density` | 1027.0 kg/m³ | 缺省按 10 ℃、35 psu 表层海水 |
| `--tray-x` / `--tray-y` / `--tray-z` | 0.40 / 0.30 / -0.18 m | 压载托盘限位与安装高度 |
| `--added-mass` | 1.3 | 横摇附加惯量系数 |
| `--target-gm` | 不启用 | 给定目标 GM 后反解压载安装高度 |

部件质量表在 `rov_ballast_trim/core.py` 的 `DEFAULT_COMPONENTS`，**换成自己机器的实测值再用结论**。

## 目录

```text
rov_ballast_trim/
├── main.py                     # 入口
├── launch/main.launch.py       # ROS 2 ExecuteProcess 包装
├── rov_ballast_trim/core.py    # 静力学、反解、报告
├── rov_ballast_trim/cli.py     # 打印表格 + 写 report.json
├── test/test_ballast_trim.py   # 18 项断言
├── tools/make_figures.py       # 文档配图（Pillow）
└── validation/offline_validation.json
```
