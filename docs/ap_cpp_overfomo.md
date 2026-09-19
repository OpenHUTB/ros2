title: AP-CPP 主动感知覆盖路径规划

# AP-CPP：主动感知覆盖路径规划

本模块把 OverFOMO 的自适应覆盖路径规划（Adaptive Coverage Path Planning）扩展为**主动感知**规划：不只是决定**飞多快**，还要决定**该去哪里看**。

完整工程源码在 [`src/ap_cpp_overfomo/`](https://github.com/Xiangyuetang91/ros2/tree/feat/ap_cpp_overfomo/src/ap_cpp_overfomo)，包含核心规划源码、ROS 包 `ap_cpp_ros`、演示脚本、测试与实验数据；完整说明见该目录下的 [README](https://github.com/Xiangyuetang91/ros2/blob/feat/ap_cpp_overfomo/src/ap_cpp_overfomo/README.md)。

---

## 1. 要解决的问题

原论文的方法预先算好一条往复式（boustrophedon）航带，开环飞行，并根据图像语义调节地速：识别得越确定就越快，冠层越密或越模糊就越慢。

它回答的是「**飞过这条航带时该飞多快**」，但没有回答「**这条航带是否值得消耗电量**」。一块田里各个位置的信息价值并不相同——上一季的产量图、粗分辨率过飞得到的 NDVI 异常、农户报告的病害地块、上一次飞行留下的空洞，都会把不确定性集中到少数几个区域。一个不能偏离航线的机器人，会把续航花在已经看明白的地方，最后带着「唯一重要那块区域仍然没搞清」的地图返航。

**AP-CPP 补上的就是这一个自由度。**

## 2. 方法

规划器维护一个对作业区域的显式**信念**（占据栅格 + 逐格信息熵），并用**滚动时域（receding horizon）**对剩余航线反复重规划。每个候选动作的评分同时权衡**航程代价**和**期望信息增益**。

由此得到的规划器是**构造性安全**的：当农田中没有任何已知信息时，一个横向偏离惩罚项会把它牢牢压回原有的往复式航带，与已发表的结果逐点一致；只有当某处存在显著的信息热点时，它才会主动绕行。

| 组成 | 作用 |
|---|---|
| `ap_cpp/grid_model.py` | 占据 + 熵信念栅格 |
| `ap_cpp/sensor.py` | 传感器视场与观测模型 |
| `ap_cpp/utility.py` | 航程代价 / 信息增益效用函数 |
| `ap_cpp/planner.py` | 滚动时域规划器（核心算法） |
| `ap_cpp/control.py` | 地速控制器 |
| `ap_cpp/mission.py` | 任务循环与终止条件 |
| `ap_cpp/geo_bridge.py` | 经纬度 ↔ 局部 NED 坐标转换 |
| `ap_cpp/airsim_driver.py` | AirSim 实飞驱动（可选） |

## 3. 运行效果

### 3.1 RViz 中的实时规划

规划器可以脱离 AirSim / Unreal 独立运行，直接发布 `nav_msgs/Path` 与 `visualization_msgs/MarkerArray` 供 RViz 显示。下图为 ROS 环境中的实际运行截图（`roslaunch ap_cpp_ros rviz_demo.launch`）：

![AP-CPP 在 RViz 中的规划结果](./img/ap_cpp_overfomo/rviz_demo.png)

覆盖航迹绘制为 `nav_msgs/Path`，信念栅格与信息热点绘制为 `visualization_msgs/MarkerArray`；地块几何取自本仓库的 `CPP/002` 定义。

```sh
roslaunch ap_cpp_ros rviz_demo.launch                          # AP-CPP
roslaunch ap_cpp_ros rviz_demo.launch reference_only:=true     # 消融（仅参考航带）
roslaunch ap_cpp_ros rviz_demo.launch source:=geojson field:=002
```

### 3.2 消融对比

在相同地块与相同先验下，对比「主动感知」与「仅按参考航带飞」两种策略。下图左侧主动感知版本会主动离开航带以消除异常热点，随后重新并回参考航带；右侧基线则始终被约束在已发表的往复式航带上，从不偏离。

<table>
  <tr>
    <th align="center">AP-CPP（主动感知）</th>
    <th align="center">Reference-only 基线</th>
  </tr>
  <tr>
    <td align="center"><img src="./img/ap_cpp_overfomo/ap_cpp_active.png" alt="AP-CPP 主动感知任务" width="430"></td>
    <td align="center"><img src="./img/ap_cpp_overfomo/ap_cpp_baseline.png" alt="仅参考航带基线任务" width="430"></td>
  </tr>
  <tr>
    <td align="center"><em>主动离开航带以消除异常热点，<br />随后重新并回参考航带。</em></td>
    <td align="center"><em>被约束在已发表的往复式航带上，<br />从不偏离。</em></td>
  </tr>
</table>

消融曲线（信息增益 / 覆盖率随步数变化）：

![消融曲线](./img/ap_cpp_overfomo/ablation_curve.png)

## 4. 快速开始

### 4.1 环境要求

规划器与演示脚本只依赖 NumPy 与 Matplotlib，**不需要 AirSim、不需要 TensorFlow、不需要 GDAL**：

```sh
pip install -r src/ap_cpp_overfomo/requirements.txt
```

### 4.2 运行演示（无需仿真器）

```sh
cd src/ap_cpp_overfomo

# 自带的合成地块（不依赖仓库数据）
python demos/run_ap_cpp_demo.py

# 使用仓库中的真实地块（Polygon002.geojson + 预生成 TurnWPs.txt 航线）
python demos/run_ap_cpp_demo.py --source geojson

# 同时运行消融基线，量化主动感知带来的收益
python demos/run_ap_cpp_demo.py --compare
```

每次运行会输出一份 JSON 任务报告与一组四联诊断图（信念熵前后对比、累计覆盖率、地块上的规划航迹、任务收敛过程）。

### 4.3 运行测试

```sh
cd src/ap_cpp_overfomo
python -m pytest tests/ -q
```

### 4.4 真实地块的农学先验

演示支持注入任意 `[0, 1]` 风险图层（上一季产量图、NDVI 异常、巡田报告、上次飞行的空洞），通过 `--prior` 与 `--prior-weight` 控制：

```sh
python demos/run_ap_cpp_demo.py --source geojson --prior anomaly --prior-weight 0.6
```

## 5. 复现说明与实测数据

本节数值由 `demos/run_ap_cpp_demo.py --compare` 在不依赖仿真器的 Python 环境下生成，与 `src/ap_cpp_overfomo/results/` 中提交的图片一一对应。ROS 节点在 `roslaunch` 下运行时另有一组指标，两次运行不可混用。

**合成地块（864 步，默认参数）**

| 指标 | AP-CPP | 参考基线 |
|---|---|---|
| 飞行步数 | 864 | 864 |
| 重规划次数 | 432 | — |
| 飞行距离 | 5183.1 m | — |
| 平均地速 | 2.17 m/s | — |
| 平均信念熵 | 0.2084（初值 1.0000） | — |
| 覆盖率 | 0.5805 | — |
| 累计信息量 | 122.327 | — |

**真实地块（`CPP/002`，`--source geojson`）**

| 指标 | 数值 |
|---|---|
| 飞行距离 | 572.7 m |
| 平均地速 | 2.63 m/s |
| 平均信念熵 | 0.2143（初值 1.0000） |
| 覆盖率 | 0.5605 |
| 累计信息量 | 10.987 |

## 6. 关于大体积数据文件

仓库约定「尽量保存文本文件，大体积数据通过永久网盘链接提供」，因此以下文件**未**纳入版本库，需要时请从网盘获取并按下述说明放置：

| 文件 | 大小 | 用途 |
|---|---|---|
| `weights0500.hdf5` | 51 MB | OverFOMO 原始语义分割模型权重，供 `main.py` 配套流程使用 |
| `CPP/00{0..4}/viewpoints_map.jpg` | 共 52 MB | 原始 OverFOMO 流程生成的可视化底图，仅 `plot_vwps_on_field.py` 使用 |
| `gif/demo.gif` | 10 MB | 上游仓库的演示动图（本模块文档未引用） |
| `AP_CPP_Assignment_Submission.zip` | 9 MB | 最终作业提交包（内容与本节目录高度重复） |

> **网盘下载地址**：`<待补充>`

放置方式：`weights0500.hdf5`、`gif/demo.gif` 与 `AP_CPP_Assignment_Submission.zip` 放在 `src/ap_cpp_overfomo/` 根目录下（动图放回 `gif/` 子目录）；`viewpoints_map.jpg` 分别放回 `CPP/000/` 至 `CPP/004/`。

**本模块的核心部分（`ap_cpp/` 规划源码、演示、测试与 ROS 包）不依赖上述任何文件，可直接运行**，见第 4 节。

---

## 参考文献

关于 OverFOMO 的原始方法与引用格式，见 `src/ap_cpp_overfomo/README.md` 的 Citation 一节。
