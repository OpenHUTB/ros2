# 雨雾天气下自适应感知降级策略的静态帧复现

「恶劣雨雾天气下多传感器感知性能对比」系列的静态帧版：不需要启动 CARLA 服务端，
用**一帧真实 CARLA Town10HD 街景 + 程序合成的雨雾退化 + MobileNet-SSD 检测器**
复现该系列第五部分「自适应感知降级策略」的核心结论：
**雨天纯相机漏检的目标，融合雷达分支后能补回**。

## 与实时渲染版的区别

| 项目 | 实时渲染版（`weather_adaptive_perception`） | 本模块 |
| --- | --- | --- |
| 图像来源 | CARLA 实时渲染四档天气 | 一帧晴天街景 + 代码合成雨雾退化 |
| 雷达 | `sensor.lidar.ray_cast` 真实点云 | 几何先验模拟（框高估距、命中概率） |
| 天气判别 | 边缘密度 / 晴天基准 | 亮度、对比度、饱和度、清晰度、暗通道均值 |
| 硬件要求 | 需 GPU 跑 CARLA | 纯 CPU，可离线复现 |

## 流程

1. 输入一帧 ClearNoon 街景（`town10_cars.png`）。
2. 天气退化：大气散射雾（远处更浓）+ 900 条雨痕 + 对比度压缩 + 轻度模糊 → HardRainNoon。
3. 检测：MobileNet-SSD（OpenCV DNN），晴天用低阈值（0.10）结果作伪真值 GT，
   两种天气按 0.22 阈值检出；加 NMS(0.45) 与几何合理性过滤。
4. 两种模式：`camera`（纯视觉）、`fusion`（视觉 ∪ 雷达几何先验分支）。
5. 按 IoU ≥ 0.5 且类别一致对齐 GT，统计四种组合的召回率。
6. 规则判别天气等级，绘制三段式对比图（晴天帧 / 雨天融合帧 / 召回柱状 + 天气特征）。

## 依赖与运行

```bash
pip install -r requirements.txt      # numpy>=1.21、opencv-python>=4.5,<5、matplotlib>=3.5
python main.py
```

模型文件（`deploy.prototxt`、`mobilenet_iter_73000.caffemodel`，约 23 MB）不进仓库，
首次运行由 `main.py` 自动下载到 `model/`；已存在则直接复用。

下载按**多个源依次回退**：先试 `raw.githubusercontent.com`，不通再换 `gh-proxy.com`
与 `jsdelivr` 镜像（国内直连 raw 常被拖到读超时，这一步决定能不能跑起来）。每个文件
下完都按期望字节数与 sha256 校验，源返回 HTML 错误页、被截断、字节数超标都判死并换
下一个源；全部失败时列出每个源的具体原因，并提示「手动下载后用 `--model-dir <目录>` 指定」。

路径含中文或空格（例如 Windows 用户名 `C:\Users\张三\...`）也能跑：`cv2.imread` 与 Caffe
加载走 C 的文件接口，读不了非 ASCII 路径，模块改用 `imdecode`/`imencode` 和「切到模型
目录用文件名加载」绕开，Linux/macOS 上行为不变。

结果写入 `results/`：

- `adaptive_perception.png`：三段式总图（上图晴天 camera、中图雨天 fusion、下图召回与天气特征）
- `panel_clear.jpg` / `panel_rain.jpg`：两张单帧检测图
- `summary.json`：伪真值数量、四种模式召回率、两档天气的特征值与判别结果

## 注意

- HardRainNoon 是程序合成的退化帧，不是 CARLA 实时渲染的雨天，光照与雨幕形态都更理想化；
  它能复现「雨雾压缩对比度、压低检出」的趋势，但不能替代真实渲染的数据。
- 雷达分支是几何先验模拟（按框高估距 + 固定种子命中概率），不是真实点云回波；
  真实点云应按「3D 包围盒内回波点数」判据实现。
- 伪真值来自晴天低阈值检测结果，衡量的是「该天气下目标还剩多少能被同一个检测器检出」，
  不含人工标注真值，因此本实验不评估误检率与框精度。
- 示例输入帧为 CARLA Town10HD 街景截图，可替换为任意 CARLA 相机帧（`python main.py -i <图片>`）。
