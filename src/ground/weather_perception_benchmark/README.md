# 恶劣雨雾天气下相机感知性能对比实验

在 CARLA 0.9.15 中以 ego 车辆车顶 RGB 相机为感知传感器，在同一场景、同一参考位姿下依次切换
ClearNoon（清晰）、SoftRainNoon（小雨）、HardRainNoon（大雨）、HeavyFog（浓雾）四档天气，
对每档天气采集的图像计算亮度（灰度均值）、对比度（灰度标准差）、边缘密度（Canny 边缘像素占比）
三个指标（每档连续取 5 帧平均），量化雨雾天气对车载视觉感知的退化程度。
测量结束后车辆开启自动驾驶，并自动合成恶劣天气下的行驶演示动图。

配套教程文档：`docs/ground/weather_perception_benchmark.md`

## 运行环境

- 操作系统：Windows 10 / 11（Linux 下启动 CARLA 用 `./CarlaUE4.sh`）
- 模拟器：CARLA 0.9.15 官方预编译版（<https://github.com/carla-simulator/carla/releases/tag/0.9.15>）
- Python：**3.10**（`carla==0.9.15` 的 wheel 仅支持 3.7~3.10）
- 依赖：见 `requirements.txt`

## 运行步骤

1. 启动 CARLA 服务端：双击解压目录下的 `CarlaUE4.exe`，等待城市三维画面出现
   （RPC 服务默认监听本机 2000 端口，解压路径不要含中文或空格）。

2. 创建并激活 Python 3.10 虚拟环境，安装依赖：

   ```bash
   py -3.10 -m venv carla_env
   carla_env\Scripts\activate
   pip install -r requirements.txt
   ```

3. 在本目录下运行：

   ```bash
   python main.py
   ```

   脚本会自动生成 ego 车辆并挂载车顶相机；测量阶段车辆刹停并复位到同一出生点，
   四档天气拍摄同一场景；测量结束后车辆自动驾驶约 10 秒并合成演示动图。

## 输出结果

运行结束后在脚本同级的 `results/` 目录下得到：

| 文件 | 说明 |
| --- | --- |
| `weather_benchmark_clear_noon.png` / `weather_benchmark_soft_rain.png` / `weather_benchmark_hard_rain.png` / `weather_benchmark_heavy_fog.png` | 四档天气样本截图 |
| `metrics.csv` | 三指标数据表 |
| `weather_benchmark_metrics.png` | 三指标对比柱状图 |
| `weather_benchmark_demo.gif` | 浓雾中自动驾驶的演示动图 |

> 输出图片的命名与教程图片目录 `docs/img/ground/` 中的文件名一致，
> 提交文档时将 `results/` 下的图片直接复制过去即可，无需再改名。

## 注意事项

- 脚本会依次尝试地图中的出生点，若全部被占用导致车辆生成失败，重启 CARLA（清空场景）后重试即可。
- `carla.WeatherParameters.ClearNoon` 等预设是共享的类属性对象，不能直接修改，
  自定义天气必须用 `carla.WeatherParameters(...)` 构造独立对象（详见代码注释）。
