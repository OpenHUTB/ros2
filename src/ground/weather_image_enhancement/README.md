# 恶劣雨雾天气下相机图像增强恢复实验

本目录是「恶劣雨雾天气下相机感知性能对比实验」（`src/ground/weather_perception_benchmark`）的续作：
实验1量化了四种天气下相机图像的退化程度（亮度、对比度、边缘密度三项指标），
本实验在相同的采集流程之上，对退化图像施加经典图像增强算法，再用相同口径的
指标量化「增强能恢复多少感知能力」。

## 增强方法

| 天气 | 方法 | 说明 |
|---|---|---|
| ClearNoon | 不处理 | 作为晴朗基准与恢复率计算的分母 |
| SoftRainNoon / HardRainNoon | CLAHE | 限制对比度自适应直方图均衡，作用于 LAB 空间 L 通道（clipLimit=2.0, tile=8x8） |
| HeavyFog | 暗通道先验去雾 | He et al. CVPR 2009 的简化实现（omega=0.95, t0=0.1, patch=15，均值滤波粗精炼透射率） |

## 使用方法

1. 启动 CARLA 服务端（解压目录下的 `CarlaUE4.exe`）
2. 安装依赖：`pip install -r requirements.txt`
3. 运行：`python main.py`

## 输出（results/ 目录）

- `enhancement_compare_*.png`：三档恶劣天气下「原图 | 增强图」并排对比
- `weather_enhancement_metrics.png`：三项指标的原图 vs 增强后分组柱状图
- `metrics.csv`：原始/增强后指标数值与恢复率
  （恢复率 = (增强 − 退化) / (晴朗基准 − 退化) × 100%）

## 依赖

- CARLA 0.9.15（Python 客户端 `carla==0.9.15`）
- numpy、opencv-python、matplotlib
