#ifndef WATER_VISION_ENHANCEMENT__IMAGE_ENHANCER_HPP_
#define WATER_VISION_ENHANCEMENT__IMAGE_ENHANCER_HPP_

#include <opencv2/opencv.hpp>
#include <string>

namespace water_vision_enhancement
{

/**
 * @brief 浑浊水体图像增强器
 *
 * 针对水下成像的三大退化因素：
 *   1. 色彩衰减（红通道被水吸收，偏蓝绿色）
 *   2. 对比度下降（悬浮颗粒散射导致雾化）
 *   3. 亮度不均
 *
 * 处理流程：白平衡 → 通道增益校正 → CLAHE 局部对比度增强 → 去散射（暗通道先验简化版）
 */
class ImageEnhancer
{
public:
  struct Params
  {
    // 色彩校正
    double red_gain{1.35};      // 红色通道增益（补偿水体吸收）
    double blue_gain{0.85};     // 蓝色通道增益（抑制偏蓝）
    double green_gain{1.0};     // 绿色通道增益

    // CLAHE 局部对比度增强
    bool   use_clahe{true};
    double clahe_clip_limit{2.0};   // CLAHE 裁剪阈值
    int    clahe_tile_grid{8};      // CLAHE 网格大小

    // 去散射
    bool   use_dehaze{true};
    double dehaze_strength{0.6};    // 去雾强度 0~1
    double atmosphere_light{0.95};  // 大气光估计值 0~1

    // 输出
    bool   output_grayscale{false}; // 是否转灰度输出
  };

  ImageEnhancer();
  explicit ImageEnhancer(const Params & params);

  /**
   * @brief 对输入图像执行完整增强流程
   * @param input  原始 BGR 图像
   * @param output 增强后的 BGR 图像（与 input 同尺寸）
   */
  void enhance(const cv::Mat & input, cv::Mat & output);

  /**
   * @brief 计算增强前后的质量指标
   * @param original  原始图像
   * @param enhanced  增强后图像
   * @param contrast  输出：局部对比度（拉普拉斯方差）
   * @param colorfulness 输出：色彩丰富度
   * @param psnr      输出：PSNR（dB），用于衡量与原图差异
   */
  void computeMetrics(
    const cv::Mat & original, const cv::Mat & enhanced,
    double & contrast, double & colorfulness, double & psnr);

  void setParams(const Params & params) { params_ = params; }
  const Params & getParams() const { return params_; }

private:
  /** 白平衡：简单灰世界假设 */
  void whiteBalance(cv::Mat & img);

  /** 通道增益校正 */
  void channelGain(cv::Mat & img);

  /** CLAHE 局部对比度增强（在 Lab 空间的 L 通道） */
  void applyCLAHE(cv::Mat & img);

  /** 去散射：基于暗通道先验的简化实现 */
  void dehaze(cv::Mat & img);

  /** 暗通道估计 */
  cv::Mat estimateDarkChannel(const cv::Mat & img, int patch_size = 15);

  Params params_;
};

}  // namespace water_vision_enhancement

#endif  // WATER_VISION_ENHANCEMENT__IMAGE_ENHANCER_HPP_
