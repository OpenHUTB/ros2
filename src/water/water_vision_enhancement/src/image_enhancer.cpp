#include "water_vision_enhancement/image_enhancer.hpp"

#include <algorithm>
#include <cmath>

namespace water_vision_enhancement
{

ImageEnhancer::ImageEnhancer()
: params_{}
{
}

ImageEnhancer::ImageEnhancer(const Params & params)
: params_(params)
{
}

void ImageEnhancer::enhance(const cv::Mat & input, cv::Mat & output)
{
  if (input.empty()) {
    output = input.clone();
    return;
  }

  // 深拷贝，避免修改原图
  cv::Mat img = input.clone();

  // 1. 色彩校正：先白平衡消除整体色偏，再施加通道增益补偿水体吸收
  //    （若先增益后白平衡，白平衡会把各通道均值重新拉回 128，抵消增益效果）
  whiteBalance(img);
  channelGain(img);

  // 2. 局部对比度增强（CLAHE）
  if (params_.use_clahe) {
    applyCLAHE(img);
  }

  // 3. 去散射（暗通道先验简化版）
  if (params_.use_dehaze) {
    dehaze(img);
  }

  output = img;
}

void ImageEnhancer::channelGain(cv::Mat & img)
{
  // 分离 BGR 通道
  std::vector<cv::Mat> channels;
  cv::split(img, channels);

  // 增益校正（水体对红光吸收强，红光增益大；蓝光偏多，蓝光增益小）
  channels[0] *= static_cast<float>(params_.blue_gain);   // B
  channels[1] *= static_cast<float>(params_.green_gain);  // G
  channels[2] *= static_cast<float>(params_.red_gain);    // R

  cv::merge(channels, img);
}

void ImageEnhancer::whiteBalance(cv::Mat & img)
{
  // 简单灰世界假设：每个通道均值趋于 128
  std::vector<cv::Mat> channels;
  cv::split(img, channels);

  for (auto & ch : channels) {
    cv::Scalar mean = cv::mean(ch);
    double scale = 128.0 / (mean[0] + 1e-6);
    scale = std::clamp(scale, 0.5, 2.0);
    ch *= static_cast<float>(scale);
  }

  cv::merge(channels, img);
}

void ImageEnhancer::applyCLAHE(cv::Mat & img)
{
  // 转到 Lab 空间，只对 L（亮度）通道做 CLAHE
  cv::Mat lab;
  cv::cvtColor(img, lab, cv::COLOR_BGR2Lab);

  std::vector<cv::Mat> lab_channels;
  cv::split(lab, lab_channels);

  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
    params_.clahe_clip_limit,
    cv::Size(params_.clahe_tile_grid, params_.clahe_tile_grid));
  clahe->apply(lab_channels[0], lab_channels[0]);

  cv::merge(lab_channels, lab);
  cv::cvtColor(lab, img, cv::COLOR_Lab2BGR);
}

void ImageEnhancer::dehaze(cv::Mat & img)
{
  // 暗通道先验去雾简化实现
  cv::Mat dark = estimateDarkChannel(img, 15);

  // 大气光由参数给定（默认 0.95）
  double atm = params_.atmosphere_light;

  // 计算透射率 t = 1 - strength * dark / atm
  cv::Mat t;
  dark.convertTo(t, CV_64F);
  t = 1.0 - params_.dehaze_strength * (t / 255.0) / atm;
  t = cv::max(t, 0.1);  // 透射率下限

  // 恢复图像 J = (I - A) / t + A
  std::vector<cv::Mat> channels;
  cv::split(img, channels);
  for (auto & ch : channels) {
    cv::Mat ch_f;
    ch.convertTo(ch_f, CV_64F);
    ch_f = (ch_f - atm * 255.0) / t + atm * 255.0;
    ch_f.convertTo(ch, CV_8U);
  }
  cv::merge(channels, img);
}

cv::Mat ImageEnhancer::estimateDarkChannel(const cv::Mat & img, int patch_size)
{
  // 暗通道：每个像素取 BGR 三通道最小值，再做最小值滤波
  cv::Mat min_channel;
  std::vector<cv::Mat> channels;
  cv::split(img, channels);
  min_channel = cv::min(channels[0], cv::min(channels[1], channels[2]));

  // 最小值滤波
  cv::Mat dark;
  cv::erode(min_channel, dark,
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(patch_size, patch_size)));

  return dark;
}

void ImageEnhancer::computeMetrics(
  const cv::Mat & original, const cv::Mat & enhanced,
  double & contrast, double & colorfulness, double & psnr)
{
  // 1. 对比度：拉普拉斯方差
  cv::Mat gray_enhanced;
  cv::cvtColor(enhanced, gray_enhanced, cv::COLOR_BGR2GRAY);
  cv::Mat lap;
  cv::Laplacian(gray_enhanced, lap, CV_64F);
  cv::Scalar mean, stddev;
  cv::meanStdDev(lap, mean, stddev);
  contrast = stddev[0] * stddev[0];

  // 2. 色彩丰富度（Hasler 公式）
  cv::Mat lab;
  cv::cvtColor(enhanced, lab, cv::COLOR_BGR2Lab);
  std::vector<cv::Mat> lab_ch;
  cv::split(lab, lab_ch);
  cv::Scalar a_mean, a_std, b_mean, b_std;
  cv::meanStdDev(lab_ch[1], a_mean, a_std);
  cv::meanStdDev(lab_ch[2], b_mean, b_std);
  double a = a_mean[0], b = b_mean[0];
  double sa = a_std[0], sb = b_std[0];
  colorfulness = std::sqrt(sa * sa + sb * sb) + 0.3 * std::sqrt(a * a + b * b);

  // 3. PSNR（与原图差异）
  cv::Mat diff;
  cv::absdiff(original, enhanced, diff);
  diff.convertTo(diff, CV_32F);
  diff = diff.mul(diff);
  cv::Scalar mse = cv::mean(diff);
  double mse_val = (mse[0] + mse[1] + mse[2]) / 3.0;
  if (mse_val < 1e-10) {
    psnr = 100.0;
  } else {
    psnr = 10.0 * std::log10(255.0 * 255.0 / mse_val);
  }
}

}  // namespace water_vision_enhancement
