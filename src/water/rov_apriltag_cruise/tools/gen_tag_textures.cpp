// 生成 AprilTag 风格标牌贴图（ArUco 6x6 字典，与检测节点使用同一字典）
// 用法: gen_tag_textures <输出目录> <起始ID> <数量>
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cstdio>
#include <string>

int main(int argc, char **argv)
{
  if (argc < 4) {
    std::printf("usage: %s <out_dir> <start_id> <count>\n", argv[0]);
    return 1;
  }
  const std::string out_dir = argv[1];
  const int start_id = std::atoi(argv[2]);
  const int count = std::atoi(argv[3]);

  cv::Ptr<cv::aruco::Dictionary> dict =
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

  // 700x700 画布，中间 600x600 为标记本体，四周留白作为静区
  for (int i = 0; i < count; ++i) {
    const int id = start_id + i;
    cv::Mat marker;
    cv::aruco::drawMarker(dict, id, 600, marker, 1);
    cv::Mat canvas(700, 700, CV_8UC1, cv::Scalar(255));
    marker.copyTo(canvas(cv::Rect(50, 50, 600, 600)));
    char path[256];
    std::snprintf(path, sizeof(path), "%s/apriltag_%d.png", out_dir.c_str(), id);
    cv::imwrite(path, canvas);
    std::printf("wrote %s\n", path);
  }
  return 0;
}
