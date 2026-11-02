#ifndef HUMANOID_REACHABILITY__WORKSPACE_ANALYZER_HPP_
#define HUMANOID_REACHABILITY__WORKSPACE_ANALYZER_HPP_

#include <cstdint>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

namespace humanoid_reachability
{

/// 单个高度切片（沿 z 方向）的占用统计
struct VoxelSlice
{
  int index{0};
  double z_min{0.0};
  double z_max{0.0};
  size_t count{0};
  Eigen::Vector2d xy_min{Eigen::Vector2d::Zero()};
  Eigen::Vector2d xy_max{Eigen::Vector2d::Zero()};
  double area{0.0};  ///< 层内占用面积（m²，体素离散估计）
};

struct WorkspaceStats
{
  size_t samples{0};
  size_t occupied{0};
  size_t surface{0};
  double volume{0.0};  ///< 体素离散估计体积（m³）
  Eigen::Vector3d min_bound{Eigen::Vector3d::Zero()};
  Eigen::Vector3d max_bound{Eigen::Vector3d::Zero()};
  double max_radius{0.0};  ///< 距肩点最远距离（m）
  std::vector<VoxelSlice> slices;
};

/// 体素占用栅格：统计可达域体积、包围盒、分层切片与表面体素
class VoxelGrid
{
public:
  VoxelGrid(double resolution, double slice_height, const Eigen::Vector3d & shoulder);

  void insert(const Eigen::Vector3d & point);

  /// 遍历全部占用体素，统计切片、包围盒并提取表面体素
  void analyze();

  const WorkspaceStats & stats() const { return stats_; }
  const std::vector<Eigen::Vector3i> & surface_voxels() const { return surface_; }
  const std::unordered_set<int64_t> & keys() const { return keys_; }
  double resolution() const { return resolution_; }
  Eigen::Vector3d voxel_center(int ix, int iy, int iz) const;

private:
  static int64_t pack(int x, int y, int z);
  static void unpack(int64_t key, int & x, int & y, int & z);

  double resolution_;
  double slice_height_;
  Eigen::Vector3d shoulder_;
  std::unordered_set<int64_t> keys_;
  std::vector<Eigen::Vector3i> surface_;
  WorkspaceStats stats_;
};

}  // namespace humanoid_reachability

#endif  // HUMANOID_REACHABILITY__WORKSPACE_ANALYZER_HPP_
