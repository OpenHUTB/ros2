#include "humanoid_reachability/workspace_analyzer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>

namespace humanoid_reachability
{
namespace
{
constexpr int kIndexBits = 21;
constexpr int64_t kIndexMask = (int64_t(1) << kIndexBits) - 1;

struct SliceAccumulator
{
  size_t count{0};
  double min_x{std::numeric_limits<double>::infinity()};
  double min_y{std::numeric_limits<double>::infinity()};
  double max_x{-std::numeric_limits<double>::infinity()};
  double max_y{-std::numeric_limits<double>::infinity()};
};
}  // namespace

VoxelGrid::VoxelGrid(double resolution, double slice_height, const Eigen::Vector3d & shoulder)
: resolution_(resolution), slice_height_(slice_height), shoulder_(shoulder)
{
}

int64_t VoxelGrid::pack(int x, int y, int z)
{
  return ((static_cast<int64_t>(x) & kIndexMask) << (2 * kIndexBits)) |
         ((static_cast<int64_t>(y) & kIndexMask) << kIndexBits) |
         (static_cast<int64_t>(z) & kIndexMask);
}

void VoxelGrid::unpack(int64_t key, int & x, int & y, int & z)
{
  x = static_cast<int>((key >> (2 * kIndexBits)) & kIndexMask);
  y = static_cast<int>((key >> kIndexBits) & kIndexMask);
  z = static_cast<int>(key & kIndexMask);

  const int sign_bit = 1 << (kIndexBits - 1);
  const int span = 1 << kIndexBits;
  if (x >= sign_bit) {
    x -= span;
  }
  if (y >= sign_bit) {
    y -= span;
  }
  if (z >= sign_bit) {
    z -= span;
  }
}

Eigen::Vector3d VoxelGrid::voxel_center(int ix, int iy, int iz) const
{
  return Eigen::Vector3d(
    (static_cast<double>(ix) + 0.5) * resolution_,
    (static_cast<double>(iy) + 0.5) * resolution_,
    (static_cast<double>(iz) + 0.5) * resolution_);
}

void VoxelGrid::insert(const Eigen::Vector3d & point)
{
  const int ix = static_cast<int>(std::floor(point.x() / resolution_));
  const int iy = static_cast<int>(std::floor(point.y() / resolution_));
  const int iz = static_cast<int>(std::floor(point.z() / resolution_));

  keys_.insert(pack(ix, iy, iz));

  ++stats_.samples;
  stats_.max_radius = std::max(stats_.max_radius, (point - shoulder_).norm());
}

void VoxelGrid::analyze()
{
  surface_.clear();
  stats_.slices.clear();
  stats_.occupied = keys_.size();
  stats_.volume = static_cast<double>(stats_.occupied) * resolution_ * resolution_ * resolution_;

  if (stats_.occupied == 0) {
    stats_.min_bound.setZero();
    stats_.max_bound.setZero();
    return;
  }

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double min_z = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();

  std::map<int, SliceAccumulator> slices;
  std::vector<Eigen::Vector3i> all;
  all.reserve(keys_.size());

  for (const int64_t key : keys_) {
    int ix = 0;
    int iy = 0;
    int iz = 0;
    unpack(key, ix, iy, iz);
    all.emplace_back(ix, iy, iz);

    const Eigen::Vector3d center = voxel_center(ix, iy, iz);
    min_x = std::min(min_x, center.x());
    min_y = std::min(min_y, center.y());
    min_z = std::min(min_z, center.z());
    max_x = std::max(max_x, center.x());
    max_y = std::max(max_y, center.y());
    max_z = std::max(max_z, center.z());

    const int slice_index = static_cast<int>(std::floor(center.z() / slice_height_));
    SliceAccumulator & slice = slices[slice_index];
    ++slice.count;
    slice.min_x = std::min(slice.min_x, center.x());
    slice.min_y = std::min(slice.min_y, center.y());
    slice.max_x = std::max(slice.max_x, center.x());
    slice.max_y = std::max(slice.max_y, center.y());
  }

  const double half = resolution_ / 2.0;
  stats_.min_bound = Eigen::Vector3d(min_x - half, min_y - half, min_z - half);
  stats_.max_bound = Eigen::Vector3d(max_x + half, max_y + half, max_z + half);

  for (auto & entry : slices) {
    VoxelSlice slice;
    slice.index = entry.first;
    slice.z_min = static_cast<double>(slice.index) * slice_height_;
    slice.z_max = slice.z_min + slice_height_;
    slice.count = entry.second.count;
    slice.xy_min = Eigen::Vector2d(entry.second.min_x, entry.second.min_y);
    slice.xy_max = Eigen::Vector2d(entry.second.max_x, entry.second.max_y);
    slice.area = static_cast<double>(slice.count) * resolution_ * resolution_;
    stats_.slices.push_back(slice);
  }

  // 表面体素：26 邻域内至少缺一个邻居
  surface_.reserve(all.size());
  for (const Eigen::Vector3i & index : all) {
    bool is_surface = false;
    for (int dx = -1; dx <= 1 && !is_surface; ++dx) {
      for (int dy = -1; dy <= 1 && !is_surface; ++dy) {
        for (int dz = -1; dz <= 1 && !is_surface; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          if (keys_.find(pack(index.x() + dx, index.y() + dy, index.z() + dz)) == keys_.end()) {
            is_surface = true;
          }
        }
      }
    }
    if (is_surface) {
      surface_.push_back(index);
    }
  }
  stats_.surface = surface_.size();
}

}  // namespace humanoid_reachability
