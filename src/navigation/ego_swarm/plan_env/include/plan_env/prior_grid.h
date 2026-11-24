#ifndef PLAN_ENV_PRIOR_GRID_H
#define PLAN_ENV_PRIOR_GRID_H

#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace plan_env {

enum class PriorCell : std::uint8_t { Unknown, Free, Occupied };

// The prior stores 5 cm axis-aligned cells in drone_map. Navigation continues to
// use its existing drone_odom grid and collision inflation.
class PriorGrid {
 public:
  static constexpr double resolution = 0.05;

  PriorGrid(const std::string& path, const std::string& expected_map_sha);

  struct Projection {
    Projection(const PriorGrid& source, const Eigen::Matrix3d& rotation, double target_resolution);
    PriorCell classify(const Eigen::Vector3d& center_map) const;

   private:
    const PriorGrid& source_;
    Eigen::Vector3d extents_;
    // Axes and combined half-widths for exact OBB/AABB overlap.
    std::array<Eigen::Vector3d, 15> axes_;
    std::array<double, 15> widths_;
    std::size_t axis_count_ = 0;
  };

 private:
  PriorCell at(const Eigen::Vector3i& index) const;
  Eigen::Vector3d origin_;
  Eigen::Vector3i size_;
  std::vector<std::uint8_t> states_;
};

}  // namespace plan_env
#endif
