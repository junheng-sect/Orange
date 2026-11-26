#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace fastlio_dll {

using Pose = Eigen::Isometry3d;
using Points = std::vector<Eigen::Vector3d>;

struct RegistrationConfig {
  double resolution = 0.10;
  int grid_memory_mib = 512;
  double distance_cap = 1.0;
  double voxel = 0.20;
  double range_min = 0.5;
  double range_max = 30.0;
  int max_points = 2000;
  int min_points = 300;
  int min_valid = 200;
  double min_coverage = 0.60;
  double inlier_distance = 0.20;
  double min_inliers = 0.50;
  double max_median = 0.10;
  double max_rmse = 0.12;
  double min_eigenvalue = 1e-4;
  double min_eigenvalue_ratio = 1e-4;
  double yaw_length = 5.0;
  double solve_ms = 80.0;
  int max_iterations = 50;
  double cauchy_scale = 0.10;

  void validate() const;
};

struct Query {
  double distance;
  Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
  bool valid = false;
};

class DistanceField {
 public:
  DistanceField(const std::string& map_path, const RegistrationConfig& config);
  Query query(const Eigen::Vector3d& point) const;
  std::size_t bytes() const { return distances_.size() * sizeof(float); }
  double buildMilliseconds() const { return build_ms_; }

 private:
  std::size_t index(int x, int y, int z) const;

  double resolution_;
  double cap_;
  double build_ms_ = 0.0;
  Eigen::Vector3d origin_;
  Eigen::Vector3i dimensions_;
  std::vector<float> distances_;
};

struct SearchRange {
  double xy;
  double z;
  double yaw;
};

struct RegistrationResult {
  Pose correction = Pose::Identity();
  std::string reason;
  int points = 0;
  int valid = 0;
  double coverage = 0.0;
  double inliers = 0.0;
  double median = 0.0;
  double rmse = 0.0;
  double min_eigenvalue = 0.0;
  double eigenvalue_ratio = 0.0;
  double score = 0.0;
  double position_innovation = 0.0;
  double yaw_innovation = 0.0;
  double solve_ms = 0.0;
  bool accepted() const { return reason.empty(); }
};

Pose yawCorrection(const Pose& value);
double yaw(const Eigen::Matrix3d& rotation);
RegistrationResult registerScan(const DistanceField& field, const RegistrationConfig& config,
                                const Points& scan, const Pose& odom_imu, const Pose& prior,
                                const SearchRange& range, bool recovery);

}  // namespace fastlio_dll
