// Distance-field residual and four-dimensional solve adapted from robotics-upo/dll
// at 04623521a21e809f8b695d65a141387fb9b1a2f9. See LICENSE.
#include "registration.hpp"

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <map>
#include <stdexcept>
#include <tuple>

#include <ceres/ceres.h>
#include <Eigen/Eigenvalues>
#include <pcl/io/pcd_io.h>

namespace fastlio_dll {
namespace {

using Clock = std::chrono::steady_clock;
using Parameters = Eigen::Vector4d;

Eigen::Matrix3d rotateYaw(double angle) {
  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

double angleDifference(double a, double b) {
  return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
}

struct Residual {
  Query query;
  Eigen::Vector4d jacobian;
};

Residual evaluate(const DistanceField& field, const Eigen::Vector3d& point,
                  const Parameters& x) {
  const Eigen::Vector3d rotated = rotateYaw(x[3]) * point;
  const auto hit = field.query(rotated + x.head<3>());
  Residual result{hit, Eigen::Vector4d::Zero()};
  result.jacobian.head<3>() = hit.gradient;
  result.jacobian[3] = hit.gradient.dot(Eigen::Vector3d(-rotated.y(), rotated.x(), 0));
  return result;
}

class Cost : public ceres::SizedCostFunction<1, 4> {
 public:
  Cost(const DistanceField& field, const Eigen::Vector3d& point)
      : field_(field), point_(point) {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const auto value = evaluate(field_, point_, Eigen::Map<const Parameters>(parameters[0]));
    residuals[0] = value.query.distance;
    if (jacobians && jacobians[0]) {
      Eigen::Map<Parameters> gradient(jacobians[0]);
      gradient = value.jacobian;
    }
    return true;
  }

 private:
  const DistanceField& field_;
  Eigen::Vector3d point_;
};

Points sample(const Points& input, const RegistrationConfig& config,
              const Eigen::Matrix3d& odom_rotation) {
  std::map<std::tuple<int, int, int>, Eigen::Vector3d> voxels;
  for (const auto& p : input) {
    if (!p.allFinite() || p.squaredNorm() < config.range_min * config.range_min ||
        p.squaredNorm() > config.range_max * config.range_max) continue;
    const Eigen::Array3d scaled = (p / config.voxel).array().floor();
    voxels.try_emplace(std::make_tuple(static_cast<int>(scaled[0]), static_cast<int>(scaled[1]),
                                      static_cast<int>(scaled[2])), p);
  }
  const auto count = std::min(voxels.size(), static_cast<std::size_t>(config.max_points));
  Points output;
  output.reserve(count);
  std::size_t index = 0;
  for (const auto& entry : voxels) {
    if ((index + 1) * count / voxels.size() > index * count / voxels.size()) {
      output.emplace_back(odom_rotation * entry.second);
    }
    ++index;
  }
  return output;
}

RegistrationResult solve(const DistanceField& field, const RegistrationConfig& config,
                         const Points& points, const Pose& odom_imu,
                         const Parameters& center, const Parameters& offset,
                         const SearchRange& range) {
  RegistrationResult result;
  result.points = static_cast<int>(points.size());
  if (result.points < config.min_points) {
    result.reason = "too_few_points";
    return result;
  }
  Parameters x = center + offset;
  ceres::Problem problem;
  auto* loss = new ceres::CauchyLoss(config.cauchy_scale);
  for (const auto& point : points) problem.AddResidualBlock(new Cost(field, point), loss, x.data());
  const Parameters bounds(range.xy, range.xy, range.z, range.yaw);
  for (int axis = 0; axis < 4; ++axis) {
    problem.SetParameterLowerBound(x.data(), axis, center[axis] - bounds[axis]);
    problem.SetParameterUpperBound(x.data(), axis, center[axis] + bounds[axis]);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.num_threads = 1;
  options.max_num_iterations = config.max_iterations;
  options.max_solver_time_in_seconds = config.solve_ms * 0.001;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  result.solve_ms = summary.total_time_in_seconds * 1000;
  if (result.solve_ms >= config.solve_ms) result.reason = "solve_timeout";
  else if (!summary.IsSolutionUsable() || summary.termination_type != ceres::CONVERGENCE ||
           !x.allFinite()) result.reason = "no_convergence";
  if (!result.reason.empty()) return result;

  const Eigen::Matrix3d rotation = rotateYaw(x[3]);
  result.correction.linear() = rotation;
  result.correction.translation() = x.head<3>() - rotation * odom_imu.translation();
  result.position_innovation = (x.head<3>() - center.head<3>()).norm();
  result.yaw_innovation = angleDifference(x[3], center[3]);

  Eigen::Matrix4d info = Eigen::Matrix4d::Zero();
  std::vector<double> distances;
  distances.reserve(points.size());
  int inliers = 0;
  double squared_inliers = 0.0;
  for (const auto& point : points) {
    auto hit = evaluate(field, point, x);
    const double d2 = hit.query.distance * hit.query.distance;
    result.score += std::log1p(d2 / (config.cauchy_scale * config.cauchy_scale));
    if (!hit.query.valid) continue;
    distances.push_back(hit.query.distance);
    if (hit.query.distance <= config.inlier_distance) {
      ++inliers;
      squared_inliers += d2;
    }
    hit.jacobian[3] /= config.yaw_length;
    const double weight = 1.0 / (1.0 + d2 / (config.cauchy_scale * config.cauchy_scale));
    info.noalias() += weight * hit.jacobian * hit.jacobian.transpose();
  }
  result.score /= points.size();
  result.valid = static_cast<int>(distances.size());
  result.coverage = static_cast<double>(result.valid) / points.size();
  result.inliers = static_cast<double>(inliers) / points.size();
  result.rmse = inliers ? std::sqrt(squared_inliers / inliers) : config.distance_cap;
  result.median = config.distance_cap;
  if (!distances.empty()) {
    auto middle = distances.begin() + distances.size() / 2;
    std::nth_element(distances.begin(), middle, distances.end());
    result.median = *middle;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen(info / result.valid, Eigen::EigenvaluesOnly);
    if (eigen.info() == Eigen::Success) {
      result.min_eigenvalue = eigen.eigenvalues()[0];
      const double largest = eigen.eigenvalues()[3];
      if (largest > 0.0) result.eigenvalue_ratio = result.min_eigenvalue / largest;
    }
  }

  if (result.valid < config.min_valid || result.coverage < config.min_coverage) {
    result.reason = "coverage";
  } else if (result.inliers < config.min_inliers || result.median > config.max_median ||
             result.rmse > config.max_rmse) {
    result.reason = "residual";
  } else if (result.min_eigenvalue < config.min_eigenvalue ||
             result.eigenvalue_ratio < config.min_eigenvalue_ratio) {
    result.reason = "degenerate";
  } else if (result.position_innovation > range.xy || result.yaw_innovation > range.yaw ||
             ((x - center).cwiseAbs().array() >= bounds.array() - 1e-5).any()) {
    result.reason = "innovation";
  }
  return result;
}

}  // namespace

void RegistrationConfig::validate() const {
  const std::array<double, 16> positive{resolution, distance_cap, voxel, range_max, min_coverage,
      inlier_distance, min_inliers, max_median, max_rmse, min_eigenvalue, min_eigenvalue_ratio,
      yaw_length, solve_ms, cauchy_scale, range_min, static_cast<double>(grid_memory_mib)};
  if (std::any_of(positive.begin(), positive.end(), [](double x) { return !std::isfinite(x) || x <= 0; }) ||
      range_min >= range_max || range_max / voxel > 1e8 || max_points < min_points ||
      min_points < min_valid || min_valid < 1 || min_coverage > 1.0 || min_inliers > 1.0 ||
      min_eigenvalue_ratio > 1.0 || max_iterations < 1 || inlier_distance > distance_cap) {
    throw std::invalid_argument("invalid DLL registration configuration");
  }
}

DistanceField::DistanceField(const std::string& map_path, const RegistrationConfig& config)
    : resolution_(config.resolution), cap_(config.distance_cap) {
  const auto started = Clock::now();
  pcl::PointCloud<pcl::PointXYZ> loaded;
  if (pcl::io::loadPCDFile(map_path, loaded) != 0) throw std::runtime_error("cannot load map PCD");
  pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Vector3d lo = Eigen::Vector3d::Constant(INFINITY);
  Eigen::Vector3d hi = -lo;
  for (const auto& p : loaded) {
    const Eigen::Vector3d v(p.x, p.y, p.z);
    if (!v.allFinite()) continue;
    map->push_back(p);
    lo = lo.cwiseMin(v);
    hi = hi.cwiseMax(v);
  }
  if (map->empty()) throw std::runtime_error("empty map PCD");

  const double padding = cap_ + resolution_;
  origin_ = lo.array() - padding;
  const auto max_cells = static_cast<std::size_t>(config.grid_memory_mib) * 1024 * 1024 / sizeof(float);
  std::size_t cells = 1;
  for (int axis = 0; axis < 3; ++axis) {
    const double size = std::ceil((hi[axis] + padding - origin_[axis]) / resolution_) + 1.0;
    if (!std::isfinite(size) || size < 2 ||
        size > static_cast<double>(max_cells / cells) || size >= INT_MAX) {
      throw std::runtime_error("distance field exceeds memory budget");
    }
    dimensions_[axis] = static_cast<int>(size);
    cells *= dimensions_[axis];
  }
  // A vertex only needs distances to map points within the truncation radius.
  // Visit that neighborhood for each map point instead of running a nearest
  // neighbor search from every vertex, including the empty parts of the box.
  const double radius = cap_ / resolution_;
  const auto lower = [](double v, double r) { return std::max(0, int(std::ceil(v - r))); };
  const auto upper = [this](int axis, double v, double r) {
    return static_cast<int>(std::min(dimensions_[axis] - 1.0, std::floor(v + r)));
  };
  distances_.assign(cells, static_cast<float>(radius * radius));
  for (const auto& point : *map) {
    const Eigen::Vector3d p = (Eigen::Vector3d(point.x, point.y, point.z) - origin_) / resolution_;
    const int x0 = lower(p.x(), radius), x1 = upper(0, p.x(), radius);
    const int y0 = lower(p.y(), radius), y1 = upper(1, p.y(), radius);
    const int z0 = lower(p.z(), radius), z1 = upper(2, p.z(), radius);
    for (int z = z0; z <= z1; ++z) {
      const double dz2 = (z - p.z()) * (z - p.z());
      for (int y = y0; y <= y1; ++y) {
        const double yz2 = dz2 + (y - p.y()) * (y - p.y());
        if (yz2 >= radius * radius) continue;
        float* row = distances_.data() + index(x0, y, z);
        for (int x = x0; x <= x1; ++x, ++row) {
          const double dx = x - p.x();
          *row = std::min(*row, static_cast<float>(yz2 + dx * dx));
        }
      }
    }
  }
  for (auto& distance : distances_) distance = std::sqrt(distance) * resolution_;
  build_ms_ = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
}

std::size_t DistanceField::index(int x, int y, int z) const {
  return (static_cast<std::size_t>(z) * dimensions_.y() + y) * dimensions_.x() + x;
}

Query DistanceField::query(const Eigen::Vector3d& point) const {
  Query result{cap_};
  const Eigen::Array3d grid = (point - origin_).array() / resolution_;
  if (!grid.isFinite().all() || (grid < 0.0).any() ||
      (grid >= dimensions_.cast<double>().array() - 1.0).any()) return result;

  const Eigen::Vector3i base = grid.cast<int>();
  const Eigen::Array3d fraction = grid - base.cast<double>().array();
  result.distance = 0.0;
  result.valid = true;
  for (int dz = 0; dz < 2; ++dz) {
    for (int dy = 0; dy < 2; ++dy) {
      for (int dx = 0; dx < 2; ++dx) {
        const double wx = dx ? fraction.x() : 1.0 - fraction.x();
        const double wy = dy ? fraction.y() : 1.0 - fraction.y();
        const double wz = dz ? fraction.z() : 1.0 - fraction.z();
        const double d = distances_[index(base.x() + dx, base.y() + dy, base.z() + dz)];
        result.distance += d * wx * wy * wz;
        result.gradient += d * Eigen::Vector3d((dx ? 1 : -1) * wy * wz,
            wx * (dy ? 1 : -1) * wz, wx * wy * (dz ? 1 : -1));
      }
    }
  }
  if (result.distance >= cap_) {
    result.distance = cap_;
    result.gradient.setZero();
  } else {
    result.gradient /= resolution_;
  }
  return result;
}

double yaw(const Eigen::Matrix3d& rotation) {
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

Pose yawCorrection(const Pose& value) {
  Pose result = Pose::Identity();
  result.linear() = rotateYaw(yaw(value.linear()));
  result.translation() = value.translation();
  return result;
}

RegistrationResult registerScan(const DistanceField& field, const RegistrationConfig& config,
                                const Points& scan, const Pose& odom_imu, const Pose& prior,
                                const SearchRange& range, bool recovery) {
  const auto begin = Clock::now();
  const Points points = sample(scan, config, odom_imu.linear());
  Parameters center;
  center << prior * odom_imu.translation(), yaw(prior.linear());
  auto best = solve(field, config, points, odom_imu, center, Parameters::Zero(), range);
  if (!recovery || best.reason == "too_few_points") return best;

  constexpr double round_ms = 750.0;
  const std::array<Parameters, 4> offsets{{
      {range.xy * 0.6, 0, 0, range.yaw * 0.6}, {-range.xy * 0.6, 0, 0, -range.yaw * 0.6},
      {0, range.xy * 0.6, range.z * 0.5, -range.yaw * 0.6},
      {0, -range.xy * 0.6, -range.z * 0.5, range.yaw * 0.6}}};
  std::vector<RegistrationResult> candidates{best};
  for (const auto& offset : offsets) {
    if (std::chrono::duration<double, std::milli>(Clock::now() - begin).count() >= round_ms) break;
    candidates.push_back(solve(field, config, points, odom_imu, center, offset, range));
  }
  const bool incomplete = candidates.size() != offsets.size() + 1 ||
      std::chrono::duration<double, std::milli>(Clock::now() - begin).count() >= round_ms ||
      std::any_of(candidates.begin(), candidates.end(), [](const auto& c) {
        return c.reason == "solve_timeout" || c.reason == "no_convergence";
      });
  for (const auto& candidate : candidates) {
    if (candidate.accepted() && (!best.accepted() || candidate.score < best.score)) best = candidate;
  }
  if (incomplete) best.reason = "recovery_incomplete";
  if (best.accepted()) {
    for (const auto& c : candidates) {
      if (!c.accepted() || best.score < 0.9 * c.score) continue;
      const auto& origin = odom_imu.translation();
      if ((best.correction * origin - c.correction * origin).norm() > 0.25 ||
          angleDifference(yaw(best.correction.linear()), yaw(c.correction.linear())) > 0.07) {
        best.reason = "recovery_ambiguous";
        break;
      }
    }
  }
  best.solve_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
  return best;
}

}  // namespace fastlio_dll
