#include "plan_env/prior_grid.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <openssl/sha.h>
#include <sstream>
#include <stdexcept>

namespace plan_env {
namespace {
constexpr double kEdgeTolerance = 1e-9;
constexpr std::size_t kMaxCells = 25000000;

bool validHash(const std::string& hash)
{
  return hash.size() == 64 && hash.find_first_not_of("0123456789abcdef") == std::string::npos;
}

std::string hashData(const std::vector<std::uint8_t>& data)
{
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(data.data(), data.size(), digest);
  std::ostringstream hex;
  hex << std::hex << std::setfill('0');
  for (const auto value : digest) hex << std::setw(2) << unsigned(value);
  return hex.str();
}
}  // namespace

constexpr double PriorGrid::resolution;

PriorGrid::PriorGrid(const std::string& path, const std::string& expected_map_sha)
{
  std::ifstream file(path, std::ios::binary);
  char line[1025];
  if (!file.getline(line, sizeof(line)) || file.eof())
    throw std::runtime_error("missing or invalid prior grid header: " + path);
  std::istringstream header(line);
  std::string magic, frame, map_sha, payload_sha, source_sha, extra;
  double spacing;
  if (!(header >> magic >> frame >> spacing >> origin_.x() >> origin_.y() >> origin_.z()
        >> size_.x() >> size_.y() >> size_.z() >> map_sha >> payload_sha) ||
      magic != "EGO_PRIOR_1" || frame != "drone_map" || spacing != resolution ||
      !validHash(map_sha) || map_sha != expected_map_sha || !validHash(payload_sha) ||
      !origin_.allFinite() || origin_.cwiseAbs().maxCoeff() > 10000 ||
      size_.minCoeff() <= 0 || size_.maxCoeff() > 1000)
    throw std::runtime_error("invalid prior grid geometry or map SHA-256: " + path);
  for (int i = 0; i < 4; ++i)
    if (!(header >> source_sha) || !validHash(source_sha))
      throw std::runtime_error("invalid prior grid source SHA-256: " + path);
  if (header >> extra) throw std::runtime_error("unexpected prior grid header fields: " + path);

  const auto cells = std::size_t(size_.x()) * size_.y() * size_.z();
  if (cells > kMaxCells) throw std::runtime_error("prior grid exceeds memory budget: " + path);
  states_.resize(cells);
  if (!file.read(reinterpret_cast<char*>(states_.data()), states_.size()) ||
      file.peek() != std::char_traits<char>::eof() ||
      *std::max_element(states_.begin(), states_.end()) > 2 || hashData(states_) != payload_sha)
    throw std::runtime_error("prior grid payload validation failed: " + path);
}

PriorCell PriorGrid::at(const Eigen::Vector3i& id) const
{
  const auto address = (std::size_t(id.x()) * size_.y() + id.y()) * size_.z() + id.z();
  return static_cast<PriorCell>(states_[address]);
}

PriorGrid::Projection::Projection(const PriorGrid& source, const Eigen::Matrix3d& rotation,
                                  double target_resolution)
    : source_(source), extents_(rotation.cwiseAbs().rowwise().sum() * (target_resolution / 2))
{
  auto add_axis = [&](Eigen::Vector3d axis) {
    const double norm = axis.norm();
    if (norm < kEdgeTolerance) return;
    axis /= norm;
    axes_[axis_count_] = axis;
    widths_[axis_count_] = (rotation.transpose() * axis).lpNorm<1>() * (target_resolution / 2)
                           + axis.lpNorm<1>() * (resolution / 2);
    ++axis_count_;
  };
  for (int i = 0; i < 3; ++i) {
    add_axis(Eigen::Vector3d::Unit(i));
    add_axis(rotation.col(i));
    for (int j = 0; j < 3; ++j)
      add_axis(rotation.col(i).cross(Eigen::Vector3d::Unit(j)));
  }
}

PriorCell PriorGrid::Projection::classify(const Eigen::Vector3d& center_map) const
{
  const Eigen::Vector3d center = center_map - source_.origin_;
  const Eigen::Vector3d lower = center - extents_;
  const Eigen::Vector3d upper = center + extents_;
  const Eigen::Vector3d extent = source_.size_.cast<double>() * resolution;
  if ((upper.array() <= 0).any() || (lower.array() >= extent.array()).any())
    return PriorCell::Unknown;

  const Eigen::Vector3i lo = (lower.cwiseMax(0) / resolution).array().floor().cast<int>();
  const Eigen::Vector3i hi = (upper.cwiseMin(extent) / resolution).array().floor().cast<int>()
                             .matrix().cwiseMin(source_.size_ - Eigen::Vector3i::Ones());
  bool all_free = (lower.array() >= -kEdgeTolerance).all() &&
                  (upper.array() <= extent.array() + kEdgeTolerance).all();
  bool found = false;
  for (int x = lo.x(); x <= hi.x(); ++x)
    for (int y = lo.y(); y <= hi.y(); ++y)
      for (int z = lo.z(); z <= hi.z(); ++z) {
        const Eigen::Vector3d delta = resolution * Eigen::Vector3d(x + .5, y + .5, z + .5) - center;
        bool intersects = true;
        for (std::size_t i = 0; i < axis_count_; ++i) {
          if (std::abs(axes_[i].dot(delta)) >= widths_[i] - kEdgeTolerance) {
            intersects = false;
            break;
          }
        }
        if (!intersects) continue;
        found = true;
        const auto state = source_.at(Eigen::Vector3i(x, y, z));
        if (state == PriorCell::Occupied) return state;
        all_free &= state == PriorCell::Free;
      }
  return found && all_free ? PriorCell::Free : PriorCell::Unknown;
}

}  // namespace plan_env
