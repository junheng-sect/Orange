#include <front_marker_vision/marker_pose_estimator.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace front_marker_vision
{
namespace
{

bool finitePoint(const cv::Point2f& point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool finiteVec3(const cv::Vec3d& value)
{
  return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

bool bilinearSample(const cv::Mat& gray, const cv::Point2f& point, double& value)
{
  const int x0 = static_cast<int>(std::floor(point.x));
  const int y0 = static_cast<int>(std::floor(point.y));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;
  if (x0 < 0 || y0 < 0 || x1 >= gray.cols || y1 >= gray.rows)
  {
    return false;
  }

  const double dx = static_cast<double>(point.x) - static_cast<double>(x0);
  const double dy = static_cast<double>(point.y) - static_cast<double>(y0);
  const double top = (1.0 - dx) * gray.at<unsigned char>(y0, x0) +
                     dx * gray.at<unsigned char>(y0, x1);
  const double bottom = (1.0 - dx) * gray.at<unsigned char>(y1, x0) +
                        dx * gray.at<unsigned char>(y1, x1);
  value = (1.0 - dy) * top + dy * bottom;
  return std::isfinite(value);
}

cv::Vec3d matToVec3(const cv::Mat& input)
{
  cv::Mat converted;
  input.reshape(1, 3).convertTo(converted, CV_64F);
  return cv::Vec3d(
    converted.at<double>(0, 0), converted.at<double>(1, 0), converted.at<double>(2, 0));
}

std::vector<cv::Point3f> squareObjectPoints(double marker_size_m)
{
  const float half = static_cast<float>(marker_size_m * 0.5);
  return {
    cv::Point3f(-half, half, 0.0F),
    cv::Point3f(half, half, 0.0F),
    cv::Point3f(half, -half, 0.0F),
    cv::Point3f(-half, -half, 0.0F),
  };
}

double reprojectionError(
  const std::vector<cv::Point3f>& object_points,
  const std::vector<cv::Point2f>& image_points,
  const cv::Vec3d& rvec,
  const cv::Vec3d& tvec,
  const cv::Mat& camera_matrix,
  const cv::Mat& dist_coeffs)
{
  std::vector<cv::Point2f> projected;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, projected);
  if (projected.size() != image_points.size() || projected.empty())
  {
    return std::numeric_limits<double>::infinity();
  }

  double squared_sum = 0.0;
  for (size_t index = 0; index < projected.size(); ++index)
  {
    const cv::Point2f delta = projected[index] - image_points[index];
    squared_sum += static_cast<double>(delta.dot(delta));
  }
  return std::sqrt(squared_sum / static_cast<double>(projected.size()));
}

bool poseFacesCamera(const cv::Vec3d& rvec, const cv::Vec3d& tvec)
{
  if (!finiteVec3(rvec) || !finiteVec3(tvec) || tvec[2] <= 0.0)
  {
    return false;
  }
  cv::Mat rotation;
  cv::Rodrigues(rvec, rotation);
  const cv::Vec3d normal(
    rotation.at<double>(0, 2), rotation.at<double>(1, 2), rotation.at<double>(2, 2));
  return normal.dot(tvec) < 0.0;
}

}  // namespace

bool evaluateMarkerContour(
  const cv::Mat& gray,
  const std::vector<cv::Point2f>& corners,
  const ContourGateOptions& options,
  ContourMetrics& metrics,
  std::string& reason)
{
  metrics = ContourMetrics();
  reason.clear();
  if (gray.empty() || gray.type() != CV_8UC1)
  {
    reason = "image must be non-empty CV_8UC1";
    return false;
  }
  if (corners.size() != 4 || !std::all_of(corners.begin(), corners.end(), finitePoint))
  {
    reason = "contour must contain four finite corners";
    return false;
  }
  if (!cv::isContourConvex(corners))
  {
    reason = "contour is not convex";
    return false;
  }

  const cv::Point2f center =
    (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25F;
  std::vector<double> edge_lengths;
  edge_lengths.reserve(4);
  for (size_t index = 0; index < corners.size(); ++index)
  {
    edge_lengths.push_back(cv::norm(corners[(index + 1) % 4] - corners[index]));
  }
  metrics.min_edge_length_px = *std::min_element(edge_lengths.begin(), edge_lengths.end());
  if (!std::isfinite(metrics.min_edge_length_px) ||
      metrics.min_edge_length_px < options.min_edge_length_px)
  {
    std::ostringstream stream;
    stream << "minimum edge " << metrics.min_edge_length_px << "px is below "
           << options.min_edge_length_px << "px";
    reason = stream.str();
    return false;
  }

  metrics.sample_offset_px = 0.0;
  for (const double edge_length : edge_lengths)
  {
    const double edge_offset = std::max(
      options.edge_sample_offset_min_px,
      std::min(
        options.edge_sample_offset_max_px,
        edge_length * options.edge_sample_offset_ratio));
    metrics.sample_offset_px = std::max(metrics.sample_offset_px, edge_offset);
  }

  const double safe_margin = metrics.sample_offset_px + 1.0;
  for (const cv::Point2f& corner : corners)
  {
    if (corner.x < safe_margin || corner.y < safe_margin ||
        corner.x >= static_cast<double>(gray.cols) - safe_margin ||
        corner.y >= static_cast<double>(gray.rows) - safe_margin)
    {
      reason = "contour is too close to the image boundary";
      return false;
    }
  }

  const int sample_count = std::max(4, options.samples_per_edge);
  std::vector<double> contrasts;
  contrasts.reserve(4);
  for (size_t index = 0; index < corners.size(); ++index)
  {
    const cv::Point2f start = corners[index];
    const cv::Point2f end = corners[(index + 1) % 4];
    const cv::Point2f edge = end - start;
    const double length = edge_lengths[index];
    const double edge_offset_px = std::max(
      options.edge_sample_offset_min_px,
      std::min(
        options.edge_sample_offset_max_px,
        length * options.edge_sample_offset_ratio));
    cv::Point2f inward(
      static_cast<float>(-edge.y / length), static_cast<float>(edge.x / length));
    const cv::Point2f midpoint = (start + end) * 0.5F;
    if ((center - midpoint).dot(inward) < 0.0F)
    {
      inward *= -1.0F;
    }

    double inside_sum = 0.0;
    double outside_sum = 0.0;
    for (int sample = 0; sample < sample_count; ++sample)
    {
      const double fraction = options.edge_sample_margin_ratio +
        (1.0 - 2.0 * options.edge_sample_margin_ratio) *
        (static_cast<double>(sample) + 0.5) / static_cast<double>(sample_count);
      const cv::Point2f on_edge = start + edge * static_cast<float>(fraction);
      double inside = 0.0;
      double outside = 0.0;
      if (!bilinearSample(
            gray, on_edge + inward * static_cast<float>(edge_offset_px), inside) ||
          !bilinearSample(
            gray, on_edge - inward * static_cast<float>(edge_offset_px), outside))
      {
        reason = "edge contrast sample lies outside the image";
        return false;
      }
      inside_sum += inside;
      outside_sum += outside;
    }
    contrasts.push_back(
      (outside_sum - inside_sum) / static_cast<double>(sample_count));
  }

  metrics.min_edge_contrast = *std::min_element(contrasts.begin(), contrasts.end());
  metrics.mean_edge_contrast =
    std::accumulate(contrasts.begin(), contrasts.end(), 0.0) /
    static_cast<double>(contrasts.size());
  if (metrics.min_edge_contrast < options.min_edge_contrast ||
      metrics.mean_edge_contrast < options.min_mean_edge_contrast)
  {
    std::ostringstream stream;
    stream << "edge contrast min=" << metrics.min_edge_contrast
           << " mean=" << metrics.mean_edge_contrast;
    reason = stream.str();
    return false;
  }
  return true;
}

double rotationDistanceRad(const cv::Vec3d& first_rvec, const cv::Vec3d& second_rvec)
{
  cv::Mat first_rotation;
  cv::Mat second_rotation;
  cv::Rodrigues(first_rvec, first_rotation);
  cv::Rodrigues(second_rvec, second_rotation);
  const cv::Mat delta = first_rotation.t() * second_rotation;
  const double cosine = std::max(
    -1.0, std::min(1.0, (cv::trace(delta)[0] - 1.0) * 0.5));
  return std::acos(cosine);
}

bool estimatePoseIppeSquare(
  const std::vector<cv::Point2f>& corners,
  double marker_size_m,
  const cv::Mat& camera_matrix,
  const cv::Mat& dist_coeffs,
  const PoseSolution* previous,
  double ambiguity_gap_px,
  double max_reprojection_error_px,
  PoseSolution& output,
  std::string& reason)
{
  output = PoseSolution();
  reason.clear();
  if (corners.size() != 4 || marker_size_m <= 0.0 || !std::isfinite(marker_size_m))
  {
    reason = "IPPE requires four corners and a positive finite marker size";
    return false;
  }

  const std::vector<cv::Point3f> object_points = squareObjectPoints(marker_size_m);
  std::vector<cv::Mat> rvec_matrices;
  std::vector<cv::Mat> tvec_matrices;
  try
  {
    cv::solvePnPGeneric(
      object_points,
      corners,
      camera_matrix,
      dist_coeffs,
      rvec_matrices,
      tvec_matrices,
      false,
      cv::SOLVEPNP_IPPE_SQUARE);
  }
  catch (const cv::Exception& exception)
  {
    reason = exception.what();
    return false;
  }

  std::vector<PoseSolution> candidates;
  for (size_t index = 0; index < rvec_matrices.size() && index < tvec_matrices.size(); ++index)
  {
    PoseSolution candidate;
    candidate.rvec = matToVec3(rvec_matrices[index]);
    candidate.tvec = matToVec3(tvec_matrices[index]);
    if (!poseFacesCamera(candidate.rvec, candidate.tvec))
    {
      continue;
    }
    candidate.reprojection_error_px = reprojectionError(
      object_points,
      corners,
      candidate.rvec,
      candidate.tvec,
      camera_matrix,
      dist_coeffs);
    if (std::isfinite(candidate.reprojection_error_px))
    {
      candidates.push_back(candidate);
    }
  }
  if (candidates.empty())
  {
    reason = "IPPE produced no finite front-facing positive-depth solution";
    return false;
  }

  std::sort(
    candidates.begin(), candidates.end(), [](const PoseSolution& first, const PoseSolution& second) {
      return first.reprojection_error_px < second.reprojection_error_px;
    });
  size_t selected = 0;
  if (previous != nullptr && candidates.size() > 1 &&
      candidates[1].reprojection_error_px - candidates[0].reprojection_error_px <= ambiguity_gap_px)
  {
    const double first_rotation = rotationDistanceRad(previous->rvec, candidates[0].rvec);
    const double second_rotation = rotationDistanceRad(previous->rvec, candidates[1].rvec);
    if (second_rotation < first_rotation ||
        (std::abs(second_rotation - first_rotation) < 1e-12 &&
         cv::norm(candidates[1].tvec - previous->tvec) <
           cv::norm(candidates[0].tvec - previous->tvec)))
    {
      selected = 1;
    }
  }

  output = candidates[selected];
  output.candidate_count = static_cast<int>(candidates.size());
  output.selected_candidate_rank = static_cast<int>(selected);
  cv::Mat rvec = (cv::Mat_<double>(3, 1) << output.rvec[0], output.rvec[1], output.rvec[2]);
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << output.tvec[0], output.tvec[1], output.tvec[2]);
  try
  {
    cv::solvePnPRefineLM(
      object_points, corners, camera_matrix, dist_coeffs, rvec, tvec);
  }
  catch (const cv::Exception& exception)
  {
    reason = exception.what();
    return false;
  }
  output.rvec = matToVec3(rvec);
  output.tvec = matToVec3(tvec);
  if (!poseFacesCamera(output.rvec, output.tvec))
  {
    reason = "refined IPPE pose is not finite, front-facing, and positive-depth";
    return false;
  }
  output.reprojection_error_px = reprojectionError(
    object_points,
    corners,
    output.rvec,
    output.tvec,
    camera_matrix,
    dist_coeffs);
  if (!std::isfinite(output.reprojection_error_px) ||
      output.reprojection_error_px > max_reprojection_error_px)
  {
    std::ostringstream stream;
    stream << "reprojection RMS " << output.reprojection_error_px << "px exceeds "
           << max_reprojection_error_px << "px";
    reason = stream.str();
    return false;
  }
  return true;
}

}  // namespace front_marker_vision
