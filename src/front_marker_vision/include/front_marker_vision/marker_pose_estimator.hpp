#ifndef FRONT_MARKER_VISION_MARKER_POSE_ESTIMATOR_HPP
#define FRONT_MARKER_VISION_MARKER_POSE_ESTIMATOR_HPP

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace front_marker_vision
{

struct ContourGateOptions
{
  double min_edge_length_px = 12.0;
  double edge_sample_offset_ratio = 0.025;
  double edge_sample_offset_min_px = 1.0;
  double edge_sample_offset_max_px = 4.0;
  double edge_sample_margin_ratio = 0.15;
  double min_edge_contrast = 25.0;
  double min_mean_edge_contrast = 40.0;
  int samples_per_edge = 32;
};

struct ContourMetrics
{
  double min_edge_length_px = 0.0;
  double sample_offset_px = 0.0;
  double min_edge_contrast = 0.0;
  double mean_edge_contrast = 0.0;
};

struct SubpixelOptions
{
  int window_size = 5;
  int max_iterations = 50;
  double epsilon = 0.001;
  double max_shift_px = 2.0;
};

struct PoseSolution
{
  cv::Vec3d rvec = cv::Vec3d(0.0, 0.0, 0.0);
  cv::Vec3d tvec = cv::Vec3d(0.0, 0.0, 0.0);
  double reprojection_error_px = 0.0;
  int candidate_count = 0;
  int selected_candidate_rank = 0;
};

bool evaluateMarkerContour(
  const cv::Mat& gray,
  const std::vector<cv::Point2f>& corners,
  const ContourGateOptions& options,
  ContourMetrics& metrics,
  std::string& reason);

bool refineMarkerCorners(
  const cv::Mat& gray,
  const std::vector<cv::Point2f>& input,
  const SubpixelOptions& options,
  std::vector<cv::Point2f>& refined,
  double& max_shift_px,
  std::string& reason);

bool estimatePoseIppeSquare(
  const std::vector<cv::Point2f>& corners,
  double marker_size_m,
  const cv::Mat& camera_matrix,
  const cv::Mat& dist_coeffs,
  const PoseSolution* previous,
  double ambiguity_gap_px,
  double max_reprojection_error_px,
  PoseSolution& output,
  std::string& reason);

double rotationDistanceRad(const cv::Vec3d& first_rvec, const cv::Vec3d& second_rvec);

}  // namespace front_marker_vision

#endif  // FRONT_MARKER_VISION_MARKER_POSE_ESTIMATOR_HPP
