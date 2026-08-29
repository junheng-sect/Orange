#ifndef FRONT_MARKER_VISION_MARKER_POSE_ESTIMATOR_HPP
#define FRONT_MARKER_VISION_MARKER_POSE_ESTIMATOR_HPP

#include <cstdint>
#include <limits>
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

struct PoseSolution
{
  cv::Vec3d rvec = cv::Vec3d(0.0, 0.0, 0.0);
  cv::Vec3d tvec = cv::Vec3d(0.0, 0.0, 0.0);
  double reprojection_error_px = 0.0;
  double reprojection_error_gap_px = std::numeric_limits<double>::infinity();
  int candidate_count = 0;
  int selected_candidate_rank = 0;
  int reference_candidate_rank = -1;
  bool ambiguous = false;
};

struct PoseReferenceOptions
{
  int confirmation_frames = 3;
  double max_rotation_step_rad = 0.35;
  double max_translation_step_m = 0.10;
  double timeout_sec = 0.5;
};

enum class PoseReferenceState
{
  UNINITIALIZED,
  BOOTSTRAP,
  TRACKING,
  SWITCH_PENDING,
};

struct PoseReferenceUpdate
{
  bool reference_committed = false;
  bool branch_flip_confirmed = false;
};

class PoseReferenceTracker
{
public:
  explicit PoseReferenceTracker(const PoseReferenceOptions& options = PoseReferenceOptions());

  const PoseSolution* referenceForFrame(double stamp_sec);
  PoseReferenceUpdate observe(const PoseSolution& solution, double stamp_sec);
  void breakSequence();

  bool hasReference() const;
  const PoseSolution& reference() const;
  PoseReferenceState state() const;
  int streak() const;
  int confirmationFrames() const;
  bool switchPending() const;
  bool posePublishAllowed(
    bool ambiguous, bool had_committed_reference_for_frame) const;
  uint64_t branchFlipCount() const;

private:
  void clearReference();
  bool geometricallyConsistent(
    const PoseSolution& first, const PoseSolution& second) const;

  PoseReferenceOptions options_;
  bool has_reference_ = false;
  PoseSolution reference_;
  double reference_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
  double last_frame_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
  bool has_pending_pose_ = false;
  PoseSolution pending_pose_;
  bool pending_matches_reference_ = true;
  int streak_ = 0;
  uint64_t branch_flip_count_ = 0;
};

bool evaluateMarkerContour(
  const cv::Mat& gray,
  const std::vector<cv::Point2f>& corners,
  const ContourGateOptions& options,
  ContourMetrics& metrics,
  std::string& reason);

bool estimatePoseIppeSquare(
  const std::vector<cv::Point2f>& corners,
  double marker_size_m,
  const cv::Mat& camera_matrix,
  const cv::Mat& dist_coeffs,
  const PoseSolution* previous,
  double max_reference_rotation_distance_rad,
  double max_reference_translation_distance_m,
  double ambiguity_gap_px,
  double max_reprojection_error_px,
  PoseSolution& output,
  std::string& reason);

double rotationDistanceRad(const cv::Vec3d& first_rvec, const cv::Vec3d& second_rvec);

}  // namespace front_marker_vision

#endif  // FRONT_MARKER_VISION_MARKER_POSE_ESTIMATOR_HPP
