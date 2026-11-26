#include <ov13855_apriltag_vision/apriltag_detector.hpp>
#include <tag36h11.h>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
namespace ov13855_apriltag_vision {
int selectDetection(const std::vector<TagDetection>& detections,
                    const std::string& mode, int fixed_id) {
  if (mode != "largest" && mode != "fixed") throw std::invalid_argument("invalid selection mode");
  int selected = -1;
  double area = -1;
  for (size_t i = 0; i < detections.size(); ++i) {
    const auto& d = detections[i];
    if (mode == "fixed" && d.id != fixed_id) continue;
    const double candidate = std::abs(cv::contourArea(d.corners));
    if (candidate > area || (candidate == area && selected >= 0 && d.id < detections[selected].id)) {
      selected = static_cast<int>(i); area = candidate;
    }
  }
  return selected;
}
AprilTagDetector::AprilTagDetector(const std::string& family, double decimate,
    int threads, bool refine, int max_dimension) : max_dimension_(max_dimension) {
  if (family != "tag36h11") throw std::invalid_argument("supported tag_family: tag36h11");
  if (!std::isfinite(decimate) || decimate < 1 || threads < 1 || threads > 64 || max_dimension < 16)
    throw std::invalid_argument("invalid AprilTag detector parameters");
  family_ = tag36h11_create();
  detector_ = apriltag_detector_create();
  if (!family_ || !detector_) {
    if (detector_) apriltag_detector_destroy(detector_);
    if (family_) tag36h11_destroy(family_);
    throw std::runtime_error("AprilTag allocation failed");
  }
  apriltag_detector_add_family(detector_, family_);
  detector_->quad_decimate = decimate;
  detector_->nthreads = threads;
  detector_->refine_edges = refine;
}
AprilTagDetector::~AprilTagDetector() {
  apriltag_detector_destroy(detector_);
  tag36h11_destroy(family_);
}
std::vector<TagDetection> AprilTagDetector::detect(const cv::Mat& gray) {
  if (gray.empty() || gray.type() != CV_8UC1) throw std::invalid_argument("expected mono8 image");
  cv::Mat input = gray;
  const int longest = std::max(gray.cols, gray.rows);
  if (longest > max_dimension_) {
    const double scale = static_cast<double>(max_dimension_) / longest;
    cv::resize(gray, input, cv::Size(std::max(1, cvRound(gray.cols * scale)),
        std::max(1, cvRound(gray.rows * scale))), 0, 0, cv::INTER_NEAREST);
  }
  image_u8_t image = {input.cols, input.rows, static_cast<int>(input.step), input.data};
  std::unique_ptr<zarray_t, decltype(&apriltag_detections_destroy)> detections(
      apriltag_detector_detect(detector_, &image), apriltag_detections_destroy);
  if (!detections) throw std::runtime_error("AprilTag detection failed");
  std::vector<TagDetection> result;
  const double sx = static_cast<double>(gray.cols) / input.cols;
  const double sy = static_cast<double>(gray.rows) / input.rows;
  for (int i = 0; i < zarray_size(detections.get()); ++i) {
    apriltag_detection_t* d;
    zarray_get(detections.get(), i, &d);
    TagDetection output; output.id = d->id;
    // Preserve native AprilTag p[0..3], matching Drone-Vision solvePnP correspondences.
    for (int k = 0; k < 4; ++k)
      output.corners.emplace_back(d->p[k][0] * sx, d->p[k][1] * sy);
    result.push_back(output);
  }
  return result;
}
}
