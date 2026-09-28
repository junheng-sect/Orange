#pragma once
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <apriltag.h>
namespace ov13855_apriltag_vision {
struct TagDetection {
  int id;
  std::vector<cv::Point2f> corners; // native decoded p[0..3]: BL, BR, TR, TL; never image-coordinate sorting
};
int selectDetection(const std::vector<TagDetection>& detections,
                    const std::string& mode, int fixed_id);
class AprilTagDetector {
 public:
  AprilTagDetector(const std::string& family, double decimate, int threads,
                   bool refine, int max_dimension);
  ~AprilTagDetector();
  AprilTagDetector(const AprilTagDetector&) = delete;
  AprilTagDetector& operator=(const AprilTagDetector&) = delete;
  std::vector<TagDetection> detect(const cv::Mat& gray);
 private:
  apriltag_family_t* family_ = nullptr;
  apriltag_detector_t* detector_ = nullptr;
  int max_dimension_;
};
}
