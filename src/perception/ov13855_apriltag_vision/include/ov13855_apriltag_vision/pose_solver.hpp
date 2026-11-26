#pragma once
#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace ov13855_apriltag_vision {
struct PoseSolution {
  cv::Vec3d rvec{0.0, 0.0, 0.0};
  cv::Vec3d tvec{0.0, 0.0, 0.0};
};
// Drone-Vision correspondence: native AprilTag p[0..3] to
// (-h,-h,0), (h,-h,0), (h,h,0), (-h,h,0). No pose history or extra filtering.
bool solveTagPose(const std::vector<cv::Point2f>& corners, double tag_size_m,
                  const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
                  PoseSolution& pose, std::string& reason);
}
