#include <ov13855_apriltag_vision/pose_solver.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>

namespace ov13855_apriltag_vision {
bool solveTagPose(const std::vector<cv::Point2f>& corners, double tag_size_m,
                  const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
                  PoseSolution& pose, std::string& reason) {
  pose = PoseSolution();
  reason.clear();
  if (corners.size() != 4 || !std::isfinite(tag_size_m) || tag_size_m <= 0.0) {
    reason = "four corners and a positive finite tag size are required";
    return false;
  }
  for (const auto& p : corners) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      reason = "non-finite image corner";
      return false;
    }
  }
  // Same object points and default solvePnP call as Drone-Vision/vision_detect.cpp.
  const double h = tag_size_m / 2.0;
  const std::vector<cv::Point3f> object_points = {
    cv::Point3f(-h,-h,0), cv::Point3f(h,-h,0),
    cv::Point3f(h,h,0), cv::Point3f(-h,h,0)
  };
  cv::Mat rvec, tvec;
  try {
    const bool ok = cv::solvePnP(object_points, corners, camera_matrix, dist_coeffs, rvec, tvec);
    if (!ok || rvec.empty() || tvec.empty() || rvec.total() < 3 || tvec.total() < 3) {
      reason = "solvePnP failed";
      return false;
    }
    for (int i = 0; i < 3; ++i) {
      const double r = rvec.at<double>(i), t = tvec.at<double>(i);
      if (!std::isfinite(r) || !std::isfinite(t)) {
        reason = "solvePnP produced non-finite rotation or translation";
        return false;
      }
      pose.rvec[i] = r;
      pose.tvec[i] = t;
    }
    if (pose.tvec[2] <= 0.0) {
      reason = "solvePnP produced non-positive camera depth";
      return false;
    }
  } catch (const cv::Exception& error) {
    reason = std::string("solvePnP failed: ") + error.what();
    return false;
  }
  return true;
}
}
