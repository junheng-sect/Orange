#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <front_marker_vision/marker_pose_estimator.hpp>

namespace
{

cv::Mat cameraMatrix()
{
  return (cv::Mat_<double>(3, 3) <<
    330.0, 0.0, 320.0,
    0.0, 330.0, 384.0,
    0.0, 0.0, 1.0);
}

std::vector<cv::Point3f> objectPoints(double size)
{
  const float half = static_cast<float>(size * 0.5);
  return {
    {-half, half, 0.0F},
    {half, half, 0.0F},
    {half, -half, 0.0F},
    {-half, -half, 0.0F},
  };
}

cv::Vec3d matToVec3(const cv::Mat& input)
{
  cv::Mat converted;
  input.reshape(1, 3).convertTo(converted, CV_64F);
  return cv::Vec3d(
    converted.at<double>(0, 0), converted.at<double>(1, 0), converted.at<double>(2, 0));
}

}  // namespace

TEST(MarkerContourGate, AcceptsBlackMarkerWithLightExteriorAndRejectsInsetContour)
{
  cv::Mat image(220, 220, CV_8UC1, cv::Scalar(220));
  cv::rectangle(image, cv::Point(40, 40), cv::Point(180, 180), cv::Scalar(10), cv::FILLED);

  front_marker_vision::ContourGateOptions options;
  front_marker_vision::ContourMetrics metrics;
  std::string reason;
  const std::vector<cv::Point2f> correct = {
    {40.0F, 40.0F}, {180.0F, 40.0F}, {180.0F, 180.0F}, {40.0F, 180.0F}};
  EXPECT_TRUE(front_marker_vision::evaluateMarkerContour(
    image, correct, options, metrics, reason)) << reason;
  EXPECT_GT(metrics.min_edge_contrast, 100.0);

  const std::vector<cv::Point2f> inset = {
    {46.0F, 46.0F}, {174.0F, 46.0F}, {174.0F, 174.0F}, {46.0F, 174.0F}};
  EXPECT_FALSE(front_marker_vision::evaluateMarkerContour(
    image, inset, options, metrics, reason));
  EXPECT_LT(metrics.mean_edge_contrast, options.min_mean_edge_contrast);
}

TEST(MarkerContourGate, RejectsShortNonConvexAndBorderContours)
{
  cv::Mat image(100, 100, CV_8UC1, cv::Scalar(128));
  front_marker_vision::ContourGateOptions options;
  front_marker_vision::ContourMetrics metrics;
  std::string reason;

  EXPECT_FALSE(front_marker_vision::evaluateMarkerContour(
    image,
    {{20.0F, 20.0F}, {25.0F, 20.0F}, {25.0F, 25.0F}, {20.0F, 25.0F}},
    options, metrics, reason));
  EXPECT_FALSE(front_marker_vision::evaluateMarkerContour(
    image,
    {{20.0F, 20.0F}, {80.0F, 20.0F}, {30.0F, 30.0F}, {20.0F, 80.0F}},
    options, metrics, reason));
  EXPECT_FALSE(front_marker_vision::evaluateMarkerContour(
    image,
    {{0.0F, 0.0F}, {80.0F, 0.0F}, {80.0F, 80.0F}, {0.0F, 80.0F}},
    options, metrics, reason));
}

TEST(MarkerIppe, RecoversKnownFrontFacingPose)
{
  constexpr double marker_size = 0.070;
  const cv::Vec3d expected_rvec(M_PI, 0.0, 0.0);
  const cv::Vec3d expected_tvec(0.02, 0.03, 0.70);
  std::vector<cv::Point2f> corners;
  cv::projectPoints(
    objectPoints(marker_size), expected_rvec, expected_tvec,
    cameraMatrix(), cv::Mat::zeros(1, 5, CV_64F), corners);

  front_marker_vision::PoseSolution output;
  std::string reason;
  ASSERT_TRUE(front_marker_vision::estimatePoseIppeSquare(
    corners,
    marker_size,
    cameraMatrix(),
    cv::Mat::zeros(1, 5, CV_64F),
    nullptr,
    0.2,
    1.0,
    output,
    reason)) << reason;
  EXPECT_NEAR(output.tvec[0], expected_tvec[0], 1e-6);
  EXPECT_NEAR(output.tvec[1], expected_tvec[1], 1e-6);
  EXPECT_NEAR(output.tvec[2], expected_tvec[2], 1e-6);
  EXPECT_LT(output.reprojection_error_px, 1e-5);
  EXPECT_GE(output.candidate_count, 1);
}

TEST(MarkerIppe, RejectsReprojectionErrorAboveThreshold)
{
  constexpr double marker_size = 0.070;
  std::vector<cv::Point2f> corners;
  cv::projectPoints(
    objectPoints(marker_size), cv::Vec3d(M_PI, 0.1, -0.05), cv::Vec3d(0.02, 0.01, 0.7),
    cameraMatrix(), cv::Mat::zeros(1, 5, CV_64F), corners);
  corners[0] += cv::Point2f(0.4F, -0.3F);
  corners[2] += cv::Point2f(-0.2F, 0.5F);

  front_marker_vision::PoseSolution output;
  std::string reason;
  EXPECT_FALSE(front_marker_vision::estimatePoseIppeSquare(
    corners,
    marker_size,
    cameraMatrix(),
    cv::Mat::zeros(1, 5, CV_64F),
    nullptr,
    0.2,
    1e-4,
    output,
    reason));
  EXPECT_EQ(reason.rfind("reprojection RMS", 0), 0U);
}

TEST(MarkerIppe, UsesRecentPoseWhenTwoSolutionsAreWithinAmbiguityGap)
{
  constexpr double marker_size = 0.070;
  const cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
  std::vector<cv::Point2f> corners;
  cv::projectPoints(
    objectPoints(marker_size),
    cv::Vec3d(M_PI, 0.02, 0.01),
    cv::Vec3d(0.02, 0.03, 1.5),
    cameraMatrix(),
    distortion,
    corners);

  std::vector<cv::Mat> candidate_rvecs;
  std::vector<cv::Mat> candidate_tvecs;
  ASSERT_GE(
    cv::solvePnPGeneric(
      objectPoints(marker_size),
      corners,
      cameraMatrix(),
      distortion,
      candidate_rvecs,
      candidate_tvecs,
      false,
      cv::SOLVEPNP_IPPE_SQUARE),
    2);
  ASSERT_GE(candidate_rvecs.size(), 2U);

  front_marker_vision::PoseSolution output;
  std::string reason;
  ASSERT_TRUE(front_marker_vision::estimatePoseIppeSquare(
    corners,
    marker_size,
    cameraMatrix(),
    distortion,
    nullptr,
    0.2,
    1.0,
    output,
    reason)) << reason;
  EXPECT_EQ(output.selected_candidate_rank, 0);

  front_marker_vision::PoseSolution previous;
  previous.rvec = matToVec3(candidate_rvecs[1]);
  previous.tvec = matToVec3(candidate_tvecs[1]);
  ASSERT_TRUE(front_marker_vision::estimatePoseIppeSquare(
    corners,
    marker_size,
    cameraMatrix(),
    distortion,
    &previous,
    0.2,
    1.0,
    output,
    reason)) << reason;
  EXPECT_EQ(output.selected_candidate_rank, 1);
}

TEST(MarkerIppe, RotationDistanceIsZeroForEquivalentRotations)
{
  EXPECT_NEAR(
    front_marker_vision::rotationDistanceRad(
      cv::Vec3d(M_PI, 0.0, 0.0), cv::Vec3d(M_PI, 0.0, 0.0)),
    0.0,
    1e-12);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
