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

front_marker_vision::PoseSolution trackerPose(
  double yaw_rad,
  double x_m,
  int selected_rank = 0,
  int reference_rank = -1,
  bool ambiguous = false)
{
  front_marker_vision::PoseSolution solution;
  solution.rvec = cv::Vec3d(0.0, 0.0, yaw_rad);
  solution.tvec = cv::Vec3d(x_m, 0.0, 1.0);
  solution.candidate_count = 2;
  solution.selected_candidate_rank = selected_rank;
  solution.reference_candidate_rank = reference_rank;
  solution.ambiguous = ambiguous;
  return solution;
}

front_marker_vision::PoseReferenceTracker bootstrappedTracker()
{
  front_marker_vision::PoseReferenceTracker tracker;
  tracker.observe(trackerPose(0.00, 0.00), 1.0);
  tracker.observe(trackerPose(0.01, 0.01), 1.1);
  tracker.observe(trackerPose(0.02, 0.02), 1.2);
  return tracker;
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
    0.35,
    0.10,
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
    0.35,
    0.10,
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
    0.35,
    0.10,
    0.2,
    1.0,
    output,
    reason)) << reason;
  EXPECT_EQ(output.selected_candidate_rank, 0);
  EXPECT_EQ(output.reference_candidate_rank, -1);
  EXPECT_TRUE(output.ambiguous);
  EXPECT_LE(output.reprojection_error_gap_px, 0.2);

  front_marker_vision::PoseSolution previous;
  previous.rvec = matToVec3(candidate_rvecs[1]);
  previous.tvec = matToVec3(candidate_tvecs[1]);
  ASSERT_TRUE(front_marker_vision::estimatePoseIppeSquare(
    corners,
    marker_size,
    cameraMatrix(),
    distortion,
    &previous,
    0.35,
    0.10,
    0.2,
    1.0,
    output,
    reason)) << reason;
  EXPECT_EQ(output.selected_candidate_rank, 1);
  EXPECT_EQ(output.reference_candidate_rank, 1);
  EXPECT_TRUE(output.ambiguous);
}

TEST(MarkerIppe, IgnoresReferenceWhenAllCandidatesAreOutsideDistanceGates)
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

  front_marker_vision::PoseSolution distant_reference;
  distant_reference.rvec = cv::Vec3d(0.0, 0.0, 0.0);
  distant_reference.tvec = cv::Vec3d(0.02, 0.03, 1.5);
  front_marker_vision::PoseSolution output;
  std::string reason;
  ASSERT_TRUE(front_marker_vision::estimatePoseIppeSquare(
    corners,
    marker_size,
    cameraMatrix(),
    distortion,
    &distant_reference,
    0.35,
    0.10,
    0.2,
    1.0,
    output,
    reason)) << reason;
  EXPECT_TRUE(output.ambiguous);
  EXPECT_EQ(output.reference_candidate_rank, -1);
  EXPECT_EQ(output.selected_candidate_rank, 0);
}

TEST(MarkerIppe, RotationDistanceIsZeroForEquivalentRotations)
{
  EXPECT_NEAR(
    front_marker_vision::rotationDistanceRad(
      cv::Vec3d(M_PI, 0.0, 0.0), cv::Vec3d(M_PI, 0.0, 0.0)),
    0.0,
    1e-12);
}

TEST(PoseReferenceTracker, CommitsOnlyAfterThreeConsistentBootstrapFrames)
{
  front_marker_vision::PoseReferenceTracker tracker;

  EXPECT_FALSE(tracker.observe(trackerPose(0.00, 0.00), 1.0).reference_committed);
  EXPECT_EQ(tracker.state(), front_marker_vision::PoseReferenceState::BOOTSTRAP);
  EXPECT_EQ(tracker.streak(), 1);
  EXPECT_FALSE(tracker.observe(trackerPose(0.01, 0.01), 1.1).reference_committed);
  EXPECT_FALSE(tracker.hasReference());

  const front_marker_vision::PoseReferenceUpdate update =
    tracker.observe(trackerPose(0.02, 0.02), 1.2);
  EXPECT_TRUE(update.reference_committed);
  EXPECT_FALSE(update.branch_flip_confirmed);
  ASSERT_TRUE(tracker.hasReference());
  EXPECT_EQ(tracker.state(), front_marker_vision::PoseReferenceState::TRACKING);
  EXPECT_NEAR(tracker.reference().tvec[0], 0.02, 1e-12);
  EXPECT_EQ(tracker.branchFlipCount(), 0U);
}

TEST(PoseReferenceTracker, AmbiguousBootstrapIsSuppressedThroughCommitFrame)
{
  front_marker_vision::PoseReferenceTracker tracker;

  const front_marker_vision::PoseSolution first =
    trackerPose(0.00, 0.00, 0, -1, true);
  EXPECT_FALSE(tracker.observe(first, 1.0).reference_committed);
  EXPECT_FALSE(tracker.posePublishAllowed(first.ambiguous, false));

  const front_marker_vision::PoseSolution second =
    trackerPose(0.01, 0.01, 0, -1, true);
  EXPECT_FALSE(tracker.observe(second, 1.1).reference_committed);
  EXPECT_FALSE(tracker.posePublishAllowed(second.ambiguous, false));

  const front_marker_vision::PoseSolution commit =
    trackerPose(0.02, 0.02, 0, -1, true);
  EXPECT_TRUE(tracker.observe(commit, 1.2).reference_committed);
  ASSERT_TRUE(tracker.hasReference());
  EXPECT_EQ(tracker.state(), front_marker_vision::PoseReferenceState::TRACKING);
  EXPECT_FALSE(tracker.posePublishAllowed(commit.ambiguous, false));

  const front_marker_vision::PoseSolution tracked =
    trackerPose(0.03, 0.03, 0, 0, true);
  EXPECT_TRUE(tracker.observe(tracked, 1.3).reference_committed);
  EXPECT_TRUE(tracker.posePublishAllowed(tracked.ambiguous, true));
}

TEST(PoseReferenceTracker, NonAmbiguousBootstrapMayPublishWithoutReference)
{
  front_marker_vision::PoseReferenceTracker tracker;
  const front_marker_vision::PoseSolution decisive = trackerPose(0.00, 0.00);

  EXPECT_FALSE(tracker.observe(decisive, 1.0).reference_committed);
  EXPECT_EQ(tracker.state(), front_marker_vision::PoseReferenceState::BOOTSTRAP);
  EXPECT_TRUE(tracker.posePublishAllowed(decisive.ambiguous, false));
}

TEST(PoseReferenceTracker, BootstrapDiscontinuityRestartsConfirmation)
{
  front_marker_vision::PoseReferenceTracker tracker;

  tracker.observe(trackerPose(0.00, 0.00), 1.0);
  tracker.observe(trackerPose(0.80, 0.01), 1.1);
  EXPECT_EQ(tracker.streak(), 1);
  EXPECT_FALSE(tracker.observe(trackerPose(0.81, 0.02), 1.2).reference_committed);
  EXPECT_TRUE(tracker.observe(trackerPose(0.82, 0.03), 1.3).reference_committed);
}

TEST(PoseReferenceTracker, OneFrameCompatibilityModeCommitsImmediately)
{
  front_marker_vision::PoseReferenceOptions options;
  options.confirmation_frames = 1;
  front_marker_vision::PoseReferenceTracker tracker(options);

  const front_marker_vision::PoseSolution bootstrap =
    trackerPose(0.00, 0.00, 0, -1, true);
  EXPECT_TRUE(tracker.observe(bootstrap, 1.0).reference_committed);
  EXPECT_TRUE(tracker.hasReference());
  EXPECT_FALSE(tracker.posePublishAllowed(bootstrap.ambiguous, false));

  const front_marker_vision::PoseSolution tracked =
    trackerPose(0.01, 0.01, 0, 0, true);
  EXPECT_TRUE(tracker.observe(tracked, 1.1).reference_committed);
  EXPECT_TRUE(tracker.posePublishAllowed(tracked.ambiguous, true));
}

TEST(PoseReferenceTracker, OneFrameAlternativeDoesNotReplaceCommittedReference)
{
  front_marker_vision::PoseReferenceTracker tracker = bootstrappedTracker();

  EXPECT_FALSE(
    tracker.observe(trackerPose(0.80, 0.03, 0, 1), 1.3).reference_committed);
  EXPECT_TRUE(tracker.switchPending());
  EXPECT_FALSE(tracker.posePublishAllowed(false, true));
  EXPECT_NEAR(tracker.reference().rvec[2], 0.02, 1e-12);

  EXPECT_FALSE(
    tracker.observe(trackerPose(0.03, 0.03, 1, 1), 1.4).reference_committed);
  EXPECT_FALSE(tracker.switchPending());
  EXPECT_TRUE(tracker.posePublishAllowed(false, true));
  EXPECT_NEAR(tracker.reference().rvec[2], 0.02, 1e-12);
  EXPECT_EQ(tracker.branchFlipCount(), 0U);
}

TEST(PoseReferenceTracker, ThreeAlternativeFramesConfirmExactlyOneBranchFlip)
{
  front_marker_vision::PoseReferenceTracker tracker = bootstrappedTracker();

  EXPECT_FALSE(
    tracker.observe(trackerPose(0.80, 0.03, 0, 1), 1.3).reference_committed);
  EXPECT_FALSE(
    tracker.observe(trackerPose(0.81, 0.04, 0, 1), 1.4).reference_committed);
  const front_marker_vision::PoseReferenceUpdate flipped =
    tracker.observe(trackerPose(0.82, 0.05, 0, 1), 1.5);
  EXPECT_TRUE(flipped.reference_committed);
  EXPECT_TRUE(flipped.branch_flip_confirmed);
  EXPECT_EQ(tracker.branchFlipCount(), 1U);
  EXPECT_FALSE(tracker.switchPending());
  EXPECT_TRUE(tracker.posePublishAllowed(false, true));

  const front_marker_vision::PoseReferenceUpdate continued =
    tracker.observe(trackerPose(0.83, 0.06, 1, 1), 1.6);
  EXPECT_TRUE(continued.reference_committed);
  EXPECT_FALSE(continued.branch_flip_confirmed);
  EXPECT_EQ(tracker.branchFlipCount(), 1U);
}

TEST(PoseReferenceTracker, CandidateRankReorderingDoesNotCountAsFlip)
{
  front_marker_vision::PoseReferenceTracker tracker = bootstrappedTracker();

  const front_marker_vision::PoseReferenceUpdate update =
    tracker.observe(trackerPose(0.03, 0.03, 1, 1), 1.3);
  EXPECT_TRUE(update.reference_committed);
  EXPECT_FALSE(update.branch_flip_confirmed);
  EXPECT_EQ(tracker.branchFlipCount(), 0U);
}

TEST(PoseReferenceTracker, SameRankLargeJumpDoesNotMatchReference)
{
  front_marker_vision::PoseReferenceTracker tracker = bootstrappedTracker();

  EXPECT_FALSE(
    tracker.observe(trackerPose(0.80, 0.03, 0, 0), 1.3).reference_committed);
  EXPECT_TRUE(tracker.switchPending());
  EXPECT_FALSE(tracker.posePublishAllowed(false, true));
  EXPECT_NEAR(tracker.reference().rvec[2], 0.02, 1e-12);
}

TEST(PoseReferenceTracker, SingleCandidateFallsBackToAbsoluteGeometry)
{
  front_marker_vision::PoseReferenceTracker tracker = bootstrappedTracker();
  front_marker_vision::PoseSolution single_candidate = trackerPose(0.80, 0.03, 0, 0);
  single_candidate.candidate_count = 1;

  EXPECT_FALSE(tracker.observe(single_candidate, 1.3).reference_committed);
  EXPECT_TRUE(tracker.switchPending());
  EXPECT_FALSE(tracker.posePublishAllowed(false, true));
}

TEST(PoseReferenceTracker, RejectedFrameBreaksConfirmationStreak)
{
  front_marker_vision::PoseReferenceTracker tracker = bootstrappedTracker();
  const double committed_yaw = tracker.reference().rvec[2];

  tracker.breakSequence();
  EXPECT_FALSE(
    tracker.observe(trackerPose(0.03, 0.03, 0, 0), 1.3).reference_committed);
  tracker.breakSequence();
  EXPECT_FALSE(
    tracker.observe(trackerPose(0.04, 0.04, 0, 0), 1.4).reference_committed);
  EXPECT_FALSE(
    tracker.observe(trackerPose(0.05, 0.05, 0, 0), 1.5).reference_committed);
  EXPECT_NEAR(tracker.reference().rvec[2], committed_yaw, 1e-12);
  EXPECT_TRUE(
    tracker.observe(trackerPose(0.06, 0.06, 0, 0), 1.6).reference_committed);
}

TEST(PoseReferenceTracker, TimeoutRollbackAndZeroStampClearReference)
{
  front_marker_vision::PoseReferenceTracker tracker = bootstrappedTracker();
  ASSERT_TRUE(tracker.hasReference());

  EXPECT_EQ(tracker.referenceForFrame(1.8), nullptr);
  EXPECT_FALSE(tracker.hasReference());

  tracker.observe(trackerPose(0.00, 0.00), 2.0);
  tracker.observe(trackerPose(0.01, 0.01), 2.1);
  tracker.observe(trackerPose(0.02, 0.02), 2.2);
  ASSERT_TRUE(tracker.hasReference());
  EXPECT_EQ(tracker.referenceForFrame(2.1), nullptr);

  tracker.observe(trackerPose(0.00, 0.00), 3.0);
  tracker.observe(trackerPose(0.01, 0.01), 3.1);
  tracker.observe(trackerPose(0.02, 0.02), 3.2);
  ASSERT_TRUE(tracker.hasReference());
  EXPECT_EQ(tracker.referenceForFrame(0.0), nullptr);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
