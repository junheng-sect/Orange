#include <gtest/gtest.h>
#include <ov13855_apriltag_vision/pose_solver.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <limits>
using namespace ov13855_apriltag_vision;
namespace {
std::vector<cv::Point3f> referencePoints(double size) {
  double h = size / 2.0;
  return {cv::Point3f(-h,-h,0), cv::Point3f(h,-h,0),
          cv::Point3f(h,h,0), cv::Point3f(-h,h,0)};
}
cv::Mat camera() {
  return (cv::Mat_<double>(3,3) << 1416.6551,0,964.044,0,1064.7061,533.7901,0,0,1);
}
cv::Mat distortion() {
  return (cv::Mat_<double>(1,5) << .229957,-.59175,.002862,-.004577,.5314);
}
std::vector<cv::Point2f> projected(const cv::Vec3d& r, const cv::Vec3d& t, double size=.012) {
  std::vector<cv::Point2f> pixels;
  cv::projectPoints(referencePoints(size),r,t,camera(),distortion(),pixels);
  return pixels;
}
}
TEST(DirectSolvePnP, MatchesDroneVisionReferenceForRotatedAndNoisyCorners) {
  for (const cv::Vec3d& r : {cv::Vec3d(3.0,.15,.1),cv::Vec3d(2.4,-.4,.6),cv::Vec3d(1.9,1.8,.2)}) {
    auto points=projected(r,cv::Vec3d(.01,-.02,.2));
    points[0]+=cv::Point2f(.2,-.1);points[2]+=cv::Point2f(-.15,.1);
    cv::Mat reference_r,reference_t;
    ASSERT_TRUE(cv::solvePnP(referencePoints(.012),points,camera(),distortion(),reference_r,reference_t));
    PoseSolution output;std::string reason;
    ASSERT_TRUE(solveTagPose(points,.012,camera(),distortion(),output,reason))<<reason;
    for (int k=0;k<3;++k) {
      EXPECT_NEAR(output.rvec[k],reference_r.at<double>(k),1e-10);
      EXPECT_NEAR(output.tvec[k],reference_t.at<double>(k),1e-10);
    }
  }
}
TEST(DirectSolvePnP, RecoversKnownPoseWithoutHistory) {
  const cv::Vec3d r(2.8,.3,.15),t(.01,-.005,.25);
  auto points=projected(r,t);PoseSolution output;std::string reason;
  ASSERT_TRUE(solveTagPose(points,.012,camera(),distortion(),output,reason))<<reason;
  EXPECT_LT(cv::norm(output.tvec-t),1e-5);
  cv::Mat actual,expected;cv::Rodrigues(output.rvec,actual);cv::Rodrigues(r,expected);
  EXPECT_LT(cv::norm(actual-expected),1e-4);
}
TEST(DirectSolvePnP, DoesNotRejectFiniteSolutionForLargeReprojectionError) {
  auto points=projected(cv::Vec3d(2.8,.3,.1),cv::Vec3d(.005,.01,.05));
  points[0]+=cv::Point2f(35,-25);
  points[2]+=cv::Point2f(-10,20);
  PoseSolution output;std::string reason;
  ASSERT_TRUE(solveTagPose(points,.012,camera(),distortion(),output,reason))<<reason;
  std::vector<cv::Point2f> reprojected;
  cv::projectPoints(referencePoints(.012),output.rvec,output.tvec,camera(),distortion(),reprojected);
  double squared=0;
  for(size_t i=0;i<points.size();++i) { auto delta=points[i]-reprojected[i];squared+=delta.dot(delta); }
  EXPECT_GT(std::sqrt(squared/4),1.0);
  EXPECT_GT(output.tvec[2],0.0);
}
TEST(DirectSolvePnP, RejectsUnusableInput) {
  PoseSolution output;std::string reason;
  EXPECT_FALSE(solveTagPose({},.012,camera(),distortion(),output,reason));
  auto points=projected(cv::Vec3d(3,.1,.2),cv::Vec3d(0,0,.2));
  EXPECT_FALSE(solveTagPose(points,0,camera(),distortion(),output,reason));
  EXPECT_FALSE(solveTagPose(points,.012,cv::Mat(),distortion(),output,reason));
  points[0].x=std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(solveTagPose(points,.012,camera(),distortion(),output,reason));
}
int main(int argc,char** argv) {testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
