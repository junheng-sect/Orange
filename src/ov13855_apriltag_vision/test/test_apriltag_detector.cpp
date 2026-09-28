#include <gtest/gtest.h>
#include <ov13855_apriltag_vision/apriltag_detector.hpp>
#include <ov13855_apriltag_vision/pose_solver.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <tag36h11.h>
#include <common/image_u8.h>
using namespace ov13855_apriltag_vision;
cv::Mat render(int id, int rotation = 0, int size = 800) {
  auto* family = tag36h11_create();
  auto* raw = apriltag_to_image(family, id);
  cv::Mat small(raw->height, raw->width, CV_8UC1, raw->buf, raw->stride);
  cv::Mat tag; cv::resize(small, tag, cv::Size(400,400), 0,0,cv::INTER_NEAREST);
  image_u8_destroy(raw); tag36h11_destroy(family);
  for (int i=0; i<rotation; ++i) cv::rotate(tag,tag,cv::ROTATE_90_CLOCKWISE);
  cv::Mat scene(size,size,CV_8UC1,cv::Scalar(255));
  tag.copyTo(scene(cv::Rect((size-400)/2,(size-400)/2,400,400)));
  return scene;
}
TEST(AprilTag, DecodedCornersTrackRotationAndCameraAxes) {
  AprilTagDetector detector("tag36h11",1.5,2,true,1280);
  cv::Mat K = (cv::Mat_<double>(3,3)<<1000,0,400,0,1000,400,0,0,1);
  cv::Mat D = cv::Mat::zeros(1,5,CV_64F);
  auto base = detector.detect(render(7));
  ASSERT_EQ(base.size(),1u); EXPECT_EQ(base[0].id,7);
  EXPECT_LT(base[0].corners[0].x,base[0].corners[1].x);
  EXPECT_GT(base[0].corners[0].y,base[0].corners[3].y);
  for(int turns=0;turns<4;++turns) {
    auto detections=detector.detect(render(7,turns)); ASSERT_EQ(detections.size(),1u);
    for(int k=0;k<4;++k) {
      cv::Point2f expected=base[0].corners[k];
      for(int t=0;t<turns;++t) expected=cv::Point2f(800-expected.y,expected.x);
      EXPECT_LT(cv::norm(expected-detections[0].corners[k]),2.0);
    }
    PoseSolution solution; std::string reason;
    ASSERT_TRUE(solveTagPose(detections[0].corners,0.012,K,D,solution,reason))<<reason;
    EXPECT_NEAR(solution.tvec[0],0,0.0001); EXPECT_NEAR(solution.tvec[1],0,0.0001);
    // apriltag_to_image includes white border: black tag is 8/10 of 400 pixels.
    EXPECT_NEAR(solution.tvec[2],1000*0.012/320,0.0005);
    cv::Mat R; cv::Rodrigues(solution.rvec,R);
    EXPECT_LT(R.at<double>(2,2),-0.98); // tag Z faces the camera, preserving old marker semantics
    EXPECT_NEAR(R.at<double>(0,0),std::cos(turns*CV_PI/2),0.03);
    EXPECT_NEAR(R.at<double>(1,0),std::sin(turns*CV_PI/2),0.03);
  }
}
TEST(AprilTag, ScalingRestoresFullImageCoordinates) {
  AprilTagDetector full("tag36h11",1.5,2,true,4096), scaled("tag36h11",1.5,2,true,1280);
  auto image=render(42,0,1600); auto a=full.detect(image),b=scaled.detect(image);
  ASSERT_EQ(a.size(),1u); ASSERT_EQ(b.size(),1u);
  for(int k=0;k<4;++k) EXPECT_LT(cv::norm(a[0].corners[k]-b[0].corners[k]),1.5);
}
TEST(AprilTag, LargestAndFixedSelection) {
  std::vector<TagDetection> detections={
    {9,{{0,0},{10,0},{10,10},{0,10}}}, {7,{{0,0},{30,0},{30,30},{0,30}}}};
  EXPECT_EQ(selectDetection(detections,"largest",0),1);
  EXPECT_EQ(selectDetection(detections,"fixed",9),0);
  EXPECT_EQ(selectDetection(detections,"fixed",88),-1);
  EXPECT_EQ(selectDetection({},"largest",0),-1);
  EXPECT_THROW(AprilTagDetector("typo",1.5,2,true,1280),std::invalid_argument);
}
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
