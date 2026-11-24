#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include <front_marker_vision/roi_tracker.hpp>

namespace
{

const cv::Size kImageSize(640, 480);

std::vector<cv::Point2f> square(float x, float y, float side)
{
  return {
    {x, y}, {x + side, y}, {x + side, y + side}, {x, y + side},
  };
}

}  // namespace

TEST(RoiTracker, StartsFullThenUsesThreeTimesMarkerSideWithMinimumSize)
{
  front_marker_vision::RoiTracker tracker;
  EXPECT_EQ(tracker.searchRect(kImageSize), cv::Rect(0, 0, 640, 480));

  ASSERT_TRUE(tracker.updateTarget(square(100.0F, 80.0F, 40.0F), kImageSize));
  EXPECT_TRUE(tracker.active());
  EXPECT_EQ(tracker.searchRect(kImageSize), cv::Rect(56, 36, 128, 128));

  ASSERT_TRUE(tracker.updateTarget(square(200.0F, 100.0F, 60.0F), kImageSize));
  EXPECT_EQ(tracker.searchRect(kImageSize), cv::Rect(140, 40, 180, 180));
}

TEST(RoiTracker, ShiftsRoiInsideBordersAndCapsEachImageDimension)
{
  front_marker_vision::RoiTracker tracker;
  ASSERT_TRUE(tracker.updateTarget(square(1.0F, 2.0F, 50.0F), kImageSize));
  EXPECT_EQ(tracker.searchRect(kImageSize), cv::Rect(0, 0, 150, 150));

  ASSERT_TRUE(tracker.updateTarget(square(589.0F, 429.0F, 50.0F), kImageSize));
  EXPECT_EQ(tracker.searchRect(kImageSize), cv::Rect(490, 330, 150, 150));

  ASSERT_TRUE(tracker.updateTarget(square(200.0F, 120.0F, 220.0F), kImageSize));
  EXPECT_EQ(tracker.searchRect(kImageSize), cv::Rect(0, 0, 640, 480));
}

TEST(RoiTracker, ExpandsTwiceAndReturnsFullAfterThirdMiss)
{
  front_marker_vision::RoiTracker tracker;
  ASSERT_TRUE(tracker.updateTarget(square(100.0F, 180.0F, 40.0F), kImageSize));
  EXPECT_EQ(tracker.searchRect(kImageSize).size(), cv::Size(128, 128));

  tracker.updateMiss(kImageSize);
  EXPECT_EQ(tracker.missCount(), 1);
  EXPECT_EQ(tracker.searchRect(kImageSize).size(), cv::Size(192, 192));

  tracker.updateMiss(kImageSize);
  EXPECT_EQ(tracker.missCount(), 2);
  EXPECT_EQ(tracker.searchRect(kImageSize).size(), cv::Size(288, 288));

  tracker.updateMiss(kImageSize);
  EXPECT_EQ(tracker.missCount(), 3);
  EXPECT_FALSE(tracker.active());
  EXPECT_EQ(tracker.searchRect(kImageSize), cv::Rect(0, 0, 640, 480));
}

TEST(RoiTracker, ImageSizeChangeAndDisableResetToFullFrame)
{
  front_marker_vision::RoiTracker tracker;
  ASSERT_TRUE(tracker.updateTarget(square(100.0F, 100.0F, 50.0F), kImageSize));
  EXPECT_TRUE(tracker.active());

  EXPECT_EQ(tracker.searchRect(cv::Size(1024, 768)), cv::Rect(0, 0, 1024, 768));
  EXPECT_FALSE(tracker.active());
  EXPECT_EQ(tracker.missCount(), 0);

  ASSERT_TRUE(tracker.updateTarget(square(100.0F, 100.0F, 50.0F), cv::Size(1024, 768)));
  tracker.setEnabled(false);
  EXPECT_FALSE(tracker.enabled());
  EXPECT_FALSE(tracker.updateTarget(square(100.0F, 100.0F, 50.0F), kImageSize));
  EXPECT_EQ(tracker.searchRect(kImageSize), cv::Rect(0, 0, 640, 480));
}

TEST(RoiTracker, RejectsInvalidCornersAndOffsetsEveryDetectedMarker)
{
  front_marker_vision::RoiTracker tracker;
  std::vector<cv::Point2f> invalid = square(10.0F, 10.0F, 20.0F);
  invalid[2].x = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(tracker.updateTarget(invalid, kImageSize));
  EXPECT_FALSE(tracker.active());

  std::vector<std::vector<cv::Point2f>> corners = {
    {{1.0F, 2.0F}, {3.0F, 4.0F}},
    {{5.0F, 6.0F}},
  };
  front_marker_vision::offsetMarkerCorners(corners, cv::Point2f(20.0F, 30.0F));
  EXPECT_EQ(corners[0][0], cv::Point2f(21.0F, 32.0F));
  EXPECT_EQ(corners[0][1], cv::Point2f(23.0F, 34.0F));
  EXPECT_EQ(corners[1][0], cv::Point2f(25.0F, 36.0F));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
