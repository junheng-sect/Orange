#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>
#include <usb_cam/mono8_utils.h>

namespace
{

void expectCopyWithPadding(int width, int height, int padding)
{
  const int stride = width + padding;
  std::vector<uint8_t> source(static_cast<size_t>(stride) * height, 0xEE);
  std::vector<uint8_t> destination(static_cast<size_t>(width) * height, 0);
  for (int row = 0; row < height; ++row)
  {
    for (int column = 0; column < width; ++column)
    {
      source[static_cast<size_t>(row) * stride + column] =
        static_cast<uint8_t>((row * 17 + column * 3) & 0xFF);
    }
  }

  ASSERT_TRUE(usb_cam::copyLumaPlane(
    source.data(), stride, width, height, destination.data(), destination.size()));
  for (int row = 0; row < height; ++row)
  {
    for (int column = 0; column < width; ++column)
    {
      EXPECT_EQ(
        source[static_cast<size_t>(row) * stride + column],
        destination[static_cast<size_t>(row) * width + column]);
    }
  }
}

TEST(Mono8Output, CopiesCommonWidthsWithPaddedStride)
{
  expectCopyWithPadding(640, 5, 16);
  expectCopyWithPadding(1024, 3, 32);
  expectCopyWithPadding(1280, 2, 64);
}

TEST(Mono8Output, RejectsInvalidArguments)
{
  std::vector<uint8_t> source(64, 1);
  std::vector<uint8_t> destination(64, 0);
  EXPECT_FALSE(usb_cam::copyLumaPlane(
    nullptr, 8, 8, 8, destination.data(), destination.size()));
  EXPECT_FALSE(usb_cam::copyLumaPlane(
    source.data(), 8, 8, 8, nullptr, destination.size()));
  EXPECT_FALSE(usb_cam::copyLumaPlane(
    source.data(), 7, 8, 8, destination.data(), destination.size()));
  EXPECT_FALSE(usb_cam::copyLumaPlane(
    source.data(), 8, 0, 8, destination.data(), destination.size()));
  EXPECT_FALSE(usb_cam::copyLumaPlane(
    source.data(), 8, 8, 0, destination.data(), destination.size()));
  EXPECT_FALSE(usb_cam::copyLumaPlane(
    source.data(), 8, 8, 8, destination.data(), destination.size() - 1));
}

}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
