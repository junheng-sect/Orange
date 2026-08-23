#include <front_marker_vision/roi_tracker.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace front_marker_vision
{
namespace
{

constexpr double kTrackingScale = 3.0;
constexpr double kMinimumSidePx = 128.0;
constexpr double kFirstMissScale = 1.5;
constexpr double kSecondMissScale = 2.25;
constexpr int kFullFrameAfterMisses = 3;

bool validImageSize(const cv::Size& size)
{
  return size.width > 0 && size.height > 0;
}

}  // namespace

void RoiTracker::setEnabled(bool enabled)
{
  if (enabled_ != enabled)
  {
    enabled_ = enabled;
    reset();
  }
}

void RoiTracker::reset()
{
  active_ = false;
  image_size_ = cv::Size();
  center_ = cv::Point2f();
  base_side_px_ = 0.0;
  current_side_px_ = 0.0;
  miss_count_ = 0;
}

bool RoiTracker::prepareImageSize(const cv::Size& image_size)
{
  if (!validImageSize(image_size))
  {
    reset();
    return false;
  }
  if (image_size_ != image_size)
  {
    active_ = false;
    center_ = cv::Point2f();
    base_side_px_ = 0.0;
    current_side_px_ = 0.0;
    miss_count_ = 0;
    image_size_ = image_size;
  }
  return true;
}

cv::Rect RoiTracker::searchRect(const cv::Size& image_size)
{
  if (!prepareImageSize(image_size))
  {
    return cv::Rect();
  }
  if (!enabled_ || !active_)
  {
    return cv::Rect(0, 0, image_size.width, image_size.height);
  }
  const cv::Rect result = centeredRect(center_, current_side_px_, image_size);
  if (result.width <= 0 || result.height <= 0)
  {
    active_ = false;
    return cv::Rect(0, 0, image_size.width, image_size.height);
  }
  return result;
}

bool RoiTracker::updateTarget(
  const std::vector<cv::Point2f>& corners, const cv::Size& image_size)
{
  if (!prepareImageSize(image_size) || !enabled_ || corners.empty())
  {
    return false;
  }

  float min_x = std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float max_x = -std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();
  for (const cv::Point2f& corner : corners)
  {
    if (!std::isfinite(corner.x) || !std::isfinite(corner.y))
    {
      return false;
    }
    min_x = std::min(min_x, corner.x);
    min_y = std::min(min_y, corner.y);
    max_x = std::max(max_x, corner.x);
    max_y = std::max(max_y, corner.y);
  }

  const double marker_side = std::max(max_x - min_x, max_y - min_y);
  if (!std::isfinite(marker_side) || marker_side <= 0.0)
  {
    return false;
  }

  center_ = cv::Point2f((min_x + max_x) * 0.5F, (min_y + max_y) * 0.5F);
  base_side_px_ = std::max(kMinimumSidePx, marker_side * kTrackingScale);
  current_side_px_ = base_side_px_;
  miss_count_ = 0;
  active_ = true;
  return true;
}

void RoiTracker::updateMiss(const cv::Size& image_size)
{
  if (!prepareImageSize(image_size) || !enabled_ || !active_)
  {
    return;
  }

  ++miss_count_;
  if (miss_count_ == 1)
  {
    current_side_px_ = base_side_px_ * kFirstMissScale;
  }
  else if (miss_count_ == 2)
  {
    current_side_px_ = base_side_px_ * kSecondMissScale;
  }
  else if (miss_count_ >= kFullFrameAfterMisses)
  {
    active_ = false;
  }
}

bool RoiTracker::enabled() const
{
  return enabled_;
}

bool RoiTracker::active() const
{
  return enabled_ && active_;
}

int RoiTracker::missCount() const
{
  return miss_count_;
}

cv::Rect RoiTracker::centeredRect(
  const cv::Point2f& center, double side, const cv::Size& image_size)
{
  if (!validImageSize(image_size) || !std::isfinite(center.x) ||
      !std::isfinite(center.y) || !std::isfinite(side) || side <= 0.0)
  {
    return cv::Rect();
  }

  const int requested_side = std::max(1, static_cast<int>(std::ceil(side)));
  const int width = std::min(requested_side, image_size.width);
  const int height = std::min(requested_side, image_size.height);
  const int maximum_x = image_size.width - width;
  const int maximum_y = image_size.height - height;
  const int x = std::max(
    0, std::min(maximum_x, static_cast<int>(std::lround(center.x - width * 0.5))));
  const int y = std::max(
    0, std::min(maximum_y, static_cast<int>(std::lround(center.y - height * 0.5))));
  return cv::Rect(x, y, width, height);
}

void offsetMarkerCorners(
  std::vector<std::vector<cv::Point2f>>& corners, const cv::Point2f& offset)
{
  if (offset.x == 0.0F && offset.y == 0.0F)
  {
    return;
  }
  for (std::vector<cv::Point2f>& marker_corners : corners)
  {
    for (cv::Point2f& corner : marker_corners)
    {
      corner += offset;
    }
  }
}

}  // namespace front_marker_vision
