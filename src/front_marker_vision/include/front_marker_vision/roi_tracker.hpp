#ifndef FRONT_MARKER_VISION_ROI_TRACKER_HPP
#define FRONT_MARKER_VISION_ROI_TRACKER_HPP

#include <vector>

#include <opencv2/core.hpp>

namespace front_marker_vision
{

class RoiTracker
{
public:
  void setEnabled(bool enabled);
  void reset();

  cv::Rect searchRect(const cv::Size& image_size);
  bool updateTarget(
    const std::vector<cv::Point2f>& corners, const cv::Size& image_size);
  void updateMiss(const cv::Size& image_size);

  bool enabled() const;
  bool active() const;
  int missCount() const;

private:
  static cv::Rect centeredRect(
    const cv::Point2f& center, double side, const cv::Size& image_size);
  bool prepareImageSize(const cv::Size& image_size);

  bool enabled_ = true;
  bool active_ = false;
  cv::Size image_size_;
  cv::Point2f center_;
  double base_side_px_ = 0.0;
  double current_side_px_ = 0.0;
  int miss_count_ = 0;
};

void offsetMarkerCorners(
  std::vector<std::vector<cv::Point2f>>& corners, const cv::Point2f& offset);

}  // namespace front_marker_vision

#endif  // FRONT_MARKER_VISION_ROI_TRACKER_HPP
