#ifndef USB_CAM_MONO8_UTILS_H
#define USB_CAM_MONO8_UTILS_H

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace usb_cam
{

inline bool copyLumaPlane(
  const uint8_t* source,
  int source_stride,
  int width,
  int height,
  uint8_t* destination,
  size_t destination_size)
{
  if (source == nullptr || destination == nullptr || width <= 0 || height <= 0 ||
      source_stride < width)
  {
    return false;
  }

  const size_t required_size =
    static_cast<size_t>(width) * static_cast<size_t>(height);
  if (destination_size < required_size)
  {
    return false;
  }

  for (int row = 0; row < height; ++row)
  {
    std::memcpy(
      destination + static_cast<size_t>(row) * static_cast<size_t>(width),
      source + static_cast<size_t>(row) * static_cast<size_t>(source_stride),
      static_cast<size_t>(width));
  }
  return true;
}

}  // namespace usb_cam

#endif  // USB_CAM_MONO8_UTILS_H
