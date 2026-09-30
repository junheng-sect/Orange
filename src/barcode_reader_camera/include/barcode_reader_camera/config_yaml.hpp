#pragma once
// YAML loading shared by decode_image and the ROS node. Keys mirror the Drone-Vision
// config.json sections `tag`, `detector` and `barcode_decoder`.
#include <string>

#include <yaml-cpp/yaml.h>

#include "barcode_reader_camera/multi_decoder.hpp"

namespace barcode_reader_camera {

// AprilTag settings used only when this package detects the tag itself (decode_image, tests).
struct TagConfig {
  std::string family = "tag36h11";
  double size_m = 0.012;
  double quad_decimate = 1.5;
  int nthreads = 4;
  bool refine_edges = true;
  int max_dimension = 1280;  // Drone-Vision tag_long
};

// Throws std::runtime_error with the offending key on invalid values.
DecoderConfig loadDecoderConfig(const YAML::Node& root);
TagConfig loadTagConfig(const YAML::Node& root);

// ROS camera_info YAML (camera_matrix/distortion_coefficients, image_width/height).
CameraModel loadCameraYaml(const std::string& path, int* width = nullptr, int* height = nullptr);

}  // namespace barcode_reader_camera
