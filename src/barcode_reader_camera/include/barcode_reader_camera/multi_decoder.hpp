#pragma once
// Camera barcode decoding ported from Drone-Vision vision_detect.cpp decode_barcodes().
// Strategy order, parameters and strategy labels are kept so logs compare 1:1 with
// vision_run.log. This header does not expose ZXing types.
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace barcode_reader_camera {

struct FormatSwitches {
  bool qr = true, dm = true, c128 = true, c39 = false;
  bool ean13 = false, pdf417 = false, aztec = false, itf = false;
};

// Required count per format (Drone-Vision expected_codes). <= 0 means "decode if seen".
struct ExpectedCounts {
  int qr = 1, dm = 1, c128 = 2, c39 = 0, ean13 = 0, pdf417 = 0, aztec = 0, itf = 0;
};

// Defaults follow the deployed Drone-Vision/publish/config.json.
struct DecoderConfig {
  bool enabled = true;
  FormatSwitches formats;
  ExpectedCounts expected;
  bool try_rotate = true;
  bool preprocess_enabled = true;
  bool clahe_enabled = true;
  double clahe_clip = 2.0;
  int clahe_grid = 8;
  int roi_2d_scale = 512;
  bool undistort_enabled = true;
  int retry_count = 2;
  int retry_interval_ms = 100;
  // Label geometry used to project the decode ROI from the tag pose.
  double label_width_m = 0.10;
  double label_height_m = 0.15;
  std::string tag_position = "center";  // center/top_left/top_right/bottom_left/bottom_right (or 中文)
  double roi_scale = 1.25;
};

// T_camera_tag from solvePnP with the Drone-Vision object points
// (-h,-h,0), (h,-h,0), (h,h,0), (-h,h,0) for AprilTag p[0..3].
struct TagPose {
  cv::Vec3d rvec;
  cv::Vec3d tvec;
};

struct CameraModel {
  cv::Mat camera_matrix;  // 3x3 CV_64F
  cv::Mat dist_coeffs;    // 1xN CV_64F, may be empty
  bool valid() const { return camera_matrix.rows == 3 && camera_matrix.cols == 3; }
};

struct DecodedCode {
  std::string format;        // QRCode/DataMatrix/Code128/Code39/EAN-13/PDF417/Aztec/ITF
  std::string code;          // decoded text (UTF-8)
  std::vector<uint8_t> bytes;
  std::string symbology_id;  // ISO/IEC 15424, e.g. ]Q1 ]d1 ]C0
  std::array<cv::Point2f, 4> corners;  // original image pixels
  cv::Point2f center;
  std::string source;        // strategy label that first produced it
};

struct DecodeOutput {
  bool success = false;
  bool partial = false;
  // "" on success; disabled/no_formats/decode_fail/partial/cancelled.
  std::string error;
  std::vector<DecodedCode> codes;
  double elapsed_ms = 0.0;
  bool used_tag_roi = false;
  std::vector<cv::Point2f> roi_quad;                   // projected label ROI, empty without a tag
  std::vector<std::array<cv::Point2f, 4>> failed_boxes;  // located but not decoded (ROI only)
  std::string report;                                  // DecodeReport line, e.g. "QR=1/1 DM=0/1 C128=2/2"
};

struct DecodeHooks {
  std::function<void(const std::string&)> log;  // one line per call, no trailing newline
  std::function<bool()> cancelled;              // polled before every decode attempt
  // Retry source replacing Drone-Vision's camera re-grab; returns false when no frame.
  std::function<bool(cv::Mat& gray)> next_frame;
};

// gray: CV_8UC1 full camera frame. tag: nullptr = no ROI (pyramid only).
DecodeOutput decodeBarcodes(const cv::Mat& gray, const TagPose* tag, const CameraModel& camera,
                            const DecoderConfig& config, const DecodeHooks& hooks = DecodeHooks());

std::string toHex(const std::vector<uint8_t>& bytes);

// Draw ROI (cyan), failed candidates (red) and decoded codes (green, "#index").
void drawDecodeOutput(cv::Mat& bgr, const DecodeOutput& output);

}  // namespace barcode_reader_camera
