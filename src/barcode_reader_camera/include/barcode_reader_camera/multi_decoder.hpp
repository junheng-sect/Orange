#pragma once
// Camera barcode decoding ported from Drone-Vision vision_detect.cpp decode_barcodes().
// Strategy order, parameters and strategy labels of the first frame are kept so logs compare 1:1
// with vision_run.log; the retry step differs (see DecoderConfig::retry_timeout_ms). This header
// does not expose ZXing types.
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
  // Codes still missing after the first frame: take new frames, each with its own tag pose, and run
  // only the label-ROI strategies (WARP/ROI) on them until this many ms after decoding started.
  // 0 = no retry. Drone-Vision retried twice on the whole frame shrunk to roi_2d_scale.
  int retry_timeout_ms = 2000;
  // Label geometry used to project the decode ROI from the tag pose.
  double label_width_m = 0.10;
  double label_height_m = 0.15;
  std::string tag_position = "center";  // center/top_left/top_right/bottom_left/bottom_right/custom (or 中文)
  // custom only: tag center relative to the label center, in the tag frame (+x right, +y tag-up).
  double tag_offset_x_m = 0.0;
  double tag_offset_y_m = 0.0;
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
  std::string source;        // strategy label that first produced it (RT-... on a retry frame)
  int frame = 0;             // frame the corners refer to: 0 = input frame, n = n-th retry frame
};

struct DecodeOutput {
  bool success = false;
  bool partial = false;
  // "" on success; disabled/no_formats/decode_fail/partial/cancelled.
  std::string error;
  std::vector<DecodedCode> codes;
  double elapsed_ms = 0.0;
  bool used_tag_roi = false;
  std::vector<cv::Point2f> roi_quad;                   // input frame's projected label ROI, empty without a tag
  std::vector<std::array<cv::Point2f, 4>> failed_boxes;  // input frame: located but not decoded (ROI only)
  std::string report;                                  // DecodeReport line, e.g. "QR=1/1 DM=0/1 C128=2/2"
  int frames = 0;        // frames scanned: 1 (input frame) + retry frames; 0 when decoding did not run
  int result_frame = 0;  // last frame that produced a new code (0 = input frame or no code)
};

struct DecodeHooks {
  std::function<void(const std::string&)> log;  // one line per call, no trailing newline
  std::function<bool()> cancelled;              // polled before every decode attempt
  // Retry source replacing Drone-Vision's camera re-grab: block up to timeout_ms for a frame newer
  // than the previous one that has its own tag pose (T_camera_tag, same convention as decodeBarcodes).
  // Returns false when none arrived in time or decoding was cancelled.
  std::function<bool(int timeout_ms, cv::Mat& gray, TagPose& pose)> next_frame;
};

// gray: CV_8UC1 full camera frame. tag: nullptr = no ROI (pyramid only).
DecodeOutput decodeBarcodes(const cv::Mat& gray, const TagPose* tag, const CameraModel& camera,
                            const DecoderConfig& config, const DecodeHooks& hooks = DecodeHooks());

std::string toHex(const std::vector<uint8_t>& bytes);

// Draw on frame `frame` of the decode (0 = input frame, n = n-th retry frame): its decoded codes
// (green, "#index"), and for the input frame also the ROI (cyan) and failed candidates (red).
void drawDecodeOutput(cv::Mat& bgr, const DecodeOutput& output, int frame = 0);

}  // namespace barcode_reader_camera
