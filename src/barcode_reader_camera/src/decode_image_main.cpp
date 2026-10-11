// Decode all barcodes in an image file with the same chain as the camera node:
// AprilTag (ov13855_apriltag_vision) -> solvePnP -> label ROI -> decodeBarcodes().
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ov13855_apriltag_vision/apriltag_detector.hpp>
#include <ov13855_apriltag_vision/pose_solver.hpp>

#include "barcode_reader_camera/config_yaml.hpp"
#include "barcode_reader_camera/multi_decoder.hpp"

using namespace barcode_reader_camera;

namespace {

void usage() {
  std::fprintf(stderr,
      "usage: decode_image <image> [--config yaml] [--layout layouts/x.yaml]\n"
      "                    [--camera-yaml K.yaml | --focal-px F] [--pose meta.yaml]\n"
      "                    [--no-tag] [--all-formats] [--free] [--annotate out.jpg] [--verbose]\n"
      "  --layout        label layout overlay (tag size, label geometry, expected codes)\n"
      "  --pose          use pose.rvec/tvec from a node-saved meta.yaml instead of detecting the tag\n"
      "  --focal-px F    pinhole focal length in pixels; default 0.6 x image diagonal\n"
      "                  (~26 mm equivalent phone lens), principal point at image center\n"
      "  --no-tag        skip AprilTag, decode with the full-frame pyramid only\n"
      "  --all-formats   enable all 8 symbologies\n"
      "  --free          set every expected count to 0 (run the whole chain, no early exit)\n");
}

std::string escape(const std::string& s) {
  std::string out;
  for (unsigned char ch : s) {
    if (ch == '"' || ch == '\\') { out.push_back('\\'); out.push_back((char)ch); }
    else if (ch < 0x20) { char buf[8]; std::snprintf(buf, sizeof(buf), "\\x%02x", ch); out += buf; }
    else out.push_back((char)ch);
  }
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  std::string image_path, config_path, layout_path, camera_yaml, annotate_path, pose_yaml;
  double focal_px = 0.0;
  bool no_tag = false, all_formats = false, free_decode = false, verbose = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto value = [&]() -> std::string {
      if (i + 1 >= argc) { usage(); std::exit(2); }
      return argv[++i];
    };
    if (a == "--config") config_path = value();
    else if (a == "--layout") layout_path = value();
    else if (a == "--camera-yaml") camera_yaml = value();
    else if (a == "--focal-px") focal_px = std::atof(value().c_str());
    else if (a == "--annotate") annotate_path = value();
    else if (a == "--pose") pose_yaml = value();
    else if (a == "--no-tag") no_tag = true;
    else if (a == "--all-formats") all_formats = true;
    else if (a == "--free") free_decode = true;
    else if (a == "--verbose") verbose = true;
    else if (a == "-h" || a == "--help") { usage(); return 0; }
    else if (!a.empty() && a[0] == '-') { usage(); return 2; }
    else image_path = a;
  }
  if (image_path.empty()) { usage(); return 2; }

  DecoderConfig config;
  TagConfig tag_config;
  try {
    const YAML::Node root = config_path.empty() ? YAML::Node() : YAML::LoadFile(config_path);
    config = loadDecoderConfig(root);
    tag_config = loadTagConfig(root);
    if (!layout_path.empty()) applyLayout(YAML::LoadFile(layout_path), config, &tag_config.size_m);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "config error: %s\n", e.what());
    return 2;
  }
  if (all_formats) {
    config.formats = FormatSwitches{true, true, true, true, true, true, true, true};
  }
  if (free_decode) config.expected = ExpectedCounts{0, 0, 0, 0, 0, 0, 0, 0};

  cv::Mat color = cv::imread(image_path, cv::IMREAD_COLOR);  // applies EXIF orientation
  if (color.empty()) {
    std::fprintf(stderr, "cannot read image %s\n", image_path.c_str());
    return 2;
  }
  cv::Mat gray;
  cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);

  CameraModel camera;
  std::string camera_source;
  if (!camera_yaml.empty()) {
    camera = loadCameraYaml(camera_yaml);
    camera_source = camera_yaml;
  } else {
    const double f = focal_px > 0 ? focal_px : 0.6 * std::hypot(gray.cols, gray.rows);
    camera.camera_matrix = (cv::Mat_<double>(3, 3) << f, 0, gray.cols / 2.0, 0, f, gray.rows / 2.0, 0, 0, 1);
    camera.dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
    camera_source = focal_px > 0 ? "--focal-px" : "estimated 0.6*diagonal";
  }
  std::printf("image: %s %dx%d\n", image_path.c_str(), gray.cols, gray.rows);
  std::printf("camera: fx=%.1f fy=%.1f cx=%.1f cy=%.1f (%s)\n",
              camera.camera_matrix.at<double>(0, 0), camera.camera_matrix.at<double>(1, 1),
              camera.camera_matrix.at<double>(0, 2), camera.camera_matrix.at<double>(1, 2),
              camera_source.c_str());

  TagPose pose;
  bool have_pose = false;
  std::vector<cv::Point2f> tag_corners;
  if (!pose_yaml.empty() && !no_tag) {
    try {
      const YAML::Node p = YAML::LoadFile(pose_yaml)["pose"];
      if (!p) throw std::runtime_error("no pose (the node had no tag for this frame)");
      const auto r = p["rvec"].as<std::vector<double>>(), t = p["tvec"].as<std::vector<double>>();
      if (r.size() != 3 || t.size() != 3) throw std::runtime_error("pose.rvec/tvec need 3 values");
      pose.rvec = cv::Vec3d(r[0], r[1], r[2]);
      pose.tvec = cv::Vec3d(t[0], t[1], t[2]);
      have_pose = true;
      std::printf("tag: pose from %s t=(%.3f, %.3f, %.3f)m\n", pose_yaml.c_str(), pose.tvec[0], pose.tvec[1],
                  pose.tvec[2]);
    } catch (const std::exception& e) {
      std::fprintf(stderr, "pose error: %s: %s\n", pose_yaml.c_str(), e.what());
      return 2;
    }
  } else if (!no_tag) {
    ov13855_apriltag_vision::AprilTagDetector detector(
        tag_config.family, tag_config.quad_decimate, tag_config.nthreads,
        tag_config.refine_edges, tag_config.max_dimension);
    const auto detections = detector.detect(gray);
    const int index = ov13855_apriltag_vision::selectDetection(detections, "largest", 0);
    if (index < 0) {
      std::printf("tag: none detected (%s) -> pyramid only\n", tag_config.family.c_str());
    } else {
      tag_corners = detections[index].corners;
      ov13855_apriltag_vision::PoseSolution solution;
      std::string reason;
      if (ov13855_apriltag_vision::solveTagPose(tag_corners, tag_config.size_m, camera.camera_matrix,
                                                camera.dist_coeffs, solution, reason)) {
        pose.rvec = solution.rvec;
        pose.tvec = solution.tvec;
        have_pose = true;
        std::printf("tag: id=%d size=%.3fm t=(%.3f, %.3f, %.3f)m corners=", detections[index].id,
                    tag_config.size_m, pose.tvec[0], pose.tvec[1], pose.tvec[2]);
        for (const auto& p : tag_corners) std::printf("(%.0f,%.0f)", p.x, p.y);
        std::printf("\n");
      } else {
        std::printf("tag: id=%d found but pose failed: %s -> pyramid only\n",
                    detections[index].id, reason.c_str());
      }
    }
  } else {
    std::printf("tag: skipped (--no-tag)\n");
  }

  DecodeHooks hooks;
  if (verbose) hooks.log = [](const std::string& line) { std::fprintf(stderr, "%s\n", line.c_str()); };
  const DecodeOutput out = decodeBarcodes(gray, have_pose ? &pose : nullptr, camera, config, hooks);

  std::printf("result: success=%d partial=%d error=\"%s\" elapsed=%.0fms tag_roi=%d report=\"%s\"\n",
              (int)out.success, (int)out.partial, out.error.c_str(), out.elapsed_ms,
              (int)out.used_tag_roi, out.report.c_str());
  for (size_t i = 0; i < out.codes.size(); ++i) {
    const auto& c = out.codes[i];
    std::printf("#%zu [%s] code=\"%s\" raw_hex=\"%s\" symbology=%s center=(%.0f,%.0f) source=%s\n",
                i + 1, c.format.c_str(), escape(c.code).c_str(), toHex(c.bytes).c_str(),
                c.symbology_id.c_str(), c.center.x, c.center.y, c.source.c_str());
  }

  if (!annotate_path.empty()) {
    cv::Mat canvas = color.clone();
    if (tag_corners.size() == 4) {
      for (int k = 0; k < 4; ++k)
        cv::line(canvas, tag_corners[k], tag_corners[(k + 1) % 4], cv::Scalar(255, 0, 255),
                 std::max(2, canvas.cols / 800));
    }
    drawDecodeOutput(canvas, out);
    if (!cv::imwrite(annotate_path, canvas)) {
      std::fprintf(stderr, "cannot write %s\n", annotate_path.c_str());
      return 2;
    }
    std::printf("annotated: %s\n", annotate_path.c_str());
  }
  return out.success ? 0 : 1;
}
