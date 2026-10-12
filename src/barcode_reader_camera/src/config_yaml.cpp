#include "barcode_reader_camera/config_yaml.hpp"

#include <cmath>
#include <stdexcept>

namespace barcode_reader_camera {
namespace {

template <typename T>
T get(const YAML::Node& node, const char* key, T fallback) {
  if (!node || !node[key]) return fallback;
  try {
    return node[key].as<T>();
  } catch (const YAML::Exception&) {
    throw std::runtime_error(std::string("invalid value for ") + key);
  }
}

void require(bool ok, const char* what) {
  if (!ok) throw std::runtime_error(std::string("invalid config: ") + what);
}

void readExpected(const YAML::Node& e, ExpectedCounts& x) {
  x.qr = get(e, "QRCode", x.qr);
  x.dm = get(e, "DataMatrix", x.dm);
  x.c128 = get(e, "Code128", x.c128);
  x.c39 = get(e, "Code39", x.c39);
  x.ean13 = get(e, "EAN13", x.ean13);
  x.pdf417 = get(e, "PDF417", x.pdf417);
  x.aztec = get(e, "Aztec", x.aztec);
  x.itf = get(e, "ITF", x.itf);
}

void readFormats(const YAML::Node& f, FormatSwitches& x) {
  x.qr = get(f, "QRCode", x.qr);
  x.dm = get(f, "DataMatrix", x.dm);
  x.c128 = get(f, "Code128", x.c128);
  x.c39 = get(f, "Code39", x.c39);
  x.ean13 = get(f, "EAN13", x.ean13);
  x.pdf417 = get(f, "PDF417", x.pdf417);
  x.aztec = get(f, "Aztec", x.aztec);
  x.itf = get(f, "ITF", x.itf);
}

}  // namespace

void validateDecoderConfig(const DecoderConfig& c) {
  require(std::isfinite(c.label_width_m) && c.label_width_m > 0, "label width > 0");
  require(std::isfinite(c.label_height_m) && c.label_height_m > 0, "label height > 0");
  require(std::isfinite(c.roi_scale) && c.roi_scale > 0, "tag.roi_scale > 0");
  static const char* positions[] = {"center", "中间", "top_left", "左上", "top_right", "右上",
                                    "bottom_left", "左下", "bottom_right", "右下", "custom", "自定义"};
  bool known = false;
  for (const char* p : positions) known = known || c.tag_position == p;
  require(known, "tag_position");
  require(std::isfinite(c.tag_offset_x_m), "tag_offset_x_meters finite");
  require(std::isfinite(c.tag_offset_y_m), "tag_offset_y_meters finite");
  const ExpectedCounts& x = c.expected;
  require(x.qr >= 0 && x.dm >= 0 && x.c128 >= 0 && x.c39 >= 0 && x.ean13 >= 0 && x.pdf417 >= 0 &&
              x.aztec >= 0 && x.itf >= 0,
          "expected_codes >= 0");
  require(c.roi_2d_scale > 0, "barcode_decoder.roi_2d_scale > 0");
  require(std::isfinite(c.clahe_clip) && c.clahe_clip > 0, "preprocess.clahe.clip_limit > 0");
  require(c.clahe_grid >= 1, "preprocess.clahe.grid_size >= 1");
  require(c.retry_timeout_ms >= 0, "retry.timeout_ms >= 0");
}

void applyLayout(const YAML::Node& layout, DecoderConfig& c, double* tag_size_m) {
  const YAML::Node label = layout["label"];
  c.label_width_m = get(label, "width_meters", c.label_width_m);
  c.label_height_m = get(label, "height_meters", c.label_height_m);
  c.tag_position = get(label, "tag_position", c.tag_position);
  c.tag_offset_x_m = get(label, "tag_offset_x_meters", c.tag_offset_x_m);
  c.tag_offset_y_m = get(label, "tag_offset_y_meters", c.tag_offset_y_m);
  readFormats(layout["formats"], c.formats);
  readExpected(layout["expected_codes"], c.expected);
  if (tag_size_m) {
    *tag_size_m = get(layout, "target_marker_size_m", *tag_size_m);
    require(std::isfinite(*tag_size_m) && *tag_size_m > 0, "layout target_marker_size_m > 0");
  }
  validateDecoderConfig(c);
}

DecoderConfig loadDecoderConfig(const YAML::Node& root) {
  DecoderConfig c;
  const YAML::Node tag = root["tag"];
  c.label_width_m = get(tag, "label_width_meters", c.label_width_m);
  c.label_height_m = get(tag, "label_height_meters", c.label_height_m);
  c.tag_position = get(tag, "tag_position", c.tag_position);
  c.tag_offset_x_m = get(tag, "tag_offset_x_meters", c.tag_offset_x_m);
  c.tag_offset_y_m = get(tag, "tag_offset_y_meters", c.tag_offset_y_m);
  c.roi_scale = get(tag, "roi_scale", c.roi_scale);

  const YAML::Node bc = root["barcode_decoder"];
  c.enabled = get(bc, "enabled", c.enabled);
  c.roi_2d_scale = get(bc, "roi_2d_scale", c.roi_2d_scale);
  readFormats(bc ? bc["formats"] : YAML::Node(), c.formats);
  readExpected(bc ? bc["expected_codes"] : YAML::Node(), c.expected);
  const YAML::Node pre = bc ? bc["preprocess"] : YAML::Node();
  c.preprocess_enabled = get(pre, "enabled", c.preprocess_enabled);
  const YAML::Node clahe = pre ? pre["clahe"] : YAML::Node();
  c.clahe_enabled = get(clahe, "enabled", c.clahe_enabled);
  c.clahe_clip = get(clahe, "clip_limit", c.clahe_clip);
  c.clahe_grid = get(clahe, "grid_size", c.clahe_grid);
  c.try_rotate = get(bc ? bc["decoder"] : YAML::Node(), "try_rotate", c.try_rotate);
  const YAML::Node retry = bc ? bc["retry"] : YAML::Node();
  c.retry_timeout_ms = get(retry, "timeout_ms", c.retry_timeout_ms);
  // Drone-Vision keys of the old whole-frame retry; reject them so an old config is not silently ignored.
  require(!retry || (!retry["count"] && !retry["interval_ms"]),
          "retry.count/interval_ms were replaced by retry.timeout_ms");
  c.undistort_enabled = get(bc ? bc["undistort"] : YAML::Node(), "enabled", c.undistort_enabled);

  validateDecoderConfig(c);
  return c;
}

TagConfig loadTagConfig(const YAML::Node& root) {
  TagConfig t;
  const YAML::Node tag = root["tag"];
  t.family = get(tag, "family", t.family);
  t.size_m = get(tag, "size_meters", t.size_m);
  const YAML::Node det = root["detector"];
  t.quad_decimate = get(det, "quad_decimate", t.quad_decimate);
  t.nthreads = get(det, "nthreads", t.nthreads);
  t.refine_edges = get(det, "refine_edges", t.refine_edges);
  require(std::isfinite(t.size_m) && t.size_m > 0, "tag.size_meters > 0");
  return t;
}

CameraModel loadCameraYaml(const std::string& path, int* width, int* height) {
  const YAML::Node root = YAML::LoadFile(path);
  const auto k = root["camera_matrix"]["data"].as<std::vector<double>>();
  require(k.size() == 9, "camera_matrix.data needs 9 values");
  CameraModel camera;
  camera.camera_matrix = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i) camera.camera_matrix.at<double>(i / 3, i % 3) = k[i];
  if (root["distortion_coefficients"]) {
    const auto d = root["distortion_coefficients"]["data"].as<std::vector<double>>();
    camera.dist_coeffs = cv::Mat(1, (int)d.size(), CV_64F);
    for (size_t i = 0; i < d.size(); ++i) camera.dist_coeffs.at<double>(0, (int)i) = d[i];
  }
  if (width) *width = get(root, "image_width", 0);
  if (height) *height = get(root, "image_height", 0);
  return camera;
}

}  // namespace barcode_reader_camera
