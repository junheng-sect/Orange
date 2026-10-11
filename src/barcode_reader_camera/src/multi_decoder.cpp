// Port of Drone-Vision vision_core/src/vision_detect.cpp decode_barcodes() and its 1D helpers
// (BUILD_ID vision_detect_fusion_fix_buildlog_20260903_01). The decode strategy chain, parameters
// and labels are unchanged; only the process-global dependencies were replaced:
//   VisionConfigSnapshot -> DecoderConfig (mapped to LegacyCfg below, original field names)
//   VisionResult buffers  -> DecodeOutput
//   vlogf/ms_now/op_aborted/g_ctx.cap->read -> DecodeHooks log/steady clock/cancelled/next_frame
//   rv/tv + r->found      -> optional TagPose + CameraModel
#include "barcode_reader_camera/multi_decoder.hpp"
#include "barcode_reader_camera/barcode_result_fusion.h"

#include <ZXing/ReadBarcode.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <limits>
#include <map>
#include <stdexcept>
#include <thread>

namespace barcode_reader_camera {
namespace {

long long ms_now() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

class Logger {
 public:
  explicit Logger(const std::function<void(const std::string&)>& sink) : sink_(sink) {}
  void operator()(const char* fmt, ...) const __attribute__((format(printf, 2, 3))) {
    if (!sink_) return;
    char buf[1024];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    std::string line(buf);
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) line.pop_back();
    sink_(line);
  }

 private:
  const std::function<void(const std::string&)>& sink_;
};

// Original VisionConfigSnapshot field names, so the ported body reads like vision_detect.cpp.
struct LegacyCfg {
  bool barcode_enabled;
  bool bc_qr, bc_dm, bc_128, bc_39, bc_ean13, bc_pdf417, bc_aztec, bc_itf;
  int exp_qr, exp_dm, exp_128, exp_39, exp_ean13, exp_pdf417, exp_aztec, exp_itf;
  int bc_roi_2d_scale;
  bool bc_try_rotate;
  int bc_retry_count, bc_retry_interval_ms;
  bool bc_preproc_enabled, bc_clahe_enabled;
  double bc_clahe_clip;
  int bc_clahe_grid;
  bool bc_undistort_enabled;
  double label_width_meters, label_height_meters;
  const char* tag_position;
  double tag_offset_x_meters, tag_offset_y_meters;
  double roi_scale;
  cv::Mat camera_matrix, dist_coeffs;
};

LegacyCfg make_legacy(const DecoderConfig& c, const CameraModel& camera) {
  LegacyCfg l;
  l.barcode_enabled = c.enabled;
  l.bc_qr = c.formats.qr; l.bc_dm = c.formats.dm; l.bc_128 = c.formats.c128; l.bc_39 = c.formats.c39;
  l.bc_ean13 = c.formats.ean13; l.bc_pdf417 = c.formats.pdf417; l.bc_aztec = c.formats.aztec;
  l.bc_itf = c.formats.itf;
  l.exp_qr = c.expected.qr; l.exp_dm = c.expected.dm; l.exp_128 = c.expected.c128;
  l.exp_39 = c.expected.c39; l.exp_ean13 = c.expected.ean13; l.exp_pdf417 = c.expected.pdf417;
  l.exp_aztec = c.expected.aztec; l.exp_itf = c.expected.itf;
  l.bc_roi_2d_scale = c.roi_2d_scale;
  l.bc_try_rotate = c.try_rotate;
  l.bc_retry_count = c.retry_count;
  l.bc_retry_interval_ms = c.retry_interval_ms;
  l.bc_preproc_enabled = c.preprocess_enabled;
  l.bc_clahe_enabled = c.clahe_enabled;
  l.bc_clahe_clip = c.clahe_clip;
  l.bc_clahe_grid = c.clahe_grid;
  l.bc_undistort_enabled = c.undistort_enabled;
  l.label_width_meters = c.label_width_m;
  l.label_height_meters = c.label_height_m;
  l.tag_position = c.tag_position.c_str();
  l.tag_offset_x_meters = c.tag_offset_x_m;
  l.tag_offset_y_meters = c.tag_offset_y_m;
  l.roi_scale = c.roi_scale;
  l.camera_matrix = camera.camera_matrix;
  l.dist_coeffs = camera.dist_coeffs;
  return l;
}

const char* barcode_format_name(ZXing::BarcodeFormat f) {
  switch (f) {
    case ZXing::BarcodeFormat::None:       return "None";
    case ZXing::BarcodeFormat::QRCode:     return "QRCode";
    case ZXing::BarcodeFormat::DataMatrix: return "DataMatrix";
    case ZXing::BarcodeFormat::Aztec:      return "Aztec";
    case ZXing::BarcodeFormat::PDF417:     return "PDF417";
    case ZXing::BarcodeFormat::Code128:    return "Code128";
    case ZXing::BarcodeFormat::Code39:     return "Code39";
    case ZXing::BarcodeFormat::EAN13:      return "EAN-13";
    case ZXing::BarcodeFormat::ITF:        return "ITF";
    default:                               return "Unknown";
  }
}

// 1D 条码的自适应二值化。与 Sobel 路径分开：默认优先保留原始条宽，
// 只有 OTSU 失败时才用局部阈值；Sobel 仅作为最后兜底。
cv::Mat barcode_1d_adaptive_threshold(const cv::Mat& gray) {
  cv::Mat bin;
  int block = std::min(gray.cols, gray.rows) >= 63 ? 51 : 31;
  if ((block & 1) == 0) ++block;
  block = std::max(3, block);
  cv::adaptiveThreshold(gray, bin, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                        cv::THRESH_BINARY, block, 5);
  return bin;
}

cv::Mat barcode_1d_sobel_adaptive(const cv::Mat& gray) {
  cv::Mat grad, abs_grad, norm, bin;
  cv::Sobel(gray, grad, CV_32F, 1, 0, 3);
  cv::convertScaleAbs(grad, abs_grad);
  cv::normalize(abs_grad, norm, 0, 255, cv::NORM_MINMAX);
  cv::adaptiveThreshold(norm, bin, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                        cv::THRESH_BINARY, 31, 5);
  return bin;
}

// 1D Candidate Locator + Rescue：Sobel/形态学只用于找候选区域，不直接送解码器。
struct Barcode1DCandidate {
  cv::RotatedRect rect;
  float score = 0.0f;
};

float long_edge_angle_deg(const cv::RotatedRect& rr) {
  cv::Point2f p[4];
  rr.points(p);
  float best_len2 = -1.0f;
  cv::Point2f a, b;
  for (int i = 0; i < 4; ++i) {
    cv::Point2f q = p[(i + 1) & 3];
    cv::Point2f d = q - p[i];
    float l2 = d.dot(d);
    if (l2 > best_len2) {
      best_len2 = l2;
      a = p[i];
      b = q;
    }
  }
  return std::atan2(b.y - a.y, b.x - a.x) * 180.0f / (float)CV_PI;
}

std::vector<Barcode1DCandidate> locate_1d_candidates(const cv::Mat& gray) {
  std::vector<Barcode1DCandidate> out;
  if (gray.empty() || gray.cols < 48 || gray.rows < 24)
    return out;

  cv::Mat gx, gy, ax, ay, grad;
  cv::Sobel(gray, gx, CV_16S, 1, 0, 3);
  cv::Sobel(gray, gy, CV_16S, 0, 1, 3);
  cv::convertScaleAbs(gx, ax);
  cv::convertScaleAbs(gy, ay);
  cv::addWeighted(ax, 1.0, ay, 0.35, 0.0, grad);

  cv::Mat bin;
  cv::threshold(grad, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  int kx = std::max(9, std::min(gray.cols / 12, 61));
  if ((kx & 1) == 0) ++kx;
  int ky = std::max(3, std::min(gray.rows / 20, 15));
  if ((ky & 1) == 0) ++ky;
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kx, ky));
  cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, kernel);
  cv::morphologyEx(bin, bin, cv::MORPH_OPEN,
                   cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  const float image_area = (float)gray.total();
  for (const auto& c : contours) {
    if (c.size() < 8) continue;
    double area = cv::contourArea(c);
    if (area < image_area * 0.0015) continue;
    cv::RotatedRect rr = cv::minAreaRect(c);
    float w = rr.size.width, h = rr.size.height;
    float short_e = std::max(1.0f, std::min(w, h));
    float long_e = std::max(w, h);
    float aspect = long_e / short_e;
    if (aspect < 2.0f || long_e < 48.0f || short_e < 10.0f) continue;
    float rect_area = std::max(1.0f, w * h);
    float fill = std::min(1.0f, (float)(area / rect_area));
    float score = aspect * 0.7f + fill * 2.0f;
    out.push_back({rr, score});
  }

  std::sort(out.begin(), out.end(), [](const Barcode1DCandidate& a, const Barcode1DCandidate& b) {
    return a.score > b.score;
  });
  if (out.size() > 3) out.resize(3);
  return out;
}

cv::Mat crop_1d_candidate_deskew(const cv::Mat& gray,
                                 const Barcode1DCandidate& cand,
                                 cv::Point2f& crop_offset_in_rotated,
                                 cv::Mat& rotation_matrix) {
  crop_offset_in_rotated = cv::Point2f();
  rotation_matrix.release();
  if (gray.empty()) return {};

  float angle = long_edge_angle_deg(cand.rect);
  rotation_matrix = cv::getRotationMatrix2D(cand.rect.center, -angle, 1.0);
  cv::Mat rotated;
  cv::warpAffine(gray, rotated, rotation_matrix, gray.size(),
                 cv::INTER_CUBIC, cv::BORDER_REPLICATE);

  float long_e = std::max(cand.rect.size.width, cand.rect.size.height);
  float short_e = std::min(cand.rect.size.width, cand.rect.size.height);
  float pad_x = std::max(16.0f, long_e * 0.20f);
  float pad_y = std::max(8.0f, short_e * 0.20f);

  int x = (int)std::floor(cand.rect.center.x - long_e * 0.5f - pad_x);
  int y = (int)std::floor(cand.rect.center.y - short_e * 0.5f - pad_y);
  int w = (int)std::ceil(long_e + 2.0f * pad_x);
  int h = (int)std::ceil(short_e + 2.0f * pad_y);
  cv::Rect roi(x, y, w, h);
  roi &= cv::Rect(0, 0, rotated.cols, rotated.rows);
  if (roi.width < 32 || roi.height < 16) return {};
  crop_offset_in_rotated = cv::Point2f((float)roi.x, (float)roi.y);
  return rotated(roi).clone();
}

struct BarcodeBox { int x0, y0, x1, y1, x2, y2, x3, y3; bool decoded; };

}  // namespace

std::string toHex(const std::vector<uint8_t>& bytes) {
  static const char* digits = "0123456789abcdef";
  std::string out;
  out.reserve(bytes.size() * 3);
  for (size_t i = 0; i < bytes.size(); ++i) {
    if (i) out.push_back(' ');
    out.push_back(digits[bytes[i] >> 4]);
    out.push_back(digits[bytes[i] & 0x0f]);
  }
  return out;
}

DecodeOutput decodeBarcodes(const cv::Mat& gray_in, const TagPose* tag, const CameraModel& camera,
                            const DecoderConfig& config, const DecodeHooks& hooks) {
  if (gray_in.empty() || gray_in.type() != CV_8UC1)
    throw std::invalid_argument("decodeBarcodes expects a non-empty CV_8UC1 image");
  DecodeOutput out;
  const LegacyCfg cfg = make_legacy(config, camera);
  const Logger vlogf(hooks.log);
  cv::Mat gray = gray_in;  // shared header; strategies never write into it

  // r->found in Drone-Vision: a tag with a valid pose. ROI projection also needs intrinsics.
  const bool tag_roi = tag != nullptr && camera.valid();
  cv::Mat rv, tv;
  if (tag_roi) {
    rv = (cv::Mat_<double>(3, 1) << tag->rvec[0], tag->rvec[1], tag->rvec[2]);
    tv = (cv::Mat_<double>(3, 1) << tag->tvec[0], tag->tvec[1], tag->tvec[2]);
  }
  bool cancelled = false;
  auto check_cancel = [&]() {
    if (!cancelled && hooks.cancelled && hooks.cancelled()) cancelled = true;
    return cancelled;
  };
  std::vector<BarcodeBox> barcode_boxes;

  using namespace ZXing;
  long long t_start = ms_now();

  vlogf("[Vision] decode_input=%dx%d tag_roi=%d expected{QR=%d DM=%d C128=%d C39=%d EAN13=%d ITF=%d}\n",
        gray.cols, gray.rows, (int)tag_roi,
        cfg.exp_qr, cfg.exp_dm, cfg.exp_128, cfg.exp_39, cfg.exp_ean13, cfg.exp_itf);

  // ── 1. 构建 1D / 2D 分离格式集（避免混合解码互相干扰） ──
  std::vector<BarcodeFormat> en2, en1, enAll;
  auto add_fmt = [&](bool on, BarcodeFormat f, bool is2d) {
    if (!on) return;
    enAll.push_back(f);
    (is2d ? en2 : en1).push_back(f);
  };
  add_fmt(cfg.bc_qr,     BarcodeFormat::QRCode,     true);
  add_fmt(cfg.bc_dm,     BarcodeFormat::DataMatrix, true);
  add_fmt(cfg.bc_aztec,  BarcodeFormat::Aztec,      true);
  add_fmt(cfg.bc_pdf417, BarcodeFormat::PDF417,     true);
  add_fmt(cfg.bc_128,    BarcodeFormat::Code128,    false);
  add_fmt(cfg.bc_39,     BarcodeFormat::Code39,     false);
  add_fmt(cfg.bc_ean13,  BarcodeFormat::EAN13,      false);
  add_fmt(cfg.bc_itf,    BarcodeFormat::ITF,        false);

  BarcodeFormats fmts_2d(std::move(en2));
  BarcodeFormats fmts_1d(std::move(en1));
  BarcodeFormats fmts_all(std::move(enAll));

  if (!cfg.barcode_enabled) {
    out.error = "disabled";
    vlogf("[Vision] Barcode: SKIPPED (disabled)\n");
    out.elapsed_ms = (double)(ms_now() - t_start);
    return out;
  }
  if (fmts_all.empty()) {
    out.error = "no_formats";
    vlogf("[Vision] Barcode: SKIPPED (no formats)\n");
    out.elapsed_ms = (double)(ms_now() - t_start);
    return out;
  }
  vlogf("[Vision] Multi-decode: 2D=%d 1D=%d\n",
        (int)!fmts_2d.empty(), (int)!fmts_1d.empty());

  // ── 2. 全帧灰度图 ──
  cv::Mat full_gray = gray;

  // ── 3. 结果收集与 Result Fusion ──
  std::vector<barcode_fusion::DecodeResult> detected;
  barcode_fusion::DecodeResultFusion result_fusion;

  struct StrategyStat {
    int attempts = 0;
    int raw_success = 0;
    int distinct_success = 0;
    long long elapsed_ms = 0;
  };
  std::map<std::string, StrategyStat> strategy_stats;
  int locator_decode_failed = 0;
  std::map<ZXing::BarcodeFormat, int> decode_failed_by_format;

  auto to_fusion_kind = [](ZXing::BarcodeFormat f) -> barcode_fusion::BarcodeKind {
    switch (f) {
      case ZXing::BarcodeFormat::QRCode:     return barcode_fusion::BarcodeKind::QRCode;
      case ZXing::BarcodeFormat::DataMatrix: return barcode_fusion::BarcodeKind::DataMatrix;
      case ZXing::BarcodeFormat::Code128:    return barcode_fusion::BarcodeKind::Code128;
      case ZXing::BarcodeFormat::Code39:     return barcode_fusion::BarcodeKind::Code39;
      case ZXing::BarcodeFormat::EAN13:      return barcode_fusion::BarcodeKind::EAN13;
      case ZXing::BarcodeFormat::PDF417:     return barcode_fusion::BarcodeKind::PDF417;
      case ZXing::BarcodeFormat::Aztec:      return barcode_fusion::BarcodeKind::Aztec;
      case ZXing::BarcodeFormat::ITF:        return barcode_fusion::BarcodeKind::ITF;
      default:                               return barcode_fusion::BarcodeKind::Unknown;
    }
  };

  auto fusion_kind_name = [](barcode_fusion::BarcodeKind kind) -> const char* {
    switch (kind) {
      case barcode_fusion::BarcodeKind::QRCode:     return "QRCode";
      case barcode_fusion::BarcodeKind::DataMatrix: return "DataMatrix";
      case barcode_fusion::BarcodeKind::Code128:    return "Code128";
      case barcode_fusion::BarcodeKind::Code39:     return "Code39";
      case barcode_fusion::BarcodeKind::EAN13:      return "EAN-13";
      case barcode_fusion::BarcodeKind::PDF417:     return "PDF417";
      case barcode_fusion::BarcodeKind::Aztec:      return "Aztec";
      case barcode_fusion::BarcodeKind::ITF:        return "ITF";
      default:                                      return "Unknown";
    }
  };

  // Distinct result accepted by fusion: keep the fusion copy (counts) and the exported code.
  auto accept = [&](const barcode_fusion::DecodeResult& fused, const Barcode& res) {
    detected.push_back(fused);
    DecodedCode code;
    code.format = fusion_kind_name(fused.format);
    code.code = fused.text;
    code.bytes = res.bytes();
    code.symbology_id = res.symbologyIdentifier();
    for (int i = 0; i < 4; ++i) code.corners[i] = cv::Point2f(fused.corners[i].x, fused.corners[i].y);
    code.center = cv::Point2f(fused.center.x, fused.center.y);
    code.source = fused.source;
    out.codes.push_back(std::move(code));
  };

  auto add_unique = [&](const Barcodes& results, double scl = 1.0,
                        int off_x = 0, int off_y = 0,
                        const char* source = "UNKNOWN") {
    for (auto& res : results) {
      if (!res.isValid()) continue;
      ZXing::BarcodeFormat fmt = res.format();
      auto pos = res.position();

      double min_x = std::numeric_limits<double>::max();
      double max_x = std::numeric_limits<double>::lowest();
      double min_y = std::numeric_limits<double>::max();
      double max_y = std::numeric_limits<double>::lowest();
      double cx = 0.0, cy = 0.0;
      std::vector<cv::Point2f> corners(4);
      for (int i = 0; i < 4; ++i) {
        corners[i].x = static_cast<float>(pos[i].x * scl + off_x);
        corners[i].y = static_cast<float>(pos[i].y * scl + off_y);
        cx += corners[i].x;
        cy += corners[i].y;
        min_x = std::min(min_x, static_cast<double>(corners[i].x));
        max_x = std::max(max_x, static_cast<double>(corners[i].x));
        min_y = std::min(min_y, static_cast<double>(corners[i].y));
        max_y = std::max(max_y, static_cast<double>(corners[i].y));
      }
      cx /= 4.0;
      cy /= 4.0;

      barcode_fusion::DecodeResult raw;
      raw.format = to_fusion_kind(fmt);
      raw.text = res.text();
      raw.center = {static_cast<float>(cx), static_cast<float>(cy)};
      raw.width = static_cast<float>(std::max(1.0, max_x - min_x));
      raw.height = static_cast<float>(std::max(1.0, max_y - min_y));
      raw.corners = {{
          {corners[0].x, corners[0].y},
          {corners[1].x, corners[1].y},
          {corners[2].x, corners[2].y},
          {corners[3].x, corners[3].y}
      }};
      raw.source = source;
      raw.valid = true;

      StrategyStat& stat = strategy_stats[source];
      stat.raw_success++;

      auto outcome = result_fusion.add(raw);
      if (outcome.duplicate) {
        vlogf("[Vision] Fusion duplicate [%s] %s center=(%.1f,%.1f) source=%s "
              "dist=%.1f iou=%.2f ref=%.1f\n",
              fusion_kind_name(raw.format), raw.text.c_str(), raw.center.x, raw.center.y,
              source, outcome.center_distance, outcome.iou, outcome.reference_long_edge);
        continue;
      } else if (outcome.same_symbol_candidate) {
        vlogf("[Vision] Fusion same-symbol kept [%s] %s center=(%.1f,%.1f) source=%s "
              "dist=%.1f iou=%.2f ref=%.1f\n",
              fusion_kind_name(raw.format), raw.text.c_str(), raw.center.x, raw.center.y,
              source, outcome.center_distance, outcome.iou, outcome.reference_long_edge);
        continue;
      }

      accept(outcome.fused, res);
      stat.distinct_success++;
      vlogf("[Vision] Multi: [%s] %s center=(%.1f,%.1f) source=%s\n",
            fusion_kind_name(outcome.fused.format), outcome.fused.text.c_str(),
            outcome.fused.center.x, outcome.fused.center.y, source);

      if (!(pos[0].x == 0 && pos[0].y == 0 &&
            pos[1].x == 0 && pos[1].y == 0)) {
        barcode_boxes.push_back({
            (int)(pos[0].x * scl + off_x), (int)(pos[0].y * scl + off_y),
            (int)(pos[1].x * scl + off_x), (int)(pos[1].y * scl + off_y),
            (int)(pos[2].x * scl + off_x), (int)(pos[2].y * scl + off_y),
            (int)(pos[3].x * scl + off_x), (int)(pos[3].y * scl + off_y),
            true});
      }
    }
  };

  // Rescue 图经过“原图 ROI -> 旋转摆正 -> bbox crop -> 可选横向放大”，用 affine 逆映射融合。
  auto add_unique_affine = [&](const Barcodes& results,
                               const cv::Mat& original_to_rotated,
                               cv::Point2f crop_offset,
                               double scale_x, double scale_y,
                               int roi_off_x, int roi_off_y,
                               const char* source) {
    cv::Mat inv;
    cv::invertAffineTransform(original_to_rotated, inv);
    for (auto& res : results) {
      if (!res.isValid()) continue;
      ZXing::BarcodeFormat fmt = res.format();
      auto pos = res.position();
      double min_x = std::numeric_limits<double>::max();
      double max_x = std::numeric_limits<double>::lowest();
      double min_y = std::numeric_limits<double>::max();
      double max_y = std::numeric_limits<double>::lowest();
      double cx = 0.0, cy = 0.0;
      std::vector<cv::Point2f> corners(4);
      for (int i = 0; i < 4; ++i) {
        double rx = pos[i].x / scale_x + crop_offset.x;
        double ry = pos[i].y / scale_y + crop_offset.y;
        double ox = inv.at<double>(0, 0) * rx + inv.at<double>(0, 1) * ry + inv.at<double>(0, 2);
        double oy = inv.at<double>(1, 0) * rx + inv.at<double>(1, 1) * ry + inv.at<double>(1, 2);
        corners[i].x = (float)(ox + roi_off_x);
        corners[i].y = (float)(oy + roi_off_y);
        cx += corners[i].x; cy += corners[i].y;
        min_x = std::min(min_x, (double)corners[i].x);
        max_x = std::max(max_x, (double)corners[i].x);
        min_y = std::min(min_y, (double)corners[i].y);
        max_y = std::max(max_y, (double)corners[i].y);
      }
      cx /= 4.0; cy /= 4.0;
      barcode_fusion::DecodeResult raw;
      raw.format = to_fusion_kind(fmt);
      raw.text = res.text();
      raw.center = {(float)cx, (float)cy};
      raw.width = (float)std::max(1.0, max_x - min_x);
      raw.height = (float)std::max(1.0, max_y - min_y);
      raw.corners = {{{corners[0].x, corners[0].y}, {corners[1].x, corners[1].y},
                      {corners[2].x, corners[2].y}, {corners[3].x, corners[3].y}}};
      raw.source = source; raw.valid = true;
      StrategyStat& stat = strategy_stats[source];
      stat.raw_success++;
      auto outcome = result_fusion.add(raw);

      if (outcome.duplicate) {
        vlogf("[Vision] Fusion duplicate [%s] %s center=(%.1f,%.1f) source=%s "
              "dist=%.1f iou=%.2f ref=%.1f [RESCUE-BLOCKED]\n",
              fusion_kind_name(raw.format), raw.text.c_str(), raw.center.x, raw.center.y,
              source, outcome.center_distance, outcome.iou, outcome.reference_long_edge);
        continue;
      }
      if (outcome.same_symbol_candidate) {
        vlogf("[Vision] Fusion same-symbol kept [%s] %s center=(%.1f,%.1f) source=%s [RESCUE-NOT-EXPORTED]\n",
              fusion_kind_name(raw.format), raw.text.c_str(), raw.center.x, raw.center.y, source);
        continue;
      }

      accept(outcome.fused, res);
      stat.distinct_success++;
      barcode_boxes.push_back({
          (int)corners[0].x, (int)corners[0].y, (int)corners[1].x, (int)corners[1].y,
          (int)corners[2].x, (int)corners[2].y, (int)corners[3].x, (int)corners[3].y, true});
      vlogf("[Vision] Rescue: [%s] %s center=(%.1f,%.1f) source=%s\n",
            fusion_kind_name(outcome.fused.format), outcome.fused.text.c_str(),
            outcome.fused.center.x, outcome.fused.center.y, source);
    }
  };

  auto record_decode_failed = [&](const Barcode& res) {
    const auto pos = res.position();
    if (pos[0].x == 0 && pos[0].y == 0 &&
        pos[1].x == 0 && pos[1].y == 0 &&
        pos[2].x == 0 && pos[2].y == 0 &&
        pos[3].x == 0 && pos[3].y == 0) {
      return;
    }
    locator_decode_failed++;
    decode_failed_by_format[res.format()]++;
    vlogf("[Vision] Locator found [%s] but decode failed\n", barcode_format_name(res.format()));
  };

  // expected > 0：有明确数量目标，达到后允许早退；expected <= 0：自由扫描，不因此判定 done。
  const bool has_required_2d =
      (cfg.bc_qr     && cfg.exp_qr > 0) ||
      (cfg.bc_dm     && cfg.exp_dm > 0) ||
      (cfg.bc_aztec  && cfg.exp_aztec > 0) ||
      (cfg.bc_pdf417 && cfg.exp_pdf417 > 0);
  const bool has_required_1d =
      (cfg.bc_128   && cfg.exp_128 > 0) ||
      (cfg.bc_39    && cfg.exp_39 > 0) ||
      (cfg.bc_ean13 && cfg.exp_ean13 > 0) ||
      (cfg.bc_itf   && cfg.exp_itf > 0);

  auto found_count = [&](ZXing::BarcodeFormat f) {
    int c = 0;
    barcode_fusion::BarcodeKind kind = to_fusion_kind(f);
    for (auto& e : detected) if (e.format == kind) c++;
    return c;
  };
  auto req_done = [&](bool enabled, int req, ZXing::BarcodeFormat f) {
    if (!enabled || req <= 0) return true;
    return found_count(f) >= req;
  };
  auto done_2d = [&]() {
    if (fmts_2d.empty()) return true;
    if (!has_required_2d) return false;
    if (!req_done(cfg.bc_qr,     cfg.exp_qr,     ZXing::BarcodeFormat::QRCode))     return false;
    if (!req_done(cfg.bc_dm,     cfg.exp_dm,     ZXing::BarcodeFormat::DataMatrix)) return false;
    if (!req_done(cfg.bc_aztec,  cfg.exp_aztec,  ZXing::BarcodeFormat::Aztec))      return false;
    if (!req_done(cfg.bc_pdf417, cfg.exp_pdf417, ZXing::BarcodeFormat::PDF417))     return false;
    return true;
  };
  auto done_1d = [&]() {
    if (fmts_1d.empty()) return true;
    if (!has_required_1d) return false;
    if (!req_done(cfg.bc_128,   cfg.exp_128,   ZXing::BarcodeFormat::Code128)) return false;
    if (!req_done(cfg.bc_39,    cfg.exp_39,    ZXing::BarcodeFormat::Code39))  return false;
    if (!req_done(cfg.bc_ean13, cfg.exp_ean13, ZXing::BarcodeFormat::EAN13))   return false;
    if (!req_done(cfg.bc_itf,   cfg.exp_itf,   ZXing::BarcodeFormat::ITF))     return false;
    return true;
  };

  // 辅助：对单张图做 ReadBarcodes，坐标自动变换到原图系统
  auto scan_image = [&](cv::Mat& img, BarcodeFormats fmts,
                        bool try_harder, bool try_rotate,
                        bool use_binarizer, int min_lines,
                        const char* label,
                        double scl = 1.0, int off_x = 0, int off_y = 0) {
    if (fmts.empty() || img.empty() || check_cancel()) return;
    ImageView view(img.data, img.cols, img.rows, ImageFormat::Lum, (int)img.step);
    ReaderOptions opts = ReaderOptions()
        .setFormats(fmts)
        .setTryHarder(try_harder)
        .setTryRotate(try_rotate)
        .setMaxNumberOfSymbols(0)
        .setReturnErrors(true);
    if (min_lines > 0) opts.setMinLineCount(min_lines);
    if (use_binarizer) opts.setBinarizer(Binarizer::LocalAverage);
    long long t0 = ms_now();
    auto results = ReadBarcodes(view, opts);
    StrategyStat& stat = strategy_stats[label];
    stat.attempts++;
    add_unique(results, scl, off_x, off_y, label);
    // 红色框只画 ROI 内定位到但解码失败的候选，并按中心距离去重。
    for (auto& res : results) {
      auto pos = res.position();
      if (pos[0].x == 0 && pos[0].y == 0 &&
          pos[1].x == 0 && pos[1].y == 0) continue;
      if (res.isValid()) continue;
      if (std::strncmp(label, "ROI", 3) != 0) continue;
      int ccx = (int)(((pos[0].x + pos[1].x + pos[2].x + pos[3].x) / 4.0) * scl + off_x);
      int ccy = (int)(((pos[0].y + pos[1].y + pos[2].y + pos[3].y) / 4.0) * scl + off_y);
      bool dup_red = false;
      for (auto& b : barcode_boxes) {
        if (b.decoded) continue;
        int bcx = (b.x0 + b.x1 + b.x2 + b.x3) / 4;
        int bcy = (b.y0 + b.y1 + b.y2 + b.y3) / 4;
        if (std::abs(bcx - ccx) < 20 && std::abs(bcy - ccy) < 20) {
          dup_red = true;
          break;
        }
      }
      if (!dup_red) {
        barcode_boxes.push_back({
            (int)(pos[0].x * scl + off_x), (int)(pos[0].y * scl + off_y),
            (int)(pos[1].x * scl + off_x), (int)(pos[1].y * scl + off_y),
            (int)(pos[2].x * scl + off_x), (int)(pos[2].y * scl + off_y),
            (int)(pos[3].x * scl + off_x), (int)(pos[3].y * scl + off_y),
            false});
      }
      record_decode_failed(res);
    }
    long long this_ms = ms_now() - t0;
    stat.elapsed_ms += this_ms;
    vlogf("[Vision] T %s: %lldms\n", label, this_ms);
  };

  // ── 4. 金字塔式多级解码：ROI → 1280 → 1920 → 全分辨率，按码型早退 ──
  auto type_done = [&](bool is2d) {
    return is2d ? done_2d() : done_1d();
  };
  bool has_required_any =
      (cfg.bc_qr && cfg.exp_qr > 0) ||
      (cfg.bc_dm && cfg.exp_dm > 0) ||
      (cfg.bc_aztec && cfg.exp_aztec > 0) ||
      (cfg.bc_pdf417 && cfg.exp_pdf417 > 0) ||
      (cfg.bc_128 && cfg.exp_128 > 0) ||
      (cfg.bc_39 && cfg.exp_39 > 0) ||
      (cfg.bc_ean13 && cfg.exp_ean13 > 0) ||
      (cfg.bc_itf && cfg.exp_itf > 0);
  auto all_done = [&]() {
    if (!has_required_any)
      return false;
    return type_done(true) && type_done(false);
  };

  // 对一张图跑一层（img2d 供 2D，img1d 供 1D；fast=true 时 tryHarder=false）
  auto run_layer_on = [&](const char* tag_name, cv::Mat& img2d, cv::Mat& img1d,
                          double scl, int off_x, int off_y, bool fast) {
    if (check_cancel()) return;
    long long l0 = ms_now();
    char lb[96];

    // 2D: 原图 → CLAHE
    if (!type_done(true) && !fmts_2d.empty()) {
      std::snprintf(lb, sizeof(lb), "%s-2d", tag_name);
      scan_image(img2d, fmts_2d, !fast, cfg.bc_try_rotate, false, 0, lb, scl, off_x, off_y);
      if (!type_done(true) && cfg.bc_preproc_enabled && cfg.bc_clahe_enabled) {
        cv::Mat clahe_img;
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
            cfg.bc_clahe_clip, cv::Size(cfg.bc_clahe_grid, cfg.bc_clahe_grid));
        clahe->apply(img2d, clahe_img);
        std::snprintf(lb, sizeof(lb), "%s-2d-clahe", tag_name);
        scan_image(clahe_img, fmts_2d, !fast, cfg.bc_try_rotate, false, 0, lb, scl, off_x, off_y);
      }
    }

    // 1D：先原始亮度图（默认 binarizer），再 LocalAverage。
    if (!type_done(false) && !fmts_1d.empty() && !img1d.empty()) {
      std::snprintf(lb, sizeof(lb), "%s-1d-raw", tag_name);
      scan_image(img1d, fmts_1d, !fast, cfg.bc_try_rotate, false, 1, lb, scl, off_x, off_y);
      if (!type_done(false)) {
        std::snprintf(lb, sizeof(lb), "%s-1d-local", tag_name);
        scan_image(img1d, fmts_1d, !fast, cfg.bc_try_rotate, true, 1, lb, scl, off_x, off_y);
      }
      if (!type_done(false)) {
        cv::Mat bin_img;
        double otsu_th = cv::threshold(img1d, bin_img, 0, 255,
                                       cv::THRESH_BINARY | cv::THRESH_OTSU);
        vlogf("[Vision] OTSU[%s] threshold=%.0f\n", tag_name, otsu_th);
        std::snprintf(lb, sizeof(lb), "%s-1d-otsu", tag_name);
        scan_image(bin_img, fmts_1d, !fast, false, false, 1, lb, scl, off_x, off_y);
        if (!type_done(false)) {
          cv::Mat adaptive = barcode_1d_adaptive_threshold(img1d);
          std::snprintf(lb, sizeof(lb), "%s-1d-adaptive", tag_name);
          scan_image(adaptive, fmts_1d, !fast, false, false, 1, lb, scl, off_x, off_y);
        }
        // Sobel 只保留为最后兜底：边缘图不是标准 barcode luminance 图。
        if (!type_done(false)) {
          cv::Mat enh = barcode_1d_sobel_adaptive(img1d);
          std::snprintf(lb, sizeof(lb), "%s-1d-sobel", tag_name);
          scan_image(enh, fmts_1d, !fast, false, false, 1, lb, scl, off_x, off_y);
        }
      }
    }

    vlogf("[Vision] Layer %s: %lldms (2D缺=%d 1D缺=%d)\n", tag_name, ms_now() - l0,
          (int)!type_done(true), (int)!type_done(false));
  };

  // 金字塔主流程：ROI → 1280(快) → 1920(细) → 全分辨率(最后防线)
  auto run_pyramid = [&](cv::Mat& src, const char* tag_name, bool with_roi) {
    // ── 4.1 AprilTag ROI 裁切解码（最高优先级，最快） ──
    if (with_roi && tag_roi && !check_cancel()) {
      // ROI 覆盖整张标签纸并留 roi_scale 倍余量，保持标签真实物理宽高比。
      double lw = cfg.label_width_meters;
      double lh = cfg.label_height_meters;
      double half_roi_w = lw * cfg.roi_scale / 2.0;
      double half_roi_h = lh * cfg.roi_scale / 2.0;
      double off_x = 0.0, off_y = 0.0;
      // 约定: tag 顶边 = 标签顶边，+y = tag 正立方向（打印方向）。
      if (std::strcmp(cfg.tag_position, "top_left") == 0 ||
          std::strcmp(cfg.tag_position, "左上") == 0) {
        off_x =  lw / 2.0; off_y = -lh / 2.0;
      } else if (std::strcmp(cfg.tag_position, "top_right") == 0 ||
                 std::strcmp(cfg.tag_position, "右上") == 0) {
        off_x = -lw / 2.0; off_y = -lh / 2.0;
      } else if (std::strcmp(cfg.tag_position, "bottom_left") == 0 ||
                 std::strcmp(cfg.tag_position, "左下") == 0) {
        off_x =  lw / 2.0; off_y =  lh / 2.0;
      } else if (std::strcmp(cfg.tag_position, "bottom_right") == 0 ||
                 std::strcmp(cfg.tag_position, "右下") == 0) {
        off_x = -lw / 2.0; off_y =  lh / 2.0;
      } else if (std::strcmp(cfg.tag_position, "custom") == 0 ||
                 std::strcmp(cfg.tag_position, "自定义") == 0) {
        // tag_offset = tag 中心相对标签中心；标签中心相对 tag 中心取反。
        off_x = -cfg.tag_offset_x_meters; off_y = -cfg.tag_offset_y_meters;
      }
      std::vector<cv::Point3f> roi_3d = {
          cv::Point3f(-half_roi_w + off_x, -half_roi_h + off_y, 0),
          cv::Point3f( half_roi_w + off_x, -half_roi_h + off_y, 0),
          cv::Point3f( half_roi_w + off_x,  half_roi_h + off_y, 0),
          cv::Point3f(-half_roi_w + off_x,  half_roi_h + off_y, 0)
      };
      std::vector<cv::Point2f> roi_2d;
      cv::projectPoints(roi_3d, rv, tv, cfg.camera_matrix, cfg.dist_coeffs, roi_2d);
      out.roi_quad = roi_2d;
      out.used_tag_roi = true;

      // ════ Homography 透视矫正：2D 1024×1024 / 1D 按标签比例长边 2048 ════
      if (roi_2d.size() == 4) {
        const int kWarp2D = 1024;
        const double roi_aspect = std::max(0.1, (lw * cfg.roi_scale) / (lh * cfg.roi_scale));
        const int kWarp1DLong = 2048;
        const int kWarp1DW = roi_aspect >= 1.0 ? kWarp1DLong :
            std::max(256, (int)std::lround(kWarp1DLong * roi_aspect));
        const int kWarp1DH = roi_aspect >= 1.0 ?
            std::max(256, (int)std::lround(kWarp1DLong / roi_aspect)) : kWarp1DLong;

        auto make_warp = [&](const cv::Size& warp_size, int interpolation,
                             cv::Mat& warped, cv::Mat& H_inv) -> bool {
          // roi_2d 按 tag 系 (-x,-y) 起逆时针：左下、右下、右上、左上（+y = 标签正立向上）。
          // 映射到矫正图同名角，使矫正图与标签正立方向一致；原实现映射到左上起，图像上下镜像。
          std::vector<cv::Point2f> dst_quad = {
              cv::Point2f(0.0f, (float)(warp_size.height - 1)),
              cv::Point2f((float)(warp_size.width - 1), (float)(warp_size.height - 1)),
              cv::Point2f((float)(warp_size.width - 1), 0.0f),
              cv::Point2f(0.0f, 0.0f)
          };
          cv::Mat H = cv::getPerspectiveTransform(roi_2d, dst_quad);
          if (H.empty()) {
            vlogf("[Vision] Homography failed: getPerspectiveTransform (warp=%dx%d)\n",
                  warp_size.width, warp_size.height);
            return false;
          }
          cv::invert(H, H_inv);
          if (H_inv.empty()) {
            vlogf("[Vision] Homography failed: invert (warp=%dx%d)\n", warp_size.width, warp_size.height);
            return false;
          }
          cv::warpPerspective(src, warped, H, warp_size, interpolation, cv::BORDER_REPLICATE);
          if (warped.empty()) {
            vlogf("[Vision] Homography warp failed (warp=%dx%d)\n", warp_size.width, warp_size.height);
            return false;
          }
          return true;
        };

        // 对透视矫正后的图像解码，并将 ZXing 返回的坐标映射回原始相机图像。
        auto scan_warped = [&](cv::Mat& wimg, BarcodeFormats fmts, const char* label,
                               bool try_rotate, const cv::Mat& H_inv,
                               bool use_binarizer, int min_lines,
                               double coord_scale = 1.0) {
          if (fmts.empty() || wimg.empty() || check_cancel()) return;

          long long t0 = ms_now();
          ImageView view(wimg.data, wimg.cols, wimg.rows, ImageFormat::Lum, (int)wimg.step);
          ReaderOptions opts = ReaderOptions()
              .setFormats(fmts)
              .setTryHarder(true)
              .setTryRotate(try_rotate)
              .setMaxNumberOfSymbols(0)
              .setReturnErrors(true);
          if (min_lines > 0) opts.setMinLineCount(min_lines);
          if (use_binarizer) opts.setBinarizer(Binarizer::LocalAverage);

          auto results = ReadBarcodes(view, opts);
          StrategyStat& stat = strategy_stats[label];
          stat.attempts++;

          for (auto& res : results) {
            if (!res.isValid()) {
              record_decode_failed(res);
              continue;
            }
            ZXing::BarcodeFormat fmt = res.format();
            auto pos = res.position();

            std::vector<cv::Point2f> s4(4), d4(4);
            for (int i = 0; i < 4; ++i) {
              s4[i] = cv::Point2f((float)(pos[i].x * coord_scale), (float)(pos[i].y * coord_scale));
            }
            cv::perspectiveTransform(s4, d4, H_inv);

            double min_x = std::numeric_limits<double>::max();
            double max_x = std::numeric_limits<double>::lowest();
            double min_y = std::numeric_limits<double>::max();
            double max_y = std::numeric_limits<double>::lowest();
            double cx = 0.0, cy = 0.0;
            for (int i = 0; i < 4; ++i) {
              cx += d4[i].x;
              cy += d4[i].y;
              min_x = std::min(min_x, static_cast<double>(d4[i].x));
              max_x = std::max(max_x, static_cast<double>(d4[i].x));
              min_y = std::min(min_y, static_cast<double>(d4[i].y));
              max_y = std::max(max_y, static_cast<double>(d4[i].y));
            }
            cx /= 4.0;
            cy /= 4.0;

            barcode_fusion::DecodeResult raw;
            raw.format = to_fusion_kind(fmt);
            raw.text = res.text();
            raw.center = {static_cast<float>(cx), static_cast<float>(cy)};
            raw.width = static_cast<float>(std::max(1.0, max_x - min_x));
            raw.height = static_cast<float>(std::max(1.0, max_y - min_y));
            raw.corners = {{
                {d4[0].x, d4[0].y}, {d4[1].x, d4[1].y},
                {d4[2].x, d4[2].y}, {d4[3].x, d4[3].y}
            }};
            raw.source = label;
            raw.valid = true;
            stat.raw_success++;

            auto outcome = result_fusion.add(raw);
            if (outcome.duplicate) {
              vlogf("[Vision] Fusion duplicate [%s] %s center=(%.1f,%.1f) source=%s "
                    "dist=%.1f iou=%.2f ref=%.1f\n",
                    fusion_kind_name(raw.format), raw.text.c_str(), raw.center.x, raw.center.y,
                    label, outcome.center_distance, outcome.iou, outcome.reference_long_edge);
              continue;
            } else if (outcome.same_symbol_candidate) {
              vlogf("[Vision] Fusion same-symbol kept [%s] %s center=(%.1f,%.1f) source=%s "
                    "dist=%.1f iou=%.2f ref=%.1f\n",
                    fusion_kind_name(raw.format), raw.text.c_str(), raw.center.x, raw.center.y,
                    label, outcome.center_distance, outcome.iou, outcome.reference_long_edge);
              continue;
            }

            accept(outcome.fused, res);
            stat.distinct_success++;
            vlogf("[Vision] Multi: [%s] %s center=(%.1f,%.1f) source=%s\n",
                  fusion_kind_name(outcome.fused.format), outcome.fused.text.c_str(),
                  outcome.fused.center.x, outcome.fused.center.y, label);

            if (!(pos[0].x == 0 && pos[0].y == 0 &&
                  pos[1].x == 0 && pos[1].y == 0)) {
              barcode_boxes.push_back({
                  (int)d4[0].x, (int)d4[0].y, (int)d4[1].x, (int)d4[1].y,
                  (int)d4[2].x, (int)d4[2].y, (int)d4[3].x, (int)d4[3].y, true});
            }
          }

          long long this_ms = ms_now() - t0;
          stat.elapsed_ms += this_ms;
          vlogf("[Vision] T %s: %lldms\n", label, this_ms);
        };

        // 2D：1024 Warp
        if (!type_done(true) && !fmts_2d.empty() && !check_cancel()) {
          cv::Mat warped_2d, H_inv_2d;
          if (make_warp(cv::Size(kWarp2D, kWarp2D), cv::INTER_LINEAR, warped_2d, H_inv_2d)) {
            scan_warped(warped_2d, fmts_2d, "WARP-2d", cfg.bc_try_rotate, H_inv_2d, false, 0);
            if (!type_done(true) && cfg.bc_preproc_enabled && cfg.bc_clahe_enabled) {
              cv::Mat warped_2d_clahe;
              cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
                  cfg.bc_clahe_clip, cv::Size(cfg.bc_clahe_grid, cfg.bc_clahe_grid));
              clahe->apply(warped_2d, warped_2d_clahe);
              scan_warped(warped_2d_clahe, fmts_2d, "WARP-2d-clahe", cfg.bc_try_rotate,
                          H_inv_2d, false, 0);
            }
          }
        }

        // 1D：2048 高采样 Warp
        if (!type_done(false) && !fmts_1d.empty() && !check_cancel()) {
          cv::Mat warped_1d, H_inv_1d;
          // 主路径：LINEAR Warp + LocalAverage。
          if (make_warp(cv::Size(kWarp1DW, kWarp1DH), cv::INTER_LINEAR, warped_1d, H_inv_1d)) {
            scan_warped(warped_1d, fmts_1d, "WARP-1d-hi-linear-raw", cfg.bc_try_rotate,
                        H_inv_1d, false, 1);
            if (!type_done(false)) {
              scan_warped(warped_1d, fmts_1d, "WARP-1d-hi-linear-local", cfg.bc_try_rotate,
                          H_inv_1d, true, 1);
            }
            // OTSU 兜底。
            if (!type_done(false)) {
              cv::Mat warped_1d_otsu;
              double otsu_th = cv::threshold(warped_1d, warped_1d_otsu, 0, 255,
                                             cv::THRESH_BINARY | cv::THRESH_OTSU);
              vlogf("[Vision] OTSU[WARP-1d-hi-linear] threshold=%.0f\n", otsu_th);
              scan_warped(warped_1d_otsu, fmts_1d, "WARP-1d-hi-linear-otsu", false,
                          H_inv_1d, false, 1);
            }
            // 局部光照不均时先尝试标准灰度自适应二值化。
            if (!type_done(false)) {
              cv::Mat warped_1d_adaptive = barcode_1d_adaptive_threshold(warped_1d);
              scan_warped(warped_1d_adaptive, fmts_1d, "WARP-1d-hi-linear-adaptive",
                          false, H_inv_1d, false, 1);
            }
            // Sobel 仅最后兜底。
            if (!type_done(false)) {
              cv::Mat warped_1d_sobel = barcode_1d_sobel_adaptive(warped_1d);
              scan_warped(warped_1d_sobel, fmts_1d, "WARP-1d-hi-linear-sobel",
                          false, H_inv_1d, false, 1);
            }
          }

          // 第二条独立重采样路径：INTER_CUBIC，只有 LINEAR 路径仍未补齐 1D 时才运行。
          if (!type_done(false) && !check_cancel()) {
            cv::Mat warped_1d_cubic, H_inv_1d_cubic;
            if (make_warp(cv::Size(kWarp1DW, kWarp1DH), cv::INTER_CUBIC,
                          warped_1d_cubic, H_inv_1d_cubic)) {
              scan_warped(warped_1d_cubic, fmts_1d, "WARP-1d-hi-cubic-raw", cfg.bc_try_rotate,
                          H_inv_1d_cubic, false, 1);
              if (!type_done(false)) {
                scan_warped(warped_1d_cubic, fmts_1d, "WARP-1d-hi-cubic-local", cfg.bc_try_rotate,
                            H_inv_1d_cubic, true, 1);
              }
              if (!type_done(false)) {
                cv::Mat cubic_otsu;
                double otsu_th = cv::threshold(warped_1d_cubic, cubic_otsu, 0, 255,
                                               cv::THRESH_BINARY | cv::THRESH_OTSU);
                vlogf("[Vision] OTSU[WARP-1d-hi-cubic] threshold=%.0f\n", otsu_th);
                scan_warped(cubic_otsu, fmts_1d, "WARP-1d-hi-cubic-otsu", false,
                            H_inv_1d_cubic, false, 1);
              }
            }
          }
        }

        vlogf("[Vision] ROI warped decode done (2D=%d, 1D=%dx%d)\n", kWarp2D, kWarp1DW, kWarp1DH);
      }

      cv::Rect roi = cv::boundingRect(roi_2d);
      roi &= cv::Rect(0, 0, src.cols, src.rows);
      if (roi.width > 40 && roi.height > 40 && roi.area() > 0 && !check_cancel()) {
        cv::Mat roi_orig = src(roi);   // 原尺寸 ROI（1D 直读用，避免降采样损失条宽）
        cv::Mat roi_img = roi_orig;
        // 小 ROI 放大到至少 1024px，大 ROI 缩到 1024px
        double s_roi = 1024.0 / std::max(roi_img.cols, roi_img.rows);
        if (s_roi != 1.0) {
          // 降采样用 LINEAR（NEAREST 会抽坏小码模块），放大才用 NEAREST
          cv::resize(roi_img, roi_img, cv::Size(), s_roi, s_roi,
                     s_roi < 1.0 ? cv::INTER_LINEAR : cv::INTER_NEAREST);
        }
        double roi_scale = 1.0 / s_roi;
        int roi_off_x = roi.x, roi_off_y = roi.y;
        vlogf("[Vision] ROI: %dx%d -> %dx%d (scale=%.3f)\n",
              roi.width, roi.height, roi_img.cols, roi_img.rows, roi_scale);

        // 1D Candidate Locator + bbox padding + deskew + rescue（常规 ROI 1D 失败后才调用）。
        auto rescue_1d_candidates = [&]() {
          if (type_done(false) || fmts_1d.empty()) return;
          auto candidates = locate_1d_candidates(roi_orig);
          if (candidates.empty()) return;
          vlogf("[LOCATOR] 1D candidates=%d\n", (int)candidates.size());
          for (size_t ci = 0; ci < candidates.size() && !type_done(false); ++ci) {
            cv::Point2f rescue_off;
            cv::Mat rotM, rescue = crop_1d_candidate_deskew(roi_orig, candidates[ci], rescue_off, rotM);
            if (rescue.empty()) continue;
            cv::Mat rescue_scaled = rescue;
            if (rescue.cols < 900 && rescue.rows >= 16) {
              cv::resize(rescue, rescue_scaled, cv::Size(), 2.0, 1.0, cv::INTER_CUBIC);
            }
            const double sx = rescue_scaled.cols / (double)rescue.cols;
            const double sy = rescue_scaled.rows / (double)rescue.rows;
            auto scan_rescue = [&](cv::Mat& img, bool use_binarizer, const char* source) {
              if (check_cancel()) return;
              ImageView view(img.data, img.cols, img.rows, ImageFormat::Lum, (int)img.step);
              ReaderOptions opts = ReaderOptions()
                  .setFormats(fmts_1d).setTryHarder(true).setTryRotate(false)
                  .setMaxNumberOfSymbols(0).setReturnErrors(true).setMinLineCount(1);
              if (use_binarizer) opts.setBinarizer(Binarizer::LocalAverage);
              long long t0 = ms_now();
              auto results = ReadBarcodes(view, opts);
              StrategyStat& stat = strategy_stats[source];
              stat.attempts++;
              add_unique_affine(results, rotM, rescue_off, sx, sy, roi.x, roi.y, source);
              stat.elapsed_ms += ms_now() - t0;
              vlogf("[Vision] T %s: %lldms\n", source, ms_now() - t0);
            };
            char lb[96];
            std::snprintf(lb, sizeof(lb), "RESCUE-1d-%d-raw", (int)ci);
            scan_rescue(rescue_scaled, false, lb);
            if (!type_done(false)) {
              std::snprintf(lb, sizeof(lb), "RESCUE-1d-%d-local", (int)ci);
              scan_rescue(rescue_scaled, true, lb);
            }
            if (!type_done(false)) {
              cv::Mat adaptive = barcode_1d_adaptive_threshold(rescue_scaled);
              std::snprintf(lb, sizeof(lb), "RESCUE-1d-%d-adaptive", (int)ci);
              scan_rescue(adaptive, false, lb);
            }
            if (!type_done(false)) {
              cv::Mat otsu;
              cv::threshold(rescue_scaled, otsu, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
              std::snprintf(lb, sizeof(lb), "RESCUE-1d-%d-otsu", (int)ci);
              scan_rescue(otsu, false, lb);
            }
          }
        };

        // ROI 1D 原尺寸直读（不缩放；Code128 对降采样敏感，先给最有把握的一枪）
        if (!type_done(false) && !fmts_1d.empty()) {
          scan_image(roi_orig, fmts_1d, true, cfg.bc_try_rotate, false, 1,
                     "ROI-1d-fullres-raw", 1.0, roi.x, roi.y);
          if (!type_done(false)) {
            scan_image(roi_orig, fmts_1d, true, cfg.bc_try_rotate, true, 1,
                       "ROI-1d-fullres-local", 1.0, roi.x, roi.y);
          }
        }

        // ROI 原图（全格式，快速第一枪）
        scan_image(roi_img, fmts_all, true, cfg.bc_try_rotate, false, 0,
                   "ROI-raw", roi_scale, roi_off_x, roi_off_y);
        // ROI 1D: LocalAverage 优先
        if (!type_done(false) && !fmts_1d.empty()) {
          scan_image(roi_img, fmts_1d, true, cfg.bc_try_rotate, true, 1,
                     "ROI-1d-local", roi_scale, roi_off_x, roi_off_y);
        }
        // ROI 1D: OTSU 兜底
        if (!type_done(false) && !fmts_1d.empty()) {
          cv::Mat roi_bin;
          cv::threshold(roi_img, roi_bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
          scan_image(roi_bin, fmts_1d, true, false, false, 1,
                     "ROI-1d-otsu", roi_scale, roi_off_x, roi_off_y);
          if (!type_done(false)) {
            cv::Mat roi_adaptive = barcode_1d_adaptive_threshold(roi_img);
            scan_image(roi_adaptive, fmts_1d, true, false, false, 1,
                       "ROI-1d-adaptive", roi_scale, roi_off_x, roi_off_y);
          }
          // Sobel-X 仅最后兜底（金属反光/非双峰极端场景）。
          if (!type_done(false)) {
            cv::Mat roi_enh = barcode_1d_sobel_adaptive(roi_img);
            scan_image(roi_enh, fmts_1d, true, false, false, 1,
                       "ROI-1d-sobel", roi_scale, roi_off_x, roi_off_y);
          }
        }
        // 常规 1D 已经尝试失败后，才做 Candidate Locator → bbox padding → deskew → rescue。
        if (!type_done(false) && !fmts_1d.empty()) {
          rescue_1d_candidates();
        }

        // ROI 2D: CLAHE
        if (!type_done(true) && !fmts_2d.empty() &&
            cfg.bc_preproc_enabled && cfg.bc_clahe_enabled) {
          cv::Mat roi_clahe;
          cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
              cfg.bc_clahe_clip, cv::Size(cfg.bc_clahe_grid, cfg.bc_clahe_grid));
          clahe->apply(roi_img, roi_clahe);
          scan_image(roi_clahe, fmts_2d, true, cfg.bc_try_rotate, false, 0,
                     "ROI-2d-clahe", roi_scale, roi_off_x, roi_off_y);
        }
        // ROI 2D 小图通道：DM 对模块像素数敏感，缩到 roi_2d_scale（默认 512）使模块落在 ~3.5px。
        if (!type_done(true) && !fmts_2d.empty()) {
          cv::Mat roi_src = src(roi);
          cv::Mat roi_small;
          double s_small = (double)cfg.bc_roi_2d_scale / std::max(roi_src.cols, roi_src.rows);
          if (s_small < 1.0)
            cv::resize(roi_src, roi_small, cv::Size(), s_small, s_small, cv::INTER_LINEAR);
          else
            cv::resize(roi_src, roi_small, cv::Size(), s_small, s_small, cv::INTER_NEAREST);
          double roi_scale_small = 1.0 / s_small;
          scan_image(roi_small, fmts_2d, true, cfg.bc_try_rotate, false, 0,
                     "ROI-2d-small", roi_scale_small, roi.x, roi.y);
        }
      }
    }

    // ── 4.2 金字塔层：小图先跑（快），大图殿后（细） ──
    int long_side = std::max(src.cols, src.rows);

    if (long_side > 1280 && !all_done() && !check_cancel()) {
      double s = 1280.0 / long_side;
      double scl = 1.0 / s;
      cv::Mat img2d, img1d;
      cv::resize(src, img2d, cv::Size(), s, s, cv::INTER_LINEAR);
      img1d.release();  // 1D 不参与激进降采样层
      char lt[32];
      std::snprintf(lt, sizeof(lt), "%s1280", tag_name);
      vlogf("[Vision] %s: %dx%d (scl=%.2f)\n", lt, img2d.cols, img2d.rows, scl);
      run_layer_on(lt, img2d, img1d, scl, 0, 0, /*fast=*/true);
    }

    if (long_side > 1920 && !all_done() && !check_cancel()) {
      double s = 1920.0 / long_side;
      double scl = 1.0 / s;
      cv::Mat img2d, img1d;
      cv::resize(src, img2d, cv::Size(), s, s, cv::INTER_LINEAR);
      img1d.release();
      char lt[32];
      std::snprintf(lt, sizeof(lt), "%s1920", tag_name);
      vlogf("[Vision] %s: %dx%d (scl=%.2f)\n", lt, img2d.cols, img2d.rows, scl);
      run_layer_on(lt, img2d, img1d, scl, 0, 0, /*fast=*/false);
    }

    if (!all_done()) {
      char lt[32];
      std::snprintf(lt, sizeof(lt), "%sFULL", tag_name);
      run_layer_on(lt, src, src, 1.0, 0, 0, /*fast=*/false);
    }
  };

  // ── 5. 主帧金字塔（带 ROI 先验） ──
  run_pyramid(full_gray, "P", true);

  // ── 6. 畸变校正兜底（2D 未命中时；需要相机内参） ──
  if (!type_done(true) && cfg.bc_undistort_enabled && !fmts_2d.empty() && !check_cancel()) {
    if (camera.valid()) {
      cv::Mat undist_img;
      cv::undistort(full_gray, undist_img, cfg.camera_matrix, cfg.dist_coeffs);
      scan_image(undist_img, fmts_2d, true, cfg.bc_try_rotate, false, 0, "UND-2d");
    } else {
      vlogf("[Vision] UND-2d skipped (no camera model)\n");
    }
  }

  // ── 7. 重试抓帧（仍有缺码型时；同样小图优先） ──
  if (!all_done() && cfg.bc_retry_count > 0 && !cancelled) {
    const long long scan_timeout_ms = 1500;
    if (!hooks.next_frame) {
      vlogf("[Vision] Still missing codes, retry skipped (no frame source)\n");
    } else {
      vlogf("[Vision] Still missing codes, retrying (%d times)...\n", cfg.bc_retry_count);
    }
    for (int rt = 0; hooks.next_frame && rt < cfg.bc_retry_count && !all_done(); rt++) {
      std::string missing;
      if (!req_done(cfg.bc_qr, cfg.exp_qr, ZXing::BarcodeFormat::QRCode)) missing += " QR";
      if (!req_done(cfg.bc_dm, cfg.exp_dm, ZXing::BarcodeFormat::DataMatrix)) missing += " DM";
      if (!req_done(cfg.bc_aztec, cfg.exp_aztec, ZXing::BarcodeFormat::Aztec)) missing += " Aztec";
      if (!req_done(cfg.bc_pdf417, cfg.exp_pdf417, ZXing::BarcodeFormat::PDF417)) missing += " PDF417";
      if (!req_done(cfg.bc_128, cfg.exp_128, ZXing::BarcodeFormat::Code128)) missing += " Code128";
      if (!req_done(cfg.bc_39, cfg.exp_39, ZXing::BarcodeFormat::Code39)) missing += " Code39";
      if (!req_done(cfg.bc_ean13, cfg.exp_ean13, ZXing::BarcodeFormat::EAN13)) missing += " EAN13";
      if (!req_done(cfg.bc_itf, cfg.exp_itf, ZXing::BarcodeFormat::ITF)) missing += " ITF";
      vlogf("[RETRY] attempt=%d missing=[%s ] distinct=%zu\n", rt + 1, missing.c_str(), detected.size());

      // 重试间检查取消。
      if (check_cancel()) break;
      if (ms_now() - t_start >= scan_timeout_ms) {
        vlogf("[Vision] Decode timeout, stop retry (elapsed=%lldms)\n", ms_now() - t_start);
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(cfg.bc_retry_interval_ms));
      cv::Mat g2;
      if (!hooks.next_frame(g2) || g2.empty()) continue;
      if (g2.channels() == 3) cv::cvtColor(g2, g2, cv::COLOR_BGR2GRAY);
      else if (g2.channels() == 4) cv::cvtColor(g2, g2, cv::COLOR_BGRA2GRAY);
      // 定向补缺：只补仍缺失的类型，直接上最有把握的通道
      if (!type_done(true) && !fmts_2d.empty()) {
        cv::Mat small;
        double s = (double)cfg.bc_roi_2d_scale / std::max(g2.cols, g2.rows);
        double scl;
        if (s < 1.0) { cv::resize(g2, small, cv::Size(), s, s, cv::INTER_LINEAR); scl = 1.0 / s; }
        else { small = g2; scl = 1.0; }
        scan_image(small, fmts_2d, true, cfg.bc_try_rotate, false, 0, "RT-2d-small", scl, 0, 0);
        if (!type_done(true) && cfg.bc_preproc_enabled && cfg.bc_clahe_enabled) {
          cv::Mat clahe_img;
          cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
              cfg.bc_clahe_clip, cv::Size(cfg.bc_clahe_grid, cfg.bc_clahe_grid));
          clahe->apply(small, clahe_img);
          scan_image(clahe_img, fmts_2d, true, cfg.bc_try_rotate, false, 0,
                     "RT-2d-small-clahe", scl, 0, 0);
        }
      }
      if (!type_done(false) && !fmts_1d.empty()) {
        scan_image(g2, fmts_1d, true, cfg.bc_try_rotate, false, 1, "RT-1d-full-raw", 1.0, 0, 0);
        if (!type_done(false)) {
          scan_image(g2, fmts_1d, true, cfg.bc_try_rotate, true, 1, "RT-1d-full-local", 1.0, 0, 0);
        }
        if (!type_done(false)) {
          cv::Mat bin;
          cv::threshold(g2, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
          scan_image(bin, fmts_1d, true, false, false, 1, "RT-1d-otsu", 1.0, 0, 0);
        }
        if (!type_done(false)) {
          cv::Mat adaptive = barcode_1d_adaptive_threshold(g2);
          scan_image(adaptive, fmts_1d, true, false, false, 1, "RT-1d-adaptive", 1.0, 0, 0);
        }
        if (!type_done(false)) {
          cv::Mat enh = barcode_1d_sobel_adaptive(g2);
          scan_image(enh, fmts_1d, true, false, false, 1, "RT-1d-sobel", 1.0, 0, 0);
        }
      }
      vlogf("[Vision] Retry %d/%d: %zu codes\n", rt + 1, cfg.bc_retry_count, detected.size());
    }
  }

  // ── 10. Fusion 摘要 ──
  {
    const auto& fs = result_fusion.stats();
    auto format_count = [](const std::map<barcode_fusion::BarcodeKind, size_t>& m,
                           barcode_fusion::BarcodeKind kind) -> size_t {
      auto it = m.find(kind);
      return it == m.end() ? 0 : it->second;
    };
    vlogf("[FUSION] raw=%zu distinct=%zu duplicates=%zu invalid=%zu\n",
          fs.raw_results, fs.distinct_results, fs.duplicates_removed, fs.invalid_results);
    vlogf("[FUSION] C128 raw=%zu distinct=%zu\n",
          format_count(fs.raw_by_format, barcode_fusion::BarcodeKind::Code128),
          format_count(fs.distinct_by_format, barcode_fusion::BarcodeKind::Code128));
    // detected 只含 duplicate 之外的结果；same-symbol 候选会进 fusion 但不导出，
    // 因此二者不一致时是提示而不是错误（Drone-Vision 日志里的 DISTINCT INVARIANT BROKEN）。
    vlogf("[FUSION-CHECK] detected=%zu fusion_distinct=%zu match=%d\n",
          detected.size(), fs.distinct_results, detected.size() == fs.distinct_results ? 1 : 0);
    for (const auto& item : strategy_stats) {
      vlogf("[STRATEGY STAT] %s attempts=%d raw=%d distinct=%d elapsed=%lldms\n",
            item.first.c_str(), item.second.attempts, item.second.raw_success,
            item.second.distinct_success, item.second.elapsed_ms);
    }
    vlogf("[LOCATOR] found_decode_failed=%d\n", locator_decode_failed);
    for (const auto& item : decode_failed_by_format) {
      vlogf("[LOCATOR] format=%s failed=%d\n", barcode_format_name(item.first), item.second);
    }
  }

  // 成功语义：有强制数量 → 须全部达到预期才算成功；expected 全 0 → 解出任意码即算成功。
  if (!detected.empty()) {
    bool decode_success = has_required_any ? all_done() : true;
    out.success = decode_success;
    out.partial = !decode_success;
    out.error = decode_success ? "" : "partial";
    vlogf("[Vision] Multi-decode: total %zu codes (%s)\n", detected.size(),
          decode_success ? (has_required_any ? "ALL required types reached"
                                             : "any result (expected=0, free decode)")
                         : "PARTIAL, required not all reached");
  } else {
    out.error = "decode_fail";
    vlogf("[Vision] Multi-decode: ALL FAILED\n");
  }
  if (cancelled) {
    out.success = false;
    out.partial = false;
    out.error = "cancelled";
    vlogf("[Vision] Multi-decode: CANCELLED\n");
  }

  // ── 11. DecodeReport：各码制命中情况 + 总耗时 ──
  {
    std::string rep;
    auto rep_add = [&](const char* name, int got, int req) {
      char buf[48];
      std::snprintf(buf, sizeof(buf), "%s%s=%d/%d", rep.empty() ? "" : " ", name, got, req);
      rep += buf;
    };
    if (cfg.bc_qr)     rep_add("QR",     found_count(ZXing::BarcodeFormat::QRCode),     cfg.exp_qr);
    if (cfg.bc_dm)     rep_add("DM",     found_count(ZXing::BarcodeFormat::DataMatrix), cfg.exp_dm);
    if (cfg.bc_aztec)  rep_add("Aztec",  found_count(ZXing::BarcodeFormat::Aztec),      cfg.exp_aztec);
    if (cfg.bc_pdf417) rep_add("PDF417", found_count(ZXing::BarcodeFormat::PDF417),     cfg.exp_pdf417);
    if (cfg.bc_128)    rep_add("C128",   found_count(ZXing::BarcodeFormat::Code128),    cfg.exp_128);
    if (cfg.bc_39)     rep_add("C39",    found_count(ZXing::BarcodeFormat::Code39),     cfg.exp_39);
    if (cfg.bc_ean13)  rep_add("EAN13",  found_count(ZXing::BarcodeFormat::EAN13),      cfg.exp_ean13);
    if (cfg.bc_itf)    rep_add("ITF",    found_count(ZXing::BarcodeFormat::ITF),        cfg.exp_itf);
    out.report = rep;
    out.elapsed_ms = (double)(ms_now() - t_start);
    vlogf("[DecodeReport] %.0fms %s\n", out.elapsed_ms, rep.c_str());
  }

  for (const auto& b : barcode_boxes) {
    if (b.decoded) continue;
    out.failed_boxes.push_back({{cv::Point2f((float)b.x0, (float)b.y0), cv::Point2f((float)b.x1, (float)b.y1),
                                 cv::Point2f((float)b.x2, (float)b.y2), cv::Point2f((float)b.x3, (float)b.y3)}});
  }
  return out;
}

void drawDecodeOutput(cv::Mat& bgr, const DecodeOutput& output) {
  const int thick = std::max(2, bgr.cols / 800);
  const double font = std::max(0.6, bgr.cols / 1600.0);
  if (output.roi_quad.size() == 4) {
    for (int k = 0; k < 4; ++k)
      cv::line(bgr, output.roi_quad[k], output.roi_quad[(k + 1) % 4], cv::Scalar(255, 255, 0), thick);
  }
  for (const auto& box : output.failed_boxes) {
    for (int k = 0; k < 4; ++k)
      cv::line(bgr, box[k], box[(k + 1) % 4], cv::Scalar(0, 0, 255), thick);
  }
  for (size_t i = 0; i < output.codes.size(); ++i) {
    const auto& c = output.codes[i];
    for (int k = 0; k < 4; ++k)
      cv::line(bgr, c.corners[k], c.corners[(k + 1) % 4], cv::Scalar(0, 255, 0), thick + 1);
    const std::string label = "#" + std::to_string(i + 1);
    cv::putText(bgr, label, c.center + cv::Point2f(-20.0f * (float)font, 12.0f * (float)font),
                cv::FONT_HERSHEY_SIMPLEX, font * 1.5, cv::Scalar(0, 0, 0), thick * 3);
    cv::putText(bgr, label, c.center + cv::Point2f(-20.0f * (float)font, 12.0f * (float)font),
                cv::FONT_HERSHEY_SIMPLEX, font * 1.5, cv::Scalar(0, 200, 255), thick);
  }
}

}  // namespace barcode_reader_camera
