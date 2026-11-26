#pragma once
// Synthetic 100x150 mm label (Drone-Vision layout: tag36h11 at the label center) for tests.
// The installed ZXing is a read-only build, so symbols come from: QR matrices generated once
// with `qrencode -t ASCII -m 0 -l M -v 1`, a small Code128-B encoder, and apriltag_to_image.
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

extern "C" {
#include <apriltag.h>
#include <tag36h11.h>
}

#include "barcode_reader_camera/multi_decoder.hpp"

namespace synthetic {

inline const std::vector<std::string>& qrTest001() {
  static const std::vector<std::string> m = {
      "111111100010001111111", "100000101100101000001", "101110100111101011101",
      "101110100100001011101", "101110101111101011101", "100000100111001000001",
      "111111101010101111111", "000000000110000000000", "101010100000100010010",
      "111001000011000100100", "000011101001001010010", "010011000011110101111",
      "000110101011001010000", "000000001000001100010", "111111100110100010110",
      "100000100000001101100", "101110101110101010011", "101110100101010000010",
      "101110101001011010101", "100000100011110111111", "111111101111011000001"};
  return m;
}

inline const std::vector<std::string>& qrTest002() {
  static const std::vector<std::string> m = {
      "111111100111001111111", "100000101010101000001", "101110100110001011101",
      "101110100000101011101", "101110101101101011101", "100000100001001000001",
      "111111101010101111111", "000000000000000000000", "101010100000100010010",
      "011000001101011100100", "001011110101001010010", "011011000101110101111",
      "111001100001001010000", "000000001010001100010", "111111100100100010110",
      "100000100100001101100", "101110101010101010011", "101110100001010000010",
      "101110101101011010101", "100000100101110111111", "111111101011011000001"};
  return m;
}

// Code128 bar/space widths for values 0..106 (106 = stop, 7 elements).
inline std::vector<int> code128Modules(const std::string& text) {
  static const char* patterns[] = {
      "212222", "222122", "222221", "121223", "121322", "131222", "122213", "122312", "132212",
      "221213", "221312", "231212", "112232", "122132", "122231", "113222", "123122", "123221",
      "223211", "221132", "221231", "213212", "223112", "312131", "311222", "321122", "321221",
      "312212", "322112", "322211", "212123", "212321", "232121", "111323", "131123", "131321",
      "112313", "132113", "132311", "211313", "231113", "231311", "112133", "112331", "132131",
      "113123", "113321", "133121", "313121", "211331", "231131", "213113", "213311", "213131",
      "311123", "311321", "331121", "312113", "312311", "332111", "314111", "221411", "431111",
      "111224", "111422", "121124", "121421", "141122", "141221", "112214", "112412", "122114",
      "122411", "142112", "142211", "241211", "221114", "413111", "241112", "134111", "111242",
      "121142", "121241", "114212", "124112", "124211", "411212", "421112", "421211", "212141",
      "214121", "412121", "111143", "111341", "131141", "114113", "114311", "411113", "411311",
      "113141", "114131", "311141", "411131", "211412", "211214", "211232", "2331112"};
  std::vector<int> values = {104};  // Start B
  int checksum = 104;
  for (size_t i = 0; i < text.size(); ++i) {
    const int v = text[i] - 32;
    values.push_back(v);
    checksum += v * (int)(i + 1);
  }
  values.push_back(checksum % 103);
  values.push_back(106);
  std::vector<int> modules;  // 1 = bar, 0 = space
  for (int v : values) {
    bool bar = true;
    for (const char* p = patterns[v]; *p; ++p, bar = !bar)
      modules.insert(modules.end(), *p - '0', bar ? 1 : 0);
  }
  return modules;
}

inline void drawMatrix(cv::Mat& img, const std::vector<std::string>& rows, cv::Point origin, int module) {
  for (size_t r = 0; r < rows.size(); ++r)
    for (size_t c = 0; c < rows[r].size(); ++c)
      if (rows[r][c] == '1')
        cv::rectangle(img, cv::Rect(origin.x + (int)c * module, origin.y + (int)r * module, module, module),
                      cv::Scalar(0), cv::FILLED);
}

inline void drawCode128(cv::Mat& img, const std::string& text, cv::Rect box) {
  const auto modules = code128Modules(text);
  const double w = (double)box.width / modules.size();
  for (size_t i = 0; i < modules.size(); ++i) {
    if (!modules[i]) continue;
    const int x0 = box.x + (int)std::lround(i * w), x1 = box.x + (int)std::lround((i + 1) * w);
    cv::rectangle(img, cv::Rect(x0, box.y, std::max(1, x1 - x0), box.height), cv::Scalar(0), cv::FILLED);
  }
}

struct Scene {
  cv::Mat gray;
  barcode_reader_camera::CameraModel camera;
  double tag_size_m = 0.012;
  std::vector<std::string> qr_texts = {"QR-TEST-001", "QR-TEST-002"};
  std::vector<std::string> c128_texts = {"C128-12345", "C128-67890"};
};

// Frontal view, px_per_mm pixels per millimetre, pinhole fx=fy=1000 at the matching distance.
inline Scene makeScene(double px_per_mm = 8.0, bool with_tag = true) {
  Scene s;
  const auto mm = [&](double v) { return (int)std::lround(v * px_per_mm); };
  s.gray = cv::Mat(mm(240), mm(160), CV_8UC1, cv::Scalar(140));  // cardboard background
  const cv::Point label(mm(30), mm(45));                          // 100x150 mm label, centered
  cv::rectangle(s.gray, cv::Rect(label.x, label.y, mm(100), mm(150)), cv::Scalar(245), cv::FILLED);

  drawMatrix(s.gray, qrTest001(), label + cv::Point(mm(8), mm(8)), mm(1.0));
  drawMatrix(s.gray, qrTest002(), label + cv::Point(mm(8), mm(117)), mm(1.0));
  drawCode128(s.gray, s.c128_texts[0], cv::Rect(label.x + mm(38), label.y + mm(10), mm(56), mm(16)));
  drawCode128(s.gray, s.c128_texts[1], cv::Rect(label.x + mm(38), label.y + mm(120), mm(56), mm(16)));

  if (with_tag) {
    apriltag_family_t* family = tag36h11_create();
    image_u8_t* tag = apriltag_to_image(family, 0);  // 10x10 cells incl. white border; black border = 8 cells
    cv::Mat cells(tag->height, tag->width, CV_8UC1, tag->buf, tag->stride);
    const int cell = mm(s.tag_size_m * 1000.0 / 8.0);
    cv::Mat big;
    cv::resize(cells, big, cv::Size(cells.cols * cell, cells.rows * cell), 0, 0, cv::INTER_NEAREST);
    const cv::Point center = label + cv::Point(mm(50), mm(75));
    big.copyTo(s.gray(cv::Rect(center.x - big.cols / 2, center.y - big.rows / 2, big.cols, big.rows)));
    image_u8_destroy(tag);
    tag36h11_destroy(family);
  }
  cv::GaussianBlur(s.gray, s.gray, cv::Size(3, 3), 0.6);  // mild optical blur

  s.camera.camera_matrix = (cv::Mat_<double>(3, 3) << 1000, 0, s.gray.cols / 2.0, 0, 1000, s.gray.rows / 2.0, 0, 0, 1);
  s.camera.dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
  return s;
}

}  // namespace synthetic
