#include "barcode_reader_camera/frame_recorder.hpp"

#include <sys/stat.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace barcode_reader_camera {
namespace {

void makeDirs(const std::string& path) {
  for (size_t pos = 0; pos != std::string::npos;) {
    pos = path.find('/', pos + 1);
    const std::string part = path.substr(0, pos);
    if (::mkdir(part.c_str(), 0775) != 0 && errno != EEXIST)
      throw std::runtime_error("mkdir " + part + ": " + std::strerror(errno));
  }
}

bool exists(const std::string& path) {
  struct stat st;
  return ::stat(path.c_str(), &st) == 0;
}

// "2026-10-11 21:40:05.123" (sep=true) or "20261011_214005_123" (sep=false), local time.
std::string formatTime(double t, bool sep) {
  const time_t s = (time_t)std::floor(t);
  const int ms = std::min(999, (int)std::lround((t - std::floor(t)) * 1000.0));
  struct tm tm;
  localtime_r(&s, &tm);
  char buf[64];
  std::strftime(buf, sizeof(buf), sep ? "%Y-%m-%d %H:%M:%S" : "%Y%m%d_%H%M%S", &tm);
  char out[80];
  std::snprintf(out, sizeof(out), sep ? "%s.%03d" : "%s_%03d", buf, ms);
  return out;
}

std::string csvField(const std::string& s) {
  std::string out = "\"";
  for (char c : s) {
    if (c == '"') out += "\"\"";
    else if (c == '\n' || c == '\r') out += ' ';
    else out += c;
  }
  return out + "\"";
}

std::string num(double v, int digits) {
  if (!std::isfinite(v)) return "";
  char buf[48];
  std::snprintf(buf, sizeof(buf), "%.*f", digits, v);
  return buf;
}

struct Metrics {
  cv::Rect roi;  // area measured: projected label ROI, or the whole frame without one
  double mean = NAN, sharpness = NAN;
  double distance = NAN, z = NAN, tilt_deg = NAN;
};

Metrics computeMetrics(const DecodeRecord& r) {
  Metrics m;
  if (r.have_pose) {
    const cv::Vec3d t = r.pose.tvec;
    m.distance = cv::norm(t);
    m.z = t[2];
    cv::Matx33d rot;
    cv::Rodrigues(r.pose.rvec, rot);
    const cv::Vec3d n(rot(0, 2), rot(1, 2), rot(2, 2));  // tag plane normal in the camera frame
    if (m.distance > 0)
      m.tilt_deg = std::acos(std::min(1.0, std::abs(n.dot(t)) / m.distance)) * 180.0 / CV_PI;
  }
  if (!r.frame.empty()) {
    const cv::Rect full(0, 0, r.frame.cols, r.frame.rows);
    m.roi = r.out.roi_quad.size() == 4 ? (cv::boundingRect(r.out.roi_quad) & full) : full;
    if (m.roi.area() > 0) {
      const cv::Mat area = r.frame(m.roi);
      m.mean = cv::mean(area)[0];
      cv::Mat lap;
      cv::Laplacian(area, lap, CV_64F);
      cv::Scalar mu, sigma;
      cv::meanStdDev(lap, mu, sigma);
      m.sharpness = sigma[0] * sigma[0];  // variance of Laplacian: higher = sharper
    }
  }
  return m;
}

std::string resultName(const DecodeRecord& r) {
  return r.success ? "ok" : (r.partial ? "partial" : "fail");
}

void emitPoint(YAML::Emitter& e, const cv::Point2f& p) {
  e << YAML::Flow << YAML::BeginSeq << p.x << p.y << YAML::EndSeq;
}

void emitQuad(YAML::Emitter& e, const cv::Point2f* p, size_t n) {
  e << YAML::Flow << YAML::BeginSeq;
  for (size_t i = 0; i < n; ++i) emitPoint(e, p[i]);
  e << YAML::EndSeq;
}

void writeText(const std::string& path, const std::string& text) {
  std::ofstream f(path);
  f << text;
  if (!f) throw std::runtime_error("cannot write " + path);
}

void writeImage(const std::string& path, const cv::Mat& img, const std::vector<int>& params) {
  if (!cv::imwrite(path, img, params)) throw std::runtime_error("cannot write " + path);
}

// ROS camera_info YAML, readable by loadCameraYaml / decode_image --camera-yaml.
std::string cameraYaml(const CameraModel& c, const cv::Size& size) {
  YAML::Emitter e;
  e << YAML::BeginMap;
  e << YAML::Key << "image_width" << YAML::Value << size.width;
  e << YAML::Key << "image_height" << YAML::Value << size.height;
  e << YAML::Key << "camera_name" << YAML::Value << "recorded_camera_info";
  e << YAML::Key << "camera_matrix" << YAML::Value << YAML::BeginMap
    << YAML::Key << "rows" << YAML::Value << 3 << YAML::Key << "cols" << YAML::Value << 3
    << YAML::Key << "data" << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (int i = 0; i < 9; ++i) e << c.camera_matrix.at<double>(i / 3, i % 3);
  e << YAML::EndSeq << YAML::EndMap;
  e << YAML::Key << "distortion_model" << YAML::Value << "plumb_bob";
  e << YAML::Key << "distortion_coefficients" << YAML::Value << YAML::BeginMap
    << YAML::Key << "rows" << YAML::Value << 1 << YAML::Key << "cols" << YAML::Value << (int)c.dist_coeffs.total()
    << YAML::Key << "data" << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (size_t i = 0; i < c.dist_coeffs.total(); ++i) e << c.dist_coeffs.at<double>((int)i);
  e << YAML::EndSeq << YAML::EndMap;
  e << YAML::EndMap;
  return std::string(e.c_str()) + "\n";
}

std::string metaYaml(const DecodeRecord& r, const Metrics& m, const std::vector<std::string>& retry_files) {
  YAML::Emitter e;
  e.SetDoublePrecision(10);
  e.SetFloatPrecision(7);
  e << YAML::BeginMap;
  e << YAML::Key << "time" << YAML::Value << formatTime(r.wall_time, true);
  e << YAML::Key << "stamp" << YAML::Value << num(r.stamp, 6);  // fixed decimals: a ROS stamp needs 16 digits
  e << YAML::Key << "frame_id" << YAML::Value << r.frame_id;
  e << YAML::Key << "result" << YAML::Value << YAML::BeginMap
    << YAML::Key << "status" << YAML::Value << resultName(r)
    << YAML::Key << "error" << YAML::Value << r.error
    << YAML::Key << "report" << YAML::Value << r.out.report
    << YAML::Key << "elapsed_s" << YAML::Value << r.elapsed_s
    << YAML::Key << "decode_ms" << YAML::Value << r.out.elapsed_ms
    << YAML::Key << "used_tag_roi" << YAML::Value << r.out.used_tag_roi << YAML::EndMap;
  e << YAML::Key << "codes" << YAML::Value << YAML::BeginSeq;
  for (const auto& c : r.out.codes) {
    e << YAML::BeginMap << YAML::Key << "format" << YAML::Value << c.format
      << YAML::Key << "code" << YAML::Value << c.code
      << YAML::Key << "raw_hex" << YAML::Value << toHex(c.bytes)
      << YAML::Key << "source" << YAML::Value << c.source
      << YAML::Key << "center" << YAML::Value;
    emitPoint(e, c.center);
    e << YAML::Key << "corners" << YAML::Value;
    emitQuad(e, c.corners.data(), 4);
    e << YAML::EndMap;
  }
  e << YAML::EndSeq;
  // Same convention as decodeBarcodes(): T_camera_tag; decode_image --pose reads rvec/tvec.
  if (r.have_pose) {
    e << YAML::Key << "pose" << YAML::Value << YAML::BeginMap
      << YAML::Key << "rvec" << YAML::Value << YAML::Flow << YAML::BeginSeq
      << r.pose.rvec[0] << r.pose.rvec[1] << r.pose.rvec[2] << YAML::EndSeq
      << YAML::Key << "tvec" << YAML::Value << YAML::Flow << YAML::BeginSeq
      << r.pose.tvec[0] << r.pose.tvec[1] << r.pose.tvec[2] << YAML::EndSeq
      << YAML::Key << "distance_m" << YAML::Value << m.distance
      << YAML::Key << "tilt_deg" << YAML::Value << m.tilt_deg << YAML::EndMap;
  }
  e << YAML::Key << "roi_quad" << YAML::Value;
  emitQuad(e, r.out.roi_quad.data(), r.out.roi_quad.size());
  e << YAML::Key << "failed_boxes" << YAML::Value << YAML::BeginSeq;
  for (const auto& b : r.out.failed_boxes) emitQuad(e, b.data(), 4);
  e << YAML::EndSeq;
  e << YAML::Key << "metrics" << YAML::Value << YAML::BeginMap
    << YAML::Key << "area" << YAML::Value << YAML::Flow << YAML::BeginSeq
    << m.roi.x << m.roi.y << m.roi.width << m.roi.height << YAML::EndSeq
    << YAML::Key << "mean" << YAML::Value << m.mean
    << YAML::Key << "laplacian_var" << YAML::Value << m.sharpness << YAML::EndMap;
  e << YAML::Key << "retries" << YAML::Value << YAML::BeginSeq;
  for (size_t i = 0; i < r.retries.size(); ++i)
    e << YAML::Flow << YAML::BeginMap << YAML::Key << "file" << YAML::Value << retry_files[i]
      << YAML::Key << "stamp" << YAML::Value << num(r.retries[i].first, 6) << YAML::EndMap;
  e << YAML::EndSeq;
  e << YAML::Key << "layout_file" << YAML::Value << r.layout_file;
  e << YAML::Key << "config_file" << YAML::Value << r.config_file;
  e << YAML::EndMap;
  return std::string(e.c_str()) + "\n";
}

}  // namespace

SaveFrames parseSaveFrames(const std::string& s) {
  if (s == "off") return SaveFrames::Off;
  if (s == "failed") return SaveFrames::Failed;
  if (s == "all") return SaveFrames::All;
  throw std::invalid_argument("save_frames must be off/failed/all, got \"" + s + "\"");
}

FrameRecorder::FrameRecorder(const std::string& dir, SaveFrames mode, bool log_attempts, size_t max_queue)
    : dir_(dir), mode_(mode), log_attempts_(log_attempts), max_queue_(std::max<size_t>(1, max_queue)) {
  while (dir_.size() > 1 && dir_.back() == '/') dir_.pop_back();
  if (!enabled()) return;
  makeDirs(dir_);
  thread_ = std::thread([this]() { run(); });
}

FrameRecorder::~FrameRecorder() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

bool FrameRecorder::submit(DecodeRecord record) {
  if (!enabled()) return true;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.size() >= max_queue_) return false;
    queue_.push_back(std::move(record));
  }
  cv_.notify_one();
  return true;
}

std::string FrameRecorder::takeError() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::string e;
  e.swap(error_);
  return e;
}

void FrameRecorder::run() {
  for (;;) {
    DecodeRecord r;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this]() { return stop_ || !queue_.empty(); });
      if (queue_.empty()) return;  // stop_ and drained
      r = std::move(queue_.front());
      queue_.pop_front();
    }
    try {
      write(r);
    } catch (const std::exception& e) {
      std::lock_guard<std::mutex> lock(mutex_);
      error_ = e.what();
    }
  }
}

void FrameRecorder::write(const DecodeRecord& r) {
  const Metrics m = computeMetrics(r);
  bool save = !r.frame.empty() && (mode_ == SaveFrames::All || (mode_ == SaveFrames::Failed && !r.success));
  // Decoding did not run (no pose / intrinsics): callers such as trigger_decode retry every 0.5 s,
  // keep one frame per kChainErrorSaveGapS so these do not bury the decode attempts.
  const bool chain_error = r.error == "no_tag" || r.error == "no_camera_info";
  if (save && chain_error) {
    if (r.wall_time - last_chain_error_save_ < kChainErrorSaveGapS) save = false;
    else last_chain_error_save_ = r.wall_time;
  }
  std::string name;
  if (save) {
    // 20261011_214005_123_ok / _partial / _decode_fail / _no_tag ...
    const std::string base = formatTime(r.wall_time, false) + "_" +
                             (r.success || r.error.empty() ? resultName(r) : r.error);
    name = base;
    for (int i = 2; exists(dir_ + "/" + name); ++i) name = base + "_" + std::to_string(i);
    const std::string d = dir_ + "/" + name;
    makeDirs(d);
    const std::vector<int> png = {cv::IMWRITE_PNG_COMPRESSION, 1};
    writeImage(d + "/frame.png", r.frame, png);
    std::vector<std::string> retry_files;
    for (size_t i = 0; i < r.retries.size(); ++i) {
      retry_files.push_back("retry_" + std::to_string(i + 1) + ".png");
      writeImage(d + "/" + retry_files.back(), r.retries[i].second, png);
    }
    if (r.camera.valid()) writeText(d + "/camera.yaml", cameraYaml(r.camera, r.frame.size()));
    writeText(d + "/meta.yaml", metaYaml(r, m, retry_files));
    std::string log;
    for (const auto& line : r.log) log += line + "\n";
    writeText(d + "/decode.log", log);
    cv::Mat preview;
    cv::cvtColor(r.frame, preview, cv::COLOR_GRAY2BGR);
    drawDecodeOutput(preview, r.out);
    if (preview.cols > 1280)
      cv::resize(preview, preview, cv::Size(), 1280.0 / preview.cols, 1280.0 / preview.cols, cv::INTER_AREA);
    writeImage(d + "/preview.jpg", preview, {cv::IMWRITE_JPEG_QUALITY, 85});
  }

  if (!log_attempts_) return;
  const std::string csv = dir_ + "/attempts.csv";
  const bool fresh = !exists(csv);
  std::ofstream f(csv, std::ios::app);
  if (fresh)
    f << "time,stamp,result,error,report,codes,sources,elapsed_ms,decode_ms,tag_roi,dist_m,z_m,tilt_deg,"
         "roi_w,roi_h,roi_mean,roi_laplacian_var,retries,dir\n";
  std::string codes, sources;
  for (const auto& c : r.out.codes) {
    codes += (codes.empty() ? "" : ";") + c.format + ":" + c.code;
    sources += (sources.empty() ? "" : ";") + c.source;
  }
  const bool has_area = !r.frame.empty() && m.roi.area() > 0;
  f << csvField(formatTime(r.wall_time, true)) << ',' << (r.stamp > 0 ? num(r.stamp, 6) : "") << ','
    << resultName(r) << ',' << csvField(r.error) << ',' << csvField(r.out.report) << ',' << csvField(codes) << ','
    << csvField(sources) << ',' << num(r.elapsed_s * 1000.0, 0) << ','
    << (r.frame.empty() ? "" : num(r.out.elapsed_ms, 0)) << ',' << (int)r.out.used_tag_roi << ','
    << num(m.distance, 3) << ',' << num(m.z, 3) << ',' << num(m.tilt_deg, 1) << ','
    << (has_area ? std::to_string(m.roi.width) : "") << ',' << (has_area ? std::to_string(m.roi.height) : "") << ','
    << num(m.mean, 1) << ',' << num(m.sharpness, 1) << ',' << r.retries.size() << ',' << csvField(name) << '\n';
  if (!f) throw std::runtime_error("cannot append " + csv);
}

}  // namespace barcode_reader_camera
