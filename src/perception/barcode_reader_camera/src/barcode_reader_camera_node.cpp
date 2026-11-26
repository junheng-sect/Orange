// ROS adapter for decodeBarcodes(): ~decode subscribes to the full-resolution image for the duration of the
// call and decodes the newest frame that has the pose of the same sensor frame from ov13855_apriltag_vision
// (nearest stamp within ~pose_match_s; waiting briefly for it), using the label ROI. While codes are missing
// it retries on newer frames that have their own pose, until retry.timeout_ms; ~cancel interrupts it.
// Every attempt can be recorded for offline analysis (~save_frames, ~log_attempts).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Trigger.h>

#include "barcode_reader_camera/BarcodeArray.h"
#include "barcode_reader_camera/BarcodeFrame.h"
#include "barcode_reader_camera/DecodeBarcodes.h"
#include "barcode_reader_camera/config_yaml.hpp"
#include "barcode_reader_camera/frame_recorder.hpp"
#include "barcode_reader_camera/multi_decoder.hpp"

namespace barcode_reader_camera {

class BarcodeReaderCameraNode {
 public:
  BarcodeReaderCameraNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    config_file_ = pnh.param<std::string>(
        "config_file", ros::package::getPath("barcode_reader_camera") + "/config/barcode_reader_camera.yaml");
    const std::string& config_file = config_file_;
    config_ = loadDecoderConfig(YAML::LoadFile(config_file));
    layout_file_ = pnh.param<std::string>("layout_file", "");
    const std::string& layout_file = layout_file_;
    if (!layout_file.empty()) {
      applyLayout(YAML::LoadFile(layout_file), config_, &layout_tag_size_m_);
      const FormatSwitches& f = config_.formats;
      const ExpectedCounts& e = config_.expected;
      std::string codes;
      auto add = [&codes](bool on, const char* name, int n) {
        if (on) codes += std::string(codes.empty() ? "" : " ") + name + "x" + std::to_string(n);
      };
      add(f.qr, "QR", e.qr); add(f.dm, "DM", e.dm); add(f.c128, "C128", e.c128); add(f.c39, "C39", e.c39);
      add(f.ean13, "EAN13", e.ean13); add(f.pdf417, "PDF417", e.pdf417); add(f.aztec, "Aztec", e.aztec);
      add(f.itf, "ITF", e.itf);
      ROS_INFO("barcode_reader_camera: layout=%s tag=%.4fm label=%.3fx%.3fm tag_position=%s "
               "offset=(%.4f,%.4f)m codes=[%s]",
               layout_file.c_str(), layout_tag_size_m_, config_.label_width_m, config_.label_height_m,
               config_.tag_position.c_str(), config_.tag_offset_x_m, config_.tag_offset_y_m, codes.c_str());
    } else {
      ROS_WARN("barcode_reader_camera: no ~layout_file, label geometry and expected_codes use built-in defaults");
    }
    vision_tag_size_param_ = pnh.param<std::string>(
        "vision_tag_size_param", "/ov13855_apriltag_vision_node/target_marker_size_m");
    require_tag_ = pnh.param("require_tag", true);
    image_wait_s_ = std::max(0.0, pnh.param("image_wait_s", 1.0));
    pose_wait_s_ = std::max(0.0, pnh.param("pose_wait_s", 0.2));
    pose_match_s_ = std::max(0.0, pnh.param("pose_match_s", 0.01));
    log_strategies_ = pnh.param("log_strategies", false);
    setupRecorder(pnh);
    nh_ = nh;
    image_topic_ = pnh.param<std::string>("image_topic", "/ov13855_full/image_raw");
    const auto info_topic = pnh.param<std::string>("camera_info_topic", "/ov13855_full/camera_info");
    const auto pose_topic = pnh.param<std::string>("pose_topic", "/front_marker/pose");

    result_pub_ = pnh.advertise<BarcodeArray>("result", 10);
    if (image_topic_ == "/ov13855_full/image_raw" || image_topic_ == "/ov13855/image_raw")
      monitor_result_ = nh_.advertise<BarcodeFrame>("/ov13855/monitor/barcodes", 1);
    info_sub_ = nh.subscribe(info_topic, 1, &BarcodeReaderCameraNode::onInfo, this);
    pose_sub_ = nh.subscribe(pose_topic, 10, &BarcodeReaderCameraNode::onPose, this);
    decode_srv_ = pnh.advertiseService("decode", &BarcodeReaderCameraNode::onDecode, this);
    cancel_srv_ = pnh.advertiseService("cancel", &BarcodeReaderCameraNode::onCancel, this);
    ROS_INFO("barcode_reader_camera: config=%s image=%s (subscribed only during ~decode) pose=%s require_tag=%d",
             config_file.c_str(), image_topic_.c_str(), pose_topic.c_str(), (int)require_tag_);
  }

 private:
  struct Frame {
    ros::Time stamp;
    std::string frame_id;
    cv::Mat gray;
    uint64_t seq = 0;
  };

  // Every node run records into its own session directory <save_dir>/<start time>_<layout>/, created
  // with the first decode; ~decode returns it as session_dir so barcode_rc_trigger writes there too.
  // ~save_frames off/failed/all: per-attempt directory with frame, retries, pose, intrinsics, log.
  // ~save_retry_frames: retry images kept per attempt besides the one that read the code and the
  // last one (<0 = all); every retry frame's stamp and pose is recorded either way.
  // ~log_attempts: one row per ~decode in <session>/attempts.csv, independent of ~save_frames.
  void setupRecorder(ros::NodeHandle& pnh) {
    save_retry_frames_ = pnh.param("save_retry_frames", 2);
    std::string root = pnh.param<std::string>("save_dir", "~/test_data/barcode");
    if (root.rfind("~/", 0) == 0 && std::getenv("HOME")) root = std::string(std::getenv("HOME")) + root.substr(1);
    const std::string mode_name = pnh.param<std::string>("save_frames", "off");
    const bool log_attempts = pnh.param("log_attempts", true);
    // Session label: layout file name without directory and extension.
    std::string label = layout_file_.substr(layout_file_.find_last_of('/') + 1);
    label = label.substr(0, label.rfind(".yaml"));
    if (label.empty()) label = "default";
    try {
      recorder_ = std::make_unique<FrameRecorder>(sessionDir(root, label, ros::WallTime::now().toSec()),
                                                  parseSaveFrames(mode_name), log_attempts);
    } catch (const std::exception& e) {
      ROS_ERROR("barcode_reader_camera: recording disabled: %s", e.what());
      return;
    }
    if (recorder_->enabled())
      ROS_INFO("barcode_reader_camera: save_frames=%s log_attempts=%d session=%s (created on first decode)",
               mode_name.c_str(), (int)log_attempts, recorder_->dir().c_str());
  }

  void onImage(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr image;
    try {
      image = cv_bridge::toCvCopy(msg, "mono8");
    } catch (const cv_bridge::Exception& e) {
      ROS_WARN_THROTTLE(5.0, "barcode_reader_camera: cannot convert image: %s", e.what());
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    frames_.push_back({msg->header.stamp, msg->header.frame_id, image->image, ++frame_seq_});
    while (frames_.size() > 5) frames_.pop_front();
    update_cv_.notify_all();
  }

  void onInfo(const sensor_msgs::CameraInfoConstPtr& msg) {
    CameraModel camera;
    camera.camera_matrix = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) camera.camera_matrix.at<double>(i / 3, i % 3) = msg->K[i];
    camera.dist_coeffs = cv::Mat(1, (int)msg->D.size(), CV_64F);
    for (size_t i = 0; i < msg->D.size(); ++i) camera.dist_coeffs.at<double>(0, (int)i) = msg->D[i];
    std::lock_guard<std::mutex> lock(mutex_);
    camera_ = camera;
    info_size_ = cv::Size((int)msg->width, (int)msg->height);
  }

  void onPose(const geometry_msgs::PoseStampedConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    poses_.push_back(*msg);
    while (poses_.size() > 30) poses_.pop_front();
    update_cv_.notify_all();
  }

  // Caller holds mutex_. The pose of the same sensor frame: poses come from the 1280x960 self path, images
  // from the 4224x3136 main path; their stamps differ by ~0.01 ms (frame period 66.7 ms), so the nearest
  // pose within pose_match_s_ is taken.
  const geometry_msgs::PoseStamped* findPose(const ros::Time& stamp) const {
    const geometry_msgs::PoseStamped* best = nullptr;
    double best_dt = pose_match_s_;
    for (const auto& p : poses_) {
      const double dt = std::abs((p.header.stamp - stamp).toSec());
      if (dt <= best_dt) {
        best = &p;
        best_dt = dt;
      }
    }
    return best;
  }

  // Caller holds mutex_. Newest received frame whose pose has arrived.
  const Frame* newestFrameWithPose() const {
    for (auto it = frames_.rbegin(); it != frames_.rend(); ++it)
      if (findPose(it->stamp)) return &*it;
    return nullptr;
  }

  // The full-resolution image (13 MB per frame) is subscribed only while ~decode runs, and the camera node
  // copies frames only while someone subscribes. Frames received outside a decode are dropped.
  class ImageSubscription {
   public:
    explicit ImageSubscription(BarcodeReaderCameraNode& node) : node_(node) {
      {
        std::lock_guard<std::mutex> lock(node_.mutex_);
        node_.frames_.clear();
      }
      node_.image_sub_ = node_.nh_.subscribe(node_.image_topic_, 2, &BarcodeReaderCameraNode::onImage, &node_);
    }
    ~ImageSubscription() {
      node_.image_sub_.shutdown();
      std::lock_guard<std::mutex> lock(node_.mutex_);
      node_.frames_.clear();
    }

   private:
    BarcodeReaderCameraNode& node_;
  };

  static TagPose toTagPose(const geometry_msgs::Pose& p) {
    const double x = p.orientation.x, y = p.orientation.y, z = p.orientation.z, w = p.orientation.w;
    const double n = std::sqrt(x * x + y * y + z * z + w * w);
    const double qx = x / n, qy = y / n, qz = z / n, qw = w / n;
    cv::Matx33d r(1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw),
                  2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw),
                  2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy));
    TagPose pose;
    cv::Rodrigues(r, pose.rvec);
    pose.tvec = cv::Vec3d(p.position.x, p.position.y, p.position.z);
    return pose;
  }

  bool onDecode(DecodeBarcodes::Request&, DecodeBarcodes::Response& res) {
    if (recorder_ && recorder_->enabled()) res.session_dir = recorder_->dir();
    std::unique_lock<std::mutex> busy(decode_mutex_, std::try_to_lock);
    if (!busy.owns_lock()) {
      res.error = "busy";
      return true;
    }
    cancel_ = false;
    const ros::WallTime started = ros::WallTime::now();
    const bool keep_log = recorder_ && recorder_->savesFrames();
    std::vector<std::string> strategy_log;
    std::vector<RetryFrameRecord> retries;  // numbered like DecodedCode::frame
    std::vector<ros::Time> retry_stamps;    // exact header stamps of retries
    BarcodeArray result;

    Frame frame;
    CameraModel camera;
    bool have_pose = false;
    TagPose pose;
    const ImageSubscription subscription(*this);
    {
      std::unique_lock<std::mutex> lock(mutex_);
      // First frame arrives ~0.2 s after subscribing (connection setup plus up to one frame period).
      update_cv_.wait_for(lock, std::chrono::duration<double>(image_wait_s_),
                          [&]() { return !frames_.empty() || cancel_.load(); });
      if (frames_.empty()) {
        result.error = cancel_.load() ? "cancelled" : "no_image";
      } else {
        // The vision node publishes a frame's pose ~50 ms after the image. Wait for any received frame
        // (more keep arriving meanwhile) to get its pose and decode the newest such frame.
        const double first_ms = (ros::WallTime::now() - started).toSec() * 1000.0;
        update_cv_.wait_for(lock, std::chrono::duration<double>(pose_wait_s_),
                            [&]() { return newestFrameWithPose() || cancel_.load(); });
        frame = frames_.back();
        if (const Frame* with_pose = newestFrameWithPose()) {
          frame = *with_pose;
          const auto* p = findPose(frame.stamp);
          pose = toTagPose(p->pose);
          have_pose = true;
          if (p->header.frame_id != frame.frame_id)
            ROS_WARN_THROTTLE(10.0, "barcode_reader_camera: pose frame %s != image frame %s",
                              p->header.frame_id.c_str(), frame.frame_id.c_str());
        }
        if (camera_.valid() && info_size_ == frame.gray.size()) camera = camera_;
        if (cancel_.load()) result.error = "cancelled";
        ROS_DEBUG("barcode_reader_camera: frame %.3f (latest %.3f) pose=%d, first image after %.0f ms, "
                  "decoding after %.0f ms", frame.stamp.toSec(), frames_.back().stamp.toSec(), (int)have_pose,
                  first_ms, (ros::WallTime::now() - started).toSec() * 1000.0);
      }
    }
    if (result.error.empty() && require_tag_ && !camera.valid()) result.error = "no_camera_info";
    if (result.error.empty() && require_tag_ && !have_pose) result.error = "no_tag";
    checkVisionTagSize();

    DecodeOutput out;
    if (result.error.empty()) {
      DecodeHooks hooks;
      hooks.cancelled = [this]() { return cancel_.load(); };
      hooks.log = [this, keep_log, &strategy_log](const std::string& line) {
        if (log_strategies_) ROS_INFO("%s", line.c_str());
        else ROS_DEBUG("%s", line.c_str());
        if (keep_log) strategy_log.push_back(line);
      };
      uint64_t last_seq = frame.seq;
      hooks.next_frame = [this, &last_seq, &retries, &retry_stamps](int timeout_ms, cv::Mat& gray, TagPose& tag) {
        std::unique_lock<std::mutex> lock(mutex_);
        // Newest frame after the previous one whose own pose has arrived (~40 ms after the image);
        // frames without a pose (tag not detected, or skipped by the vision node) are not tried.
        const Frame* next = nullptr;
        const geometry_msgs::PoseStamped* next_pose = nullptr;
        update_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&]() {
          if (cancel_.load()) return true;
          for (auto it = frames_.rbegin(); it != frames_.rend() && it->seq > last_seq; ++it) {
            if ((next_pose = findPose(it->stamp))) {
              next = &*it;
              return true;
            }
          }
          return false;
        });
        if (cancel_.load() || !next) return false;
        last_seq = next->seq;
        gray = next->gray;  // shares the buffer, no copy
        tag = toTagPose(next_pose->pose);
        RetryFrameRecord rec;
        rec.index = (int)retries.size() + 1;
        rec.stamp = next->stamp.toSec();
        rec.pose = tag;
        rec.gray = gray;
        retries.push_back(std::move(rec));
        retry_stamps.push_back(next->stamp);
        return true;
      };
      out = decodeBarcodes(frame.gray, have_pose && camera.valid() ? &pose : nullptr, camera, config_, hooks);
      result.success = out.success;
      result.partial = out.partial;
      result.error = out.error;
      result.used_tag_roi = out.used_tag_roi;
      result.report = out.report;
      for (const auto& c : out.codes) {
        Barcode b;
        b.format = c.format;
        b.code = c.code;
        b.raw_hex = toHex(c.bytes);
        b.symbology_id = c.symbology_id;
        b.center_x = c.center.x;
        b.center_y = c.center.y;
        for (int i = 0; i < 4; ++i) {
          b.corners[2 * i] = c.corners[i].x;
          b.corners[2 * i + 1] = c.corners[i].y;
        }
        b.source = c.source;
        result.codes.push_back(b);
      }
    }
    // The result refers to the frame that produced the last new code: a retry frame when retries read it.
    // Its stamp lets callers look up the pose of that frame (barcode_rc_trigger tag.source=img).
    const int result_index = out.result_frame;
    const bool from_retry = result_index > 0 && result_index <= (int)retries.size();
    result.header.stamp = from_retry ? retry_stamps[result_index - 1] : frame.stamp;
    result.header.frame_id = frame.frame_id;
    result.elapsed_s = (float)(ros::WallTime::now() - started).toSec();
    result_pub_.publish(result);
    if (monitor_result_ && monitor_result_.getNumSubscribers()) {
      for (size_t i = 0; i <= retries.size(); ++i) {
        BarcodeFrame observation;
        observation.header = result.header;
        observation.header.stamp = i ? retry_stamps[i - 1] : frame.stamp;
        const cv::Mat& image = i ? retries[i - 1].gray : frame.gray;
        observation.width = image.cols;
        observation.height = image.rows;
        observation.selfpath = image_topic_ == "/ov13855/image_raw";
        observation.status = result.error.empty() ? result.report : result.error;
        for (size_t j = 0; j < out.codes.size(); ++j)
          if (out.codes[j].frame == static_cast<int>(i)) observation.codes.push_back(result.codes.at(j));
        if (!observation.codes.empty() || (result.codes.empty() && i == 0)) monitor_result_.publish(observation);
      }
    }

    if (result.success) {
      ROS_INFO("barcode_reader_camera: %zu codes (%s) in %.0f ms, frames=%d result_frame=%d tag_roi=%d",
               result.codes.size(), result.report.c_str(), result.elapsed_s * 1000.0, out.frames,
               out.result_frame, (int)result.used_tag_roi);
    } else if (result.error == "decode_fail" || result.error == "partial") {
      // 定位到了但没读出/没读齐，是测距实验里的正常结果；链路问题（no_image/no_tag 等）仍然告警。
      ROS_INFO("barcode_reader_camera: decode %s, %zu codes (%s) in %.0f ms, frames=%d",
               result.error.c_str(), result.codes.size(), result.report.c_str(), result.elapsed_s * 1000.0,
               out.frames);
    } else {
      ROS_WARN("barcode_reader_camera: decode %s, %zu codes (%s) in %.0f ms", result.error.c_str(),
               result.codes.size(), result.report.c_str(), result.elapsed_s * 1000.0);
    }

    if (recorder_ && recorder_->enabled()) {
      DecodeRecord rec;
      rec.wall_time = started.toSec();
      rec.stamp = frame.stamp.toSec();
      rec.frame_id = frame.frame_id;
      rec.frame = frame.gray;
      // Keep the first save_retry_frames_ retry images, the one that read the code and the last one.
      const bool keep_images = recorder_->savesFrames();
      for (auto& f : retries) {
        const bool keep = keep_images && (save_retry_frames_ < 0 || f.index <= save_retry_frames_ ||
                                          f.index == result_index || f.index == (int)retries.size());
        if (!keep) f.gray.release();
      }
      rec.retries = std::move(retries);
      rec.have_pose = have_pose;
      rec.pose = pose;
      rec.camera = camera;
      rec.success = result.success;
      rec.partial = result.partial;
      rec.error = result.error;
      rec.elapsed_s = result.elapsed_s;
      rec.out = std::move(out);
      rec.log = std::move(strategy_log);
      rec.layout_file = layout_file_;
      rec.config_file = config_file_;
      if (!recorder_->submit(std::move(rec)))
        ROS_WARN_THROTTLE(5.0, "barcode_reader_camera: recorder busy, attempt not recorded");
      const std::string error = recorder_->takeError();
      if (!error.empty()) ROS_WARN_THROTTLE(10.0, "barcode_reader_camera: recording failed: %s", error.c_str());
    }

    res.success = result.success;
    res.partial = result.partial;
    res.error = result.error;
    res.elapsed_s = result.elapsed_s;
    res.used_tag_roi = result.used_tag_roi;
    res.report = result.report;
    res.codes = result.codes;
    res.header = result.header;
    return true;
  }

  // The pose (and so the ROI) is scaled by the vision node's tag size; warn if it was started with
  // a different layout. Warning only: a wrong ROI still falls back to the full-frame pyramid.
  void checkVisionTagSize() {
    if (layout_tag_size_m_ <= 0.0 || vision_tag_size_param_.empty()) return;
    double vision_size = 0.0;
    if (!ros::param::getCached(vision_tag_size_param_, vision_size)) return;
    if (std::abs(vision_size - layout_tag_size_m_) > 1e-6)
      ROS_WARN_THROTTLE(10.0, "barcode_reader_camera: %s=%.4fm but layout tag size is %.4fm; "
                        "start vision and reader with the same layout",
                        vision_tag_size_param_.c_str(), vision_size, layout_tag_size_m_);
  }

  bool onCancel(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    std::unique_lock<std::mutex> busy(decode_mutex_, std::try_to_lock);
    if (busy.owns_lock()) {
      res.success = false;
      res.message = "no decode in progress";
      return true;
    }
    cancel_ = true;
    update_cv_.notify_all();
    res.success = true;
    res.message = "cancel requested";
    return true;
  }

  DecoderConfig config_;
  std::string config_file_, layout_file_;
  double layout_tag_size_m_ = 0.0;  // 0 = no layout file
  std::string vision_tag_size_param_;
  bool require_tag_ = true;
  double image_wait_s_ = 1.0;   // max wait for the first image after subscribing
  double pose_wait_s_ = 0.2;    // max wait after the first image for a received frame's pose
  double pose_match_s_ = 0.01;  // max |pose stamp - image stamp| for the same sensor frame
  bool log_strategies_ = false;
  ros::NodeHandle nh_;
  std::string image_topic_;
  std::unique_ptr<FrameRecorder> recorder_;  // background writer; drains its queue on shutdown
  int save_retry_frames_ = 2;

  std::mutex mutex_;
  std::condition_variable update_cv_;  // new frame, new pose or cancel
  std::deque<Frame> frames_;
  uint64_t frame_seq_ = 0;
  std::deque<geometry_msgs::PoseStamped> poses_;
  CameraModel camera_;
  cv::Size info_size_;

  std::mutex decode_mutex_;
  std::atomic<bool> cancel_{false};

  ros::Publisher result_pub_, monitor_result_;
  ros::Subscriber image_sub_, info_sub_, pose_sub_;
  ros::ServiceServer decode_srv_, cancel_srv_;
};

}  // namespace barcode_reader_camera

int main(int argc, char** argv) {
  ros::init(argc, argv, "barcode_reader_camera");
  ros::NodeHandle nh, pnh("~");
  try {
    barcode_reader_camera::BarcodeReaderCameraNode node(nh, pnh);
    ros::AsyncSpinner spinner(4);  // image callbacks keep flowing while ~decode runs
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception& e) {
    ROS_FATAL("barcode_reader_camera: %s", e.what());
    return 1;
  }
  return 0;
}
