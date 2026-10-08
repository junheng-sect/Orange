// ROS adapter for decodeBarcodes(): ~decode decodes the latest camera frame, using the label ROI
// when ov13855_apriltag_vision published a pose with the same stamp; ~cancel interrupts it.
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>

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
#include "barcode_reader_camera/DecodeBarcodes.h"
#include "barcode_reader_camera/config_yaml.hpp"
#include "barcode_reader_camera/multi_decoder.hpp"

namespace barcode_reader_camera {

class BarcodeReaderCameraNode {
 public:
  BarcodeReaderCameraNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    const std::string config_file = pnh.param<std::string>(
        "config_file", ros::package::getPath("barcode_reader_camera") + "/config/barcode_reader_camera.yaml");
    config_ = loadDecoderConfig(YAML::LoadFile(config_file));
    const std::string layout_file = pnh.param<std::string>("layout_file", "");
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
    max_frame_age_s_ = pnh.param("max_frame_age_s", 1.0);
    log_strategies_ = pnh.param("log_strategies", false);
    const auto image_topic = pnh.param<std::string>("image_topic", "/ov13855/image_raw");
    const auto info_topic = pnh.param<std::string>("camera_info_topic", "/ov13855/camera_info");
    const auto pose_topic = pnh.param<std::string>("pose_topic", "/front_marker/pose");

    result_pub_ = pnh.advertise<BarcodeArray>("result", 10);
    annotated_pub_ = pnh.advertise<sensor_msgs::Image>("image_annotated", 1);
    image_sub_ = nh.subscribe(image_topic, 2, &BarcodeReaderCameraNode::onImage, this);
    info_sub_ = nh.subscribe(info_topic, 1, &BarcodeReaderCameraNode::onInfo, this);
    pose_sub_ = nh.subscribe(pose_topic, 10, &BarcodeReaderCameraNode::onPose, this);
    decode_srv_ = pnh.advertiseService("decode", &BarcodeReaderCameraNode::onDecode, this);
    cancel_srv_ = pnh.advertiseService("cancel", &BarcodeReaderCameraNode::onCancel, this);
    ROS_INFO("barcode_reader_camera: config=%s image=%s pose=%s require_tag=%d",
             config_file.c_str(), image_topic.c_str(), pose_topic.c_str(), (int)require_tag_);
  }

 private:
  struct Frame {
    ros::Time stamp;
    std::string frame_id;
    cv::Mat gray;
    uint64_t seq = 0;
  };

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
    frame_cv_.notify_all();
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
  }

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
    std::unique_lock<std::mutex> busy(decode_mutex_, std::try_to_lock);
    if (!busy.owns_lock()) {
      res.error = "busy";
      return true;
    }
    cancel_ = false;
    const ros::WallTime started = ros::WallTime::now();
    BarcodeArray result;

    Frame frame;
    CameraModel camera;
    bool have_pose = false;
    TagPose pose;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (frames_.empty() || (ros::Time::now() - frames_.back().stamp).toSec() > max_frame_age_s_) {
        result.error = "no_image";
      } else {
        frame = frames_.back();
        if (camera_.valid() && info_size_ == frame.gray.size()) camera = camera_;
        for (auto it = poses_.rbegin(); it != poses_.rend(); ++it) {
          if (it->header.stamp == frame.stamp) {
            pose = toTagPose(it->pose);
            have_pose = true;
            if (it->header.frame_id != frame.frame_id)
              ROS_WARN_THROTTLE(10.0, "barcode_reader_camera: pose frame %s != image frame %s",
                                it->header.frame_id.c_str(), frame.frame_id.c_str());
            break;
          }
        }
      }
    }
    if (result.error.empty() && require_tag_ && !camera.valid()) result.error = "no_camera_info";
    if (result.error.empty() && require_tag_ && !have_pose) result.error = "no_tag";
    checkVisionTagSize();

    DecodeOutput out;
    if (result.error.empty()) {
      DecodeHooks hooks;
      hooks.cancelled = [this]() { return cancel_.load(); };
      if (log_strategies_) hooks.log = [](const std::string& line) { ROS_INFO("%s", line.c_str()); };
      else hooks.log = [](const std::string& line) { ROS_DEBUG("%s", line.c_str()); };
      uint64_t last_seq = frame.seq;
      hooks.next_frame = [this, &last_seq](cv::Mat& gray) {
        std::unique_lock<std::mutex> lock(mutex_);
        const bool got = frame_cv_.wait_for(lock, std::chrono::seconds(1), [&]() {
          return cancel_.load() || (!frames_.empty() && frames_.back().seq > last_seq);
        });
        if (!got || cancel_.load()) return false;
        last_seq = frames_.back().seq;
        gray = frames_.back().gray;
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
    result.header.stamp = frame.stamp;
    result.header.frame_id = frame.frame_id;
    result.elapsed_s = (float)(ros::WallTime::now() - started).toSec();
    result_pub_.publish(result);
    if (!frame.gray.empty() && annotated_pub_.getNumSubscribers() > 0) {
      cv_bridge::CvImage annotated(result.header, "bgr8");
      cv::cvtColor(frame.gray, annotated.image, cv::COLOR_GRAY2BGR);
      drawDecodeOutput(annotated.image, out);
      annotated_pub_.publish(annotated.toImageMsg());
    }
    if (result.success) {
      ROS_INFO("barcode_reader_camera: %zu codes (%s) in %.0f ms, tag_roi=%d", result.codes.size(),
               result.report.c_str(), result.elapsed_s * 1000.0, (int)result.used_tag_roi);
    } else {
      ROS_WARN("barcode_reader_camera: decode %s, %zu codes (%s) in %.0f ms", result.error.c_str(),
               result.codes.size(), result.report.c_str(), result.elapsed_s * 1000.0);
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
    frame_cv_.notify_all();
    res.success = true;
    res.message = "cancel requested";
    return true;
  }

  DecoderConfig config_;
  double layout_tag_size_m_ = 0.0;  // 0 = no layout file
  std::string vision_tag_size_param_;
  bool require_tag_ = true;
  double max_frame_age_s_ = 1.0;
  bool log_strategies_ = false;

  std::mutex mutex_;
  std::condition_variable frame_cv_;
  std::deque<Frame> frames_;
  uint64_t frame_seq_ = 0;
  std::deque<geometry_msgs::PoseStamped> poses_;
  CameraModel camera_;
  cv::Size info_size_;

  std::mutex decode_mutex_;
  std::atomic<bool> cancel_{false};

  ros::Publisher result_pub_, annotated_pub_;
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
