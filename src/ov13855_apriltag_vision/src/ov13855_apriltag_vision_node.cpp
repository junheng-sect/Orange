#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <ov13855_apriltag_vision/HomogeneousTransform.h>
#include <ov13855_apriltag_vision/apriltag_detector.hpp>
#include <ov13855_apriltag_vision/pose_solver.hpp>
#include <ov13855_apriltag_vision/TagObservationArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>
namespace {
bool finiteVec3(const cv::Vec3d& v) {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
bool rotationToQuaternion(const cv::Mat& rotation, geometry_msgs::Quaternion& quaternion)
{
  if (rotation.rows != 3 || rotation.cols != 3 || rotation.type() != CV_64F)
  {
    return false;
  }

  const double r00 = rotation.at<double>(0, 0);
  const double r01 = rotation.at<double>(0, 1);
  const double r02 = rotation.at<double>(0, 2);
  const double r10 = rotation.at<double>(1, 0);
  const double r11 = rotation.at<double>(1, 1);
  const double r12 = rotation.at<double>(1, 2);
  const double r20 = rotation.at<double>(2, 0);
  const double r21 = rotation.at<double>(2, 1);
  const double r22 = rotation.at<double>(2, 2);
  const double trace = r00 + r11 + r22;

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 0.0;
  if (trace > 0.0)
  {
    const double scale = std::sqrt(trace + 1.0) * 2.0;
    w = 0.25 * scale;
    x = (r21 - r12) / scale;
    y = (r02 - r20) / scale;
    z = (r10 - r01) / scale;
  }
  else if (r00 > r11 && r00 > r22)
  {
    const double scale = std::sqrt(1.0 + r00 - r11 - r22) * 2.0;
    w = (r21 - r12) / scale;
    x = 0.25 * scale;
    y = (r01 + r10) / scale;
    z = (r02 + r20) / scale;
  }
  else if (r11 > r22)
  {
    const double scale = std::sqrt(1.0 + r11 - r00 - r22) * 2.0;
    w = (r02 - r20) / scale;
    x = (r01 + r10) / scale;
    y = 0.25 * scale;
    z = (r12 + r21) / scale;
  }
  else
  {
    const double scale = std::sqrt(1.0 + r22 - r00 - r11) * 2.0;
    w = (r10 - r01) / scale;
    x = (r02 + r20) / scale;
    y = (r12 + r21) / scale;
    z = 0.25 * scale;
  }

  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (!std::isfinite(norm) || norm <= 1e-12)
  {
    return false;
  }
  quaternion.x = x / norm;
  quaternion.y = y / norm;
  quaternion.z = z / norm;
  quaternion.w = w / norm;
  return std::isfinite(quaternion.x) && std::isfinite(quaternion.y) &&
         std::isfinite(quaternion.z) && std::isfinite(quaternion.w);
}

// INTER_AREA is only fast for integer factors (2560 -> 960 costs ~40 ms CPU on RK3588):
// halve with INTER_AREA, then finish the remaining < 2x step with INTER_LINEAR.
cv::Mat downscale(const cv::Mat& image, double scale)
{
  const cv::Size target(cvRound(image.cols * scale), cvRound(image.rows * scale));
  cv::Mat current = image;
  while (current.cols / 2 >= target.width && current.rows / 2 >= target.height)
  {
    cv::Mat half;
    cv::resize(current, half, cv::Size(current.cols / 2, current.rows / 2), 0, 0, cv::INTER_AREA);
    current = half;
  }
  if (current.size() != target)
  {
    cv::Mat resized;
    cv::resize(current, resized, target, 0, 0, cv::INTER_LINEAR);
    current = resized;
  }
  return current;
}

}

class Ov13855AprilTagVisionNode {
 public:
  Ov13855AprilTagVisionNode() : pnh_("~"), image_transport_(nh_) {
    loadParameters();
    detector_.reset(new ov13855_apriltag_vision::AprilTagDetector(
        tag_family_, quad_decimate_, nthreads_, refine_edges_, detection_max_dimension_));
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 1);
    if (pnh_.param("publish_detections", false)) {
      detections_pub_ = pnh_.advertise<ov13855_apriltag_vision::TagObservationArray>("detections", 2);
    }
    transform_debug_pub_ = nh_.advertise<ov13855_apriltag_vision::HomogeneousTransform>(transform_debug_topic_, 1);
    status_pub_ = nh_.advertise<std_msgs::String>(status_topic_, 1, true);
    processing_time_pub_ = nh_.advertise<std_msgs::Float64>(processing_time_topic_, 10);
    // image_transport adds /compressed etc.; preview is the downscaled view meant for remote RViz.
    if (publish_annotated_) annotated_pub_ = image_transport_.advertise(annotated_image_topic_, 1);
    if (publish_preview_) preview_pub_ = image_transport_.advertise(preview_image_topic_, 1);
    image_sub_ = nh_.subscribe(image_topic_, 1, &Ov13855AprilTagVisionNode::imageCallback,
                              this, ros::TransportHints().tcpNoDelay());
    camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1, &Ov13855AprilTagVisionNode::cameraInfoCallback, this);
    publishStatus("WAIT_CAMERA_INFO: waiting for " + camera_info_topic_, true);
    ROS_INFO_STREAM("ov13855_apriltag_vision | image=" << image_topic_ << " | family=" << tag_family_
        << " | selection=" << selection_mode_ << " | tag_size=" << target_marker_size_m_
        << "m | solver=SOLVEPNP_ITERATIVE (Drone-Vision) | pose=" << pose_topic_);
  }
 private:
  void loadParameters() {
    pnh_.param<std::string>("image_topic", image_topic_, "/ov13855/image_raw");
    pnh_.param<std::string>("camera_info_topic", camera_info_topic_, "/ov13855/camera_info");
    pnh_.param<std::string>("pose_topic", pose_topic_, "/front_marker/pose");
    pnh_.param<std::string>("transform_debug_topic", transform_debug_topic_, "/ov13855_apriltag_vision/debug/homogeneous_transform");
    pnh_.param<std::string>("processing_time_topic", processing_time_topic_, "/ov13855_apriltag_vision/debug/processing_time_ms");
    pnh_.param<std::string>("annotated_image_topic", annotated_image_topic_, "/ov13855_apriltag_vision/image_annotated");
    pnh_.param<std::string>("status_topic", status_topic_, "/ov13855_apriltag_vision/status");
    pnh_.param<std::string>("tag_family", tag_family_, "tag36h11");
    pnh_.param<std::string>("selection_mode", selection_mode_, "largest");
    pnh_.param("target_marker_id", fixed_marker_id_, 0);
    pnh_.param("target_marker_size_m", target_marker_size_m_, 0.012);
    pnh_.param("quad_decimate", quad_decimate_, 1.5);
    pnh_.param("nthreads", nthreads_, 4);
    pnh_.param("refine_edges", refine_edges_, true);
    pnh_.param("detection_max_dimension", detection_max_dimension_, 1280);
    pnh_.param("publish_marker_tf", publish_marker_tf_, true);
    pnh_.param<std::string>("marker_frame", marker_frame_, "front_marker");
    pnh_.param("publish_annotated", publish_annotated_, true);
    pnh_.param("annotated_rate_hz", annotated_rate_hz_, 5.0);
    pnh_.param("publish_preview", publish_preview_, true);
    pnh_.param<std::string>("preview_image_topic", preview_image_topic_, "/ov13855_apriltag_vision/image_preview");
    pnh_.param("preview_max_dimension", preview_max_dimension_, 960);
    pnh_.param("preview_rate_hz", preview_rate_hz_, 0.0);
    if (fixed_marker_id_ < 0 || (selection_mode_ != "largest" && selection_mode_ != "fixed"))
      throw std::runtime_error("selection_mode must be largest/fixed and target_marker_id non-negative");
    if (!std::isfinite(target_marker_size_m_) || target_marker_size_m_ <= 0.0)
      throw std::runtime_error("target_marker_size_m must be positive and finite");
    if (publish_marker_tf_ && marker_frame_.empty()) throw std::runtime_error("marker_frame must not be empty");
    if (!std::isfinite(annotated_rate_hz_)) throw std::runtime_error("annotated_rate_hz must be finite");
    if (!std::isfinite(preview_rate_hz_)) throw std::runtime_error("preview_rate_hz must be finite");
    if (publish_preview_ && preview_max_dimension_ <= 0)
      throw std::runtime_error("preview_max_dimension must be positive");
  }
  void invalidateCameraInfo(const std::string& reason) {
    has_camera_info_ = false;
    camera_info_error_ = reason;
    publishStatus("INVALID_CAMERA_INFO: " + reason);
  }
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& message) {
    if (message->width == 0 || message->height == 0 || message->header.frame_id.empty()) {
      invalidateCameraInfo("image dimensions and frame_id are required"); return;
    }
    if (message->distortion_model != "plumb_bob" && message->distortion_model != "rational_polynomial") {
      invalidateCameraInfo("unsupported distortion model"); return;
    }
    for (double value : message->K) {
      if (!std::isfinite(value)) { invalidateCameraInfo("non-finite camera matrix"); return; }
    }
    if (message->K[0] <= 0.0 || message->K[4] <= 0.0) {
      invalidateCameraInfo("fx and fy must be positive"); return;
    }
    const size_t size = message->D.size();
    if (!(size == 0 || size == 4 || size == 5 || size == 8 || size == 12 || size == 14)) {
      invalidateCameraInfo("unsupported distortion coefficient count"); return;
    }
    for (double value : message->D) {
      if (!std::isfinite(value)) { invalidateCameraInfo("non-finite distortion"); return; }
    }
    camera_matrix_ = cv::Mat(3,3,CV_64F);
    std::copy(message->K.begin(), message->K.end(), camera_matrix_.ptr<double>());
    dist_coeffs_ = cv::Mat::zeros(1, static_cast<int>(size ? size : 5), CV_64F);
    std::copy(message->D.begin(), message->D.end(), dist_coeffs_.ptr<double>());
    camera_info_frame_ = message->header.frame_id;
    camera_info_width_ = message->width;
    camera_info_height_ = message->height;
    has_camera_info_ = true;
    camera_info_error_.clear();
  }
  void imageCallback(const sensor_msgs::ImageConstPtr& message) {
    const auto start = ros::WallTime::now();
    ++frame_count_;
    cv_bridge::CvImageConstPtr source;
    cv::Mat gray;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    int selected = -1;
    ov13855_apriltag_vision::PoseSolution pose;
    bool published = false;
    std::string status;
    try {
      if (message->encoding == sensor_msgs::image_encodings::MONO8) {
        source = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::MONO8);
        gray = source->image;
      } else if (message->encoding == sensor_msgs::image_encodings::RGB8) {
        source = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::RGB8);
        cv::cvtColor(source->image, gray, cv::COLOR_RGB2GRAY);
      } else {
        source = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8);
        cv::cvtColor(source->image, gray, cv::COLOR_BGR2GRAY);
      }
      const auto detections = detector_->detect(gray);
      for (const auto& detection : detections) {
        ids.push_back(detection.id);
        corners.push_back(detection.corners);
      }
      selected = ov13855_apriltag_vision::selectDetection(detections, selection_mode_, fixed_marker_id_);
      if (selected >= 0) { ++marker_count_; target_marker_id_ = ids[selected]; }
      if (!has_camera_info_) {
        status = selected >= 0 ? "DETECTED_2D: ID" + std::to_string(target_marker_id_) : "WAIT_MARKER_2D: no matching tag";
        status += " | POSE_DISABLED: " + (camera_info_error_.empty() ? "waiting for CameraInfo" : camera_info_error_);
      } else if (message->width != camera_info_width_ || message->height != camera_info_height_) {
        status = "IMAGE_CAMERA_INFO_SIZE_MISMATCH: image and CameraInfo dimensions differ";
      } else if (message->header.frame_id.empty() || camera_info_frame_ != message->header.frame_id) {
        status = "FRAME_MISMATCH: image and CameraInfo require the same non-empty frame_id";
      } else {
        if (detections_pub_ && detections_pub_.getNumSubscribers() > 0)
          publishDetections(message->header, detections);
        if (selected < 0) {
          status = "WAIT_MARKER: no matching tag";
        } else {
          std::string reason;
          if (ov13855_apriltag_vision::solveTagPose(corners[selected], target_marker_size_m_,
              camera_matrix_, dist_coeffs_, pose, reason)) {
            published = publishPoseAndDebug(message, pose.rvec, pose.tvec);
            if (!published) reason = "cannot construct finite pose transform";
          }
          if (published) {
            ++detect_count_;
            std::ostringstream text;
            text << std::fixed << std::setprecision(4) << "DETECTED: ID" << target_marker_id_
                 << " xyz_camera=(" << pose.tvec[0] << "," << pose.tvec[1] << "," << pose.tvec[2]
                 << ")m pipeline=SOLVEPNP_ITERATIVE";
            status = text.str();
          } else {
            ++pose_reject_count_;
            status = "SOLVEPNP_FAILED: ID" + std::to_string(target_marker_id_) + " " + reason;
          }
        }
      }
    } catch (const std::exception& error) {
      status = std::string("IMAGE_PROCESSING_ERROR: ") + error.what();
    }
    publishStatus(status);
    if (!gray.empty()) {
      publishDebugImages(message, gray, corners, ids, selected,
                         published ? &pose.rvec : nullptr, published ? &pose.tvec : nullptr, status);
    }
    publishProcessingTime(start);
  }
  static bool rateAllows(double rate_hz, ros::Time& last_time, const ros::Time& now)
  {
    if (rate_hz <= 0.0)
    {
      return true;
    }
    const double period = 1.0 / rate_hz;
    if (last_time.isZero() || (now - last_time).toSec() >= period)
    {
      last_time = now;
      return true;
    }
    return false;
  }

  void publishProcessingTime(const ros::WallTime& start)
  {
    std_msgs::Float64 message;
    const double elapsed_ms = (ros::WallTime::now() - start).toSec() * 1000.0;
    message.data = std::isfinite(elapsed_ms) && elapsed_ms >= 0.0 ? elapsed_ms : 0.0;
    processing_time_pub_.publish(message);
  }

  void publishDetections(
      const std_msgs::Header& header,
      const std::vector<ov13855_apriltag_vision::TagDetection>& detections)
  {
    ov13855_apriltag_vision::TagObservationArray message;
    message.header = header;
    message.family = tag_family_;
    const float h = static_cast<float>(target_marker_size_m_ / 2.0);
    const std::vector<cv::Point3f> object{{-h,-h,0}, {h,-h,0}, {h,h,0}, {-h,h,0}};
    for (const auto& detection : detections) {
      ov13855_apriltag_vision::PoseSolution pose;
      std::string reason;
      if (!ov13855_apriltag_vision::solveTagPose(detection.corners, target_marker_size_m_,
                                               camera_matrix_, dist_coeffs_, pose, reason))
        continue;
      ov13855_apriltag_vision::TagObservation observation;
      observation.id = detection.id;
      cv::Mat rotation;
      cv::Rodrigues(pose.rvec, rotation);
      if (!rotationToQuaternion(rotation, observation.pose.orientation)) continue;
      observation.pose.position.x = pose.tvec[0];
      observation.pose.position.y = pose.tvec[1];
      observation.pose.position.z = pose.tvec[2];
      std::vector<cv::Point2f> projected;
      cv::projectPoints(object, pose.rvec, pose.tvec, camera_matrix_, dist_coeffs_, projected);
      observation.reprojection_rms_px = cv::norm(projected, detection.corners, cv::NORM_L2) / 2.0;
      if (std::isfinite(observation.reprojection_rms_px)) message.tags.push_back(observation);
    }
    detections_pub_.publish(message);
  }

  bool publishPoseAndDebug(
    const sensor_msgs::ImageConstPtr& image_message,
    const cv::Vec3d& rvec,
    const cv::Vec3d& tvec)
  {
    cv::Mat rotation;
    cv::Rodrigues(rvec, rotation);
    if (rotation.rows != 3 || rotation.cols != 3)
    {
      return false;
    }

    if (!finiteVec3(tvec))
    {
      return false;
    }

    geometry_msgs::PoseStamped pose;
    pose.header = image_message->header;
    pose.pose.position.x = tvec[0];
    pose.pose.position.y = tvec[1];
    pose.pose.position.z = tvec[2];
    if (!rotationToQuaternion(rotation, pose.pose.orientation))
    {
      return false;
    }

    ov13855_apriltag_vision::HomogeneousTransform transform;
    transform.header = pose.header;
    transform.marker_id = target_marker_id_;
    for (int row = 0; row < 3; ++row)
    {
      for (int column = 0; column < 3; ++column)
      {
        const double value = rotation.at<double>(row, column);
        if (!std::isfinite(value))
        {
          return false;
        }
        transform.matrix[static_cast<size_t>(row * 4 + column)] = value;
      }
      transform.matrix[static_cast<size_t>(row * 4 + 3)] = tvec[row];
    }
    transform.matrix[12] = 0.0;
    transform.matrix[13] = 0.0;
    transform.matrix[14] = 0.0;
    transform.matrix[15] = 1.0;

    pose_pub_.publish(pose);
    transform_debug_pub_.publish(transform);
    if (publish_marker_tf_)
    {
      if (pose.header.frame_id == marker_frame_)
      {
        ROS_ERROR_THROTTLE(
          1.0, "Refusing marker TF with identical parent and child frame '%s'",
          marker_frame_.c_str());
        return true;
      }

      geometry_msgs::TransformStamped marker_transform;
      marker_transform.header = pose.header;
      marker_transform.child_frame_id = marker_frame_;
      marker_transform.transform.translation.x = pose.pose.position.x;
      marker_transform.transform.translation.y = pose.pose.position.y;
      marker_transform.transform.translation.z = pose.pose.position.z;
      marker_transform.transform.rotation = pose.pose.orientation;
      marker_tf_broadcaster_.sendTransform(marker_transform);
    }
    return true;
  }

  void publishDebugImages(
    const sensor_msgs::ImageConstPtr& source,
    const cv::Mat& gray,
    const std::vector<std::vector<cv::Point2f>>& corners,
    const std::vector<int>& ids,
    int selected_index,
    const cv::Vec3d* rvec,
    const cv::Vec3d* tvec,
    const std::string& status)
  {
    const ros::Time now = ros::Time::now();
    if (publish_annotated_ && annotated_pub_.getNumSubscribers() > 0 &&
        rateAllows(annotated_rate_hz_, last_annotated_time_, now))
    {
      publishAnnotatedImage(annotated_pub_, source, gray, corners, ids, selected_index, rvec, tvec, status, 1.0);
    }
    if (publish_preview_ && preview_pub_.getNumSubscribers() > 0 &&
        rateAllows(preview_rate_hz_, last_preview_time_, now))
    {
      const int longest = std::max(gray.cols, gray.rows);
      const double scale = longest > preview_max_dimension_ ?
        static_cast<double>(preview_max_dimension_) / longest : 1.0;
      publishAnnotatedImage(preview_pub_, source, gray, corners, ids, selected_index, rvec, tvec, status, scale);
    }
  }

  // Draws at the output resolution (scale <= 1) so overlay text stays readable in the preview.
  void publishAnnotatedImage(
    const image_transport::Publisher& publisher,
    const sensor_msgs::ImageConstPtr& source,
    const cv::Mat& gray,
    const std::vector<std::vector<cv::Point2f>>& corners,
    const std::vector<int>& ids,
    int selected_index,
    const cv::Vec3d* rvec,
    const cv::Vec3d* tvec,
    const std::string& status,
    double scale)
  {
    cv::Mat image;
    try
    {
      if (source->encoding == sensor_msgs::image_encodings::MONO8)
      {
        image = gray;
      }
      else if (source->encoding == sensor_msgs::image_encodings::RGB8)
      {
        const cv_bridge::CvImageConstPtr rgb =
          cv_bridge::toCvShare(source, sensor_msgs::image_encodings::RGB8);
        cv::cvtColor(rgb->image, image, cv::COLOR_RGB2BGR);
      }
      else
      {
        image = cv_bridge::toCvShare(source, sensor_msgs::image_encodings::BGR8)->image;
      }
    }
    catch (const cv_bridge::Exception& exception)
    {
      ROS_ERROR_THROTTLE(1.0, "Annotated image conversion failed: %s", exception.what());
      return;
    }
    // Resize before colour conversion; never draw into the shared source buffer.
    const cv::Mat scaled = scale < 1.0 ? downscale(image, scale) : image;
    cv::Mat annotated;
    if (scaled.channels() == 1)
    {
      cv::cvtColor(scaled, annotated, cv::COLOR_GRAY2BGR);
    }
    else
    {
      annotated = scaled.data == image.data ? scaled.clone() : scaled;
    }
    if (!ids.empty())
    {
      for (size_t i = 0; i < corners.size(); ++i)
      {
        for (int k = 0; k < 4; ++k)
          cv::line(annotated, corners[i][k] * scale, corners[i][(k + 1) % 4] * scale, cv::Scalar(0,255,0), 2);
        cv::putText(annotated, std::to_string(ids[i]), corners[i][0] * scale,
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);
      }
    }
    if (selected_index >= 0 && rvec != nullptr && tvec != nullptr)
    {
      // Distortion acts on normalised coordinates, so scaling fx/fy/cx/cy matches the resized image.
      cv::Mat camera_matrix = camera_matrix_.clone();
      cv::Mat focal_rows = camera_matrix.rowRange(0, 2);
      focal_rows *= scale;
      cv::drawFrameAxes(
        annotated, camera_matrix, dist_coeffs_, *rvec, *tvec, target_marker_size_m_ * 0.5);
    }
    std::ostringstream summary;
    summary << "F=" << frame_count_ << " Seen=" << marker_count_ << " OK=" << detect_count_
            << " SolveFailed=" << pose_reject_count_;
    cv::putText(
      annotated, summary.str(), cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX, 0.55,
      cv::Scalar(255, 255, 255), 1);
    cv::putText(
      annotated, status, cv::Point(10, 48), cv::FONT_HERSHEY_SIMPLEX, 0.48,
      status.rfind("DETECTED", 0) == 0 ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 1);

    cv_bridge::CvImage output;
    output.header = source->header;
    output.encoding = sensor_msgs::image_encodings::BGR8;
    output.image = annotated;
    publisher.publish(output.toImageMsg());
  }

  void publishStatus(const std::string& text, bool force = false)
  {
    const ros::Time now = ros::Time::now();
    if (!force && text == last_status_ && !last_status_time_.isZero() &&
        (now - last_status_time_).toSec() < 1.0)
    {
      return;
    }
    last_status_ = text;
    last_status_time_ = now;
    std_msgs::String message;
    message.data = text;
    status_pub_.publish(message);
  }


  ros::NodeHandle nh_, pnh_;
  ros::Subscriber image_sub_, camera_info_sub_;
  image_transport::ImageTransport image_transport_;
  ros::Publisher detections_pub_, pose_pub_, transform_debug_pub_, status_pub_, processing_time_pub_;
  image_transport::Publisher annotated_pub_, preview_pub_;
  tf2_ros::TransformBroadcaster marker_tf_broadcaster_;
  std::string image_topic_, camera_info_topic_, pose_topic_, transform_debug_topic_;
  std::string processing_time_topic_, annotated_image_topic_, preview_image_topic_, status_topic_, tag_family_, selection_mode_;
  std::string marker_frame_, camera_info_frame_, camera_info_error_, last_status_;
  int fixed_marker_id_ = 0, target_marker_id_ = -1, nthreads_ = 4, detection_max_dimension_ = 1280;
  int preview_max_dimension_ = 960;
  double target_marker_size_m_ = 0.012, quad_decimate_ = 1.5, annotated_rate_hz_ = 5.0, preview_rate_hz_ = 0.0;
  bool refine_edges_ = true, publish_marker_tf_ = true, publish_annotated_ = true, publish_preview_ = true;
  bool has_camera_info_ = false;
  uint32_t camera_info_width_ = 0, camera_info_height_ = 0;
  uint64_t frame_count_ = 0, marker_count_ = 0, detect_count_ = 0, pose_reject_count_ = 0;
  std::unique_ptr<ov13855_apriltag_vision::AprilTagDetector> detector_;
  cv::Mat camera_matrix_, dist_coeffs_;
  ros::Time last_annotated_time_, last_preview_time_, last_status_time_;
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "ov13855_apriltag_vision_node");
  ros::NodeHandle lifetime;
  try { Ov13855AprilTagVisionNode node; ros::spin(); }
  catch (const std::exception& error) { ROS_FATAL("ov13855_apriltag_vision: %s", error.what()); return 1; }
  return 0;
}
