#include <algorithm>
#include <cmath>
#include <cctype>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <front_marker_vision/HomogeneousTransform.h>
#include <front_marker_vision/marker_pose_estimator.hpp>
#include <front_marker_vision/roi_tracker.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>

namespace
{

std::string uppercase(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::toupper(character));
  });
  return value;
}

bool finiteVec3(const cv::Vec3d& value)
{
  return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

const char* poseReferenceStateName(front_marker_vision::PoseReferenceState state)
{
  switch (state)
  {
    case front_marker_vision::PoseReferenceState::UNINITIALIZED:
      return "UNINITIALIZED";
    case front_marker_vision::PoseReferenceState::BOOTSTRAP:
      return "BOOTSTRAP";
    case front_marker_vision::PoseReferenceState::TRACKING:
      return "TRACKING";
    case front_marker_vision::PoseReferenceState::SWITCH_PENDING:
      return "SWITCH_PENDING";
  }
  return "UNKNOWN";
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

}  // namespace

class FrontMarkerVisionNode
{
public:
  FrontMarkerVisionNode()
    : nh_()
    , pnh_("~")
  {
    loadParameters();
    roi_tracker_.setEnabled(enable_roi_);
    configureDictionary();

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 1);
    transform_debug_pub_ =
      nh_.advertise<front_marker_vision::HomogeneousTransform>(transform_debug_topic_, 1);
    status_pub_ = nh_.advertise<std_msgs::String>(status_topic_, 1, true);
    processing_time_pub_ = nh_.advertise<std_msgs::Float64>(processing_time_topic_, 10);
    if (publish_annotated_)
    {
      annotated_pub_ = nh_.advertise<sensor_msgs::Image>(annotated_image_topic_, 1);
    }

    const ros::TransportHints transport_hints = ros::TransportHints().tcpNoDelay();
    image_sub_ = nh_.subscribe(image_topic_, 1, &FrontMarkerVisionNode::imageCallback, this, transport_hints);
    camera_info_sub_ = nh_.subscribe(
      camera_info_topic_, 1, &FrontMarkerVisionNode::cameraInfoCallback, this);

    publishStatus("WAIT_CAMERA_INFO: waiting for " + camera_info_topic_, true);
    ROS_INFO_STREAM("front_marker_vision_node started | image=" << image_topic_
                    << " | camera_info=" << camera_info_topic_
                    << " | pose=" << pose_topic_
                    << " | dictionary=" << dictionary_name_
                    << " | target=ID" << target_marker_id_
                    << " size=" << target_marker_size_m_ << "m"
                    << " | contour_gate=" << (enable_contour_gate_ ? "true" : "false")
                    << " | roi=" << (enable_roi_ ? "adaptive" : "full-frame")
                    << " | solver=" << pose_solver_
                    << " | reference_confirm_frames="
                    << pose_reference_options_.confirmation_frames
                    << " | marker_tf=" << (publish_marker_tf_ ? marker_frame_ : "disabled")
                    << " | annotated=" << (publish_annotated_ ? "true" : "false"));
  }

private:
  void loadParameters()
  {
    pnh_.param<std::string>("image_topic", image_topic_, "/usb_cam/image_raw");
    pnh_.param<std::string>("camera_info_topic", camera_info_topic_, "/usb_cam/camera_info");
    pnh_.param<std::string>("pose_topic", pose_topic_, "/front_marker/pose");
    pnh_.param<std::string>(
      "transform_debug_topic", transform_debug_topic_,
      "/front_marker_vision/debug/homogeneous_transform");
    pnh_.param<std::string>(
      "processing_time_topic", processing_time_topic_,
      "/front_marker_vision/debug/processing_time_ms");
    pnh_.param<std::string>(
      "annotated_image_topic", annotated_image_topic_, "/front_marker_vision/image_annotated");
    pnh_.param<std::string>(
      "status_topic", status_topic_, "/front_marker_vision/status");
    pnh_.param<std::string>("aruco_dictionary", dictionary_name_, "DICT_5X5_1000");
    pnh_.param("target_marker_id", target_marker_id_, 33);
    pnh_.param("target_marker_size_m", target_marker_size_m_, 0.070);
    pnh_.param("publish_marker_tf", publish_marker_tf_, true);
    pnh_.param<std::string>("marker_frame", marker_frame_, "front_marker");
    pnh_.param("publish_annotated", publish_annotated_, true);
    pnh_.param("annotated_rate_hz", annotated_rate_hz_, 5.0);
    pnh_.param("enable_contour_gate", enable_contour_gate_, true);
    pnh_.param("enable_roi", enable_roi_, true);
    pnh_.param<std::string>("pose_solver", pose_solver_, "IPPE_SQUARE");
    pnh_.param("min_edge_length_px", contour_options_.min_edge_length_px, 12.0);
    pnh_.param("edge_sample_offset_ratio", contour_options_.edge_sample_offset_ratio, 0.025);
    pnh_.param("edge_sample_offset_min_px", contour_options_.edge_sample_offset_min_px, 1.0);
    pnh_.param("edge_sample_offset_max_px", contour_options_.edge_sample_offset_max_px, 4.0);
    pnh_.param("edge_sample_margin_ratio", contour_options_.edge_sample_margin_ratio, 0.15);
    pnh_.param("min_edge_contrast", contour_options_.min_edge_contrast, 25.0);
    pnh_.param("min_mean_edge_contrast", contour_options_.min_mean_edge_contrast, 40.0);
    pnh_.param("edge_samples_per_side", contour_options_.samples_per_edge, 32);
    pnh_.param("ippe_ambiguity_gap_px", ippe_ambiguity_gap_px_, 0.2);
    pnh_.param("max_reprojection_error_px", max_reprojection_error_px_, 1.0);
    pnh_.param("pose_continuity_timeout_sec", pose_continuity_timeout_sec_, 0.5);
    pnh_.param(
      "ippe_reference_confirm_frames", pose_reference_options_.confirmation_frames, 3);
    pnh_.param(
      "ippe_reference_max_rotation_step_rad",
      pose_reference_options_.max_rotation_step_rad,
      0.35);
    pnh_.param(
      "ippe_reference_max_translation_step_m",
      pose_reference_options_.max_translation_step_m,
      0.10);
    pose_reference_options_.timeout_sec = pose_continuity_timeout_sec_;

    pose_solver_ = uppercase(pose_solver_);

    if (target_marker_id_ < 0)
    {
      throw std::runtime_error("target_marker_id must be non-negative");
    }
    if (!std::isfinite(target_marker_size_m_) || target_marker_size_m_ <= 0.0)
    {
      throw std::runtime_error("target_marker_size_m must be finite and greater than zero");
    }
    if (publish_marker_tf_ && marker_frame_.empty())
    {
      throw std::runtime_error("marker_frame must not be empty when publish_marker_tf is true");
    }
    if (!(pose_solver_ == "LEGACY" || pose_solver_ == "IPPE_SQUARE"))
    {
      throw std::runtime_error("pose_solver must be LEGACY or IPPE_SQUARE");
    }
    if (contour_options_.min_edge_length_px <= 0.0 ||
        contour_options_.edge_sample_offset_ratio <= 0.0 ||
        contour_options_.edge_sample_offset_min_px <= 0.0 ||
        contour_options_.edge_sample_offset_max_px < contour_options_.edge_sample_offset_min_px ||
        contour_options_.edge_sample_margin_ratio < 0.0 ||
        contour_options_.edge_sample_margin_ratio >= 0.5 ||
        contour_options_.samples_per_edge < 4)
    {
      throw std::runtime_error("invalid contour gate parameters");
    }
    if (!std::isfinite(ippe_ambiguity_gap_px_) || ippe_ambiguity_gap_px_ < 0.0 ||
        !std::isfinite(max_reprojection_error_px_) || max_reprojection_error_px_ <= 0.0 ||
        !std::isfinite(pose_continuity_timeout_sec_) || pose_continuity_timeout_sec_ <= 0.0 ||
        pose_reference_options_.confirmation_frames < 1 ||
        !std::isfinite(pose_reference_options_.max_rotation_step_rad) ||
        pose_reference_options_.max_rotation_step_rad <= 0.0 ||
        !std::isfinite(pose_reference_options_.max_translation_step_m) ||
        pose_reference_options_.max_translation_step_m <= 0.0)
    {
      throw std::runtime_error("invalid IPPE parameters");
    }
    pose_reference_tracker_ =
      front_marker_vision::PoseReferenceTracker(pose_reference_options_);
  }

  void configureDictionary()
  {
    const std::map<std::string, int> dictionary_map = {
      {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
      {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
      {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
      {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
      {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
      {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
      {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
      {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
      {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
      {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
      {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
      {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
      {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
      {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
      {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
      {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
      {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
      {"DICT_APRILTAG_16H5", cv::aruco::DICT_APRILTAG_16h5},
      {"DICT_APRILTAG_25H9", cv::aruco::DICT_APRILTAG_25h9},
      {"DICT_APRILTAG_36H10", cv::aruco::DICT_APRILTAG_36h10},
      {"DICT_APRILTAG_36H11", cv::aruco::DICT_APRILTAG_36h11},
    };

    const std::string key = uppercase(dictionary_name_);
    const auto iterator = dictionary_map.find(key);
    if (iterator == dictionary_map.end())
    {
      throw std::runtime_error("unsupported aruco_dictionary: " + dictionary_name_);
    }
    dictionary_name_ = key;
    dictionary_ = cv::aruco::getPredefinedDictionary(iterator->second);
    detector_parameters_ = cv::aruco::DetectorParameters::create();
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& message)
  {
    if (message->width == 0 || message->height == 0)
    {
      invalidateCameraInfo("INVALID_CAMERA_INFO: image dimensions must be positive");
      return;
    }
    if (message->header.frame_id.empty())
    {
      invalidateCameraInfo("INVALID_CAMERA_INFO: frame_id must not be empty");
      return;
    }
    for (const double value : message->K)
    {
      if (!std::isfinite(value))
      {
        invalidateCameraInfo("INVALID_CAMERA_INFO: K contains non-finite values");
        return;
      }
    }
    if (message->K[0] <= 0.0 || message->K[4] <= 0.0)
    {
      invalidateCameraInfo("INVALID_CAMERA_INFO: fx and fy must be finite and positive");
      return;
    }

    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
      message->K[0], message->K[1], message->K[2],
      message->K[3], message->K[4], message->K[5],
      message->K[6], message->K[7], message->K[8]);

    const size_t distortion_size = message->D.size();
    if (!(distortion_size == 0 || distortion_size == 4 || distortion_size == 5 ||
          distortion_size == 8 || distortion_size == 12 || distortion_size == 14))
    {
      invalidateCameraInfo("INVALID_CAMERA_INFO: unsupported distortion coefficient count");
      return;
    }

    if (message->D.empty())
    {
      dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    }
    else
    {
      dist_coeffs_ = cv::Mat(1, static_cast<int>(message->D.size()), CV_64F);
      for (size_t index = 0; index < message->D.size(); ++index)
      {
        if (!std::isfinite(message->D[index]))
        {
          invalidateCameraInfo("INVALID_CAMERA_INFO: distortion contains non-finite values");
          return;
        }
        dist_coeffs_.at<double>(0, static_cast<int>(index)) = message->D[index];
      }
    }

    camera_info_frame_ = message->header.frame_id;
    camera_info_width_ = message->width;
    camera_info_height_ = message->height;
    has_camera_info_ = true;
    camera_info_error_.clear();
  }

  void invalidateCameraInfo(const std::string& error)
  {
    const bool changed = has_camera_info_ || camera_info_error_ != error;
    has_camera_info_ = false;
    camera_info_error_ = error;
    if (changed)
    {
      publishStatus(error, true);
    }
  }

  bool shouldPublishAnnotated(const ros::Time& now)
  {
    if (!publish_annotated_)
    {
      return false;
    }
    if (annotated_rate_hz_ <= 0.0)
    {
      return true;
    }
    const double period = 1.0 / annotated_rate_hz_;
    if (last_annotated_time_.isZero() || (now - last_annotated_time_).toSec() >= period)
    {
      last_annotated_time_ = now;
      return true;
    }
    return false;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& message)
  {
    const ros::WallTime processing_start = ros::WallTime::now();
    ++frame_count_;
    const double frame_stamp_sec = message->header.stamp.toSec();
    const front_marker_vision::PoseSolution* committed_reference = nullptr;
    if (pose_solver_ == "IPPE_SQUARE")
    {
      committed_reference = pose_reference_tracker_.referenceForFrame(frame_stamp_sec);
    }
    cv_bridge::CvImageConstPtr source_image;
    cv::Mat gray_image;
    try
    {
      if (message->encoding == sensor_msgs::image_encodings::MONO8)
      {
        source_image = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::MONO8);
        gray_image = source_image->image;
      }
      else if (message->encoding == sensor_msgs::image_encodings::RGB8)
      {
        source_image = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::RGB8);
        cv::cvtColor(source_image->image, gray_image, cv::COLOR_RGB2GRAY);
      }
      else
      {
        source_image = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8);
        cv::cvtColor(source_image->image, gray_image, cv::COLOR_BGR2GRAY);
      }
    }
    catch (const cv_bridge::Exception& exception)
    {
      roi_tracker_.reset();
      publishStatus(appendReferenceStatus(
        std::string("IMAGE_CONVERSION_ERROR: ") + exception.what()));
      pose_reference_tracker_.breakSequence();
      publishProcessingTime(processing_start);
      return;
    }

    const cv::Size image_size = gray_image.size();
    const cv::Rect full_frame(0, 0, image_size.width, image_size.height);
    if (has_camera_info_ &&
        (static_cast<uint32_t>(gray_image.cols) != camera_info_width_ ||
         static_cast<uint32_t>(gray_image.rows) != camera_info_height_))
    {
      roi_tracker_.reset();
      std::ostringstream status;
      status << "IMAGE_CAMERA_INFO_SIZE_MISMATCH: image=" << gray_image.cols << "x"
             << gray_image.rows << " camera_info=" << camera_info_width_ << "x"
             << camera_info_height_;
      const std::string status_text = appendRoiStatus(
        appendReferenceStatus(status.str()), full_frame, image_size);
      publishStatus(status_text);
      publishAnnotatedImage(
        message, gray_image, {}, {}, -1, nullptr, nullptr, full_frame, status_text);
      pose_reference_tracker_.breakSequence();
      publishProcessingTime(processing_start);
      return;
    }

    cv::Rect search_rect = roi_tracker_.searchRect(image_size);
    if (search_rect.width <= 0 || search_rect.height <= 0 ||
        (search_rect & full_frame) != search_rect)
    {
      roi_tracker_.reset();
      search_rect = full_frame;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(
      gray_image(search_rect), dictionary_, corners, ids, detector_parameters_);
    front_marker_vision::offsetMarkerCorners(
      corners, cv::Point2f(static_cast<float>(search_rect.x), static_cast<float>(search_rect.y)));

    int selected_index = -1;
    for (size_t index = 0; index < ids.size(); ++index)
    {
      if (ids[index] == target_marker_id_)
      {
        selected_index = static_cast<int>(index);
        break;
      }
    }

    front_marker_vision::ContourMetrics contour_metrics;
    std::string contour_rejection_reason;
    bool contour_accepted = selected_index >= 0;
    if (selected_index >= 0)
    {
      ++marker_count_;
      if (enable_contour_gate_ && !front_marker_vision::evaluateMarkerContour(
          gray_image,
          corners[static_cast<size_t>(selected_index)],
          contour_options_,
          contour_metrics,
          contour_rejection_reason))
      {
        contour_accepted = false;
      }
    }

    if (contour_accepted)
    {
      if (!roi_tracker_.updateTarget(
          corners[static_cast<size_t>(selected_index)], image_size))
      {
        roi_tracker_.updateMiss(image_size);
      }
    }
    else
    {
      roi_tracker_.updateMiss(image_size);
    }

    if (!has_camera_info_)
    {
      std::ostringstream stream;
      if (selected_index >= 0)
      {
        stream << "DETECTED_2D: ID" << target_marker_id_;
      }
      else
      {
        stream << "WAIT_MARKER_2D: ID" << target_marker_id_ << " not detected";
      }
      stream << " | POSE_DISABLED: "
             << (camera_info_error_.empty() ? "waiting for CameraInfo" : camera_info_error_);
      const std::string status = appendRoiStatus(
        appendReferenceStatus(stream.str()), search_rect, image_size);
      publishStatus(status);
      publishAnnotatedImage(
        message, gray_image, corners, ids, selected_index, nullptr, nullptr, search_rect, status);
      pose_reference_tracker_.breakSequence();
      publishProcessingTime(processing_start);
      return;
    }
    if (message->header.frame_id.empty())
    {
      const std::string status = appendRoiStatus(
        appendReferenceStatus("INVALID_IMAGE_FRAME: frame_id must not be empty"),
        search_rect,
        image_size);
      publishStatus(status);
      publishAnnotatedImage(
        message, gray_image, corners, ids, selected_index, nullptr, nullptr, search_rect, status);
      pose_reference_tracker_.breakSequence();
      publishProcessingTime(processing_start);
      return;
    }
    if (camera_info_frame_ != message->header.frame_id)
    {
      const std::string status = appendRoiStatus(
        appendReferenceStatus("FRAME_MISMATCH: image and CameraInfo frame_id differ"),
        search_rect,
        image_size);
      publishStatus(status);
      publishAnnotatedImage(
        message, gray_image, corners, ids, selected_index, nullptr, nullptr, search_rect, status);
      pose_reference_tracker_.breakSequence();
      publishProcessingTime(processing_start);
      return;
    }

    cv::Vec3d rvec;
    cv::Vec3d tvec;
    bool pose_valid = false;
    bool has_ippe_solution = false;
    bool reference_observed = false;
    front_marker_vision::PoseSolution ippe_solution;
    int selected_candidate_rank = -1;
    int reference_candidate_rank = -1;
    bool ambiguous_pose = false;
    std::string status;
    if (selected_index < 0)
    {
      std::ostringstream stream;
      stream << "WAIT_MARKER: ID" << target_marker_id_ << " not detected";
      status = stream.str();
    }
    else
    {
      const std::vector<cv::Point2f>& working_corners =
        corners[static_cast<size_t>(selected_index)];
      double reprojection_error_px = 0.0;
      int ippe_candidate_count = 0;
      std::string rejection_reason;
      std::string rejection_stage;

      if (!contour_accepted)
      {
        rejection_stage = "CONTOUR";
        rejection_reason = contour_rejection_reason;
      }

      if (rejection_stage.empty() && pose_solver_ == "IPPE_SQUARE")
      {
        front_marker_vision::PoseSolution solution;
        if (!front_marker_vision::estimatePoseIppeSquare(
              working_corners,
              target_marker_size_m_,
              camera_matrix_,
              dist_coeffs_,
              committed_reference,
              pose_reference_options_.max_rotation_step_rad,
              pose_reference_options_.max_translation_step_m,
              ippe_ambiguity_gap_px_,
              max_reprojection_error_px_,
              solution,
              rejection_reason))
        {
          rejection_stage = rejection_reason.rfind("reprojection RMS", 0) == 0 ?
            "REPROJECTION" : "IPPE";
        }
        else
        {
          ippe_solution = solution;
          has_ippe_solution = true;
          rvec = solution.rvec;
          tvec = solution.tvec;
          reprojection_error_px = solution.reprojection_error_px;
          ippe_candidate_count = solution.candidate_count;
          selected_candidate_rank = solution.selected_candidate_rank;
          reference_candidate_rank = solution.reference_candidate_rank;
          ambiguous_pose = solution.ambiguous;
        }
      }
      else if (rejection_stage.empty())
      {
        std::vector<std::vector<cv::Point2f>> selected_corners(1, working_corners);
        std::vector<cv::Vec3d> rvecs;
        std::vector<cv::Vec3d> tvecs;
        cv::aruco::estimatePoseSingleMarkers(
          selected_corners, target_marker_size_m_, camera_matrix_, dist_coeffs_, rvecs, tvecs);
        if (!rvecs.empty() && !tvecs.empty() &&
            finiteVec3(rvecs.front()) && finiteVec3(tvecs.front()))
        {
          rvec = rvecs.front();
          tvec = tvecs.front();
        }
        else
        {
          rejection_stage = "POSE";
          rejection_reason = "legacy pose is empty or non-finite";
        }
      }

      if (rejection_stage.empty() && !poseOutputIsValid(rvec, tvec))
      {
        rejection_stage = "POSE";
        rejection_reason = "transform contains a non-finite rotation or translation";
      }

      if (rejection_stage.empty() && pose_solver_ == "IPPE_SQUARE" && has_ippe_solution)
      {
        pose_reference_tracker_.observe(ippe_solution, frame_stamp_sec);
        reference_observed = true;
        if (!pose_reference_tracker_.posePublishAllowed())
        {
          rejection_stage = "AMBIGUITY_PENDING";
          std::ostringstream stream;
          stream << "candidate branch differs from committed reference; confirmation "
                 << pose_reference_tracker_.streak() << "/"
                 << pose_reference_tracker_.confirmationFrames();
          rejection_reason = stream.str();
        }
      }

      if (rejection_stage.empty())
      {
        pose_valid = publishPoseAndDebug(message, rvec, tvec);
        if (!pose_valid)
        {
          rejection_stage = "POSE";
          rejection_reason = "transform contains a non-finite rotation";
        }
      }

      if (pose_valid)
      {
        ++detect_count_;
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(3)
               << "DETECTED: ID" << target_marker_id_
               << " xyz_camera=(" << tvec[0] << "," << tvec[1] << "," << tvec[2] << ")m"
               << " pipeline=" << pose_solver_;
        if (pose_solver_ == "IPPE_SQUARE")
        {
          stream << " reproj=" << reprojection_error_px << "px"
                 << " candidates=" << ippe_candidate_count;
        }
        status = stream.str();
      }
      else
      {
        if (rejection_stage.rfind("CONTOUR", 0) == 0)
        {
          ++contour_reject_count_;
        }
        else if (rejection_stage == "REPROJECTION")
        {
          ++reprojection_reject_count_;
        }
        else
        {
          ++pose_reject_count_;
        }
        std::ostringstream stream;
        stream << "REJECTED_" << rejection_stage << ": ID" << target_marker_id_
               << " " << rejection_reason;
        status = stream.str();
      }
    }

    status = appendReferenceStatus(
      status, selected_candidate_rank, reference_candidate_rank, ambiguous_pose);
    status = appendRoiStatus(status, search_rect, image_size);
    publishStatus(status);
    publishAnnotatedImage(
      message, gray_image, corners, ids, selected_index,
      pose_valid ? &rvec : nullptr, pose_valid ? &tvec : nullptr, search_rect, status);
    publishProcessingTime(processing_start);
    if (pose_solver_ == "IPPE_SQUARE" && !reference_observed)
    {
      pose_reference_tracker_.breakSequence();
    }
  }

  std::string appendRoiStatus(
    const std::string& status, const cv::Rect& search_rect, const cv::Size& image_size) const
  {
    std::ostringstream stream;
    stream << status << " | roi=";
    const cv::Rect full_frame(0, 0, image_size.width, image_size.height);
    if (search_rect == full_frame)
    {
      stream << "FULL";
      if (!enable_roi_)
      {
        stream << " disabled";
      }
    }
    else
    {
      stream << search_rect.x << "," << search_rect.y << ","
             << search_rect.width << "x" << search_rect.height;
    }
    stream << " miss=" << roi_tracker_.missCount();
    if (enable_roi_ && search_rect != full_frame && !roi_tracker_.active())
    {
      stream << " next=FULL";
    }
    return stream.str();
  }

  std::string appendReferenceStatus(
    const std::string& status,
    int selected_candidate_rank = -1,
    int reference_candidate_rank = -1,
    bool ambiguous_pose = false) const
  {
    std::ostringstream stream;
    stream << status;
    if (pose_solver_ == "IPPE_SQUARE")
    {
      stream << " | ref_state=" << poseReferenceStateName(pose_reference_tracker_.state())
             << " ref_streak=" << pose_reference_tracker_.streak() << "/"
             << pose_reference_tracker_.confirmationFrames()
             << " switch_pending=" << (pose_reference_tracker_.switchPending() ? 1 : 0)
             << " selected_rank=" << selected_candidate_rank
             << " reference_rank=" << reference_candidate_rank
             << " ambiguous=" << (ambiguous_pose ? 1 : 0)
             << " branch_flips=" << pose_reference_tracker_.branchFlipCount();
    }
    else
    {
      stream << " | ref_state=DISABLED ref_streak=0/"
             << pose_reference_tracker_.confirmationFrames()
             << " switch_pending=0 selected_rank=-1 reference_rank=-1"
             << " ambiguous=0 branch_flips=0";
    }
    return stream.str();
  }

  void publishProcessingTime(const ros::WallTime& start)
  {
    std_msgs::Float64 message;
    const double elapsed_ms = (ros::WallTime::now() - start).toSec() * 1000.0;
    message.data = std::isfinite(elapsed_ms) && elapsed_ms >= 0.0 ? elapsed_ms : 0.0;
    processing_time_pub_.publish(message);
  }

  bool poseOutputIsValid(const cv::Vec3d& rvec, const cv::Vec3d& tvec) const
  {
    if (!finiteVec3(rvec) || !finiteVec3(tvec))
    {
      return false;
    }
    cv::Mat rotation;
    try
    {
      cv::Rodrigues(rvec, rotation);
    }
    catch (const cv::Exception&)
    {
      return false;
    }
    geometry_msgs::Quaternion quaternion;
    return rotationToQuaternion(rotation, quaternion);
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

    front_marker_vision::HomogeneousTransform transform;
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

  void publishAnnotatedImage(
    const sensor_msgs::ImageConstPtr& source,
    const cv::Mat& gray,
    const std::vector<std::vector<cv::Point2f>>& corners,
    const std::vector<int>& ids,
    int selected_index,
    const cv::Vec3d* rvec,
    const cv::Vec3d* tvec,
    const cv::Rect& search_rect,
    const std::string& status)
  {
    if (annotated_pub_.getNumSubscribers() == 0)
    {
      return;
    }
    if (!shouldPublishAnnotated(ros::Time::now()))
    {
      return;
    }

    cv::Mat annotated;
    try
    {
      if (source->encoding == sensor_msgs::image_encodings::MONO8)
      {
        cv::cvtColor(gray, annotated, cv::COLOR_GRAY2BGR);
      }
      else if (source->encoding == sensor_msgs::image_encodings::RGB8)
      {
        const cv_bridge::CvImageConstPtr rgb =
          cv_bridge::toCvShare(source, sensor_msgs::image_encodings::RGB8);
        cv::cvtColor(rgb->image, annotated, cv::COLOR_RGB2BGR);
      }
      else
      {
        const cv_bridge::CvImageConstPtr bgr =
          cv_bridge::toCvShare(source, sensor_msgs::image_encodings::BGR8);
        annotated = bgr->image.clone();
      }
    }
    catch (const cv_bridge::Exception& exception)
    {
      ROS_ERROR_THROTTLE(1.0, "Annotated image conversion failed: %s", exception.what());
      return;
    }
    if (!ids.empty())
    {
      cv::aruco::drawDetectedMarkers(annotated, corners, ids);
    }
    if (selected_index >= 0 && rvec != nullptr && tvec != nullptr)
    {
      cv::aruco::drawAxis(
        annotated, camera_matrix_, dist_coeffs_, *rvec, *tvec, target_marker_size_m_ * 0.5);
    }
    const cv::Rect full_frame(0, 0, annotated.cols, annotated.rows);
    if (enable_roi_ && search_rect != full_frame)
    {
      cv::rectangle(annotated, search_rect, cv::Scalar(0, 255, 255), 2);
      std::ostringstream roi_label;
      roi_label << "ROI " << search_rect.width << "x" << search_rect.height
                << " miss=" << roi_tracker_.missCount();
      cv::putText(
        annotated, roi_label.str(),
        cv::Point(search_rect.x + 4, std::max(18, search_rect.y + 18)),
        cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(0, 255, 255), 1);
    }

    std::ostringstream summary;
    summary << "F=" << frame_count_ << " Seen=" << marker_count_ << " OK=" << detect_count_
            << " Reject=" << contour_reject_count_ << "/" << pose_reject_count_
            << "/" << reprojection_reject_count_;
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
    annotated_pub_.publish(output.toImageMsg());
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

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher transform_debug_pub_;
  ros::Publisher annotated_pub_;
  ros::Publisher status_pub_;
  ros::Publisher processing_time_pub_;
  tf2_ros::TransformBroadcaster marker_tf_broadcaster_;

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string pose_topic_;
  std::string transform_debug_topic_;
  std::string processing_time_topic_;
  std::string annotated_image_topic_;
  std::string status_topic_;
  std::string dictionary_name_;
  std::string camera_info_frame_;
  std::string camera_info_error_;
  int target_marker_id_ = 33;
  double target_marker_size_m_ = 0.070;
  bool publish_marker_tf_ = true;
  std::string marker_frame_ = "front_marker";
  uint32_t camera_info_width_ = 0;
  uint32_t camera_info_height_ = 0;
  bool publish_annotated_ = true;
  double annotated_rate_hz_ = 5.0;
  bool enable_contour_gate_ = true;
  bool enable_roi_ = true;
  std::string pose_solver_ = "IPPE_SQUARE";
  front_marker_vision::ContourGateOptions contour_options_;
  double ippe_ambiguity_gap_px_ = 0.2;
  double max_reprojection_error_px_ = 1.0;
  double pose_continuity_timeout_sec_ = 0.5;
  front_marker_vision::PoseReferenceOptions pose_reference_options_;
  front_marker_vision::PoseReferenceTracker pose_reference_tracker_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  bool has_camera_info_ = false;
  front_marker_vision::RoiTracker roi_tracker_;

  ros::Time last_annotated_time_;
  ros::Time last_status_time_;
  std::string last_status_;
  uint64_t frame_count_ = 0;
  uint64_t marker_count_ = 0;
  uint64_t detect_count_ = 0;
  uint64_t contour_reject_count_ = 0;
  uint64_t pose_reject_count_ = 0;
  uint64_t reprojection_reject_count_ = 0;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "front_marker_vision_node");
  try
  {
    FrontMarkerVisionNode node;
    ros::spin();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL_STREAM("front_marker_vision_node failed: " << exception.what());
    return 1;
  }
  return 0;
}
