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
#include <front_marker_tracking/HomogeneousTransform.h>
#include <front_marker_tracking/marker_pose_estimator.hpp>
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

}  // namespace

class FrontMarkerDetectorNode
{
public:
  FrontMarkerDetectorNode()
    : nh_()
    , pnh_("~")
  {
    loadParameters();
    configureDictionary();

    transform_pub_ = nh_.advertise<front_marker_tracking::HomogeneousTransform>(transform_topic_, 1);
    status_pub_ = nh_.advertise<std_msgs::String>(status_topic_, 1, true);
    processing_time_pub_ = pnh_.advertise<std_msgs::Float64>("processing_time_ms", 10);
    diagnostic_pub_ = pnh_.advertise<std_msgs::String>("diagnostic", 10);
    if (publish_annotated_)
    {
      annotated_pub_ = nh_.advertise<sensor_msgs::Image>(annotated_image_topic_, 1);
    }

    const ros::TransportHints transport_hints = ros::TransportHints().tcpNoDelay();
    image_sub_ = nh_.subscribe(image_topic_, 1, &FrontMarkerDetectorNode::imageCallback, this, transport_hints);
    camera_info_sub_ = nh_.subscribe(
      camera_info_topic_, 1, &FrontMarkerDetectorNode::cameraInfoCallback, this);

    publishStatus("WAIT_CAMERA_INFO: waiting for " + camera_info_topic_, true);
    ROS_INFO_STREAM("front_marker_detector_node started | image=" << image_topic_
                    << " | camera_info=" << camera_info_topic_
                    << " | dictionary=" << dictionary_name_
                    << " | target=ID" << target_marker_id_
                    << " size=" << target_marker_size_m_ << "m"
                    << " | contour_gate=" << (enable_contour_gate_ ? "true" : "false")
                    << " | corners=" << corner_refinement_method_
                    << " | solver=" << pose_solver_
                    << " | annotated=" << (publish_annotated_ ? "true" : "false"));
  }

private:
  void loadParameters()
  {
    pnh_.param<std::string>("image_topic", image_topic_, "/usb_cam/image_raw");
    pnh_.param<std::string>("camera_info_topic", camera_info_topic_, "/usb_cam/camera_info");
    pnh_.param<std::string>("transform_topic", transform_topic_, "/front_marker_tracking/transform");
    pnh_.param<std::string>(
      "annotated_image_topic", annotated_image_topic_, "/front_marker_tracking/image_annotated");
    pnh_.param<std::string>(
      "status_topic", status_topic_, "/front_marker_tracking/detector_status");
    pnh_.param<std::string>("aruco_dictionary", dictionary_name_, "DICT_5X5_1000");
    pnh_.param<std::string>(
      "expected_camera_frame", expected_camera_frame_, "usb_camera_optical_frame");
    pnh_.param("target_marker_id", target_marker_id_, 33);
    pnh_.param("target_marker_size_m", target_marker_size_m_, 0.193);
    pnh_.param("expected_image_width", expected_image_width_, 640);
    pnh_.param("expected_image_height", expected_image_height_, 480);
    pnh_.param("publish_annotated", publish_annotated_, false);
    pnh_.param("annotated_rate_hz", annotated_rate_hz_, 5.0);
    pnh_.param("enable_contour_gate", enable_contour_gate_, false);
    pnh_.param<std::string>("corner_refinement_method", corner_refinement_method_, "NONE");
    pnh_.param<std::string>("pose_solver", pose_solver_, "LEGACY");
    pnh_.param("min_edge_length_px", contour_options_.min_edge_length_px, 12.0);
    pnh_.param("edge_sample_offset_ratio", contour_options_.edge_sample_offset_ratio, 0.025);
    pnh_.param("edge_sample_offset_min_px", contour_options_.edge_sample_offset_min_px, 1.0);
    pnh_.param("edge_sample_offset_max_px", contour_options_.edge_sample_offset_max_px, 4.0);
    pnh_.param("edge_sample_margin_ratio", contour_options_.edge_sample_margin_ratio, 0.15);
    pnh_.param("min_edge_contrast", contour_options_.min_edge_contrast, 25.0);
    pnh_.param("min_mean_edge_contrast", contour_options_.min_mean_edge_contrast, 40.0);
    pnh_.param("edge_samples_per_side", contour_options_.samples_per_edge, 32);
    pnh_.param("subpix_window_size", subpixel_options_.window_size, 5);
    pnh_.param("subpix_max_iterations", subpixel_options_.max_iterations, 50);
    pnh_.param("subpix_epsilon", subpixel_options_.epsilon, 0.001);
    pnh_.param("subpix_max_shift_px", subpixel_options_.max_shift_px, 2.0);
    pnh_.param("ippe_ambiguity_gap_px", ippe_ambiguity_gap_px_, 0.2);
    pnh_.param("max_reprojection_error_px", max_reprojection_error_px_, 1.0);
    pnh_.param("pose_continuity_timeout_sec", pose_continuity_timeout_sec_, 0.5);

    corner_refinement_method_ = uppercase(corner_refinement_method_);
    pose_solver_ = uppercase(pose_solver_);

    if (target_marker_id_ < 0)
    {
      throw std::runtime_error("target_marker_id must be non-negative");
    }
    if (!std::isfinite(target_marker_size_m_) || target_marker_size_m_ <= 0.0)
    {
      throw std::runtime_error("target_marker_size_m must be finite and greater than zero");
    }
    if (expected_image_width_ <= 0 || expected_image_height_ <= 0)
    {
      throw std::runtime_error("expected image dimensions must be positive");
    }
    if (!(corner_refinement_method_ == "NONE" || corner_refinement_method_ == "SUBPIX"))
    {
      throw std::runtime_error("corner_refinement_method must be NONE or SUBPIX");
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
    if (subpixel_options_.window_size <= 0 || subpixel_options_.max_iterations <= 0 ||
        subpixel_options_.epsilon <= 0.0 || subpixel_options_.max_shift_px <= 0.0)
    {
      throw std::runtime_error("invalid subpixel parameters");
    }
    if (ippe_ambiguity_gap_px_ < 0.0 || max_reprojection_error_px_ <= 0.0 ||
        pose_continuity_timeout_sec_ <= 0.0)
    {
      throw std::runtime_error("invalid IPPE parameters");
    }
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
    if (static_cast<int>(message->width) != expected_image_width_ ||
        static_cast<int>(message->height) != expected_image_height_)
    {
      std::ostringstream status;
      status << "INVALID_CAMERA_INFO: expected " << expected_image_width_ << "x"
             << expected_image_height_ << " but received " << message->width << "x"
             << message->height;
      invalidateCameraInfo(status.str());
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
    has_camera_info_ = true;
    camera_info_error_.clear();
    if (!expected_camera_frame_.empty() && camera_info_frame_ != expected_camera_frame_)
    {
      ROS_WARN_THROTTLE(
        5.0, "CameraInfo frame is '%s', expected '%s'", camera_info_frame_.c_str(),
        expected_camera_frame_.c_str());
    }
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
    cv_bridge::CvImageConstPtr source_image;
    cv::Mat color_image;
    cv::Mat gray_image;
    try
    {
      if (message->encoding == sensor_msgs::image_encodings::MONO8)
      {
        source_image = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::MONO8);
        gray_image = source_image->image;
        cv::cvtColor(gray_image, color_image, cv::COLOR_GRAY2BGR);
      }
      else
      {
        source_image = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8);
        color_image = source_image->image;
        cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);
      }
    }
    catch (const cv_bridge::Exception& exception)
    {
      publishStatus(std::string("IMAGE_CONVERSION_ERROR: ") + exception.what());
      publishProcessingTime(processing_start);
      return;
    }

    if (gray_image.cols != expected_image_width_ ||
        gray_image.rows != expected_image_height_)
    {
      std::ostringstream status;
      status << "INVALID_IMAGE_SIZE: expected " << expected_image_width_ << "x"
             << expected_image_height_ << " but received " << gray_image.cols << "x"
             << gray_image.rows;
      publishStatus(status.str());
      publishAnnotatedImage(message, color_image, {}, {}, -1, nullptr, nullptr, status.str());
      publishProcessingTime(processing_start);
      return;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray_image, dictionary_, corners, ids, detector_parameters_);

    int selected_index = -1;
    for (size_t index = 0; index < ids.size(); ++index)
    {
      if (ids[index] == target_marker_id_)
      {
        selected_index = static_cast<int>(index);
        break;
      }
    }

    if (!has_camera_info_)
    {
      std::ostringstream stream;
      if (selected_index >= 0)
      {
        ++marker_count_;
        stream << "DETECTED_2D: ID" << target_marker_id_;
      }
      else
      {
        stream << "WAIT_MARKER_2D: ID" << target_marker_id_ << " not detected";
      }
      stream << " | POSE_DISABLED: "
             << (camera_info_error_.empty() ? "waiting for CameraInfo" : camera_info_error_);
      const std::string status = stream.str();
      publishStatus(status);
      publishDiagnostic(status);
      publishAnnotatedImage(
        message, color_image, corners, ids, selected_index, nullptr, nullptr, status);
      publishProcessingTime(processing_start);
      return;
    }
    if (!camera_info_frame_.empty() && !message->header.frame_id.empty() &&
        camera_info_frame_ != message->header.frame_id)
    {
      publishStatus("FRAME_MISMATCH: image and CameraInfo frame_id differ");
      publishProcessingTime(processing_start);
      return;
    }

    cv::Vec3d rvec;
    cv::Vec3d tvec;
    bool pose_valid = false;
    std::string status;
    if (selected_index < 0)
    {
      std::ostringstream stream;
      stream << "WAIT_MARKER: ID" << target_marker_id_ << " not detected";
      status = stream.str();
    }
    else
    {
      ++marker_count_;
      std::vector<cv::Point2f> working_corners = corners[static_cast<size_t>(selected_index)];
      front_marker_tracking::ContourMetrics contour_metrics;
      double max_subpixel_shift_px = 0.0;
      double reprojection_error_px = 0.0;
      int ippe_candidate_count = 0;
      std::string rejection_reason;
      std::string rejection_stage;

      if (enable_contour_gate_ && !front_marker_tracking::evaluateMarkerContour(
          gray_image,
          working_corners,
          contour_options_,
          contour_metrics,
          rejection_reason))
      {
        rejection_stage = "CONTOUR_RAW";
      }

      if (rejection_stage.empty() && corner_refinement_method_ == "SUBPIX")
      {
        std::vector<cv::Point2f> refined;
        if (!front_marker_tracking::refineMarkerCorners(
              gray_image,
              working_corners,
              subpixel_options_,
              refined,
              max_subpixel_shift_px,
              rejection_reason))
        {
          rejection_stage = "SUBPIX";
        }
        else
        {
          working_corners = refined;
        }
      }

      if (rejection_stage.empty() && enable_contour_gate_ &&
          !front_marker_tracking::evaluateMarkerContour(
            gray_image,
            working_corners,
            contour_options_,
            contour_metrics,
            rejection_reason))
      {
        rejection_stage = "CONTOUR_REFINED";
      }

      if (rejection_stage.empty() && pose_solver_ == "IPPE_SQUARE")
      {
        const front_marker_tracking::PoseSolution* previous = nullptr;
        if (has_previous_pose_ && !message->header.stamp.isZero() && !previous_pose_stamp_.isZero())
        {
          const double age = (message->header.stamp - previous_pose_stamp_).toSec();
          if (age >= 0.0 && age <= pose_continuity_timeout_sec_)
          {
            previous = &previous_pose_;
          }
        }
        front_marker_tracking::PoseSolution solution;
        if (!front_marker_tracking::estimatePoseIppeSquare(
              working_corners,
              target_marker_size_m_,
              camera_matrix_,
              dist_coeffs_,
              previous,
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
          rvec = solution.rvec;
          tvec = solution.tvec;
          reprojection_error_px = solution.reprojection_error_px;
          ippe_candidate_count = solution.candidate_count;
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

      corners[static_cast<size_t>(selected_index)] = working_corners;
      if (rejection_stage.empty())
      {
        pose_valid = publishTransform(message, rvec, tvec);
        if (!pose_valid)
        {
          rejection_stage = "POSE";
          rejection_reason = "transform contains a non-finite rotation";
        }
      }

      if (pose_valid)
      {
        ++detect_count_;
        previous_pose_.rvec = rvec;
        previous_pose_.tvec = tvec;
        previous_pose_.reprojection_error_px = reprojection_error_px;
        previous_pose_.candidate_count = ippe_candidate_count;
        previous_pose_stamp_ = message->header.stamp;
        has_previous_pose_ = true;
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(3)
               << "DETECTED: ID" << target_marker_id_
               << " xyz_camera=(" << tvec[0] << "," << tvec[1] << "," << tvec[2] << ")m"
               << " pipeline=" << pose_solver_;
        if (corner_refinement_method_ == "SUBPIX")
        {
          stream << " subpix_shift=" << max_subpixel_shift_px << "px";
        }
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
        else if (rejection_stage == "SUBPIX")
        {
          ++subpixel_reject_count_;
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

    publishStatus(status);
    publishDiagnostic(status);
    publishAnnotatedImage(
      message, color_image, corners, ids, selected_index,
      pose_valid ? &rvec : nullptr, pose_valid ? &tvec : nullptr, status);
    publishProcessingTime(processing_start);
  }

  void publishProcessingTime(const ros::WallTime& start)
  {
    std_msgs::Float64 message;
    message.data = (ros::WallTime::now() - start).toSec() * 1000.0;
    processing_time_pub_.publish(message);
  }

  void publishDiagnostic(const std::string& text)
  {
    std_msgs::String message;
    message.data = text;
    diagnostic_pub_.publish(message);
  }

  bool publishTransform(
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

    front_marker_tracking::HomogeneousTransform output;
    output.header = image_message->header;
    if (output.header.frame_id.empty())
    {
      output.header.frame_id = camera_info_frame_;
    }
    output.marker_id = target_marker_id_;
    for (int row = 0; row < 3; ++row)
    {
      for (int column = 0; column < 3; ++column)
      {
        const double value = rotation.at<double>(row, column);
        if (!std::isfinite(value))
        {
          return false;
        }
        output.matrix[static_cast<size_t>(row * 4 + column)] = value;
      }
      output.matrix[static_cast<size_t>(row * 4 + 3)] = tvec[row];
    }
    output.matrix[12] = 0.0;
    output.matrix[13] = 0.0;
    output.matrix[14] = 0.0;
    output.matrix[15] = 1.0;
    transform_pub_.publish(output);
    return true;
  }

  void publishAnnotatedImage(
    const sensor_msgs::ImageConstPtr& source,
    const cv::Mat& color,
    const std::vector<std::vector<cv::Point2f>>& corners,
    const std::vector<int>& ids,
    int selected_index,
    const cv::Vec3d* rvec,
    const cv::Vec3d* tvec,
    const std::string& status)
  {
    if (!shouldPublishAnnotated(ros::Time::now()))
    {
      return;
    }

    cv::Mat annotated = color.clone();
    if (!ids.empty())
    {
      cv::aruco::drawDetectedMarkers(annotated, corners, ids);
    }
    if (selected_index >= 0 && rvec != nullptr && tvec != nullptr)
    {
      cv::aruco::drawAxis(
        annotated, camera_matrix_, dist_coeffs_, *rvec, *tvec, target_marker_size_m_ * 0.5);
    }

    std::ostringstream summary;
    summary << "F=" << frame_count_ << " Seen=" << marker_count_ << " OK=" << detect_count_
            << " Reject=" << contour_reject_count_ << "/" << subpixel_reject_count_
            << "/" << pose_reject_count_ << "/" << reprojection_reject_count_;
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
  ros::Publisher transform_pub_;
  ros::Publisher annotated_pub_;
  ros::Publisher status_pub_;
  ros::Publisher processing_time_pub_;
  ros::Publisher diagnostic_pub_;

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string transform_topic_;
  std::string annotated_image_topic_;
  std::string status_topic_;
  std::string dictionary_name_;
  std::string expected_camera_frame_;
  std::string camera_info_frame_;
  std::string camera_info_error_;
  int target_marker_id_ = 33;
  double target_marker_size_m_ = 0.193;
  int expected_image_width_ = 640;
  int expected_image_height_ = 480;
  bool publish_annotated_ = false;
  double annotated_rate_hz_ = 5.0;
  bool enable_contour_gate_ = false;
  std::string corner_refinement_method_ = "NONE";
  std::string pose_solver_ = "LEGACY";
  front_marker_tracking::ContourGateOptions contour_options_;
  front_marker_tracking::SubpixelOptions subpixel_options_;
  double ippe_ambiguity_gap_px_ = 0.2;
  double max_reprojection_error_px_ = 1.0;
  double pose_continuity_timeout_sec_ = 0.5;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  bool has_camera_info_ = false;
  bool has_previous_pose_ = false;
  front_marker_tracking::PoseSolution previous_pose_;
  ros::Time previous_pose_stamp_;

  ros::Time last_annotated_time_;
  ros::Time last_status_time_;
  std::string last_status_;
  uint64_t frame_count_ = 0;
  uint64_t marker_count_ = 0;
  uint64_t detect_count_ = 0;
  uint64_t contour_reject_count_ = 0;
  uint64_t subpixel_reject_count_ = 0;
  uint64_t pose_reject_count_ = 0;
  uint64_t reprojection_reject_count_ = 0;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "front_marker_detector_node");
  try
  {
    FrontMarkerDetectorNode node;
    ros::spin();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL_STREAM("front_marker_detector_node failed: " << exception.what());
    return 1;
  }
  return 0;
}
