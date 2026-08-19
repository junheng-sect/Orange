#include <cerrno>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <sys/stat.h>

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

namespace
{

std::string trim(const std::string& input)
{
  const std::string whitespace = " \t\r\n";
  const size_t first = input.find_first_not_of(whitespace);
  if (first == std::string::npos)
  {
    return "";
  }
  const size_t last = input.find_last_not_of(whitespace);
  return input.substr(first, last - first + 1);
}

bool isSafeDevicePath(const std::string& path)
{
  if (path.rfind("/dev/", 0) != 0)
  {
    return false;
  }
  for (const char character : path)
  {
    const bool allowed =
      (character >= 'a' && character <= 'z') ||
      (character >= 'A' && character <= 'Z') ||
      (character >= '0' && character <= '9') ||
      character == '/' || character == '.' || character == '_' || character == '-';
    if (!allowed)
    {
      return false;
    }
  }
  return true;
}

std::string resolveDevice(const std::string& path)
{
  char resolved[PATH_MAX];
  if (realpath(path.c_str(), resolved) == nullptr)
  {
    std::ostringstream message;
    message << "cannot resolve camera device '" << path << "': " << std::strerror(errno);
    throw std::runtime_error(message.str());
  }

  struct stat device_stat;
  if (stat(resolved, &device_stat) != 0 || !S_ISCHR(device_stat.st_mode))
  {
    throw std::runtime_error("resolved camera path is not a character device: " + std::string(resolved));
  }
  return resolved;
}

std::string videoDeviceName(const std::string& resolved_device)
{
  const size_t separator = resolved_device.find_last_of('/');
  const std::string basename = separator == std::string::npos ?
    resolved_device : resolved_device.substr(separator + 1);
  const std::string sysfs_path = "/sys/class/video4linux/" + basename + "/name";
  std::ifstream stream(sysfs_path);
  if (!stream)
  {
    throw std::runtime_error("cannot read V4L2 device identity: " + sysfs_path);
  }
  std::string name;
  std::getline(stream, name);
  return trim(name);
}

}  // namespace

class FrontMarkerCsiCamera
{
public:
  FrontMarkerCsiCamera()
    : nh_(),
      pnh_("~"),
      width_(720),
      height_(576),
      framerate_(15),
      calibrated_(false)
  {
    pnh_.param<std::string>("device", device_, "/dev/video-camera0");
    pnh_.param<int>("width", width_, 720);
    pnh_.param<int>("height", height_, 576);
    pnh_.param<int>("framerate", framerate_, 15);
    pnh_.param<std::string>("frame_id", frame_id_, "cam1_optical_frame");
    pnh_.param<std::string>("camera_name", camera_name_, "ov13855_cam1");
    pnh_.param<std::string>("camera_info_url", camera_info_url_, "");
    pnh_.param<std::string>("image_topic", image_topic_, "/cam1/image_raw");
    pnh_.param<std::string>("camera_info_topic", camera_info_topic_, "/cam1/camera_info");

    validateParameters();
    resolved_device_ = resolveDevice(device_);
    const std::string product_name = videoDeviceName(resolved_device_);
    if (product_name != "rkisp_mainpath")
    {
      std::ostringstream message;
      message << "camera device '" << device_ << "' resolves to " << resolved_device_
              << " (" << product_name << "), expected RKISP mainpath; raw Bayer/CIF devices are rejected";
      throw std::runtime_error(message.str());
    }

    camera_info_manager_.reset(
      new camera_info_manager::CameraInfoManager(nh_, camera_name_, camera_info_url_));
    if (!camera_info_url_.empty())
    {
      calibrated_ = camera_info_manager_->isCalibrated();
      if (!calibrated_)
      {
        throw std::runtime_error("camera_info_url was provided but no valid calibration could be loaded");
      }
      const sensor_msgs::CameraInfo loaded = camera_info_manager_->getCameraInfo();
      if (loaded.width != static_cast<uint32_t>(width_) ||
          loaded.height != static_cast<uint32_t>(height_))
      {
        std::ostringstream message;
        message << "calibration resolution " << loaded.width << "x" << loaded.height
                << " does not match capture resolution " << width_ << "x" << height_;
        throw std::runtime_error(message.str());
      }
    }
    else
    {
      ROS_WARN_STREAM(
        "No OV13855 calibration was provided. Publishing uncalibrated CameraInfo (K[0]=0); "
        "metric Marker pose must remain disabled.");
    }

    image_publisher_ = nh_.advertise<sensor_msgs::Image>(image_topic_, 1);
    camera_info_publisher_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_info_topic_, 1);
    openCapture();
    ROS_INFO_STREAM(
      "CAM1 CSI camera ready: " << device_ << " -> " << resolved_device_
      << " (rkisp_mainpath), " << width_ << "x" << height_ << "@" << framerate_
      << "Hz, image=" << image_topic_ << ", camera_info=" << camera_info_topic_
      << ", frame_id=" << frame_id_ << ", transport=raw-only");
  }

  int run()
  {
    int consecutive_failures = 0;
    while (ros::ok())
    {
      cv::Mat frame;
      if (!capture_.read(frame) || frame.empty())
      {
        ++consecutive_failures;
        ROS_ERROR_STREAM("CSI frame capture failed (" << consecutive_failures << "/5)");
        if (consecutive_failures >= 5)
        {
          return EXIT_FAILURE;
        }
        ros::spinOnce();
        continue;
      }
      consecutive_failures = 0;

      if (frame.cols != width_ || frame.rows != height_ || frame.type() != CV_8UC3)
      {
        std::ostringstream message;
        message << "unexpected GStreamer output: " << frame.cols << "x" << frame.rows
                << " type=" << frame.type() << ", expected " << width_ << "x" << height_
                << " CV_8UC3";
        ROS_FATAL_STREAM(message.str());
        return EXIT_FAILURE;
      }

      std_msgs::Header header;
      header.stamp = ros::Time::now();
      header.frame_id = frame_id_;
      sensor_msgs::ImagePtr image = cv_bridge::CvImage(
        header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();

      sensor_msgs::CameraInfo camera_info;
      if (calibrated_)
      {
        camera_info = camera_info_manager_->getCameraInfo();
      }
      else
      {
        camera_info.width = static_cast<uint32_t>(width_);
        camera_info.height = static_cast<uint32_t>(height_);
      }
      camera_info.header = header;
      image_publisher_.publish(image);
      camera_info_publisher_.publish(camera_info);
      ros::spinOnce();
    }
    return EXIT_SUCCESS;
  }

private:
  void validateParameters() const
  {
    if (!isSafeDevicePath(device_))
    {
      throw std::runtime_error("device must be a safe absolute /dev path");
    }
    if (width_ <= 0 || height_ <= 0 || framerate_ <= 0)
    {
      throw std::runtime_error("width, height, and framerate must be positive integers");
    }
    if (frame_id_.empty() || camera_name_.empty() || image_topic_.empty() ||
        camera_info_topic_.empty())
    {
      throw std::runtime_error(
        "frame_id, camera_name, image_topic, and camera_info_topic must not be empty");
    }
  }

  void openCapture()
  {
    std::ostringstream pipeline;
    pipeline << "v4l2src device=" << device_ << " io-mode=4 ! "
             << "video/x-raw,format=NV12,width=" << width_ << ",height=" << height_
             << ",framerate=" << framerate_ << "/1 ! "
             << "videoconvert ! video/x-raw,format=BGR ! "
             << "appsink drop=true max-buffers=1 sync=false";
    if (!capture_.open(pipeline.str(), cv::CAP_GSTREAMER))
    {
      throw std::runtime_error("failed to open CAM1 with the validated GStreamer pipeline");
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher image_publisher_;
  ros::Publisher camera_info_publisher_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  cv::VideoCapture capture_;
  std::string device_;
  std::string resolved_device_;
  std::string frame_id_;
  std::string camera_name_;
  std::string camera_info_url_;
  std::string image_topic_;
  std::string camera_info_topic_;
  int width_;
  int height_;
  int framerate_;
  bool calibrated_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "front_marker_csi_camera_node");
  try
  {
    FrontMarkerCsiCamera camera;
    return camera.run();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL_STREAM("CAM1 CSI camera startup failed: " << exception.what());
    return EXIT_FAILURE;
  }
}
