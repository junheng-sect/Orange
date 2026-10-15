#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
namespace {
struct Pipeline {
  GstElement* pipeline = nullptr;
  GstElement* sink = nullptr;
  ~Pipeline() {
    if (pipeline) gst_element_set_state(pipeline, GST_STATE_NULL);
    if (sink) gst_object_unref(sink);
    if (pipeline) gst_object_unref(pipeline);
  }
};
// RKISP needs enough capture buffers to keep DMA running while a frame is
// downstream. The default two-buffer pool loses every third frame on this board.
// This is the capture pool, not an application queue: appsink remains latest-only.
GstPadProbeReturn configureCaptureAllocation(GstPad*, GstPadProbeInfo* probe, gpointer) {
  GstQuery* query = GST_PAD_PROBE_INFO_QUERY(probe);
  if (GST_QUERY_TYPE(query) != GST_QUERY_ALLOCATION) return GST_PAD_PROBE_OK;
  GstCaps* caps = nullptr;
  gst_query_parse_allocation(query, &caps, nullptr);
  GstVideoInfo video;
  gst_video_info_init(&video);
  if (!caps || !gst_video_info_from_caps(&video, caps)) return GST_PAD_PROBE_OK;
  // v4l2src chooses its own compatible pool. Request at least four buffers;
  // the source may reserve additional buffers for its own capture requirements.
  gst_query_add_allocation_pool(query, nullptr, video.size, 4, 0);
  gst_query_add_allocation_meta(query, GST_VIDEO_META_API_TYPE, nullptr);
  ROS_INFO("Requesting at least 4 V4L2 capture buffers; appsink queue remains 1");
  return GST_PAD_PROBE_HANDLED;
}
void checkBus(GstElement* pipeline) {
  GstBus* bus = gst_element_get_bus(pipeline);
  GstMessage* message = gst_bus_pop_filtered(bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
  gst_object_unref(bus);
  if (!message) return;
  std::string detail = "camera stream ended";
  if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
    GError* error = nullptr; gchar* debug = nullptr;
    gst_message_parse_error(message, &error, &debug);
    detail = error ? error->message : "GStreamer error";
    if (debug) detail += std::string(" | ") + debug;
    g_clear_error(&error); g_free(debug);
  }
  gst_message_unref(message); throw std::runtime_error(detail);
}
ros::Time captureStamp(GstElement* pipeline, GstSample* sample) {
  // Map running-time PTS to ROS time, preserving capture age. Not an exposure timestamp.
  const ros::Time now = ros::Time::now();
  const GstSegment* segment = gst_sample_get_segment(sample);
  const GstClockTime pts = GST_BUFFER_PTS(gst_sample_get_buffer(sample));
  GstClock* clock = gst_element_get_clock(pipeline);
  if (clock && segment && segment->format == GST_FORMAT_TIME && GST_CLOCK_TIME_IS_VALID(pts)) {
    const auto running = gst_segment_to_running_time(segment, GST_FORMAT_TIME, pts);
    const auto current = gst_clock_get_time(clock);
    const auto base = gst_element_get_base_time(pipeline);
    gst_object_unref(clock);
    if (GST_CLOCK_TIME_IS_VALID(running) && current >= base && current - base >= running) {
      const double age = static_cast<double>(current - base - running) / GST_SECOND;
      if (age <= now.toSec()) return now - ros::Duration(age);
    }
  } else if (clock) gst_object_unref(clock);
  ROS_WARN_THROTTLE(5, "No usable capture PTS; using ROS receipt time");
  return now;
}
void validateInfo(const sensor_msgs::CameraInfo& info, int width, int height) {
  if (info.width != static_cast<unsigned>(width) || info.height != static_cast<unsigned>(height))
    throw std::runtime_error("calibration resolution differs from requested camera resolution");
  for (double value : info.K) if (!std::isfinite(value)) throw std::runtime_error("non-finite K");
  for (double value : info.D) if (!std::isfinite(value)) throw std::runtime_error("non-finite distortion");
  if (info.K[0] <= 0 || info.K[4] <= 0 || info.distortion_model != "plumb_bob" || info.D.size() != 5)
    throw std::runtime_error("invalid camera calibration");
}
void run() {
  ros::NodeHandle nh, pnh("~");
  std::string device, name, frame, url, image_topic, info_topic;
  int width, height, fps; double timeout;
  pnh.param<std::string>("video_device", device, "/dev/video12");
  pnh.param<std::string>("camera_name", name, "ov13855_1280x960");
  pnh.param<std::string>("camera_frame_id", frame, "ov13855_optical_frame");
  pnh.param<std::string>("camera_info_url", url, "package://ov13855_cam/config/ov13855_1280x960.yaml");
  pnh.param<std::string>("image_topic", image_topic, "/ov13855/image_raw");
  pnh.param<std::string>("camera_info_topic", info_topic, "/ov13855/camera_info");
  pnh.param("image_width", width, 1280); pnh.param("image_height", height, 960);
  pnh.param("framerate", fps, 15); pnh.param("frame_timeout_sec", timeout, 5.0);
  if (width <= 0 || height <= 0 || fps <= 0 || frame.empty() || !std::isfinite(timeout) || timeout <= 0 || device.empty())
    throw std::runtime_error("invalid camera configuration");
  if (ros::Time::isSimTime()) throw std::runtime_error("live camera requires wall-clock ROS time");
  camera_info_manager::CameraInfoManager manager(pnh, name);
  if (!manager.loadCameraInfo(url) || !manager.isCalibrated()) throw std::runtime_error("cannot load calibration");
  validateInfo(manager.getCameraInfo(), width, height);
  // image_transport adds <image_topic>/compressed etc.; plugins encode only while subscribed.
  image_transport::ImageTransport transport(nh);
  auto image_pub = transport.advertise(image_topic, 1);
  auto info_pub = nh.advertise<sensor_msgs::CameraInfo>(info_topic, 1);
  Pipeline stream; GError* error = nullptr;
  // No videoconvert: the NV12 Y plane is already the mono8 image. videoconvert recomputed luma through a
  // colour-matrix conversion on one CPU core (~0.9 core at 2560x1440, 5.4 fps at 4224x3136).
  const std::string description = "v4l2src name=source ! video/x-raw,format=NV12,width=" + std::to_string(width) +
    ",height=" + std::to_string(height) + ",framerate=" + std::to_string(fps) +
    "/1 ! appsink name=frames sync=false drop=true max-buffers=1 wait-on-eos=false";
  stream.pipeline = gst_parse_launch(description.c_str(), &error);
  if (error || !stream.pipeline) {
    std::string detail = error ? error->message : "cannot create pipeline";
    g_clear_error(&error); throw std::runtime_error(detail);
  }
  // Set device as a property, not as pipeline source text.
  GstElement* source = gst_bin_get_by_name(GST_BIN(stream.pipeline), "source");
  g_object_set(source, "device", device.c_str(), nullptr);
  GstPad* source_pad = gst_element_get_static_pad(source, "src");
  gst_pad_add_probe(source_pad, GST_PAD_PROBE_TYPE_QUERY_DOWNSTREAM,
                    configureCaptureAllocation, nullptr, nullptr);
  gst_object_unref(source_pad);
  gst_object_unref(source);
  stream.sink = gst_bin_get_by_name(GST_BIN(stream.pipeline), "frames");
  if (gst_element_set_state(stream.pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    checkBus(stream.pipeline); throw std::runtime_error("camera failed to start");
  }
  auto last_frame = ros::WallTime::now(); bool first = true;
  while (ros::ok()) {
    ros::spinOnce(); checkBus(stream.pipeline);
    std::unique_ptr<GstSample, decltype(&gst_sample_unref)> sample(
      gst_app_sink_try_pull_sample(GST_APP_SINK(stream.sink), 200 * GST_MSECOND), gst_sample_unref);
    if (!sample) {
      if ((ros::WallTime::now() - last_frame).toSec() > timeout) throw std::runtime_error("camera frame timeout");
      continue;
    }
    last_frame = ros::WallTime::now(); GstVideoInfo video; gst_video_info_init(&video);
    if (!gst_video_info_from_caps(&video, gst_sample_get_caps(sample.get())) ||
        GST_VIDEO_INFO_WIDTH(&video) != width || GST_VIDEO_INFO_HEIGHT(&video) != height ||
        GST_VIDEO_INFO_FORMAT(&video) != GST_VIDEO_FORMAT_NV12 || GST_VIDEO_INFO_FPS_D(&video) <= 0 ||
        GST_VIDEO_INFO_FPS_N(&video) != fps * GST_VIDEO_INFO_FPS_D(&video))
      throw std::runtime_error("negotiated camera caps differ from requested width/height/fps/format");
    // Publishing Y as mono8 assumes full-range (0-255) luma; limited range (16-235) would lose contrast.
    const auto range = GST_VIDEO_INFO_COLORIMETRY(&video).range;
    if (range == GST_VIDEO_COLOR_RANGE_16_235) throw std::runtime_error("limited-range NV12; Y plane is not full-range mono8");
    if (range != GST_VIDEO_COLOR_RANGE_0_255) ROS_WARN_ONCE("NV12 colour range unknown; assuming full-range Y plane");
    auto info = manager.getCameraInfo(); validateInfo(info, width, height);
    info.header.stamp = captureStamp(stream.pipeline, sample.get()); info.header.frame_id = frame;
    info_pub.publish(info);
    // Without image subscribers (the full-resolution path between barcode reads) the frame is still pulled so
    // the ISP keeps streaming, but the Y plane is neither copied nor published.
    if (image_pub.getNumSubscribers() > 0) {
      sensor_msgs::Image image; image.header = info.header;
      image.width = width; image.height = height; image.encoding = "mono8";
      image.is_bigendian = false; image.step = width; image.data.resize(static_cast<size_t>(width) * height);
      GstVideoFrame mapped;
      if (!gst_video_frame_map(&mapped, &video, gst_sample_get_buffer(sample.get()), GST_MAP_READ))
        throw std::runtime_error("cannot map camera frame");
      // Plane 0 of NV12 is the full-resolution Y plane; the UV plane is never read.
      const auto* data = static_cast<const unsigned char*>(GST_VIDEO_FRAME_PLANE_DATA(&mapped, 0));
      const int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&mapped, 0);
      if (stride < width) { gst_video_frame_unmap(&mapped); throw std::runtime_error("invalid frame stride"); }
      for (int row = 0; row < height; ++row)
        std::memcpy(image.data.data() + static_cast<size_t>(row) * width, data + static_cast<size_t>(row) * stride, width);
      gst_video_frame_unmap(&mapped);
      image_pub.publish(image);
    }
    if (first) {
      gchar* colorimetry = gst_video_colorimetry_to_string(&GST_VIDEO_INFO_COLORIMETRY(&video));
      ROS_INFO("Streaming %s %dx%d @ %d caps fps, NV12 Y plane as mono8 (colorimetry %s) to %s while subscribed, "
               "frame=%s", device.c_str(), width, height, fps, colorimetry ? colorimetry : "unknown",
               image_topic.c_str(), frame.c_str());
      g_free(colorimetry); first = false;
    }
  }
}
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "ov13855_cam");
  ros::NodeHandle lifetime; // Keep ROS logging alive while run() unwinds on startup failure.
  gst_init(nullptr, nullptr);
  try { run(); } catch (const std::exception& error) { ROS_FATAL("ov13855_cam: %s", error.what()); return 1; }
  return 0;
}
