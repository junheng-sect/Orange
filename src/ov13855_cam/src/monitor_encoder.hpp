#pragma once

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

// Task drivers announce intent before waiting for the selfpath. The viewer never
// publishes task image topics and voluntarily releases its own device.
class CaptureLease {
 public:
  CaptureLease(bool viewer) : viewer_(viewer) {
    const auto base = std::string("/tmp/ov13855-") + std::to_string(getuid());
    intent_ = open((base + ".intent").c_str(), O_CREAT|O_CLOEXEC|O_RDWR|O_NOFOLLOW, 0600);
    owner_ = open((base + ".capture").c_str(), O_CREAT|O_CLOEXEC|O_RDWR|O_NOFOLLOW, 0600);
    if (intent_ < 0 || owner_ < 0 || flock(intent_, viewer ? (LOCK_EX | LOCK_NB) : LOCK_SH) < 0)
      throw std::runtime_error("selfpath reserved by another camera");
    for (int i = 0; flock(owner_, LOCK_EX|LOCK_NB) < 0; ++i) {
      if (viewer || i >= 30) throw std::runtime_error("selfpath already in use");
      usleep(100000);
    }
    if (viewer) flock(intent_, LOCK_UN);
  }
  bool yieldToTask() {
    if (!viewer_) return false;
    if (flock(intent_, LOCK_EX|LOCK_NB) < 0) return true;
    flock(intent_, LOCK_UN);
    return false;
  }
  ~CaptureLease() { close(owner_); close(intent_); }
 private:
  bool viewer_;
  int intent_, owner_;
};

class MonitorEncoder {
 public:
  MonitorEncoder(ros::NodeHandle& nh, bool standalone) {
    pub_ = nh.advertise<sensor_msgs::CompressedImage>("/ov13855/monitor/h264", 2);
    source_ = standalone ? "standalone" : "task";
    thread_ = std::thread(&MonitorEncoder::run, this);
  }
  ~MonitorEncoder() {
    { std::lock_guard<std::mutex> lock(mutex_); stopping_ = true; }
    ready_.notify_one(); thread_.join();
    if (pending_) gst_sample_unref(pending_);
  }
  void offer(GstSample* sample, const std_msgs::Header& header) {
    if (!pub_.getNumSubscribers()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_) gst_sample_unref(pending_);
    pending_ = gst_sample_ref(sample); header_ = header; ready_.notify_one();
  }
 private:
  ros::Publisher pub_;
  std::string source_;
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable ready_;
  bool stopping_ = false;
  GstSample* pending_ = nullptr;
  std_msgs::Header header_;
  GstElement *pipe_ = nullptr, *in_ = nullptr, *out_ = nullptr;
  void reset() {
    if (pipe_) gst_element_set_state(pipe_, GST_STATE_NULL);
    if (in_) gst_object_unref(in_);
    if (out_) gst_object_unref(out_);
    if (pipe_) gst_object_unref(pipe_);
    pipe_ = in_ = out_ = nullptr;
  }
  void encode(GstSample* sample, const std_msgs::Header& header) {
    if (!pipe_) {
      GError* err = nullptr;
      pipe_ = gst_parse_launch("appsrc name=input is-live=true format=time ! "
          "mpph264enc rc-mode=cbr bps=3000000 bps-min=2400000 bps-max=3300000 gop=15 header-mode=each-idr profile=baseline ! "
          "video/x-h264,stream-format=byte-stream,alignment=au ! appsink name=output sync=false async=false", &err);
      if (err) { std::string detail(err->message); g_error_free(err); throw std::runtime_error(detail); }
      if (!pipe_) throw std::runtime_error("missing MPP pipeline");
      in_ = gst_bin_get_by_name(GST_BIN(pipe_), "input");
      out_ = gst_bin_get_by_name(GST_BIN(pipe_), "output");
      gst_app_src_set_caps(GST_APP_SRC(in_), gst_sample_get_caps(sample));
      if (gst_element_set_state(pipe_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        throw std::runtime_error("MPP startup failed");
    }
    // Give the hardware its own buffer so encoder latency cannot exhaust RKISP's capture pool.
    GstBuffer* input = gst_buffer_copy_deep(gst_sample_get_buffer(sample));
    if (!input || gst_app_src_push_buffer(GST_APP_SRC(in_), input) != GST_FLOW_OK)
      throw std::runtime_error("MPP refused NV12 frame");
    std::unique_ptr<GstSample, decltype(&gst_sample_unref)> result(
      gst_app_sink_try_pull_sample(GST_APP_SINK(out_), 650*GST_MSECOND), gst_sample_unref);
    if (!result) throw std::runtime_error("MPP returned no H.264 frame");
    auto* buf = gst_sample_get_buffer(result.get());
    sensor_msgs::CompressedImage msg;
    msg.header = header; msg.format = "h264/" + source_;
    msg.data.resize(gst_buffer_get_size(buf));
    gst_buffer_extract(buf, 0, msg.data.data(), msg.data.size());
    pub_.publish(msg);
  }
  void run() {
    for (;;) {
      GstSample* sample;
      std_msgs::Header header;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait_for(lock, std::chrono::milliseconds(250), [&]{return stopping_ || pending_;});
        if (stopping_) break;
        sample = pending_; pending_ = nullptr; header = header_;
      }
      if (!sample) { if (!pub_.getNumSubscribers()) reset(); continue; }
      try { encode(sample, header); }
      catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(5,"OV13855 monitor: %s",e.what());
        sensor_msgs::CompressedImage error;
        error.format = std::string("error/") + e.what(); pub_.publish(error);
        reset(); std::this_thread::sleep_for(std::chrono::milliseconds(600));
      }
      gst_sample_unref(sample);
    }
    reset();
  }
};
