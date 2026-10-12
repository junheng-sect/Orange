// rostest: publish a synthetic label (image + camera_info + tag pose with the same stamp),
// then exercise ~decode, ~result and ~cancel of barcode_reader_camera_node.
#include <dirent.h>

#include <atomic>
#include <fstream>
#include <mutex>
#include <set>
#include <string>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <ov13855_apriltag_vision/apriltag_detector.hpp>
#include <ov13855_apriltag_vision/pose_solver.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Trigger.h>
#include <yaml-cpp/yaml.h>

#include "barcode_reader_camera/BarcodeArray.h"
#include "barcode_reader_camera/DecodeBarcodes.h"
#include "synthetic_label.hpp"

namespace {

geometry_msgs::Pose detectPose(const synthetic::Scene& scene) {
  ov13855_apriltag_vision::AprilTagDetector detector("tag36h11", 1.5, 4, true, 1280);
  const auto detections = detector.detect(scene.gray);
  const int index = ov13855_apriltag_vision::selectDetection(detections, "largest", 0);
  EXPECT_GE(index, 0);
  ov13855_apriltag_vision::PoseSolution solution;
  std::string reason;
  EXPECT_TRUE(ov13855_apriltag_vision::solveTagPose(detections[index].corners, scene.tag_size_m,
                                                    scene.camera.camera_matrix, scene.camera.dist_coeffs,
                                                    solution, reason));
  cv::Matx33d r;
  cv::Rodrigues(solution.rvec, r);
  // Rotation matrix -> quaternion (same convention ov13855_apriltag_vision publishes).
  geometry_msgs::Pose pose;
  const double tr = r(0, 0) + r(1, 1) + r(2, 2);
  if (tr > 0) {
    const double s = std::sqrt(tr + 1.0) * 2;
    pose.orientation.w = 0.25 * s;
    pose.orientation.x = (r(2, 1) - r(1, 2)) / s;
    pose.orientation.y = (r(0, 2) - r(2, 0)) / s;
    pose.orientation.z = (r(1, 0) - r(0, 1)) / s;
  } else if (r(0, 0) > r(1, 1) && r(0, 0) > r(2, 2)) {
    const double s = std::sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2;
    pose.orientation.w = (r(2, 1) - r(1, 2)) / s;
    pose.orientation.x = 0.25 * s;
    pose.orientation.y = (r(0, 1) + r(1, 0)) / s;
    pose.orientation.z = (r(0, 2) + r(2, 0)) / s;
  } else if (r(1, 1) > r(2, 2)) {
    const double s = std::sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2;
    pose.orientation.w = (r(0, 2) - r(2, 0)) / s;
    pose.orientation.x = (r(0, 1) + r(1, 0)) / s;
    pose.orientation.y = 0.25 * s;
    pose.orientation.z = (r(1, 2) + r(2, 1)) / s;
  } else {
    const double s = std::sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2;
    pose.orientation.w = (r(1, 0) - r(0, 1)) / s;
    pose.orientation.x = (r(0, 2) + r(2, 0)) / s;
    pose.orientation.y = (r(1, 2) + r(2, 1)) / s;
    pose.orientation.z = 0.25 * s;
  }
  pose.position.x = solution.tvec[0];
  pose.position.y = solution.tvec[1];
  pose.position.z = solution.tvec[2];
  return pose;
}

class Publisher {
 public:
  explicit Publisher(const synthetic::Scene& scene) : scene_(scene), pose_(detectPose(scene)) {
    cv::blur(scene.gray, blurred_gray_, cv::Size(41, 1));  // motion blur: nothing decodes
    ros::NodeHandle nh;
    image_pub_ = nh.advertise<sensor_msgs::Image>("/test/image_raw", 1);
    info_pub_ = nh.advertise<sensor_msgs::CameraInfo>("/test/camera_info", 1);
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/test/pose", 1);
    thread_ = std::thread([this]() { run(); });
  }
  ~Publisher() {
    stop_ = true;
    thread_.join();
  }
  std::atomic<bool> publish_pose{true};
  std::atomic<int> pose_delay_ms{0};  // > 0: pose after its image, like the real vision node
  std::atomic<bool> blurred{false};   // publish motion-blurred frames (the pose is still published)

 private:
  void run() {
    ros::Rate rate(10);
    while (!stop_ && ros::ok()) {
      std_msgs::Header header;
      header.stamp = ros::Time::now();
      header.frame_id = "ov13855_optical_frame";
      sensor_msgs::CameraInfo info;
      info.header = header;
      info.width = scene_.gray.cols;
      info.height = scene_.gray.rows;
      for (int i = 0; i < 9; ++i) info.K[i] = scene_.camera.camera_matrix.at<double>(i / 3, i % 3);
      info.D.assign(5, 0.0);
      info_pub_.publish(info);
      geometry_msgs::PoseStamped pose;
      pose.header = header;
      pose.pose = pose_;
      const int delay_ms = pose_delay_ms;
      if (publish_pose && delay_ms <= 0) pose_pub_.publish(pose);  // pose buffered before its image
      image_pub_.publish(cv_bridge::CvImage(header, "mono8", blurred ? blurred_gray_ : scene_.gray).toImageMsg());
      if (publish_pose && delay_ms > 0) {
        ros::Duration(delay_ms / 1000.0).sleep();
        pose_pub_.publish(pose);
      }
      rate.sleep();
    }
  }
  const synthetic::Scene& scene_;
  cv::Mat blurred_gray_;
  geometry_msgs::Pose pose_;
  ros::Publisher image_pub_, info_pub_, pose_pub_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
};

}  // namespace

TEST(BarcodeReaderCameraNode, DecodeWithTagRoiThenNoTag) {
  const auto scene = synthetic::makeScene();
  ros::NodeHandle nh;
  std::mutex results_mutex;
  std::vector<barcode_reader_camera::BarcodeArray> results;
  auto sub = nh.subscribe<barcode_reader_camera::BarcodeArray>(
      "/barcode_reader_camera/result", 10, [&](const barcode_reader_camera::BarcodeArray::ConstPtr& m) {
        std::lock_guard<std::mutex> lock(results_mutex);
        results.push_back(*m);
      });
  ASSERT_TRUE(ros::service::waitForService("/barcode_reader_camera/decode", 20000));

  Publisher publisher(scene);
  ros::Duration(1.5).sleep();

  barcode_reader_camera::DecodeBarcodes srv;
  ASSERT_TRUE(ros::service::call("/barcode_reader_camera/decode", srv));
  EXPECT_TRUE(srv.response.success) << srv.response.error << " " << srv.response.report;
  EXPECT_TRUE(srv.response.used_tag_roi);
  std::multiset<std::string> got;
  for (const auto& c : srv.response.codes) {
    got.insert(c.format + ":" + c.code);
    EXPECT_FALSE(c.raw_hex.empty());
  }
  EXPECT_EQ(got, (std::multiset<std::string>{"QRCode:QR-TEST-001", "QRCode:QR-TEST-002",
                                             "Code128:C128-12345", "Code128:C128-67890"}));

  publisher.publish_pose = false;  // images keep coming, no matching pose -> no_tag
  ros::Duration(0.5).sleep();
  ASSERT_TRUE(ros::service::call("/barcode_reader_camera/decode", srv));
  EXPECT_FALSE(srv.response.success);
  EXPECT_EQ(srv.response.error, "no_tag");
  EXPECT_TRUE(srv.response.codes.empty());

  std_srvs::Trigger cancel;
  ASSERT_TRUE(ros::service::call("/barcode_reader_camera/cancel", cancel));
  EXPECT_FALSE(cancel.response.success);  // nothing running

  ros::Time deadline = ros::Time::now() + ros::Duration(2.0);
  auto count = [&]() { std::lock_guard<std::mutex> lock(results_mutex); return results.size(); };
  while (count() < 2 && ros::Time::now() < deadline) ros::Duration(0.05).sleep();
  std::lock_guard<std::mutex> lock(results_mutex);
  ASSERT_EQ(results.size(), 2u);
  EXPECT_TRUE(results[0].success);
  EXPECT_EQ(results[0].codes.size(), 4u);
  EXPECT_EQ(results[1].error, "no_tag");
}

TEST(BarcodeReaderCameraNode, WaitsForPoseThatArrivesAfterImage) {
  const auto scene = synthetic::makeScene();
  ASSERT_TRUE(ros::service::waitForService("/barcode_reader_camera/decode", 20000));
  Publisher publisher(scene);
  publisher.pose_delay_ms = 60;  // > half the 100 ms frame period: the latest frame usually has no pose yet
  ros::Duration(1.0).sleep();
  for (int i = 0; i < 6; ++i) {
    barcode_reader_camera::DecodeBarcodes srv;
    ASSERT_TRUE(ros::service::call("/barcode_reader_camera/decode", srv));
    EXPECT_TRUE(srv.response.success) << i << ": " << srv.response.error << " " << srv.response.report;
    EXPECT_TRUE(srv.response.used_tag_roi) << i;
    ros::Duration(0.037 * (i + 1)).sleep();  // vary the phase against the frame period
  }
}

TEST(BarcodeReaderCameraNode, RetriesNewFramesUntilDecoded) {
  const auto scene = synthetic::makeScene();
  ASSERT_TRUE(ros::service::waitForService("/barcode_reader_camera/decode", 20000));
  Publisher publisher(scene);
  publisher.blurred = true;
  ros::Duration(1.0).sleep();

  barcode_reader_camera::DecodeBarcodes srv;
  const ros::Time called = ros::Time::now();
  std::thread sharpen([&]() {
    ros::Duration(0.6).sleep();
    publisher.blurred = false;  // the drone steadied: later frames are sharp
  });
  const bool called_ok = ros::service::call("/barcode_reader_camera/decode", srv);
  sharpen.join();
  ASSERT_TRUE(called_ok);
  EXPECT_TRUE(srv.response.success) << srv.response.error << " " << srv.response.report;
  EXPECT_EQ(srv.response.codes.size(), 4u);
  for (const auto& c : srv.response.codes) EXPECT_EQ(c.source.rfind("RT-", 0), 0u) << c.source;
  // The result refers to the sharp retry frame, not to the blurred frame the call started with.
  EXPECT_GT((srv.response.header.stamp - called).toSec(), 0.4);
  EXPECT_LT(srv.response.elapsed_s, 2.0f + 0.5f);
}

namespace {

const char* kSaveDir = "/tmp/barcode_reader_camera_node_test";  // ~save_dir in node.test

std::set<std::string> savedDirs() {
  std::set<std::string> out;
  if (DIR* d = opendir(kSaveDir)) {
    while (const dirent* e = readdir(d)) {
      const std::string n = e->d_name;
      if (n != "." && n != ".." && n != "attempts.csv") out.insert(n);
    }
    closedir(d);
  }
  return out;
}

size_t csvRows() {
  std::ifstream f(std::string(kSaveDir) + "/attempts.csv");
  size_t n = 0;
  for (std::string l; std::getline(f, l);) ++n;
  return n;
}

bool readable(const std::string& path) { return std::ifstream(path).good(); }

}  // namespace

TEST(BarcodeReaderCameraNode, RecordsEachAttempt) {
  const auto scene = synthetic::makeScene();
  ASSERT_TRUE(ros::service::waitForService("/barcode_reader_camera/decode", 20000));
  Publisher publisher(scene);
  ros::Duration(1.0).sleep();
  const auto before = savedDirs();
  const size_t rows_before = csvRows();

  barcode_reader_camera::DecodeBarcodes srv;
  ASSERT_TRUE(ros::service::call("/barcode_reader_camera/decode", srv));
  ASSERT_TRUE(srv.response.success) << srv.response.error;

  // Written in the background after the response: wait for the directory and the CSV row.
  std::string added;
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(5.0);
  while (ros::WallTime::now() < deadline) {
    for (const auto& d : savedDirs())
      if (!before.count(d) && readable(std::string(kSaveDir) + "/" + d + "/meta.yaml")) added = d;
    if (!added.empty() && csvRows() > rows_before) break;
    ros::WallDuration(0.05).sleep();
  }
  ASSERT_FALSE(added.empty());
  EXPECT_EQ(added.substr(added.size() - 3), "_ok");
  const std::string d = std::string(kSaveDir) + "/" + added;
  EXPECT_TRUE(readable(d + "/frame.png"));
  EXPECT_TRUE(readable(d + "/camera.yaml"));
  EXPECT_TRUE(readable(d + "/preview.jpg"));
  std::ifstream meta(d + "/meta.yaml");
  const std::string text((std::istreambuf_iterator<char>(meta)), std::istreambuf_iterator<char>());
  EXPECT_NE(text.find("rvec:"), std::string::npos);
  EXPECT_NE(text.find("QR-TEST-001"), std::string::npos);
  EXPECT_EQ(csvRows(), std::max<size_t>(rows_before, 1) + 1);  // header written on first use
}

TEST(BarcodeReaderCameraNode, FailedAttemptRecordsRetryFrames) {
  const auto scene = synthetic::makeScene();
  ASSERT_TRUE(ros::service::waitForService("/barcode_reader_camera/decode", 20000));
  Publisher publisher(scene);
  publisher.blurred = true;
  ros::Duration(1.0).sleep();
  const auto before = savedDirs();

  barcode_reader_camera::DecodeBarcodes srv;
  ASSERT_TRUE(ros::service::call("/barcode_reader_camera/decode", srv));
  EXPECT_EQ(srv.response.error, "decode_fail");
  // Retries until retry.timeout_ms (2000 ms default, from decode start) plus at most one frame.
  EXPECT_GT(srv.response.elapsed_s, 1.9f);
  EXPECT_LT(srv.response.elapsed_s, 2.0f + 0.5f);

  std::string added;
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(10.0);
  while (added.empty() && ros::WallTime::now() < deadline) {
    for (const auto& d : savedDirs())
      if (!before.count(d) && readable(std::string(kSaveDir) + "/" + d + "/meta.yaml")) added = d;
    ros::WallDuration(0.05).sleep();
  }
  ASSERT_FALSE(added.empty());
  EXPECT_EQ(added.substr(added.size() - 12), "_decode_fail");
  const std::string d = std::string(kSaveDir) + "/" + added;
  const YAML::Node meta = YAML::LoadFile(d + "/meta.yaml");
  const YAML::Node retries = meta["retries"];
  const int frames = meta["result"]["frames"].as<int>();
  ASSERT_EQ((int)retries.size(), frames - 1);
  // QR + Code128 here: the first frame's full chain takes ~0.75 s and each retry frame ~0.45 s (1D ROI
  // strategies on a 1000x1500 px ROI), so 2 s fit 3 retry frames on the board. QR-only layouts take ~20 ms.
  EXPECT_GE(frames, 3);
  // Every retry frame keeps its stamp and pose; images only for the first 2 (save_retry_frames) and the last.
  int saved = 0;
  double last_stamp = meta["stamp"].as<double>();
  for (size_t i = 0; i < retries.size(); ++i) {
    EXPECT_EQ(retries[i]["index"].as<int>(), (int)i + 1);
    EXPECT_EQ(retries[i]["tvec"].size(), 3u);
    EXPECT_GT(retries[i]["stamp"].as<double>(), last_stamp);
    last_stamp = retries[i]["stamp"].as<double>();
    const std::string file = retries[i]["file"].as<std::string>();
    const bool expect_saved = i < 2 || i + 1 == retries.size();
    EXPECT_EQ(!file.empty(), expect_saved) << i;
    if (!file.empty()) {
      EXPECT_TRUE(readable(d + "/" + file)) << file;
      ++saved;
    }
  }
  EXPECT_EQ(saved, 3);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_barcode_reader_camera_node");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  return RUN_ALL_TESTS();
}
