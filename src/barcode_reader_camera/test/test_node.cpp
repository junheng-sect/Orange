// rostest: publish a synthetic label (image + camera_info + tag pose with the same stamp),
// then exercise ~decode, ~result and ~cancel of barcode_reader_camera_node.
#include <atomic>
#include <mutex>
#include <set>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <ov13855_apriltag_vision/apriltag_detector.hpp>
#include <ov13855_apriltag_vision/pose_solver.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Trigger.h>

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
      if (publish_pose) {  // pose first so it is buffered before its image arrives
        geometry_msgs::PoseStamped pose;
        pose.header = header;
        pose.pose = pose_;
        pose_pub_.publish(pose);
      }
      image_pub_.publish(cv_bridge::CvImage(header, "mono8", scene_.gray).toImageMsg());
      rate.sleep();
    }
  }
  const synthetic::Scene& scene_;
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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_barcode_reader_camera_node");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  return RUN_ALL_TESTS();
}
