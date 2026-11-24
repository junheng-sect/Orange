// FrameRecorder: session directory, per-attempt directories by mode, attempts.csv, and files that
// decode_image can replay.
#include <dirent.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/stat.h>

#include <ctime>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include "barcode_reader_camera/config_yaml.hpp"
#include "barcode_reader_camera/frame_recorder.hpp"

using namespace barcode_reader_camera;

namespace {

std::string makeTempDir() {
  char tmpl[] = "/tmp/frame_recorder_test_XXXXXX";
  const char* d = mkdtemp(tmpl);
  EXPECT_NE(d, nullptr);
  return d ? d : "";
}

std::vector<std::string> subdirs(const std::string& dir) {
  std::vector<std::string> out;
  if (DIR* d = opendir(dir.c_str())) {
    while (const dirent* e = readdir(d)) {
      const std::string n = e->d_name;
      if (n != "." && n != ".." && n.rfind("attempts", 0) != 0) out.push_back(n);
    }
    closedir(d);
  }
  return out;
}

std::vector<std::string> lines(const std::string& path) {
  std::ifstream f(path);
  std::vector<std::string> out;
  for (std::string l; std::getline(f, l);) out.push_back(l);
  return out;
}

bool readable(const std::string& path) { return std::ifstream(path).good(); }

bool isDir(const std::string& path) {
  struct stat st;
  return ::stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

bool endsWith(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

DecodeRecord makeRecord(bool success, const std::string& error) {
  DecodeRecord r;
  r.wall_time = 1760190005.123;
  r.stamp = 1760190005.05;
  r.frame_id = "ov13855_optical_frame";
  r.frame = cv::Mat(144, 256, CV_8UC1);
  cv::randu(r.frame, 0, 255);
  r.have_pose = true;
  r.pose.rvec = cv::Vec3d(2.95965, -0.054603, -0.973757);
  r.pose.tvec = cv::Vec3d(-0.0306827, -0.0125738, 0.362314);
  for (int i = 1; i <= 2; ++i) {  // retry 1 kept, retry 2 image dropped by the node
    RetryFrameRecord f;
    f.index = i;
    f.stamp = 1760190005.05 + 0.0667 * i;
    f.pose = r.pose;
    f.pose.tvec[0] += 0.001 * i;
    if (i == 1) f.gray = cv::Mat(144, 256, CV_8UC1, cv::Scalar(77));
    r.retries.push_back(f);
  }
  r.camera.camera_matrix = (cv::Mat_<double>(3, 3) << 188.75, 0, 128.7, 0, 142.9, 71.9, 0, 0, 1);
  r.camera.dist_coeffs = (cv::Mat_<double>(1, 5) << 0.182839, -0.557115, -0.001278, -0.000407, 0.498597);
  r.success = success;
  r.error = error;
  r.elapsed_s = 0.032;
  r.out.success = success;
  r.out.error = error;
  r.out.elapsed_ms = 31.0;
  r.out.used_tag_roi = true;
  r.out.report = success ? "QR=1/1" : "QR=0/1";
  r.out.frames = 3;
  r.out.roi_quad = {{90, 94}, {141, 97}, {138, 14}, {89, 26}};
  if (success) {
    DecodedCode c;
    c.format = "QRCode";
    c.code = "12345678";
    c.bytes = {0x31, 0x32};
    c.corners = {{{105, 37}, {119, 35}, {119, 49}, {106, 51}}};
    c.center = {112, 43};
    c.source = "WARP-2d";
    r.out.codes.push_back(c);
  } else {
    r.out.failed_boxes.push_back({{{105, 37}, {119, 35}, {119, 49}, {106, 51}}});
  }
  r.log = {"[Vision] T WARP-2d: 13ms", "[DecodeReport] 32ms QR=1/1"};
  r.layout_file = "layouts/qr_top_tag_bottom.yaml";
  r.config_file = "barcode_reader_camera.yaml";
  return r;
}

}  // namespace

TEST(FrameRecorder, AllModeWritesReplayableAttempt) {
  const std::string dir = makeTempDir() + "/frames";  // created by the recorder
  const DecodeRecord rec = makeRecord(true, "");
  {
    FrameRecorder recorder(dir, SaveFrames::All, true);
    ASSERT_TRUE(recorder.submit(rec));
  }  // destructor drains the queue
  const auto dirs = subdirs(dir);
  ASSERT_EQ(dirs.size(), 1u);
  EXPECT_TRUE(endsWith(dirs[0], "4005_123_ok")) << dirs[0];  // local time: YYYYmmdd_HHMMSS_mmm_<result>
  const std::string d = dir + "/" + dirs[0];

  const cv::Mat frame = cv::imread(d + "/frame.png", cv::IMREAD_UNCHANGED);
  ASSERT_FALSE(frame.empty());
  EXPECT_EQ(cv::norm(frame, rec.frame, cv::NORM_INF), 0.0);
  EXPECT_FALSE(cv::imread(d + "/retry_1.png", cv::IMREAD_UNCHANGED).empty());
  EXPECT_NE(access((d + "/preview.jpg").c_str(), F_OK), 0);
  EXPECT_EQ(lines(d + "/decode.log"), rec.log);

  int w = 0, h = 0;
  const CameraModel cam = loadCameraYaml(d + "/camera.yaml", &w, &h);
  EXPECT_EQ(w, 256);
  EXPECT_EQ(h, 144);
  EXPECT_LT(cv::norm(cam.camera_matrix, rec.camera.camera_matrix, cv::NORM_INF), 1e-9);
  EXPECT_LT(cv::norm(cam.dist_coeffs, rec.camera.dist_coeffs, cv::NORM_INF), 1e-9);

  const YAML::Node meta = YAML::LoadFile(d + "/meta.yaml");
  const auto rvec = meta["pose"]["rvec"].as<std::vector<double>>();
  const auto tvec = meta["pose"]["tvec"].as<std::vector<double>>();
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(rvec[i], rec.pose.rvec[i], 1e-9);
    EXPECT_NEAR(tvec[i], rec.pose.tvec[i], 1e-9);
  }
  EXPECT_NEAR(meta["pose"]["distance_m"].as<double>(), cv::norm(rec.pose.tvec), 1e-6);
  EXPECT_EQ(meta["result"]["status"].as<std::string>(), "ok");
  EXPECT_EQ(meta["codes"][0]["code"].as<std::string>(), "12345678");
  EXPECT_EQ(meta["codes"][0]["source"].as<std::string>(), "WARP-2d");
  ASSERT_EQ(meta["retries"].size(), 2u);  // every retry frame, image or not
  EXPECT_EQ(meta["retries"][0]["file"].as<std::string>(), "retry_1.png");
  EXPECT_EQ(meta["retries"][1]["file"].as<std::string>(), "");
  EXPECT_EQ(meta["retries"][1]["index"].as<int>(), 2);
  EXPECT_NEAR(meta["retries"][1]["tvec"][0].as<double>(), rec.pose.tvec[0] + 0.002, 1e-9);
  EXPECT_FALSE(readable(d + "/retry_2.png"));
  EXPECT_EQ(meta["result"]["frames"].as<int>(), 3);
  EXPECT_EQ(meta["codes"][0]["frame"].as<int>(), 0);
  EXPECT_EQ(meta["roi_quad"].size(), 4u);
  EXPECT_EQ(meta["stamp"].as<std::string>(), "1760190005.050000");

  const auto csv = lines(dir + "/attempts.csv");
  ASSERT_EQ(csv.size(), 2u);
  EXPECT_EQ(csv[0].rfind("time,stamp,result,", 0), 0u);
  EXPECT_NE(csv[1].find(",ok,"), std::string::npos);
  EXPECT_NE(csv[1].find("\"QRCode:12345678\""), std::string::npos);
  EXPECT_NE(csv[1].find("\"WARP-2d\""), std::string::npos);
  EXPECT_NE(csv[1].find(",0.364,0.362,"), std::string::npos);  // dist_m, z_m
  EXPECT_TRUE(endsWith(csv[1], ",3,0,\"" + dirs[0] + "\""));  // frames, result_frame, dir
}

TEST(FrameRecorder, KeepsResultFrameForDesktopPreview) {
  const std::string dir = makeTempDir();
  DecodeRecord rec = makeRecord(true, "");
  rec.retries[1].gray = cv::Mat(144, 256, CV_8UC1, cv::Scalar(200));
  rec.out.codes[0].frame = 2;
  rec.out.codes[0].source = "RT-WARP-2d";
  rec.out.result_frame = 2;
  {
    FrameRecorder recorder(dir, SaveFrames::All, true);
    ASSERT_TRUE(recorder.submit(rec));
  }
  const auto dirs = subdirs(dir);
  ASSERT_EQ(dirs.size(), 1u);
  const std::string d = dir + "/" + dirs[0];
  EXPECT_FALSE(cv::imread(d + "/retry_2.png", cv::IMREAD_UNCHANGED).empty());
  EXPECT_NE(access((d + "/preview.jpg").c_str(), F_OK), 0);
  const YAML::Node meta = YAML::LoadFile(d + "/meta.yaml");
  EXPECT_EQ(meta["result"]["result_frame"].as<int>(), 2);
  EXPECT_EQ(meta["codes"][0]["frame"].as<int>(), 2);
  const auto csv = lines(dir + "/attempts.csv");
  ASSERT_EQ(csv.size(), 2u);
  EXPECT_TRUE(endsWith(csv[1], ",3,2,\"" + dirs[0] + "\""));
}

TEST(FrameRecorder, SessionDirIsNamedByStartTimeAndLabel) {
  const std::string root = makeTempDir();
  const double t = 1760190005.9;
  const time_t s = (time_t)t;
  struct tm tm;
  localtime_r(&s, &tm);
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm);
  const std::string expected = root + "/" + stamp + "_qr_top_tag_bottom";
  const std::string first = sessionDir(root + "/", "qr_top_tag_bottom", t);
  EXPECT_EQ(first, expected);
  EXPECT_FALSE(isDir(first));  // only the path, nothing created
  ASSERT_EQ(mkdir(first.c_str(), 0775), 0);
  EXPECT_EQ(sessionDir(root, "qr_top_tag_bottom", t), expected + "_2");  // a restart within the same second
}

TEST(FrameRecorder, CreatesSessionDirWithFirstRecord) {
  const std::string dir = makeTempDir() + "/session/run";
  {
    FrameRecorder recorder(dir, SaveFrames::Off, true);
    EXPECT_FALSE(isDir(dir));  // a run without decodes leaves nothing behind
    ASSERT_TRUE(recorder.submit(makeRecord(true, "")));
  }
  EXPECT_TRUE(isDir(dir));
  EXPECT_EQ(lines(dir + "/attempts.csv").size(), 2u);
}

TEST(FrameRecorder, FailedModeSkipsSuccessfulFrames) {
  const std::string dir = makeTempDir();
  {
    FrameRecorder recorder(dir, SaveFrames::Failed, true);
    ASSERT_TRUE(recorder.submit(makeRecord(true, "")));
    DecodeRecord fail = makeRecord(false, "decode_fail");
    fail.wall_time += 1.0;
    ASSERT_TRUE(recorder.submit(fail));
  }
  const auto dirs = subdirs(dir);
  ASSERT_EQ(dirs.size(), 1u);
  EXPECT_TRUE(endsWith(dirs[0], "_decode_fail"));
  const YAML::Node meta = YAML::LoadFile(dir + "/" + dirs[0] + "/meta.yaml");
  EXPECT_EQ(meta["result"]["status"].as<std::string>(), "fail");
  EXPECT_EQ(meta["failed_boxes"].size(), 1u);
  const auto csv = lines(dir + "/attempts.csv");
  ASSERT_EQ(csv.size(), 3u);  // both attempts logged
  EXPECT_TRUE(endsWith(csv[1], ",\"\""));  // success row: no directory
}

TEST(FrameRecorder, OffModeLogsCsvOnlyIncludingNoImage) {
  const std::string dir = makeTempDir();
  {
    FrameRecorder recorder(dir, SaveFrames::Off, true);
    EXPECT_FALSE(recorder.savesFrames());
    DecodeRecord none;  // ~decode with no frame (no_image)
    none.wall_time = 1760190005.0;
    none.error = "no_image";
    ASSERT_TRUE(recorder.submit(none));
    ASSERT_TRUE(recorder.submit(makeRecord(false, "decode_fail")));
  }
  EXPECT_TRUE(subdirs(dir).empty());
  const auto csv = lines(dir + "/attempts.csv");
  ASSERT_EQ(csv.size(), 3u);
  EXPECT_NE(csv[1].find(",fail,\"no_image\","), std::string::npos);
}

TEST(FrameRecorder, ThrottlesFramesWithoutDecode) {
  const std::string dir = makeTempDir();
  {
    FrameRecorder recorder(dir, SaveFrames::All, true, 16);  // 5 records back to back
    for (double dt : {0.0, 0.5, 1.0, 2.1}) {  // trigger_decode retrying no_tag every 0.5 s
      DecodeRecord r = makeRecord(false, "no_tag");
      r.have_pose = false;
      r.wall_time += dt;
      ASSERT_TRUE(recorder.submit(r));
    }
    DecodeRecord fail = makeRecord(false, "decode_fail");
    fail.wall_time += 2.2;
    ASSERT_TRUE(recorder.submit(fail));
  }
  const auto dirs = subdirs(dir);
  EXPECT_EQ(dirs.size(), 3u);  // no_tag at 0 s and 2.1 s, plus the decode failure
  EXPECT_EQ(lines(dir + "/attempts.csv").size(), 6u);
}

TEST(FrameRecorder, DisabledCreatesNothing) {
  const std::string dir = makeTempDir() + "/unused";
  FrameRecorder recorder(dir, SaveFrames::Off, false);
  EXPECT_FALSE(recorder.enabled());
  EXPECT_TRUE(recorder.submit(makeRecord(true, "")));
  DIR* d = opendir(dir.c_str());
  EXPECT_EQ(d, nullptr);
  if (d) closedir(d);
}

TEST(FrameRecorder, ParseSaveFrames) {
  EXPECT_EQ(parseSaveFrames("off"), SaveFrames::Off);
  EXPECT_EQ(parseSaveFrames("failed"), SaveFrames::Failed);
  EXPECT_EQ(parseSaveFrames("all"), SaveFrames::All);
  EXPECT_THROW(parseSaveFrames("yes"), std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
