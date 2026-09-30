#include <algorithm>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <ov13855_apriltag_vision/apriltag_detector.hpp>
#include <ov13855_apriltag_vision/pose_solver.hpp>

#include "barcode_reader_camera/multi_decoder.hpp"
#include "synthetic_label.hpp"

using namespace barcode_reader_camera;

namespace {

DecoderConfig testConfig() {
  DecoderConfig c;  // label geometry defaults match the synthetic scene
  c.formats = FormatSwitches{true, false, true, false, false, false, false, false};  // QR + Code128
  c.expected = ExpectedCounts{2, 0, 2, 0, 0, 0, 0, 0};
  return c;
}

bool detectPose(const synthetic::Scene& scene, TagPose& pose) {
  ov13855_apriltag_vision::AprilTagDetector detector("tag36h11", 1.5, 4, true, 1280);
  const auto detections = detector.detect(scene.gray);
  const int index = ov13855_apriltag_vision::selectDetection(detections, "largest", 0);
  if (index < 0) return false;
  ov13855_apriltag_vision::PoseSolution solution;
  std::string reason;
  if (!ov13855_apriltag_vision::solveTagPose(detections[index].corners, scene.tag_size_m,
                                             scene.camera.camera_matrix, scene.camera.dist_coeffs,
                                             solution, reason))
    return false;
  pose.rvec = solution.rvec;
  pose.tvec = solution.tvec;
  return true;
}

std::multiset<std::string> texts(const DecodeOutput& out) {
  std::multiset<std::string> s;
  for (const auto& c : out.codes) s.insert(c.format + ":" + c.code);
  return s;
}

bool anyLineContains(const std::vector<std::string>& lines, const std::string& needle) {
  return std::any_of(lines.begin(), lines.end(),
                     [&](const std::string& l) { return l.find(needle) != std::string::npos; });
}

const std::multiset<std::string> kAll = {"QRCode:QR-TEST-001", "QRCode:QR-TEST-002",
                                         "Code128:C128-12345", "Code128:C128-67890"};

}  // namespace

TEST(Code128Encoder, ModuleCountAndQuietZoneFree) {
  // Start + 10 data + checksum = 12 symbols x 11 modules, stop = 13 modules.
  EXPECT_EQ(synthetic::code128Modules("C128-12345").size(), 12u * 11u + 13u);
}

TEST(MultiDecoder, TagRoiDecodesAllCodesWithRawBytes) {
  const auto scene = synthetic::makeScene();
  TagPose pose;
  ASSERT_TRUE(detectPose(scene, pose));
  EXPECT_NEAR(pose.tvec[2], 0.125, 0.01);  // 8 px/mm at fx=1000

  std::vector<std::string> lines;
  DecodeHooks hooks;
  hooks.log = [&](const std::string& l) { lines.push_back(l); };
  const auto out = decodeBarcodes(scene.gray, &pose, scene.camera, testConfig(), hooks);

  EXPECT_TRUE(out.success) << out.error << " " << out.report;
  EXPECT_FALSE(out.partial);
  EXPECT_EQ(out.error, "");
  EXPECT_TRUE(out.used_tag_roi);
  EXPECT_EQ(out.roi_quad.size(), 4u);
  EXPECT_EQ(texts(out), kAll);
  for (const auto& c : out.codes) {
    EXPECT_EQ(c.bytes, std::vector<uint8_t>(c.code.begin(), c.code.end()));
    EXPECT_TRUE(c.source.rfind("WARP", 0) == 0 || c.source.rfind("ROI", 0) == 0) << c.source;
    EXPECT_EQ(c.symbology_id.substr(0, 2), c.format == "QRCode" ? "]Q" : "]C");
  }
  // Every expected count reached inside the ROI: the full-frame pyramid must not run.
  EXPECT_FALSE(anyLineContains(lines, "P1280"));
  EXPECT_FALSE(anyLineContains(lines, "PFULL"));
  EXPECT_EQ(out.report, "QR=2/2 C128=2/2");
}

TEST(MultiDecoder, NoTagFallsBackToPyramid) {
  const auto scene = synthetic::makeScene(8.0, false);
  const auto out = decodeBarcodes(scene.gray, nullptr, scene.camera, testConfig());
  EXPECT_TRUE(out.success) << out.error << " " << out.report;
  EXPECT_FALSE(out.used_tag_roi);
  EXPECT_TRUE(out.roi_quad.empty());
  EXPECT_EQ(texts(out), kAll);
  for (const auto& c : out.codes) EXPECT_EQ(c.source[0], 'P') << c.source;
}

TEST(MultiDecoder, FreeDecodeRunsWholeChainWithoutDuplicates) {
  const auto scene = synthetic::makeScene();
  TagPose pose;
  ASSERT_TRUE(detectPose(scene, pose));
  auto config = testConfig();
  config.expected = ExpectedCounts{0, 0, 0, 0, 0, 0, 0, 0};
  std::vector<std::string> lines;
  DecodeHooks hooks;
  hooks.log = [&](const std::string& l) { lines.push_back(l); };
  const auto out = decodeBarcodes(scene.gray, &pose, scene.camera, config, hooks);
  EXPECT_TRUE(out.success);  // expected all 0: any code counts as success
  EXPECT_EQ(texts(out), kAll);  // many strategies hit the same codes; fusion keeps one each
  EXPECT_TRUE(anyLineContains(lines, "PFULL"));
  EXPECT_TRUE(anyLineContains(lines, "Fusion duplicate"));
}

TEST(MultiDecoder, UnmetExpectationIsPartial) {
  const auto scene = synthetic::makeScene();
  TagPose pose;
  ASSERT_TRUE(detectPose(scene, pose));
  auto config = testConfig();
  config.expected.c128 = 3;
  config.retry_count = 0;
  const auto out = decodeBarcodes(scene.gray, &pose, scene.camera, config);
  EXPECT_FALSE(out.success);
  EXPECT_TRUE(out.partial);
  EXPECT_EQ(out.error, "partial");
  EXPECT_EQ(texts(out), kAll);
  EXPECT_EQ(out.report, "QR=2/2 C128=2/3");
}

TEST(MultiDecoder, DisabledFormatsAreNotScanned) {
  const auto scene = synthetic::makeScene();
  TagPose pose;
  ASSERT_TRUE(detectPose(scene, pose));
  auto config = testConfig();
  config.formats = FormatSwitches{true, false, false, false, false, false, false, false};
  std::vector<std::string> lines;
  DecodeHooks hooks;
  hooks.log = [&](const std::string& l) { lines.push_back(l); };
  const auto out = decodeBarcodes(scene.gray, &pose, scene.camera, config, hooks);
  EXPECT_TRUE(out.success);
  EXPECT_EQ(texts(out), (std::multiset<std::string>{"QRCode:QR-TEST-001", "QRCode:QR-TEST-002"}));
  EXPECT_FALSE(anyLineContains(lines, "-1d-"));
}

TEST(MultiDecoder, RetryUsesNextFrameOnlyWhenCodesMissing) {
  const auto scene = synthetic::makeScene();
  TagPose pose;
  ASSERT_TRUE(detectPose(scene, pose));
  int calls = 0;
  DecodeHooks hooks;
  hooks.next_frame = [&](cv::Mat& gray) { ++calls; gray = scene.gray; return true; };
  decodeBarcodes(scene.gray, &pose, scene.camera, testConfig(), hooks);
  EXPECT_EQ(calls, 0);  // everything found in the first frame

  auto config = testConfig();
  config.expected.c128 = 3;  // unreachable
  config.retry_interval_ms = 0;
  const auto out = decodeBarcodes(scene.gray, &pose, scene.camera, config, hooks);
  EXPECT_LE(calls, config.retry_count);  // also bounded by the 1500 ms decode budget
  EXPECT_EQ(texts(out), kAll);           // retry duplicates are fused away
}

TEST(MultiDecoder, CancelStopsDecoding) {
  const auto scene = synthetic::makeScene();
  TagPose pose;
  ASSERT_TRUE(detectPose(scene, pose));
  DecodeHooks hooks;
  hooks.cancelled = []() { return true; };
  const auto out = decodeBarcodes(scene.gray, &pose, scene.camera, testConfig(), hooks);
  EXPECT_EQ(out.error, "cancelled");
  EXPECT_FALSE(out.success);
  EXPECT_TRUE(out.codes.empty());
}

TEST(MultiDecoder, DisabledAndNoFormats) {
  const auto scene = synthetic::makeScene();
  auto config = testConfig();
  config.enabled = false;
  EXPECT_EQ(decodeBarcodes(scene.gray, nullptr, scene.camera, config).error, "disabled");
  config = testConfig();
  config.formats = FormatSwitches{false, false, false, false, false, false, false, false};
  EXPECT_EQ(decodeBarcodes(scene.gray, nullptr, scene.camera, config).error, "no_formats");
  EXPECT_THROW(decodeBarcodes(cv::Mat(), nullptr, scene.camera, config), std::invalid_argument);
}

TEST(MultiDecoder, ToHex) {
  EXPECT_EQ(toHex({0x4e, 0x45, 0x00, 0xff}), "4e 45 00 ff");
  EXPECT_EQ(toHex({}), "");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
