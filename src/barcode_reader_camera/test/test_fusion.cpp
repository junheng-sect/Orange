// Drone-Vision vision_core/tests/barcode_result_fusion_test.cpp, run under gtest.
#include <gtest/gtest.h>

#include "barcode_reader_camera/barcode_result_fusion.h"

#include <cmath>
#include <cstdio>
#include <string>

#define CHECK(cond) EXPECT_TRUE(cond) << #cond

static barcode_fusion::DecodeResult make_result(
    const std::string& text,
    barcode_fusion::BarcodeKind kind,
    float x,
    float y,
    float width,
    float height,
    const std::string& source)
{
    barcode_fusion::DecodeResult r;
    r.format = kind;
    r.text = text;
    r.center = {x, y};
    r.width = width;
    r.height = height;
    r.source = source;
    r.valid = true;
    return r;
}

static barcode_fusion::DecodeResult make_rotated_result(
    const std::string& text,
    barcode_fusion::BarcodeKind kind,
    float x,
    float y,
    float long_edge,
    float short_edge,
    float angle_deg,
    const std::string& source)
{
    barcode_fusion::DecodeResult r;
    r.format = kind;
    r.text = text;
    r.center = {x, y};
    r.width = std::abs(long_edge * std::cos(angle_deg * 3.14159265F)) +
              std::abs(short_edge * std::sin(angle_deg * 3.14159265F));
    r.height = std::abs(long_edge * std::sin(angle_deg * 3.14159265F)) +
               std::abs(short_edge * std::cos(angle_deg * 3.14159265F));
    r.source = source;
    r.valid = true;

    const float half_long = long_edge / 2.0F;
    const float half_short = short_edge / 2.0F;
    const float dx_long = half_long * std::cos(angle_deg * 3.14159265F);
    const float dy_long = half_long * std::sin(angle_deg * 3.14159265F);
    const float dx_short = -half_short * std::sin(angle_deg * 3.14159265F);
    const float dy_short = half_short * std::cos(angle_deg * 3.14159265F);
    r.corners = {{
        {x + dx_long + dx_short, y + dy_long + dy_short},
        {x - dx_long + dx_short, y - dy_long + dy_short},
        {x - dx_long - dx_short, y - dy_long - dy_short},
        {x + dx_long - dx_short, y + dy_long - dy_short}
    }};
    return r;
}

static void test_same_code_from_two_strategies_is_merged(void)
{
    barcode_fusion::DecodeResultFusion fusion;

    auto first = make_result("NEXT90157324",
                             barcode_fusion::BarcodeKind::Code128,
                             2200.0F, 575.0F,
                             120.0F, 40.0F,
                             "WARP-1d-hi-cubic");
    auto second = make_result("NEXT90157324",
                              barcode_fusion::BarcodeKind::Code128,
                              2190.0F, 575.0F,
                              118.0F, 42.0F,
                              "ROI-1d-fullres");

    auto a = fusion.add(first);
    auto b = fusion.add(second);

    CHECK(!a.duplicate);
    CHECK(b.duplicate);
    CHECK(fusion.fused().size() == 1);
    CHECK(fusion.stats().raw_results == 2);
    CHECK(fusion.stats().distinct_results == 1);
    CHECK(fusion.stats().duplicates_removed == 1);
}

static void test_same_text_far_apart_is_two_physical_codes(void)
{
    barcode_fusion::DecodeResultFusion fusion;

    auto first = make_result("NEXT90157324",
                             barcode_fusion::BarcodeKind::Code128,
                             1000.0F, 500.0F,
                             120.0F, 40.0F,
                             "A");
    auto second = make_result("NEXT90157324",
                              barcode_fusion::BarcodeKind::Code128,
                              1800.0F, 500.0F,
                              120.0F, 40.0F,
                              "B");

    fusion.add(first);
    fusion.add(second);

    CHECK(fusion.fused().size() == 2);
    CHECK(fusion.stats().distinct_results == 2);
    CHECK(fusion.stats().duplicates_removed == 0);
}

static void test_different_text_close_together_is_not_merged(void)
{
    barcode_fusion::DecodeResultFusion fusion;

    auto first = make_result("NEXT90157324",
                             barcode_fusion::BarcodeKind::Code128,
                             1000.0F, 500.0F,
                             120.0F, 40.0F,
                             "A");
    auto second = make_result("ABC123456",
                              barcode_fusion::BarcodeKind::Code128,
                              1020.0F, 505.0F,
                              120.0F, 40.0F,
                              "B");

    fusion.add(first);
    fusion.add(second);

    CHECK(fusion.fused().size() == 2);
}

static void test_different_format_is_not_merged(void)
{
    barcode_fusion::DecodeResultFusion fusion;

    auto first = make_result("12345678",
                             barcode_fusion::BarcodeKind::QRCode,
                             1000.0F, 500.0F,
                             80.0F, 80.0F,
                             "A");
    auto second = make_result("12345678",
                              barcode_fusion::BarcodeKind::DataMatrix,
                              1010.0F, 505.0F,
                              80.0F, 80.0F,
                              "B");

    fusion.add(first);
    fusion.add(second);

    CHECK(fusion.fused().size() == 2);
}

static void test_retry_duplicate_does_not_increase_count(void)
{
    barcode_fusion::DecodeResultFusion fusion;

    auto first = make_result("RETRY-001",
                             barcode_fusion::BarcodeKind::Code128,
                             1200.0F, 600.0F,
                             100.0F, 30.0F,
                             "attempt-1");
    auto second = make_result("RETRY-001",
                              barcode_fusion::BarcodeKind::Code128,
                              1208.0F, 604.0F,
                              104.0F, 28.0F,
                              "attempt-2");

    fusion.add(first);
    fusion.add(second);

    CHECK(fusion.fused().size() == 1);
    CHECK(fusion.stats().success_by_source.at("attempt-1") == 1);
    CHECK(fusion.stats().success_by_source.at("attempt-2") == 1);
}

static void test_same_slanted_1d_symbol_from_two_strategies_is_merged(void)
{
    barcode_fusion::DecodeResultFusion fusion;

    // This mirrors the real WARP-1d-hi-linear / WARP-1d-hi-cubic duplicate:
    // the two centers were only 12.6px apart, but axis-aligned box IoU was
    // unstable for the slanted Code128.
    auto first = make_rotated_result("huhuhuhhh",
                                     barcode_fusion::BarcodeKind::Code128,
                                     2141.1F, 463.9F,
                                     280.0F, 32.0F,
                                     8.0F,
                                     "WARP-1d-hi-linear");
    auto second = make_rotated_result("huhuhuhhh",
                                      barcode_fusion::BarcodeKind::Code128,
                                      2128.5F, 464.8F,
                                      268.0F, 30.0F,
                                      11.0F,
                                      "WARP-1d-hi-cubic");

    auto a = fusion.add(first);
    auto b = fusion.add(second);

    CHECK(a.duplicate == false);
    CHECK(b.duplicate);
    CHECK(b.center_distance > 0.0F);
    CHECK(b.iou > 0.15F);
    CHECK(fusion.fused().size() == 1);
}

static void test_same_1d_text_with_center_drift_is_merged(void)
{
    barcode_fusion::DecodeResultFusion fusion;

    // The 2026-09-02 log also had NEXT90157324 at 56.7px center distance
    // from ROI-1d and P1920-1d.  A slanted symbol can produce this amount of
    // drift while remaining one physical code.
    auto first = make_rotated_result("NEXT90157324",
                                     barcode_fusion::BarcodeKind::Code128,
                                     2284.6F, 531.9F,
                                     240.0F, 28.0F,
                                     -14.0F,
                                     "ROI-1d");
    auto second = make_rotated_result("NEXT90157324",
                                      barcode_fusion::BarcodeKind::Code128,
                                      2228.0F, 535.3F,
                                      228.0F, 26.0F,
                                      -11.0F,
                                      "P1920-1d");

    auto a = fusion.add(first);
    auto b = fusion.add(second);

    CHECK(a.duplicate == false);
    CHECK(b.duplicate);
    CHECK(fusion.fused().size() == 1);
}

static void test_same_1d_text_far_apart_is_not_merged_even_with_corners(void)
{
    barcode_fusion::DecodeResultFusion fusion;

    auto first = make_rotated_result("NEXT90157324",
                                     barcode_fusion::BarcodeKind::Code128,
                                     1000.0F, 500.0F,
                                     240.0F, 28.0F,
                                     12.0F,
                                     "A");
    auto second = make_rotated_result("NEXT90157324",
                                      barcode_fusion::BarcodeKind::Code128,
                                      1300.0F, 560.0F,
                                      232.0F, 27.0F,
                                      14.0F,
                                      "B");

    auto a = fusion.add(first);
    auto b = fusion.add(second);

    CHECK(a.duplicate == false);
    CHECK(b.duplicate == false);
    CHECK(b.same_symbol_candidate);
    CHECK(b.center_distance > 0.0F);
    CHECK(fusion.fused().size() == 2);
}

static void test_invalid_result_is_not_counted(void)
{
    barcode_fusion::DecodeResultFusion fusion;

    auto result = make_result("INVALID",
                              barcode_fusion::BarcodeKind::Code128,
                              0.0F, 0.0F,
                              10.0F, 10.0F,
                              "A");
    result.valid = false;

    fusion.add(result);

    CHECK(fusion.fused().empty());
    CHECK(fusion.stats().invalid_results == 1);
}

TEST(BarcodeResultFusion, same_code_from_two_strategies_is_merged) { test_same_code_from_two_strategies_is_merged(); }
TEST(BarcodeResultFusion, same_text_far_apart_is_two_physical_codes) { test_same_text_far_apart_is_two_physical_codes(); }
TEST(BarcodeResultFusion, different_text_close_together_is_not_merged) { test_different_text_close_together_is_not_merged(); }
TEST(BarcodeResultFusion, different_format_is_not_merged) { test_different_format_is_not_merged(); }
TEST(BarcodeResultFusion, retry_duplicate_does_not_increase_count) { test_retry_duplicate_does_not_increase_count(); }
TEST(BarcodeResultFusion, invalid_result_is_not_counted) { test_invalid_result_is_not_counted(); }
TEST(BarcodeResultFusion, same_slanted_1d_symbol_from_two_strategies_is_merged) { test_same_slanted_1d_symbol_from_two_strategies_is_merged(); }
TEST(BarcodeResultFusion, same_1d_text_with_center_drift_is_merged) { test_same_1d_text_with_center_drift_is_merged(); }
TEST(BarcodeResultFusion, same_1d_text_far_apart_is_not_merged_even_with_corners) { test_same_1d_text_far_apart_is_not_merged_even_with_corners(); }

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
