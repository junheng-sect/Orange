#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

namespace barcode_fusion {

enum class BarcodeKind {
    Unknown = 0,
    QRCode,
    DataMatrix,
    Code128,
    Code39,
    EAN13,
    PDF417,
    Aztec,
    ITF,
};

struct Point2F {
    float x = 0.0F;
    float y = 0.0F;
};

struct DecodeResult {
    BarcodeKind format = BarcodeKind::Unknown;
    std::string text;
    Point2F center;
    float width = 0.0F;
    float height = 0.0F;
    std::array<Point2F, 4> corners{};
    std::string source;
    float confidence = 1.0F;
    int transform_id = 0;
    bool valid = true;
};

struct FusionStats {
    std::size_t raw_results = 0;
    std::size_t distinct_results = 0;
    std::size_t duplicates_removed = 0;
    std::size_t invalid_results = 0;
    std::map<BarcodeKind, std::size_t> raw_by_format;
    std::map<BarcodeKind, std::size_t> distinct_by_format;
    std::map<std::string, std::size_t> success_by_source;
};

struct FusionOutcome {
    DecodeResult fused;
    bool duplicate = false;
    bool same_symbol_candidate = false;
    float center_distance = 0.0F;
    float iou = 0.0F;
    float reference_long_edge = 0.0F;
};

inline bool is_finite_point(const Point2F& p)
{
    return std::isfinite(p.x) && std::isfinite(p.y);
}

inline bool has_valid_geometry(const DecodeResult& r)
{
    return is_finite_point(r.center) &&
           r.width > 0.0F &&
           r.height > 0.0F &&
           std::isfinite(r.width) &&
           std::isfinite(r.height);
}

inline float rectangle_overlap_area(const DecodeResult& a, const DecodeResult& b)
{
    const float a_left = a.center.x - a.width / 2.0F;
    const float a_right = a.center.x + a.width / 2.0F;
    const float a_top = a.center.y - a.height / 2.0F;
    const float a_bottom = a.center.y + a.height / 2.0F;

    const float b_left = b.center.x - b.width / 2.0F;
    const float b_right = b.center.x + b.width / 2.0F;
    const float b_top = b.center.y - b.height / 2.0F;
    const float b_bottom = b.center.y + b.height / 2.0F;

    const float overlap_w = std::min(a_right, b_right) - std::max(a_left, b_left);
    const float overlap_h = std::min(a_bottom, b_bottom) - std::max(a_top, b_top);
    if (overlap_w <= 0.0F || overlap_h <= 0.0F) {
        return 0.0F;
    }
    return overlap_w * overlap_h;
}

inline float rectangle_iou(const DecodeResult& a, const DecodeResult& b)
{
    const float area_a = a.width * a.height;
    const float area_b = b.width * b.height;
    const float union_area = area_a + area_b - rectangle_overlap_area(a, b);
    if (union_area <= 0.0F) {
        return 0.0F;
    }
    return rectangle_overlap_area(a, b) / union_area;
}

inline float quad_area(const std::array<Point2F, 4>& corners)
{
    float twice_area = 0.0F;
    for (std::size_t i = 0; i < corners.size(); ++i) {
        const Point2F& p = corners[i];
        const Point2F& q = corners[(i + 1) % corners.size()];
        twice_area += p.x * q.y - q.x * p.y;
    }
    return std::abs(twice_area) / 2.0F;
}

inline bool has_valid_corners(const DecodeResult& r)
{
    if (!std::all_of(r.corners.begin(), r.corners.end(), is_finite_point)) {
        return false;
    }
    return quad_area(r.corners) > 1.0F;
}

inline float distance(const Point2F& a, const Point2F& b)
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

inline float quad_long_edge(const std::array<Point2F, 4>& corners)
{
    float longest = 0.0F;
    for (std::size_t i = 0; i < corners.size(); ++i) {
        longest = std::max(longest, distance(corners[i], corners[(i + 1) % corners.size()]));
    }
    return longest;
}

inline float quad_angle_deg(const std::array<Point2F, 4>& corners)
{
    float longest = 0.0F;
    float angle = 0.0F;
    for (std::size_t i = 0; i < corners.size(); ++i) {
        const float dx = corners[(i + 1) % corners.size()].x - corners[i].x;
        const float dy = corners[(i + 1) % corners.size()].y - corners[i].y;
        const float length = std::sqrt(dx * dx + dy * dy);
        if (length <= longest) {
            continue;
        }
        longest = length;
        angle = std::atan2(dy, dx) * 180.0F / 3.14159265358979323846F;
    }
    if (angle < 0.0F) {
        angle += 180.0F;
    }
    return angle;
}

inline float angle_delta_deg(float a, float b)
{
    float delta = std::abs(a - b);
    if (delta > 90.0F) {
        delta = 180.0F - delta;
    }
    return delta;
}

inline Point2F line_intersection(const Point2F& p1, const Point2F& p2,
                                 const Point2F& q1, const Point2F& q2)
{
    const float edge_x = q2.x - q1.x;
    const float edge_y = q2.y - q1.y;
    const float d1 = edge_x * (p1.y - q1.y) - edge_y * (p1.x - q1.x);
    const float d2 = edge_x * (p2.y - q1.y) - edge_y * (p2.x - q1.x);
    const float denominator = d1 - d2;
    if (std::abs(denominator) < 1.0e-6F) {
        return p2;
    }
    const float t = d1 / denominator;
    return {p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y)};
}

inline float polygon_overlap_area(const std::array<Point2F, 4>& subject_in,
                                  const std::array<Point2F, 4>& clip_in)
{
    std::array<Point2F, 4> clip = clip_in;
    if (((clip[1].x - clip[0].x) * (clip[2].y - clip[0].y) -
         (clip[1].y - clip[0].y) * (clip[2].x - clip[0].x)) < 0.0F) {
        std::reverse(clip.begin(), clip.end());
    }

    std::vector<Point2F> output(subject_in.begin(), subject_in.end());
    for (std::size_t edge = 0; edge < clip.size() && !output.empty(); ++edge) {
        const Point2F& a = clip[edge];
        const Point2F& b = clip[(edge + 1) % clip.size()];
        const auto inside = [&](const Point2F& p) {
            return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x) >= 0.0F;
        };

        const std::vector<Point2F> input = output;
        output.clear();
        for (std::size_t i = 0; i < input.size(); ++i) {
            const Point2F& current = input[i];
            const Point2F& previous = input[(i + input.size() - 1) % input.size()];
            const bool current_inside = inside(current);
            const bool previous_inside = inside(previous);

            if (current_inside != previous_inside) {
                output.push_back(line_intersection(previous, current, a, b));
            }
            if (current_inside) {
                output.push_back(current);
            }
        }
    }

    if (output.size() < 3) {
        return 0.0F;
    }

    float twice_area = 0.0F;
    for (std::size_t i = 0; i < output.size(); ++i) {
        const Point2F& p = output[i];
        const Point2F& q = output[(i + 1) % output.size()];
        twice_area += p.x * q.y - q.x * p.y;
    }
    return std::abs(twice_area) / 2.0F;
}

inline float polygon_iou(const DecodeResult& a, const DecodeResult& b)
{
    const float area_a = quad_area(a.corners);
    const float area_b = quad_area(b.corners);
    const float overlap = polygon_overlap_area(a.corners, b.corners);
    const float union_area = area_a + area_b - overlap;
    return union_area <= 0.0F ? 0.0F : overlap / union_area;
}

struct GeometryMetrics {
    float center_distance = 0.0F;
    float iou = 0.0F;
    float reference_long_edge = 0.0F;
    float angle_delta = 0.0F;
    bool corners_used = false;
};

inline GeometryMetrics geometry_metrics(const DecodeResult& a, const DecodeResult& b)
{
    const bool corners_used = has_valid_corners(a) && has_valid_corners(b);
    const float long_edge_a = corners_used
        ? quad_long_edge(a.corners)
        : std::max(a.width, a.height);
    const float long_edge_b = corners_used
        ? quad_long_edge(b.corners)
        : std::max(b.width, b.height);

    GeometryMetrics metrics;
    metrics.center_distance = distance(a.center, b.center);
    metrics.iou = corners_used ? polygon_iou(a, b) : rectangle_iou(a, b);
    metrics.reference_long_edge = std::min(long_edge_a, long_edge_b);
    metrics.angle_delta = corners_used
        ? angle_delta_deg(quad_angle_deg(a.corners), quad_angle_deg(b.corners))
        : 0.0F;
    metrics.corners_used = corners_used;
    return metrics;
}

inline float center_distance(const DecodeResult& a, const DecodeResult& b)
{
    const float dx = a.center.x - b.center.x;
    const float dy = a.center.y - b.center.y;
    return std::sqrt(dx * dx + dy * dy);
}

inline bool is_duplicate(const DecodeResult& a, const DecodeResult& b)
{
    if (a.format != b.format || a.text != b.text) {
        return false;
    }
    if (!has_valid_geometry(a) || !has_valid_geometry(b)) {
        return false;
    }

    const GeometryMetrics metrics = geometry_metrics(a, b);
    const bool is_1d = a.format == BarcodeKind::Code128 ||
                       a.format == BarcodeKind::Code39 ||
                       a.format == BarcodeKind::EAN13 ||
                       a.format == BarcodeKind::ITF;

    if (metrics.iou >= 0.80F) {
        return true;
    }

    if (is_1d) {
        // A perspective/restoration path can make axis-aligned bounding boxes
        // almost disjoint for the same slanted symbol.  Corners plus a size
        // normalized center constraint are more stable for 1D geometry.
        const float center_threshold = std::max(8.0F, 0.40F * metrics.reference_long_edge);
        return metrics.angle_delta <= 35.0F &&
               metrics.center_distance <= center_threshold &&
               metrics.iou >= 0.25F;
    }

    const float center_threshold = std::max(4.0F, 0.25F * metrics.reference_long_edge);
    return metrics.center_distance <= center_threshold && metrics.iou >= 0.35F;
}

class DecodeResultFusion {
public:
    void reset()
    {
        fused_.clear();
        stats_ = FusionStats{};
    }

    FusionOutcome add(const DecodeResult& raw)
    {
        stats_.raw_results++;
        stats_.raw_by_format[raw.format]++;

        if (!raw.valid || raw.text.empty() || raw.format == BarcodeKind::Unknown) {
            stats_.invalid_results++;
            return {raw, false};
        }

        stats_.success_by_source[raw.source]++;

        bool same_symbol_candidate = false;
        GeometryMetrics last_metrics;
        for (auto& existing : fused_) {
            const GeometryMetrics metrics = geometry_metrics(existing, raw);
            if (is_duplicate(existing, raw)) {
                stats_.duplicates_removed++;

                if (raw.confidence > existing.confidence) {
                    existing = raw;
                }
                return {existing, true, true, metrics.center_distance,
                        metrics.iou, metrics.reference_long_edge};
            }
            if (existing.format == raw.format && existing.text == raw.text) {
                same_symbol_candidate = true;
                last_metrics = metrics;
            }
        }

        fused_.push_back(raw);
        stats_.distinct_results++;
        stats_.distinct_by_format[raw.format]++;
        return {raw, false, same_symbol_candidate, last_metrics.center_distance,
                last_metrics.iou, last_metrics.reference_long_edge};
    }

    const std::vector<DecodeResult>& fused() const
    {
        return fused_;
    }

    const FusionStats& stats() const
    {
        return stats_;
    }

private:
    std::vector<DecodeResult> fused_;
    FusionStats stats_;
};

} // namespace barcode_fusion
