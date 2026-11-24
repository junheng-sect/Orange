#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <sensor_msgs/Image.h>

// One independent Matroska stream, referenced to the bag's OPEN event.
class GrayVideo {
public:
    GrayVideo(const std::string& path, int width, int height, int fps);
    ~GrayVideo();
    void write(const sensor_msgs::Image& image, int64_t pts_ms);
    void finish();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
