#include "gray_video.hpp"
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
}

namespace {
void check(int rc, const char* op) {
    if (rc < 0) {
        char buffer[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(rc, buffer, sizeof(buffer));
        throw std::runtime_error(std::string(op) + ": " + buffer);
    }
}
}
struct GrayVideo::Impl {
    AVFormatContext* mux = nullptr;
    AVCodecContext* codec = nullptr;
    AVStream* stream = nullptr;
    AVFrame* frame = nullptr;
    AVPacket* packet = nullptr;
    int writeback_fd = -1;
    int64_t scheduled_bytes = 0;
    ~Impl() {
        if (writeback_fd >= 0) ::close(writeback_fd);
        av_packet_free(&packet);
        av_frame_free(&frame);
        avcodec_free_context(&codec);
        if (mux) {
            if (mux->pb) avio_closep(&mux->pb);
            avformat_free_context(mux);
        }
    }
};
GrayVideo::GrayVideo(const std::string& path, int w, int h, int fps) : impl_(new Impl) {
    auto& i = *impl_;
    check(avformat_alloc_output_context2(&i.mux, nullptr, "matroska", path.c_str()), "allocate mux");
    auto* enc = avcodec_find_encoder(AV_CODEC_ID_FFVHUFF);
    if (!enc) throw std::runtime_error("FFVHUFF encoder unavailable");
    i.stream = avformat_new_stream(i.mux, nullptr);
    i.codec = avcodec_alloc_context3(enc);
    i.frame = av_frame_alloc();
    i.packet = av_packet_alloc();
    if (!i.stream || !i.codec || !i.frame || !i.packet) throw std::bad_alloc();
    i.codec->height=h; i.codec->width=w; i.codec->pix_fmt=AV_PIX_FMT_GRAY8;
    i.codec->time_base={1,1000}; i.codec->framerate={fps,1};
    i.codec->thread_count=1; // Each camera owns a worker; packets are emitted in source-frame order.
    if (i.mux->oformat->flags & AVFMT_GLOBALHEADER) i.codec->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    check(avcodec_open2(i.codec,enc,nullptr),"open FFVHUFF");
    check(avcodec_parameters_from_context(i.stream->codecpar,i.codec),"copy codec parameters");
    i.stream->time_base=i.codec->time_base; i.stream->avg_frame_rate={fps,1};
    i.frame->format=i.codec->pix_fmt; i.frame->width=w; i.frame->height=h;
    check(av_frame_get_buffer(i.frame,32),"allocate gray frame");
    check(avio_open(&i.mux->pb,path.c_str(),AVIO_FLAG_WRITE),"open video");
    check(avformat_write_header(i.mux,nullptr),"write Matroska header");
    i.writeback_fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);

}
GrayVideo::~GrayVideo() = default;
void GrayVideo::write(const sensor_msgs::Image& image,int64_t pts_ms) {
    auto& i=*impl_;
    check(av_frame_make_writable(i.frame),"make frame writable");
    for (unsigned y=0;y<image.height;++y)
        std::memcpy(i.frame->data[0]+y*i.frame->linesize[0],image.data.data()+size_t(y)*image.step,image.width);
    i.frame->pts=pts_ms;
    check(avcodec_send_frame(i.codec,i.frame),"encode");
    check(avcodec_receive_packet(i.codec,i.packet),"receive frame");
    if (i.packet->pts!=pts_ms) throw std::runtime_error("unexpected encoder frame order");
    i.packet->stream_index=i.stream->index;
    av_packet_rescale_ts(i.packet,i.codec->time_base,i.stream->time_base);
    check(av_interleaved_write_frame(i.mux,i.packet),"write frame");
    av_packet_unref(i.packet);
    check(i.mux->pb->error,"video I/O");
    // Schedule completed extents gradually. A failed advisory request leaves normal kernel writeback in use.
    const int64_t chunk = 4 * 1024 * 1024;
    const int64_t through = i.mux->pb->pos / chunk * chunk;
    if (i.writeback_fd >= 0 && through > i.scheduled_bytes) {
        if (::sync_file_range(i.writeback_fd, i.scheduled_bytes, through - i.scheduled_bytes,
                              SYNC_FILE_RANGE_WRITE) < 0) {
            ::close(i.writeback_fd);
            i.writeback_fd = -1;
        }
        i.scheduled_bytes = through;
    }
}
void GrayVideo::finish() {
    auto& i=*impl_;
    check(avcodec_send_frame(i.codec,nullptr),"finish encoder");
    if (avcodec_receive_packet(i.codec,i.packet)!=AVERROR_EOF) throw std::runtime_error("unexpected delayed packet");
    check(av_write_trailer(i.mux),"finish Matroska");
    avio_flush(i.mux->pb);
    check(i.mux->pb->error,"flush video");
    check(avio_closep(&i.mux->pb),"close video");
}
