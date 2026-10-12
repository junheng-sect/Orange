#pragma once
// Decode attempt recorder for offline analysis: one CSV row per ~decode call, plus (by mode) a
// directory with the decoded frame, retry frames, pose, intrinsics and the strategy log, so the
// attempt can be replayed with decode_image. Files are written by a background thread; submit()
// never blocks the decode service.
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "barcode_reader_camera/multi_decoder.hpp"

namespace barcode_reader_camera {

enum class SaveFrames { Off, Failed, All };

// "off" / "failed" / "all"; throws std::invalid_argument otherwise.
SaveFrames parseSaveFrames(const std::string& s);

// One retry frame scanned by decodeBarcodes (numbered like DecodedCode::frame).
struct RetryFrameRecord {
  int index = 0;  // 1 = first retry frame
  double stamp = 0.0;
  TagPose pose;
  cv::Mat gray;  // empty when this frame's image is not kept (only some retry frames are saved)
};

struct DecodeRecord {
  double wall_time = 0.0;  // seconds since epoch, when ~decode was called
  double stamp = 0.0;      // input frame's header stamp; 0 when no frame was picked
  std::string frame_id;
  cv::Mat frame;                          // CV_8UC1 input frame given to decodeBarcodes; may be empty
  std::vector<RetryFrameRecord> retries;  // every retry frame decodeBarcodes scanned, in order
  bool have_pose = false;
  TagPose pose;
  CameraModel camera;  // may be invalid
  bool success = false;
  bool partial = false;
  std::string error;     // ~decode error string
  double elapsed_s = 0.0;  // whole ~decode call
  DecodeOutput out;      // empty when decodeBarcodes did not run
  std::vector<std::string> log;  // strategy log lines
  std::string layout_file;
  std::string config_file;
};

class FrameRecorder {
 public:
  // dir: created if missing. log_attempts: append a row to <dir>/attempts.csv for every record.
  FrameRecorder(const std::string& dir, SaveFrames mode, bool log_attempts, size_t max_queue = 4);
  ~FrameRecorder();  // writes everything still queued
  FrameRecorder(const FrameRecorder&) = delete;
  FrameRecorder& operator=(const FrameRecorder&) = delete;

  bool enabled() const { return mode_ != SaveFrames::Off || log_attempts_; }
  bool savesFrames() const { return mode_ != SaveFrames::Off; }
  // False when the queue is full (record dropped).
  bool submit(DecodeRecord record);
  // Last write error, cleared on read ("" if none).
  std::string takeError();
  const std::string& dir() const { return dir_; }

 private:
  void run();
  void write(const DecodeRecord& r);
  void rotateStaleCsv();

  static constexpr double kChainErrorSaveGapS = 2.0;

  std::string dir_;
  SaveFrames mode_;
  bool log_attempts_;
  size_t max_queue_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<DecodeRecord> queue_;
  bool stop_ = false;
  std::string error_;
  double last_chain_error_save_ = -1e9;  // worker thread only
  std::thread thread_;
};

}  // namespace barcode_reader_camera
