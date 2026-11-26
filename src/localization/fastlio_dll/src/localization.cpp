#include "registration.hpp"

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <boost/property_tree/json_parser.hpp>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <openssl/evp.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>

namespace fastlio_dll {
namespace {

constexpr double radians(double degrees) { return degrees * 3.14159265358979323846 / 180.0; }

template <class Point>
std::optional<Pose> toPose(const Point& p, const geometry_msgs::Quaternion& q) {
  const Eigen::Vector3d translation(p.x, p.y, p.z);
  const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
  if (!translation.allFinite() || !rotation.coeffs().allFinite() ||
      std::abs(rotation.norm() - 1.0) > 0.01) return std::nullopt;
  Pose result = Pose::Identity();
  result.translation() = translation;
  result.linear() = rotation.normalized().toRotationMatrix();
  return result;
}

template <class Point>
void fillPose(const Pose& pose, Point& p, geometry_msgs::Quaternion& q) {
  p.x = pose.translation().x();
  p.y = pose.translation().y();
  p.z = pose.translation().z();
  const Eigen::Quaterniond rotation(pose.linear());
  q.x = rotation.x();
  q.y = rotation.y();
  q.z = rotation.z();
  q.w = rotation.w();
}

RegistrationConfig readRegistrationConfig(const ros::NodeHandle& nh) {
  RegistrationConfig c;
  const auto read = [&nh](const std::string& name, auto& value) {
    nh.param("registration/" + name, value, value);
  };
  read("resolution", c.resolution);
  read("grid_memory_mib", c.grid_memory_mib);
  read("distance_cap", c.distance_cap);
  read("voxel", c.voxel);
  read("range_min", c.range_min);
  read("range_max", c.range_max);
  read("max_points", c.max_points);
  read("min_points", c.min_points);
  read("min_valid", c.min_valid);
  read("min_coverage", c.min_coverage);
  read("inlier_distance", c.inlier_distance);
  read("min_inliers", c.min_inliers);
  read("max_median", c.max_median);
  read("max_rmse", c.max_rmse);
  read("min_eigenvalue", c.min_eigenvalue);
  read("min_eigenvalue_ratio", c.min_eigenvalue_ratio);
  read("yaw_length", c.yaw_length);
  read("solve_ms", c.solve_ms);
  read("max_iterations", c.max_iterations);
  read("cauchy_scale", c.cauchy_scale);
  c.validate();
  return c;
}

std::string getMap(const ros::NodeHandle& nh) {
  std::string map;
  if (!nh.getParam("map_pcd", map) || map.empty()) {
    throw std::invalid_argument("map_pcd is required");
  }
  return map;
}

bool seedAligned(const Pose& pose) {
  return std::acos(std::clamp(pose.linear()(2, 2), -1.0, 1.0)) <= radians(3);
}

std::optional<Pose> getSeed(const ros::NodeHandle& nh) {
  std::vector<double> values;
  if (!nh.hasParam("seed_map_odom")) return std::nullopt;
  if (!nh.getParam("seed_map_odom", values) || values.size() != 7) {
    throw std::invalid_argument("seed_map_odom must be [x, y, z, qx, qy, qz, qw]");
  }
  geometry_msgs::Point p;
  geometry_msgs::Quaternion q;
  p.x = values[0]; p.y = values[1]; p.z = values[2];
  q.x = values[3]; q.y = values[4]; q.z = values[5]; q.w = values[6];
  const auto pose = toPose(p, q);
  if (!pose || !seedAligned(*pose)) {
    throw std::invalid_argument("seed invalid or map/odom gravity axes disagree by more than 3 degrees");
  }
  return yawCorrection(*pose);
}

struct Job {
  ros::Time stamp;
  std::uint64_t revision;
  Pose odom_imu;
  Pose prior;
  Points scan;
  bool recovering;
};

struct Completed {
  ros::Time stamp;
  std::uint64_t revision;
  Pose odom_imu;
  bool recovering;
  RegistrationResult result;
};

struct Confirmation {
  Pose first_correction;
  Pose last_correction;
  ros::Time start;
  int scans;
};

enum class State { Initializing, Tracking, Degraded, Recovering, WaitingForSeed };

const char* stateName(State state) {
  switch (state) {
    case State::Initializing: return "INITIALIZING";
    case State::Tracking: return "TRACKING";
    case State::Degraded: return "DEGRADED";
    case State::Recovering: return "RELOCALIZING";
    case State::WaitingForSeed: return "WAITING_FOR_SEED";
  }
  return "UNKNOWN";
}

using Tree = boost::property_tree::ptree;

std::string sha256(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot open " + path);
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("cannot initialize SHA-256");
  }
  std::array<char, 65536> buffer;
  while (input.read(buffer.data(), buffer.size()) || input.gcount() > 0) {
    if (EVP_DigestUpdate(context.get(), buffer.data(), input.gcount()) != 1) {
      throw std::runtime_error("cannot hash " + path);
    }
  }
  std::array<unsigned char, 32> digest;
  unsigned int size = 0;
  if (!input.eof() || input.bad() || EVP_DigestFinal_ex(context.get(), digest.data(), &size) != 1 ||
      size != digest.size()) throw std::runtime_error("cannot finish hashing " + path);
  std::ostringstream hex;
  hex << std::hex << std::setfill('0');
  for (const auto byte : digest) hex << std::setw(2) << static_cast<unsigned int>(byte);
  return hex.str();
}

std::string verifiedMap(const ros::NodeHandle& nh) {
  const auto map = getMap(nh);
  std::string legacy;
  nh.param("legacy_map_dir", legacy, std::string());
  if (legacy.empty()) return map;
  if (legacy.front() != '/' || map.front() != '/' || nh.hasParam("seed_map_odom") ||
      map.substr(map.find_last_of('/') + 1) != "map_world.pcd") {
    throw std::invalid_argument("observer requires absolute map paths and a legacy session seed");
  }
  if (map == legacy + "/map_world.pcd") return map;
  const auto crop = map.substr(0, map.find_last_of('/'));
  Tree manifest;
  boost::property_tree::read_json(crop + "/manifest.json", manifest);
  if (manifest.get<std::string>("map_frame") != "drone_map" ||
      manifest.get<std::string>("source.map_dir") != legacy) {
    throw std::invalid_argument("observer crop does not share the original drone_map frame");
  }
  const auto expect = [&manifest](const std::string& file, const Tree::path_type& key) {
    if (sha256(file) != manifest.get<std::string>(key, "")) {
      throw std::invalid_argument("observer map SHA-256 mismatch: " + file);
    }
  };
  expect(legacy + "/map_world.pcd", Tree::path_type("source.map_sha256", '.'));
  expect(legacy + "/meta.json", Tree::path_type("source.meta_sha256", '.'));
  expect(map, Tree::path_type("sha256/map_world.pcd", '/'));
  expect(crop + "/meta.json", Tree::path_type("sha256/meta.json", '/'));
  ROS_INFO_STREAM("DLL observer verified original map and crop: " << legacy << " -> " << crop);
  return map;
}

Tree readJson(const std_msgs::String& message) {
  std::istringstream input(message.data);
  Tree tree;
  boost::property_tree::read_json(input, tree);
  return tree;
}

ros::Time jsonStamp(const Tree& tree) {
  const double seconds = tree.get<double>("stamp");
  if (!std::isfinite(seconds) || seconds <= 0.0 || seconds >= 4294967296.0) {
    throw std::invalid_argument("invalid relocalization timestamp");
  }
  return ros::Time().fromSec(seconds);
}

template <std::size_t N>
std::array<double, N> jsonVector(const Tree& tree, const std::string& path) {
  std::array<double, N> value;
  const auto& children = tree.get_child(path);
  if (children.size() != N) throw std::invalid_argument("wrong vector size");
  std::size_t index = 0;
  for (const auto& item : children) {
    if (!item.first.empty()) throw std::invalid_argument("expected JSON array");
    value[index++] = item.second.get_value<double>();
  }
  return value;
}

struct LegacyPose {
  Pose pose;
  ros::Time stamp;
  std::string publisher;
};

struct LegacyStatus {
  ros::Time stamp;
  ros::WallTime received;
  std::string publisher;
  bool valid;
};

class Localizer {
 public:
  Localizer() : private_("~"), config_(readRegistrationConfig(private_)),
                seed_(getSeed(private_)), field_(verifiedMap(private_), config_) {
    private_.param("match_hz", match_hz_, 2.0);
    if (!std::isfinite(match_hz_) || match_hz_ <= 0.0 || match_hz_ > 10.0) {
      throw std::invalid_argument("match_hz must be in (0, 10]");
    }
    std::string anchor;
    private_.param("seed_not_before_ns", anchor, std::string("0"));
    if (anchor.empty() || anchor.find_first_not_of("0123456789") != std::string::npos) {
      throw std::invalid_argument("seed_not_before_ns must be an unsigned nanosecond stamp");
    }
    seed_not_before_.fromNSec(std::stoull(anchor));
    private_.param("legacy_map_dir", legacy_map_dir_, std::string());
    if (observing()) {
      detail_ = "waiting_for_legacy_seed";
      legacy_result_sub_ = nh_.subscribe("/drone_reloc/result", 4, &Localizer::onLegacyResult, this);
      legacy_status_sub_ = nh_.subscribe("/drone_reloc/status", 4, &Localizer::onLegacyStatus, this);
      legacy_tf_sub_ = nh_.subscribe("/tf", 20, &Localizer::onLegacyTf, this);
    } else {
      broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
      seed_sub_ = nh_.subscribe("/dll_localization/seed", 1, &Localizer::onSeed, this);
    }
    if (seed_) {
      state_ = State::Initializing;
      detail_ = "validating_seed";
    }

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/dll_localization/pose", 10);
    correction_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/dll_localization/correction", 10);
    status_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/dll_localization/status", 10, true);
    static_sub_ = nh_.subscribe("/tf_static", 10, &Localizer::onStatic, this);
    local_sub_ = nh_.subscribe("/fastlio/odom_world", 20, &Localizer::onLocal, this);
    raw_sub_.subscribe(nh_, "/Odometry", 12);
    cloud_sub_.subscribe(nh_, "/cloud_registered_body", 12);
    sync_ = std::make_unique<Sync>(raw_sub_, cloud_sub_, 12);
    sync_->registerCallback(boost::bind(&Localizer::onScan, this, boost::placeholders::_1,
                                        boost::placeholders::_2));
    status_timer_ = nh_.createWallTimer(ros::WallDuration(1), &Localizer::onStatusTimer, this);
    worker_ = std::thread(&Localizer::run, this);
    ROS_INFO("DLL ready: distance field %.1f MiB, built in %.3f s, %.1f Hz",
             field_.bytes() / 1048576.0, field_.buildMilliseconds() / 1000.0, match_hz_);
  }

  ~Localizer() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
      pending_.reset();
    }
    available_.notify_one();
    worker_.join();
  }

 private:
  using Sync = message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>;

  bool observing() const { return !legacy_map_dir_.empty(); }

  void resetSeed(const std::string& reason) {
    seed_.reset();
    correction_.reset();
    last_accepted_ = ros::Time();
    last_applied_ = ros::Time();
    active_legacy_seed_ = ros::Time();
    detail_ = reason;
    transition(State::WaitingForSeed);
    publishStatus();
  }

  void startSeed(const Pose& pose) {
    seed_ = yawCorrection(pose);
    correction_.reset();
    last_accepted_ = ros::Time();
    last_applied_ = ros::Time();
    failed_rounds_ = 0;
    rejects_ = 0;
    detail_ = "validating_seed";
    transition(State::Initializing);
    publishStatus();
  }

  void rejectLegacy(const std::string& reason) {
    legacy_result_.reset();
    if (seed_ || detail_ != reason) resetSeed(reason);
  }

  void onLegacyResult(const ros::MessageEvent<std_msgs::String const>& event) {
    try {
      const auto data = readJson(*event.getMessage());
      if (!data.get<bool>("ok", false) || !data.get<bool>("confirmed", false)) {
        rejectLegacy("legacy_result_unconfirmed");
        return;
      }
      if (data.get<std::string>("map", "") != legacy_map_dir_) {
        rejectLegacy("legacy_map_mismatch");
        return;
      }
      const auto t = jsonVector<3>(data, "transform.translation");
      const auto q = jsonVector<4>(data, "transform.q_xyzw");
      geometry_msgs::Point point;
      point.x = t[0]; point.y = t[1]; point.z = t[2];
      geometry_msgs::Quaternion rotation;
      rotation.x = q[0]; rotation.y = q[1]; rotation.z = q[2]; rotation.w = q[3];
      const auto pose = toPose(point, rotation);
      if (!pose || !seedAligned(*pose)) {
        rejectLegacy("legacy_seed_invalid");
        return;
      }
      const auto stamp = jsonStamp(data);
      if (stamp <= consumed_legacy_seed_) return;
      if (!alignment_epoch_.isZero() && stamp < alignment_epoch_) {
        rejectLegacy("legacy_result_before_epoch");
        return;
      }
      if (seed_) resetSeed("legacy_seed_changed");
      legacy_result_ = LegacyPose{*pose, stamp, event.getPublisherName()};
    } catch (const std::exception&) {
      rejectLegacy("legacy_result_invalid");
    }
  }

  void onLegacyStatus(const ros::MessageEvent<std_msgs::String const>& event) {
    try {
      const auto data = readJson(*event.getMessage());
      const bool valid = data.get<bool>("valid", false) &&
                         data.get<std::string>("state", "") == "LOCALIZED";
      legacy_status_ = LegacyStatus{jsonStamp(data), ros::WallTime::now(),
                                    event.getPublisherName(), valid};
      if (!valid && seed_) rejectLegacy("legacy_localization_invalid");
    } catch (const std::exception&) {
      legacy_status_.reset();
      if (seed_) rejectLegacy("legacy_status_invalid");
    }
  }

  void onLegacyTf(const ros::MessageEvent<tf2_msgs::TFMessage const>& event) {
    for (const auto& tf : event.getMessage()->transforms) {
      if (tf.header.frame_id != "drone_map" || tf.child_frame_id != "drone_odom") continue;
      const auto pose = toPose(tf.transform.translation, tf.transform.rotation);
      if (!pose || tf.header.stamp.isZero()) {
        legacy_tf_.reset();
        if (seed_) rejectLegacy("legacy_tf_invalid");
        return;
      }
      legacy_tf_ = LegacyPose{*pose, tf.header.stamp, event.getPublisherName()};
      legacy_tf_wall_ = ros::WallTime::now();
    }
  }

  void refreshLegacy() {
    if (!legacy_result_ || !odom_from_camera_ || alignment_epoch_.isZero()) return;
    const auto& result = *legacy_result_;
    if (result.stamp < alignment_epoch_) {
      rejectLegacy("legacy_result_before_epoch");
      return;
    }
    const auto now = ros::Time::now();
    const auto wall = ros::WallTime::now();
    if (last_local_.isZero() || (wall - last_local_wall_).toSec() > 0.5 ||
        !legacy_status_ || !legacy_status_->valid || legacy_status_->publisher != result.publisher ||
        (legacy_status_->stamp - result.stamp).toSec() < -0.01 ||
        (wall - legacy_status_->received).toSec() > 2.5 ||
        std::abs((now - legacy_status_->stamp).toSec()) > 2.5) {
      if (seed_) rejectLegacy("legacy_or_local_input_stale");
      return;
    }
    if (!legacy_tf_ || legacy_tf_->publisher != result.publisher ||
        legacy_tf_->stamp < alignment_epoch_ ||
        (wall - legacy_tf_wall_).toSec() > 0.5 ||
        std::abs((now - legacy_tf_->stamp).toSec()) > 0.5) {
      if (seed_) rejectLegacy("legacy_tf_stale");
      return;
    }
    if (!legacy_tf_->pose.matrix().isApprox(result.pose.matrix(), 1e-4)) {
      if (seed_ || detail_ != "legacy_tf_mismatch") resetSeed("legacy_tf_mismatch");
      return;
    }
    if (!seed_ && result.stamp > consumed_legacy_seed_) {
      active_legacy_seed_ = consumed_legacy_seed_ = result.stamp;
      startSeed(result.pose);
    }
  }

  void onSeed(const geometry_msgs::TransformStamped::ConstPtr& message) {
    const auto pose = toPose(message->transform.translation, message->transform.rotation);
    if (!pose || !seedAligned(*pose) || message->header.frame_id != "drone_map" ||
        message->child_frame_id != "drone_odom") {
      ROS_WARN("DLL seed must be a finite, gravity-aligned drone_map -> drone_odom transform");
      return;
    }
    startSeed(*pose);
  }

  void onStatic(const tf2_msgs::TFMessage::ConstPtr& message) {
    for (const auto& transform : message->transforms) {
      if (transform.header.frame_id != "drone_odom" || transform.child_frame_id != "camera_init") continue;
      const auto aligned = toPose(transform.transform.translation, transform.transform.rotation);
      if (observing()) {
        if (!aligned || transform.header.stamp.isZero()) {
          resetAlignment("local_alignment_invalid");
          return;
        }
        if (odom_from_camera_ && (alignment_epoch_ != transform.header.stamp ||
            !odom_from_camera_->isApprox(*aligned, 1e-9))) {
          resetAlignment("local_alignment_reset");
        }
        alignment_epoch_ = transform.header.stamp;
        odom_from_camera_ = aligned;
        return;
      }
      if (!aligned || (odom_from_camera_ && !odom_from_camera_->isApprox(*aligned, 1e-9))) {
        fail("gravity alignment changed or invalid; restart LIO and DLL together");
        return;
      }
      odom_from_camera_ = aligned;
    }
  }

  void resetAlignment(const std::string& reason) {
    odom_from_camera_.reset();
    legacy_result_.reset();
    legacy_status_.reset();
    legacy_tf_.reset();
    alignment_epoch_ = consumed_legacy_seed_ = ros::Time();
    last_local_ = last_pair_ = last_job_ = last_candidate_ = ros::Time();
    resetSeed(reason);
  }

  void onScan(const nav_msgs::Odometry::ConstPtr& odom,
              const sensor_msgs::PointCloud2::ConstPtr& cloud) {
    const auto stamp = cloud->header.stamp;
    if (stamp.isZero() || odom->header.frame_id != "camera_init" ||
        odom->child_frame_id != "body" || cloud->header.frame_id != "body") return;
    if (!last_pair_.isZero() && stamp < last_pair_) {
      if (observing()) resetAlignment("local_scan_time_reset");
      else fail("raw scan time went backwards; restart the replay");
      return;
    }
    last_pair_ = stamp;
    if (observing() && stamp < alignment_epoch_) return;
    if (!odom_from_camera_ || stamp < seed_not_before_ || state_ == State::WaitingForSeed ||
        (!last_job_.isZero() && (stamp - last_job_).toSec() < 1.0 / match_hz_ - 1e-6)) return;

    const auto raw_pose = toPose(odom->pose.pose.position, odom->pose.pose.orientation);
    if (!raw_pose) return;
    Points scan;
    try {
      if (cloud->is_bigendian || cloud->point_step < 12 ||
          static_cast<std::size_t>(cloud->row_step) !=
              static_cast<std::size_t>(cloud->width) * cloud->point_step ||
          cloud->data.empty() || cloud->data.size() !=
              static_cast<std::size_t>(cloud->row_step) * cloud->height) {
        throw std::invalid_argument("unsupported point cloud layout");
      }
      for (const std::string axis : {"x", "y", "z"}) {
        const auto field = std::find_if(cloud->fields.begin(), cloud->fields.end(),
            [&axis](const auto& f) { return f.name == axis; });
        if (field == cloud->fields.end() || field->datatype != sensor_msgs::PointField::FLOAT32 ||
            field->count != 1 || field->offset > cloud->point_step - sizeof(float)) {
          throw std::invalid_argument("point cloud must contain scalar FLOAT32 xyz");
        }
      }
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x"), y(*cloud, "y"), z(*cloud, "z");
      scan.reserve(static_cast<std::size_t>(cloud->width) * cloud->height);
      for (; x != x.end(); ++x, ++y, ++z) scan.emplace_back(*x, *y, *z);
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(1.0, "DLL cannot read body cloud: %s", error.what());
      return;
    }

    const bool recovering = state_ == State::Recovering;
    const Pose& prior = confirmation_ ? confirmation_->last_correction : (correction_ ? *correction_ : *seed_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_ = Job{stamp, revision_, *odom_from_camera_ * *raw_pose, prior, std::move(scan), recovering};
    }
    last_job_ = stamp;
    available_.notify_one();
  }

  void run() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
      available_.wait(lock, [this] { return stop_ || pending_.has_value(); });
      if (stop_) return;
      Job job = std::move(*pending_);
      pending_.reset();
      lock.unlock();
      const SearchRange range = job.recovering ? SearchRange{1.0, 0.5, radians(15)}
                                                : SearchRange{0.50, 0.50, radians(5)};
      RegistrationResult result;
      try {
        result = registerScan(field_, config_, job.scan, job.odom_imu, job.prior, range, job.recovering);
      } catch (const std::exception& error) {
        ROS_ERROR("DLL registration failed: %s", error.what());
        result.reason = "registration_error";
      }
      lock.lock();
      completed_ = Completed{job.stamp, job.revision, job.odom_imu, job.recovering, std::move(result)};
    }
  }

  void transition(State state) {
    state_ = state;
    confirmation_.reset();
    ++revision_;
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.reset();
  }

  void fail(const std::string& reason) {
    ROS_ERROR_STREAM("DLL: " << reason);
    ros::shutdown();
  }

  void onLocal(const nav_msgs::Odometry::ConstPtr& message) {
    const auto stamp = message->header.stamp;
    if (stamp.isZero() || message->header.frame_id != "drone_odom" ||
        message->child_frame_id != "drone_base") return;
    if (!last_local_.isZero() && stamp <= last_local_) {
      if (stamp < last_local_) {
        if (observing()) resetAlignment("local_odometry_time_reset");
        else fail("local odometry went backwards; restart the replay");
      }
      return;
    }
    const auto pose = toPose(message->pose.pose.position, message->pose.pose.orientation);
    if (!pose) return;
    last_local_ = stamp;
    last_local_wall_ = ros::WallTime::now();
    if (observing()) {
      if (stamp < alignment_epoch_) return;
      refreshLegacy();
    }

    std::optional<Completed> result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (completed_ && completed_->stamp < stamp) {
        result = std::move(completed_);
        completed_.reset();
      }
    }
    if (result && result->revision == revision_ && result->stamp > last_candidate_) submit(*result);
    if (correction_ && (state_ == State::Tracking || state_ == State::Degraded)) {
      const double age = (stamp - last_accepted_).toSec();
      if (age >= 3.0 || rejects_ >= 5) {
        detail_ = "correction_stale";
        failed_rounds_ = 0;
        transition(State::Recovering);
        publishStatus();
      } else if (age >= 1.5 && state_ == State::Tracking) {
        state_ = State::Degraded;
        detail_ = "correction_aging";
        publishStatus();
      }
    }
    if (!correction_ || (observing() && state_ == State::WaitingForSeed)) return;
    geometry_msgs::TransformStamped edge;
    edge.header.stamp = stamp;
    edge.header.frame_id = "drone_map";
    edge.child_frame_id = "drone_odom";
    fillPose(*correction_, edge.transform.translation, edge.transform.rotation);
    if (broadcaster_) broadcaster_->sendTransform(edge);
    correction_pub_.publish(edge);

    geometry_msgs::PoseStamped global;
    global.header = edge.header;
    fillPose(*correction_ * *pose, global.pose.position, global.pose.orientation);
    pose_pub_.publish(global);
  }

  void submit(Completed& candidate) {
    last_candidate_ = candidate.stamp;
    last_candidate_recovering_ = candidate.recovering;
    last_result_ = candidate.result;
    if ((last_local_ - candidate.stamp).toSec() > 1.0) last_result_.reason = "late_candidate";
    detail_ = last_result_.reason;
    if (!last_result_.accepted()) {
      ++rejected_;
      ++rejects_;
      confirmation_.reset();
      if (state_ == State::Recovering && ++failed_rounds_ >= 3) transition(State::WaitingForSeed);
    } else if (state_ == State::Tracking || state_ == State::Degraded) {
      accept(candidate);
    } else {
      const auto& p = candidate.odom_imu.translation();
      if (confirmation_) {
        const auto& first = confirmation_->first_correction;
        const auto& current = candidate.result.correction;
        if ((first * p - current * p).norm() > 0.10 ||
            std::abs(yaw(current.linear() * first.linear().transpose())) > radians(2.0)) {
          confirmation_.reset();
          if (state_ == State::Recovering && ++failed_rounds_ >= 3) transition(State::WaitingForSeed);
        }
      }
      if (state_ != State::WaitingForSeed) {
        if (!confirmation_) {
          confirmation_ = Confirmation{candidate.result.correction, candidate.result.correction,
                                       candidate.stamp, 0};
        }
        confirmation_->last_correction = candidate.result.correction;
        ++confirmation_->scans;
        detail_ = "confirming";
        if (confirmation_->scans >= 3 && (candidate.stamp - confirmation_->start).toSec() >= 0.5) {
          transition(State::Tracking);
          accept(candidate);
        }
      } else {
        detail_ = "inconsistent_recovery";
      }
    }
    publishStatus();
  }

  void accept(const Completed& candidate) {
    correction_ = candidate.result.correction;
    last_accepted_ = candidate.stamp;
    last_applied_ = last_local_;
    ++accepted_;
    rejects_ = 0;
    state_ = State::Tracking;
    detail_ = "accepted";
  }

  void onStatusTimer(const ros::WallTimerEvent&) {
    if (observing()) refreshLegacy();
    publishStatus();
  }

  void publishStatus() {
    const bool local_fresh = !last_local_.isZero() &&
        (ros::WallTime::now() - last_local_wall_).toSec() <= 0.5;
    diagnostic_msgs::DiagnosticArray output;
    output.header.stamp = last_local_;
    auto& status = output.status.emplace_back();
    status.name = "fastlio_dll";
    status.hardware_id = "MID-360S";
    status.level = !local_fresh ? diagnostic_msgs::DiagnosticStatus::STALE :
        state_ == State::Tracking ? diagnostic_msgs::DiagnosticStatus::OK :
                                   diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = stateName(state_);
    if (observing() && (status.message != reported_state_ || status.level != reported_level_)) {
      ROS_INFO_STREAM("DLL " << status.message << ": " << detail_ << (local_fresh ? "" : " (local input stale)"));
      reported_state_ = status.message;
      reported_level_ = status.level;
    }
    const auto append = [&status](const std::string& key, const std::string& value) {
      auto& pair = status.values.emplace_back();
      pair.key = key;
      pair.value = value;
    };
    const auto number = [&append](const std::string& key, auto value) {
      append(key, std::to_string(value));
    };
    append("reason", detail_);
    append("local_input", local_fresh ? "fresh" : "stale");
    if (observing()) {
      number("legacy_seed_stamp_ns", active_legacy_seed_.toNSec());
      number("local_epoch_stamp_ns", alignment_epoch_.toNSec());
    }
    number("candidate_stamp_ns", last_candidate_.toNSec());
    number("accepted_stamp_ns", last_accepted_.toNSec());
    number("applied_stamp_ns", last_applied_.toNSec());
    number("accepted_count", accepted_);
    number("rejected_count", rejected_);
    number("correction_age_s", last_accepted_.isZero() ? -1.0 : (last_local_ - last_accepted_).toSec());
    number("points", last_result_.points);
    number("valid", last_result_.valid);
    number("coverage", last_result_.coverage);
    number("inliers", last_result_.inliers);
    number("distance_median_m", last_result_.median);
    number("inlier_rmse_m", last_result_.rmse);
    number("solve_ms", last_result_.solve_ms);
    append("registration_mode", last_candidate_recovering_ ? "recovery" : "tracking");
    number("min_eigenvalue", last_result_.min_eigenvalue);
    number("eigenvalue_ratio", last_result_.eigenvalue_ratio);
    number("position_innovation_m", last_result_.position_innovation);
    number("yaw_innovation_rad", last_result_.yaw_innovation);
    number("field_bytes", field_.bytes());
    number("field_build_ms", field_.buildMilliseconds());
    status_pub_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_;
  RegistrationConfig config_;
  std::optional<Pose> seed_;
  DistanceField field_;
  ros::Subscriber static_sub_, local_sub_, seed_sub_, legacy_result_sub_, legacy_status_sub_, legacy_tf_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> raw_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  std::unique_ptr<Sync> sync_;
  ros::Publisher pose_pub_, correction_pub_, status_pub_;
  ros::WallTimer status_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::string legacy_map_dir_;
  std::string reported_state_;
  int reported_level_ = -1;
  std::optional<LegacyPose> legacy_result_, legacy_tf_;
  std::optional<LegacyStatus> legacy_status_;
  ros::Time alignment_epoch_, active_legacy_seed_, consumed_legacy_seed_;
  ros::WallTime legacy_tf_wall_;

  double match_hz_;
  std::optional<Pose> odom_from_camera_, correction_;
  std::optional<Confirmation> confirmation_;
  State state_ = State::WaitingForSeed;
  std::string detail_ = "seed_required";
  ros::Time last_pair_, last_job_, last_local_, last_candidate_, last_accepted_, last_applied_;
  ros::Time seed_not_before_;
  ros::WallTime last_local_wall_;
  RegistrationResult last_result_;
  bool last_candidate_recovering_ = false;
  std::uint64_t revision_ = 0, accepted_ = 0, rejected_ = 0;
  int rejects_ = 0, failed_rounds_ = 0;

  std::mutex mutex_;
  std::condition_variable available_;
  std::optional<Job> pending_;
  std::optional<Completed> completed_;
  bool stop_ = false;
  std::thread worker_;
};

}  // namespace
}  // namespace fastlio_dll

int main(int argc, char** argv) {
  ros::init(argc, argv, "fastlio_dll");
  try {
    fastlio_dll::Localizer localizer;
    ros::spin();
  } catch (const std::exception& error) {
    std::fprintf(stderr, "DLL startup failed: %s\n", error.what());
    return 1;
  }
  return 0;
}
