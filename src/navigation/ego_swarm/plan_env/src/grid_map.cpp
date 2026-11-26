#include "plan_env/grid_map.h"

#include <boost/property_tree/json_parser.hpp>
#include <sstream>
#include <std_msgs/Bool.h>

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

void GridMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;
  node_.param("grid_map/nav_radius", nav_radius_, 0.4);
  node_.param("grid_map/nav_half_height", nav_half_height_, 0.25);
  std::vector<double> lo,hi;
  if(!node_.getParam("/drone_nav/config/bounds_min",lo)||!node_.getParam("/drone_nav/config/bounds_max",hi)||lo.size()!=3||hi.size()!=3) throw std::runtime_error("navigation bounds required");
  for(int i=0;i<3;++i){nav_min_[i]=lo[i];nav_max_[i]=hi[i];}
  std::vector<double> body;
  if(!node_.getParam("/drone_nav/config/body_half_extent",body)||body.size()!=3||!(body[0]>0&&body[1]>0&&body[2]>0)) throw std::runtime_error("navigation body_half_extent required");
  nav_body_xy_=std::max(body[0],body[1]);nav_body_z_=body[2];
  node_.param("/drone_nav/config/allow_unknown_above",nav_unknown_above_free_,false);
  if(nav_unknown_above_free_) ROS_WARN("[grid_map] allow_unknown_above: unknown cells above observed free space are treated as free");
  check_service_=node_.advertiseService("check_trajectory", &GridMap::checkPath,this);

  /* get parameter */
  double x_size, y_size, z_size;
  node_.param("grid_map/resolution", mp_.resolution_, -1.0);
  node_.param("grid_map/map_size_x", x_size, -1.0);
  node_.param("grid_map/map_size_y", y_size, -1.0);
  node_.param("grid_map/map_size_z", z_size, -1.0);
  node_.param("grid_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
  node_.param("grid_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
  node_.param("grid_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
  node_.param("grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);

  node_.param("grid_map/fx", mp_.fx_, -1.0);
  node_.param("grid_map/fy", mp_.fy_, -1.0);
  node_.param("grid_map/cx", mp_.cx_, -1.0);
  node_.param("grid_map/cy", mp_.cy_, -1.0);

  node_.param("grid_map/use_depth_filter", mp_.use_depth_filter_, true);
  node_.param("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
  node_.param("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  node_.param("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  node_.param("grid_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  node_.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  node_.param("grid_map/skip_pixel", mp_.skip_pixel_, -1);

  node_.param("grid_map/p_hit", mp_.p_hit_, 0.70);
  node_.param("grid_map/p_miss", mp_.p_miss_, 0.35);
  node_.param("grid_map/p_min", mp_.p_min_, 0.12);
  node_.param("grid_map/p_max", mp_.p_max_, 0.97);
  node_.param("grid_map/p_occ", mp_.p_occ_, 0.80);
  node_.param("grid_map/min_ray_length", mp_.min_ray_length_, -0.1);
  node_.param("grid_map/max_ray_length", mp_.max_ray_length_, -0.1);

  node_.param("grid_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);
  node_.param("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);
  node_.param("grid_map/virtual_ceil_yp", mp_.virtual_ceil_yp_, -0.1);
  node_.param("grid_map/virtual_ceil_yn", mp_.virtual_ceil_yn_, -0.1);

  node_.param("grid_map/show_occ_time", mp_.show_occ_time_, false);
  node_.param("grid_map/pose_type", mp_.pose_type_, 1);

  node_.param("grid_map/frame_id", mp_.frame_id_, string("world"));
  node_.param("grid_map/local_map_margin", mp_.local_map_margin_, 1);
  node_.param("grid_map/ground_height", mp_.ground_height_, 1.0);

  node_.param("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_, 1.0);

  if( mp_.virtual_ceil_height_ - mp_.ground_height_ > z_size)
  {
    mp_.virtual_ceil_height_ = mp_.ground_height_ + z_size;
  }

  if(mp_.resolution_<=0 || x_size<=0 || y_size<=0 || z_size<=0 || mp_.skip_pixel_<1 || (x_size*y_size*z_size/std::pow(mp_.resolution_,3))>6000000) throw std::runtime_error("invalid map geometry or memory budget");
  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  /* init callback */

  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "grid_map/depth", 50));
  extrinsic_sub_ = node_.subscribe<nav_msgs::Odometry>(
      "/vins_estimator/extrinsic", 10, &GridMap::extrinsicCallback, this); //sub

  if (mp_.pose_type_ == POSE_STAMPED)
  {
    pose_sub_.reset(
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "grid_map/pose", 25));

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCallback, this, _1, _2));
  }
  else if (mp_.pose_type_ == ODOMETRY)
  {
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "grid_map/odom", 100, ros::TransportHints().tcpNoDelay()));

    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCallback, this, _1, _2));
  }

  // use odometry and point cloud
  nav_cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_,"grid_map/cloud",3));
  nav_sensor_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_,"grid_map/odom",10));
  nav_cloud_sync_.reset(new message_filters::Synchronizer<CloudOdomPolicy>(CloudOdomPolicy(10),*nav_cloud_sub_,*nav_sensor_sub_));
  nav_cloud_sync_->registerCallback(boost::bind(&GridMap::cloudOdomCallback,this,_1,_2));

  occ_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::updateOccupancyCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.11), &GridMap::visCallback, this);

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflate", 10);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;
  md_.last_occ_update_time_.fromSec(0);

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;

  md_.flag_depth_odom_timeout_ = false;
  md_.flag_use_depth_fusion = false;

  md_.local_bound_min_.setZero();
  md_.local_bound_max_.setZero();
  initPrior();

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

void GridMap::initPrior()
{
  std::string path, map_sha;
  node_.param<std::string>("grid_map/prior_grid_file", path, "");
  if (path.empty()) return;
  prior_required_ = true;
  if (mp_.frame_id_ != "drone_odom" ||
      !node_.getParam("/drone_nav/config/map_id", prior_map_id_) || prior_map_id_.empty() ||
      !node_.getParam("grid_map/prior_map_sha256", map_sha) ||
      !nav_min_.allFinite() || !nav_max_.allFinite() || (nav_min_.array() >= nav_max_.array()).any())
    throw std::runtime_error("prior grid requires drone_odom, map_id, verified SHA-256 and navigation bounds");

  prior_grid_.reset(new plan_env::PriorGrid(path, map_sha));
  prior_flags_.assign(md_.occupancy_buffer_.size(), 0);
  prior_tf_.reset(new tf2_ros::Buffer);
  prior_tf_listener_.reset(new tf2_ros::TransformListener(*prior_tf_));
  prior_result_sub_ = node_.subscribe("/drone_reloc/result", 1, &GridMap::onPriorRelocResult, this);
  prior_status_sub_ = node_.subscribe("/drone_reloc/status", 1, &GridMap::onPriorRelocStatus, this);
  prior_ready_pub_ = node_.advertise<std_msgs::Bool>("grid_map/prior_ready", 1, true);
  prior_ready_pub_.publish(std_msgs::Bool());
  prior_timer_ = node_.createWallTimer(ros::WallDuration(0.1), &GridMap::refreshPrior, this);
  ROS_INFO_STREAM("[grid_map] verified prior grid " << path << ", waiting for confirmed localization");
}

void GridMap::onPriorRelocResult(const std_msgs::String::ConstPtr& message)
{
  prior_confirmed_ = false;
  try {
    std::istringstream input(message->data);
    boost::property_tree::ptree result;
    boost::property_tree::read_json(input, result);
    prior_confirmed_ = result.get<bool>("confirmed", false) &&
                       result.get<std::string>("map", "") == prior_map_id_;
  } catch (const boost::property_tree::ptree_error&) {
    ROS_WARN_THROTTLE(5, "[grid_map] invalid relocalization result");
  }
  if (!prior_confirmed_) prior_alignment_ok_ = false;
}

void GridMap::onPriorRelocStatus(const std_msgs::String::ConstPtr& message)
{
  prior_localized_ = false;
  try {
    std::istringstream input(message->data);
    boost::property_tree::ptree status;
    boost::property_tree::read_json(input, status);
    prior_localized_ = status.get<bool>("valid", false) &&
                       status.get<std::string>("state", "") == "LOCALIZED";
    prior_status_at_ = ros::WallTime::now();
  } catch (const boost::property_tree::ptree_error&) {
    ROS_WARN_THROTTLE(5, "[grid_map] invalid relocalization status");
  }
  if (!prior_localized_) prior_alignment_ok_ = false;
}

void GridMap::refreshPrior(const ros::WallTimerEvent&)
{
  const bool was_aligned = prior_alignment_ok_;
  prior_alignment_ok_ = false;
  if (prior_confirmed_ && prior_localized_ && !prior_transform_changed_ &&
      (ros::WallTime::now() - prior_status_at_).toSec() <= 1.5) {
    try {
      const auto tf = prior_tf_->lookupTransform("drone_map", mp_.frame_id_, ros::Time(0));
      const double age = (ros::Time::now() - tf.header.stamp).toSec();
      Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                           tf.transform.rotation.y, tf.transform.rotation.z);
      const Eigen::Vector3d p(tf.transform.translation.x, tf.transform.translation.y,
                              tf.transform.translation.z);
      if (!tf.header.stamp.isZero() && age >= -0.05 && age <= mp_.odom_depth_timeout_ &&
          p.allFinite() && q.coeffs().allFinite() && std::abs(q.norm() - 1.0) < 1e-3) {
        Eigen::Isometry3d map_from_odom = Eigen::Isometry3d::Identity();
        map_from_odom.linear() = q.normalized().toRotationMatrix();
        map_from_odom.translation() = p;
        if (!prior_projected_ && prior_grid_) {
          prior_projected_ = projectPrior(map_from_odom);
          prior_map_from_odom_ = map_from_odom;
          prior_grid_.reset();
        }
        if (prior_projected_) {
          if ((prior_map_from_odom_.translation() - p).norm() > .01 ||
              (prior_map_from_odom_.linear() - map_from_odom.linear()).norm() > .01) {
            prior_transform_changed_ = true;
            ROS_ERROR("[grid_map] prior alignment moved, restart navigation after relocalizing");
          } else {
            prior_alignment_ok_ = true;
          }
        }
      }
    } catch (const tf2::TransformException&) {
      // Relocalization has not yet supplied a transform in this session.
    }
  }
  if (was_aligned && !prior_alignment_ok_)
    ROS_WARN("[grid_map] lost prior grid alignment");
  const bool ready = !getOdomDepthTimeout();
  if (ready != prior_reported_ready_) {
    std_msgs::Bool status;
    status.data = ready;
    prior_ready_pub_.publish(status);
    prior_reported_ready_ = ready;
  }
}

bool GridMap::projectPrior(const Eigen::Isometry3d& map_from_odom)
{
  const int xy = std::ceil(nav_radius_ / mp_.resolution_);
  const int z_pad = std::ceil(nav_half_height_ / mp_.resolution_);
  const Eigen::Vector3d padding = mp_.resolution_ * Eigen::Vector3d(xy, xy, z_pad);
  // Include obstacles whose inflated shell can touch the allowed navigation volume.
  const Eigen::Vector3d lower = (nav_min_ - padding).cwiseMax(mp_.map_min_boundary_);
  const Eigen::Vector3d upper = (nav_max_ + padding).cwiseMin(mp_.map_max_boundary_);
  if ((lower.array() >= upper.array()).any()) return false;

  Eigen::Vector3i lo, hi;
  posToIndex(lower, lo);
  posToIndex(upper, hi);
  boundIndex(lo);
  boundIndex(hi);
  const plan_env::PriorGrid::Projection projection(*prior_grid_, map_from_odom.linear(), mp_.resolution_);
  std::size_t free_count = 0, occupied_count = 0;
  for (int x = lo.x(); x <= hi.x(); ++x)
    for (int y = lo.y(); y <= hi.y(); ++y)
      for (int z = lo.z(); z <= hi.z(); ++z) {
        const Eigen::Vector3i id(x, y, z);
        Eigen::Vector3d center;
        indexToPos(id, center);
        const auto state = projection.classify(map_from_odom * center);
        if (state == plan_env::PriorCell::Unknown) continue;
        const int adr = toAddress(id);
        const bool occupied = state == plan_env::PriorCell::Occupied;
        prior_flags_[adr] = Seeded;
        md_.occupancy_buffer_[adr] = occupied ? mp_.clamp_max_log_ : mp_.clamp_min_log_;
        if (occupied) ++occupied_count;
        else ++free_count;
      }
  if (free_count == 0 && occupied_count == 0) {
    ROS_ERROR("[grid_map] prior has no evidence inside the aligned navigation volume");
    return false;
  }
  md_.local_bound_min_ = lo;
  md_.local_bound_max_ = hi;
  inflateNavigationMap();
  ROS_INFO_STREAM("[grid_map] initialized prior in drone_odom: " << free_count
                  << " free, " << occupied_count << " occupied");
  return true;
}

void GridMap::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;
  int skip_pix = mp_.skip_pixel_;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_r_m_;

  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v+=skip_pix)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u+=skip_pix)
      {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240)
          std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else
  {

    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_r_m_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
      {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_)
        {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          if (*row_ptr == 0)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }
          else if (depth < mp_.depth_filter_mindist_)
          {
            continue;
          }
          else if (depth > mp_.depth_filter_maxdist_)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }

          // project to world frame
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;
          // if (!isInMap(pt_world)) {
          //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
          // }

          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          // check consistency with last image, disabled...
          if (false)
          {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
            {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_)
              {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            }
            else
            {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* maintain camera pose for consistency check */

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_r_m_ = md_.camera_r_m_;
  md_.last_depth_image_ = md_.depth_image_;
}

void GridMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return;

  ros::Time t1, t2;

  if(md_.raycast_num_==120){std::fill(md_.flag_rayend_.begin(),md_.flag_rayend_.end(),-1);std::fill(md_.flag_traverse_.begin(),md_.flag_traverse_.end(),-1);md_.raycast_num_=0;}
  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true;

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    const bool live_hit = md_.count_hit_[idx_ctns] > 0;
    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (!prior_flags_.empty()) {
      const bool seeded = prior_flags_[idx_ctns] & Seeded;
      prior_flags_[idx_ctns] |= SeenLive;
      if (seeded && live_hit) {
        md_.occupancy_buffer_[idx_ctns] = mp_.clamp_max_log_;
        continue;
      }
    }

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }

    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void GridMap::clearAndInflateLocalMap()
{
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // clear data outside the local range

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    {

      for (int z = min_cut_m(2); z < min_cut(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    {

      for (int y = min_cut_m(1); y < min_cut(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    {

      for (int x = min_cut_m(0); x < min_cut(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  // inflate occupied voxels to compensate robot size

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // int inf_step_z = 1;
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // clear outdated data
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_)
        {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k)
          {
            inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            if (idx_inf < 0 ||
                idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2))
            {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }

  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.5) {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_) - 1;
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
  }
}

void GridMap::visCallback(const ros::TimerEvent & /*event*/)
{

  publishMapInflate(true);
  publishMap();
}

void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  if (md_.last_occ_update_time_.toSec() < 1.0 ) md_.last_occ_update_time_ = ros::Time::now();
  
  if (!md_.occ_need_update_)
  {
    if ( md_.flag_use_depth_fusion && (ros::Time::now() - md_.last_occ_update_time_).toSec() > mp_.odom_depth_timeout_ )
    {
      ROS_ERROR("odom or depth lost! ros::Time::now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f", 
        ros::Time::now().toSec(), md_.last_occ_update_time_.toSec(), mp_.odom_depth_timeout_);
      md_.flag_depth_odom_timeout_ = true;
    }
    return;
  }
  md_.last_occ_update_time_ = ros::Time::now();

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  projectDepthImage();
  // t2 = ros::Time::now();
  raycastProcess();
  // t3 = ros::Time::now();

  if (md_.local_updated_)
    clearAndInflateLocalMap();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
}

void GridMap::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                const geometry_msgs::PoseStampedConstPtr &pose)
{
  if (prior_required_) return;
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_r_m_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                       pose->pose.orientation.y, pose->pose.orientation.z)
                        .toRotationMatrix();
  if (isInMap(md_.camera_pos_))
  {
    md_.has_odom_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }

  md_.flag_use_depth_fusion = true;
}

void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  if(odom->header.frame_id != mp_.frame_id_ || odom->header.stamp <= last_odom_stamp_) return;
  Eigen::Vector3d p(odom->pose.pose.position.x,odom->pose.pose.position.y,odom->pose.pose.position.z);
  if(!p.allFinite() || !isInMap(p)) return;
  md_.camera_pos_=p; md_.has_odom_=true;
  last_odom_stamp_=odom->header.stamp; last_odom_wall_=ros::WallTime::now();
}

void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{
  if((prior_required_ && !prior_projected_) || !md_.has_odom_ || img->header.frame_id!=mp_.frame_id_ || img->header.stamp<=last_cloud_stamp_ ||
     img->header.stamp!=last_odom_stamp_ || (ros::Time::now()-img->header.stamp).toSec()>.3) return;
  const auto start = mp_.show_occ_time_ ? ros::WallTime::now() : ros::WallTime();
  pcl::PointCloud<pcl::PointXYZ> cloud; pcl::fromROSMsg(*img,cloud);
  md_.proj_points_.clear();
  for(const auto &p:cloud) if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))
    md_.proj_points_.emplace_back(p.x,p.y,p.z);
  if(md_.proj_points_.empty()) return;
  md_.proj_points_cnt=md_.proj_points_.size();
  raycastProcess();
  // Upstream traversal excludes its terminal voxel: the physically occupied sensor cell is known free.
  Eigen::Vector3i origin_id;posToIndex(md_.camera_pos_,origin_id);
  const int origin_address = toAddress(origin_id);
  if(isUnknown(origin_id)) md_.occupancy_buffer_[origin_address]=mp_.clamp_min_log_;
  if(!prior_flags_.empty()) prior_flags_[origin_address] |= SeenLive;
  inflateNavigationMap();
  last_cloud_stamp_=img->header.stamp; last_cloud_wall_=ros::WallTime::now();md_.has_cloud_=true;
  if (mp_.show_occ_time_)
    ROS_INFO_STREAM("[grid_map] cloud_update_ms=" << (last_cloud_wall_ - start).toSec() * 1000);
}

void GridMap::inflateNavigationMap()
{
  auto& inflated = md_.occupancy_buffer_inflate_;
  std::fill(inflated.begin(), inflated.end(), 0);
  const int radius = std::ceil(nav_radius_ / mp_.resolution_);
  const int half_height = std::ceil(nav_half_height_ / mp_.resolution_);
  const auto& size = mp_.map_voxel_num_;
  int address = 0;
  for (int x = 0; x < size.x(); ++x)
    for (int y = 0; y < size.y(); ++y)
      for (int z = 0; z < size.z(); ++z, ++address) {
        if (md_.occupancy_buffer_[address] <= mp_.min_occupancy_log_) continue;
        const int z_lo = std::max(0, z - half_height);
        const int z_hi = std::min(size.z() - 1, z + half_height);
        for (int nx = std::max(0, x - radius); nx <= std::min(size.x() - 1, x + radius); ++nx)
          for (int ny = std::max(0, y - radius); ny <= std::min(size.y() - 1, y + radius); ++ny) {
            const auto start = inflated.begin() + toAddress(Eigen::Vector3i(nx, ny, z_lo));
            std::fill_n(start, z_hi - z_lo + 1, 1);
          }
      }
}

std::string GridMap::unknownColumnReason(const Eigen::Vector3d &pos)
{
 if(!isInMap(pos)) return "outside map";
 if(!nav_unknown_above_free_) return "allow_unknown_above=false";
 Eigen::Vector3i id;posToIndex(pos,id);
 for(int z=id.z()-1;z>=0;--z) {
  Eigen::Vector3i below(id.x(),id.y(),z);
  if(isUnknown(below) || onlyPriorFree(toAddress(below))) continue;
  Eigen::Vector3d p;indexToPos(below,p);
  return "first observed below at "+std::to_string(p.x())+","+std::to_string(p.y())+","+std::to_string(p.z())+
    " raw="+(getOccupancy(below)?"occupied":"free")+
    " inflated="+std::to_string(int(md_.occupancy_buffer_inflate_[toAddress(below)]));
 }
 return "no observed voxel below in column "+std::to_string(id.x())+","+std::to_string(id.y());
}

bool GridMap::checkPath(traj_utils::CheckTrajectory::Request &req, traj_utils::CheckTrajectory::Response &res)
{
 res.valid=false;res.reason="stale map or invalid frame";
 if(getOdomDepthTimeout() || req.path.header.frame_id!=mp_.frame_id_ || req.path.poses.empty() || req.path.poses.size()>2000) return true;
 Eigen::Vector3d prev; bool first=true;
 for(const auto &pose:req.path.poses) {
  Eigen::Vector3d p(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
  if(!p.allFinite()) {res.reason="nonfinite path";return true;}
  if(first) prev=p;
  int steps=std::max(1,int(ceil((p-prev).norm()/(mp_.resolution_*0.5))));
  if(steps>2000) {res.reason="path segment too long";return true;}
  for(int k=0;k<=steps;++k) {
   Eigen::Vector3d c=prev+(p-prev)*(double(k)/steps);
   // Obstacles anywhere in the radius/half_height footprint are caught by the inflation layer.
   // Only the measured body volume must be observed free: open sky above the lidar never returns
   // a ray, so the clearance shell beyond the body may stay unknown outdoors. With allow_unknown_above
   // the body volume may also be unknown when it sits above observed free space (isUnknownBlocking).
   if(getInflateOccupancy(c)!=0) {
    if(!isInMap(c) || (c.array()<nav_min_.array()).any() || (c.array()>nav_max_.array()).any() ||
       c.z()<mp_.ground_height_+nav_half_height_ || c.z()>mp_.virtual_ceil_height_-nav_half_height_)
      res.reason="outside flight volume";
    else {
      Eigen::Vector3i id;posToIndex(c,id);
      // Known collisions must never be hidden by an unknown centre (the caller
      // may defer observing a future goal, but must still reject known obstacles).
      if(md_.occupancy_buffer_inflate_[toAddress(id)]) res.reason="occupied inflated footprint";
      else res.reason="unknown path center";
    }
    res.reason += " at "+std::to_string(c.x())+","+std::to_string(c.y())+","+std::to_string(c.z());
    if(isUnknownBlocking(c)) res.reason += "; "+unknownColumnReason(c);
    return true;
   }
   int r=std::min(ceil(nav_body_xy_/mp_.resolution_),ceil(nav_radius_/mp_.resolution_));
   int h=std::min(ceil(nav_body_z_/mp_.resolution_),ceil(nav_half_height_/mp_.resolution_));
   for(int dx=-r;dx<=r;++dx) for(int dy=-r;dy<=r;++dy) for(int dz=-h;dz<=h;++dz) {
    Eigen::Vector3d q=c+mp_.resolution_*Eigen::Vector3d(dx,dy,dz);
    if(!isInMap(q)||isUnknownBlocking(q)) {res.reason="unknown footprint at "+std::to_string(q.x())+","+std::to_string(q.y())+","+std::to_string(q.z())+"; "+unknownColumnReason(q);return true;}
   }
  }
  prev=p;first=false;
 }
 res.valid=true;res.reason="clear";return true;
}

void GridMap::publishMap()
{

  if (map_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info)
{

  if (map_inf_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  if (all_info)
  {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

bool GridMap::odomValid() { return md_.has_odom_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void GridMap::extrinsicCallback(const nav_msgs::OdometryConstPtr &odom)
{
  Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
  Eigen::Matrix3d cam2body_r_m = cam2body_q.toRotationMatrix();
  md_.cam2body_.block<3, 3>(0, 0) = cam2body_r_m;
  md_.cam2body_(0, 3) = odom->pose.pose.position.x;
  md_.cam2body_(1, 3) = odom->pose.pose.position.y;
  md_.cam2body_(2, 3) = odom->pose.pose.position.z;
  md_.cam2body_(3, 3) = 1.0;
}

void GridMap::depthOdomCallback(const sensor_msgs::ImageConstPtr &img,
                                const nav_msgs::OdometryConstPtr &odom)
{
  if (prior_required_) return;
  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;
  md_.flag_use_depth_fusion = true;
}
