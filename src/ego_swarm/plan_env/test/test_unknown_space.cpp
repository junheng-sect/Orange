#include <gtest/gtest.h>
#include <plan_env/grid_map.h>

// Exercise the actual map policy and CheckTrajectory on a private master, without flight inputs.
class GridMapUnknownSpaceTest : public ::testing::Test {
protected:
  GridMap map;
  void SetUp() override {
    ros::Time::init();
    map.mp_.resolution_ = 0.1;
    map.mp_.resolution_inv_ = 10.;
    map.mp_.map_origin_ = Eigen::Vector3d(-2., -2., -1.);
    map.mp_.map_size_ = Eigen::Vector3d(4., 4., 4.);
    map.mp_.map_min_boundary_ = map.mp_.map_origin_;
    map.mp_.map_max_boundary_ = map.mp_.map_origin_ + map.mp_.map_size_;
    map.mp_.map_voxel_num_ = Eigen::Vector3i(40, 40, 40);
    map.mp_.clamp_min_log_ = -2.;
    map.mp_.min_occupancy_log_ = 1.;
    map.mp_.ground_height_ = -1.;
    map.mp_.virtual_ceil_height_ = 3.;
    map.mp_.odom_depth_timeout_ = 100.;
    map.mp_.frame_id_ = "drone_odom";
    map.md_.occupancy_buffer_.assign(40*40*40, -2.01);
    map.md_.occupancy_buffer_inflate_.assign(40*40*40, 0);
    map.nav_min_ = Eigen::Vector3d(-1.5, -1.5, .4);
    map.nav_max_ = Eigen::Vector3d(1.5, 1.5, 2.5);
    map.nav_radius_ = .4;
    map.nav_half_height_ = .3;
    map.nav_body_xy_ = .27;
    map.nav_body_z_ = .15;
    map.nav_unknown_above_free_ = true;
    map.last_cloud_wall_ = map.last_odom_wall_ = ros::WallTime::now();
  }
  Eigen::Vector3d point(double z) { return Eigen::Vector3d(-.047704, -.333470, z); }
  void evidence(const Eigen::Vector3d& pos, bool occupied, bool inflated) {
    Eigen::Vector3i id; map.posToIndex(pos, id);
    const int adr = map.toAddress(id);
    map.md_.occupancy_buffer_[adr] = occupied ? 2. : -2.;
    map.md_.occupancy_buffer_inflate_[adr] = inflated;
  }
  void groundRays() {
    // Observed free rays under the entire test10 takeoff footprint, near inflated ground.
    for(int x=15;x<=23;++x) for(int y=15;y<=23;++y) {
      const int adr = map.toAddress(Eigen::Vector3i(x,y,10));
      map.md_.occupancy_buffer_[adr] = -2.;
      map.md_.occupancy_buffer_inflate_[adr] = 0;
    }
    evidence(point(.05), false, true);
  }
  traj_utils::CheckTrajectory::Response check() {
    traj_utils::CheckTrajectory::Request req;
    req.path.header.frame_id = "drone_odom";
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = -.047704;
    pose.pose.position.y = -.033470;
    pose.pose.position.z = .968028;
    req.path.poses.push_back(pose);
    traj_utils::CheckTrajectory::Response res;
    map.checkPath(req,res);
    return res;
  }
  void strict() { map.nav_unknown_above_free_ = false; }
  void inflateOnly(const Eigen::Vector3d& pos) {
    Eigen::Vector3i id;map.posToIndex(pos,id);
    map.md_.occupancy_buffer_inflate_[map.toAddress(id)]=1;
  }
};

TEST_F(GridMapUnknownSpaceTest, FreeRayInGroundInflationDoesNotBlockSky) {
  evidence(point(.05), false, true);
  EXPECT_FALSE(map.isUnknownBlocking(point(.768028)));
}
TEST_F(GridMapUnknownSpaceTest, EntirelyUnobservedColumnStillBlocks) {
  EXPECT_TRUE(map.isUnknownBlocking(point(.768028)));
}
TEST_F(GridMapUnknownSpaceTest, FirstRawObstacleBlocksDespiteFreeRayBelow) {
  evidence(point(.05), false, false);
  evidence(point(.45), true, true);
  EXPECT_TRUE(map.isUnknownBlocking(point(.768028)));
}
TEST_F(GridMapUnknownSpaceTest, StrictIndoorPolicyStillBlocksUnknown) {
  evidence(point(.05), false, false);
  strict();
  EXPECT_TRUE(map.isUnknownBlocking(point(.768028)));
}
TEST_F(GridMapUnknownSpaceTest, QueryInflationStillBlocksCollision) {
  evidence(point(.05), false, true);
  evidence(point(.768028), false, true);
  EXPECT_EQ(map.getInflateOccupancy(point(.768028)),1);
}
TEST_F(GridMapUnknownSpaceTest, RecordedTakeoffFootprintAboveFreeGroundRaysPasses) {
  groundRays();
  const auto res=check();
  EXPECT_TRUE(res.valid) << res.reason;
}
TEST_F(GridMapUnknownSpaceTest, OccupiedTakeoffEnvelopeStillRejected) {
  groundRays();
  evidence(Eigen::Vector3d(-.047704,-.033470,.968028), false, true);
  const auto res=check();
  EXPECT_FALSE(res.valid);
  EXPECT_NE(res.reason.find("occupied inflated footprint"),std::string::npos);
}
TEST_F(GridMapUnknownSpaceTest, MissingFootprintColumnReportsWhyItBlocks) {
  groundRays();
  evidence(point(.05), true, true);
  const auto res=check();
  EXPECT_FALSE(res.valid);
  EXPECT_NE(res.reason.find("raw=occupied"),std::string::npos) << res.reason;
}

TEST_F(GridMapUnknownSpaceTest, KnownCollisionTakesPriorityOverUnknownCenter) {
  // No supporting rays: the centre is unknown as well as inside an observed obstacle's inflation.
  inflateOnly(Eigen::Vector3d(-.047704,-.033470,.968028));
  const auto res=check();
  EXPECT_FALSE(res.valid);
  EXPECT_NE(res.reason.find("occupied inflated footprint"),std::string::npos) << res.reason;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "grid_map_unknown_space_test",
            ros::init_options::AnonymousName | ros::init_options::NoSigintHandler |
            ros::init_options::NoRosout);
  ::testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
