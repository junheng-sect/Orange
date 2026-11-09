#include <plan_manage/planner_manager.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <sstream>
#include <unordered_set>
#include <chrono>

using Eigen::Vector3d;
using Eigen::Vector3i;
class Demo {
  ros::NodeHandle n_{"/ego_demo"};
  ego_planner::EGOPlannerManager manager_;
  ego_planner::PlanningVisualization::Ptr vis_;
  ros::Publisher map_pub_,path_pub_,marker_pub_,status_pub_,slice_pub_;
  ros::Subscriber goal_sub_;
  ros::ServiceServer example_;
  ros::Timer labels_;
  Vector3d start_;
  Vector3d goal_;
  bool have_goal_=false;
  std::string label_="Loading static map";
  double altitude_,resolution_;
  bool ready_=false;
  int source_points_=0,occupied_cells_=0;
  void status(const std::string& s) {
    std_msgs::String m;m.data=s;status_pub_.publish(m);ROS_INFO_STREAM(s);
    std::ofstream("/results/latest.json")<<s<<"\n";
    if(s.find("PLANNED")!=std::string::npos)label_="OFFLINE | EGO path ready";
    else if(s.find("READY")!=std::string::npos)label_="OFFLINE | choose 2D Nav Goal";
    else if(s.find("PLANNING")!=std::string::npos)label_="OFFLINE | planning...";
    else label_="OFFLINE | rejected goal / no valid path";
  }
  void marker(int id,int type,const Vector3d& p,float r,float g,float b,const std::string& text="") {
    visualization_msgs::Marker m;m.header.frame_id="world";m.header.stamp=ros::Time::now();m.ns="demo";m.id=id;m.type=type;m.action=m.ADD;m.pose.orientation.w=1;
    m.pose.position.x=p.x();m.pose.position.y=p.y();m.pose.position.z=p.z();m.color.r=r;m.color.g=g;m.color.b=b;m.color.a=1;
    m.scale.x=m.scale.y=m.scale.z=(type==m.TEXT_VIEW_FACING?0.55:0.6);m.text=text;marker_pub_.publish(m);
  }
  bool clear(const Vector3d& p) {return manager_.grid_map_->getInflateOccupancy(p)==0;}
  bool segmentClear(const Vector3d& a,const Vector3d& b) {
    int k=std::max(1,int(std::ceil((b-a).norm()/.04)));
    for(int i=0;i<=k;++i) if(!clear(a+(b-a)*(double(i)/k)))return false;
    return true;
  }
 public:
  Demo() {
    if(std::string(getenv("ROS_MASTER_URI")?getenv("ROS_MASTER_URI"):"")!="http://127.0.0.1:11311")throw std::runtime_error("Offline demo requires its container-local master");
    n_.param("altitude",altitude_,1.3446101049);n_.param("grid_map/resolution",resolution_,.2);start_=Vector3d(0,0,altitude_);
    map_pub_=n_.advertise<sensor_msgs::PointCloud2>("map",1,true);path_pub_=n_.advertise<nav_msgs::Path>("path",1,true);
    marker_pub_=n_.advertise<visualization_msgs::Marker>("markers",30,true);status_pub_=n_.advertise<std_msgs::String>("status",1,true);
    slice_pub_=n_.advertise<sensor_msgs::PointCloud2>("obstacles_at_height",1,true);
    vis_.reset(new ego_planner::PlanningVisualization(n_));manager_.initPlanModules(n_,vis_);manager_.deliverTrajToOptimizer();manager_.setDroneIdtoOpt();
    manager_.grid_map_->resetBuffer();
    std::string file;n_.param<std::string>("map_file",file,"/maps/selected.pcd");
    pcl::PointCloud<pcl::PointXYZ> points;if(pcl::io::loadPCDFile(file,points)<0)throw std::runtime_error("Cannot read saved map");source_points_=points.size();
    std::unordered_set<int> unique;std::vector<Vector3i> cells;
    for(const auto& p:points) {
      Vector3d q(p.x,p.y,p.z);if(!q.allFinite()||!manager_.grid_map_->isInMap(q))continue;
      Vector3i id;manager_.grid_map_->posToIndex(q,id);int key=manager_.grid_map_->toAddress(id);
      if(unique.insert(key).second)cells.push_back(id);
    }
    occupied_cells_=cells.size();
    // Static occupied cells include every PCD point in the bounded grid.
    // Inflate cell centres conservatively for a 0.4 m radius / 0.25 m half height.
    std::vector<Vector3i> offsets;
    int r=std::ceil(.4/resolution_)+1,h=std::ceil(.25/resolution_)+1;
    for(int x=-r;x<=r;++x)for(int y=-r;y<=r;++y)for(int z=-h;z<=h;++z)
      if(std::hypot(x*resolution_,y*resolution_)<=.4+resolution_*1.42)offsets.emplace_back(x,y,z);
    for(const auto& c:cells)for(const auto& d:offsets) {
      Vector3i id=c+d;if(!manager_.grid_map_->isInMap(id))continue;
      Vector3d p;manager_.grid_map_->indexToPos(id,p);manager_.grid_map_->setOccupied(p);
    }
    // A bounded demonstration flight slab, not a terrain-following model.
    for(double x=-63.9;x<64.;x+=resolution_)for(double y=-63.9;y<64.;y+=resolution_) {
      for(double z=-.4;z<.3;z+=resolution_)manager_.grid_map_->setOccupied(Vector3d(x,y,z));
      for(double z=3.1;z<4.;z+=resolution_)manager_.grid_map_->setOccupied(Vector3d(x,y,z));
      // Keep this click demo close to the user's requested 1.5 m AGL.
      manager_.grid_map_->setOccupied(Vector3d(x,y,altitude_-.35));
      manager_.grid_map_->setOccupied(Vector3d(x,y,altitude_+.35));
    }
    sensor_msgs::PointCloud2 map;pcl::toROSMsg(points,map);map.header.frame_id="world";map.header.stamp=ros::Time::now();map_pub_.publish(map);
    pcl::PointCloud<pcl::PointXYZ> slice;
    for(int ix=0;ix<640;++ix)for(int iy=0;iy<640;++iy) {
      Vector3d p(-64+(ix+.5)*resolution_,-64+(iy+.5)*resolution_,altitude_);
      if(manager_.grid_map_->getInflateOccupancy(p)>0)slice.push_back(pcl::PointXYZ(p.x(),p.y(),p.z()));
    }
    sensor_msgs::PointCloud2 obstacle;pcl::toROSMsg(slice,obstacle);obstacle.header=map.header;slice_pub_.publish(obstacle);
    marker(0,visualization_msgs::Marker::SPHERE,start_,0,.8,0);marker(1,visualization_msgs::Marker::TEXT_VIEW_FACING,start_+Vector3d(0,0,1.1),0,.5,0,"START: map origin, 1.5 m AGL");
    goal_sub_=n_.subscribe("goal_click",1,&Demo::goal,this);example_=n_.advertiseService("plan_example",&Demo::example,this);
    ready_=clear(start_);status(ready_?"{\"state\":\"READY\",\"offline\":true}":"{\"state\":\"START_OCCUPIED\",\"offline\":true}");
    labels_=n_.createTimer(ros::Duration(.5),[this](const ros::TimerEvent&){
      marker(0,visualization_msgs::Marker::SPHERE,start_,0,.8,0);
      marker(1,visualization_msgs::Marker::TEXT_VIEW_FACING,start_+Vector3d(0,0,1.1),0,.5,0,"START: map origin, 1.5 m AGL");
      marker(3,visualization_msgs::Marker::TEXT_VIEW_FACING,start_+Vector3d(0,-2,2.0),.15,.15,.15,label_);
      if(have_goal_)marker(2,visualization_msgs::Marker::SPHERE,goal_,1,.4,0);
    });
  }
  void goal(const geometry_msgs::PoseStamped::ConstPtr& m) {
    if(m->header.frame_id!="world"){status("{\"state\":\"WRONG_FRAME\"}");return;}
    plan(Vector3d(m->pose.position.x,m->pose.position.y,altitude_));
  }
  bool example(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& out) {
    std::vector<double> g;n_.param("example_goal",g,std::vector<double>{-15.,0.});
    out.success=g.size()==2&&plan(Vector3d(g[0],g[1],altitude_));out.message=out.success?"EGO trajectory accepted":"No accepted EGO trajectory";return true;
  }
  bool plan(const Vector3d& target) {
    nav_msgs::Path path;path.header.frame_id="world";path.header.stamp=ros::Time::now();path_pub_.publish(path);
    std::ofstream("/results/path.csv")<<"x,y,z\n";
    goal_=target;have_goal_=target.allFinite();
    marker(2,visualization_msgs::Marker::SPHERE,target,1,.4,0);
    if(!ready_||!target.allFinite()||!clear(target)){status("{\"state\":\"GOAL_OR_START_OCCUPIED\",\"success\":false}");return false;}
    if((target-start_).norm()<.3){status("{\"state\":\"GOAL_TOO_CLOSE\",\"success\":false}");return false;}
    status("{\"state\":\"PLANNING\",\"offline\":true}");
    bool direct=segmentClear(start_,target);auto begin=std::chrono::steady_clock::now();
    for(int attempt=0;attempt<4;++attempt) {
      bool ok=manager_.reboundReplan(start_,Vector3d::Zero(),Vector3d::Zero(),target,Vector3d::Zero(),true,attempt>0);
      if(!ok)continue;
      auto& spline=manager_.local_data_.position_traj_;double duration=spline.getTimeSum();
      if(!std::isfinite(duration)||duration<=0||duration>1000)continue;
      std::vector<Vector3d> pts;bool valid=true;Vector3d prev=start_;
      for(double t=0;t<duration+.02;t+=.02) {
        Vector3d p=spline.evaluateDeBoorT(std::min(t,duration));
        if(!p.allFinite()||std::abs(p.z()-altitude_)>.25||!segmentClear(prev,p)){valid=false;break;}
        pts.push_back(p);prev=p;
      }
      if(!valid||pts.empty()||(pts.front()-start_).norm()>.05||(pts.back()-target).norm()>.08)continue;
      double length=0;std::ofstream csv("/results/path.csv");csv<<"x,y,z\n";
      for(size_t i=0;i<pts.size();++i) {
        geometry_msgs::PoseStamped pose;pose.header=path.header;pose.pose.orientation.w=1;pose.pose.position.x=pts[i].x();pose.pose.position.y=pts[i].y();pose.pose.position.z=pts[i].z();path.poses.push_back(pose);
        csv<<pts[i].x()<<","<<pts[i].y()<<","<<pts[i].z()<<"\n";if(i)length+=(pts[i]-pts[i-1]).norm();
      }
      path_pub_.publish(path);
      double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
      std::ostringstream report;report<<"{\"state\":\"PLANNED\",\"success\":true,\"offline\":true,\"source_points\":"<<source_points_<<",\"occupied_cells\":"<<occupied_cells_<<",\"goal\":["<<target.x()<<","<<target.y()<<","<<target.z()<<"],\"length_m\":"<<length<<",\"straight_line_blocked\":"<<(direct?"false":"true")<<",\"sampled_clear\":true,\"planning_ms\":"<<ms<<",\"samples\":"<<pts.size()<<"}";
      std::ofstream("/results/latest.json")<<report.str()<<"\n";status(report.str());return true;
    }
    status("{\"state\":\"NO_ACCEPTED_PATH\",\"success\":false,\"offline\":true}");return false;
  }
};
int main(int argc,char** argv) {ros::init(argc,argv,"ego_saved_map_planner");srand(42);try{Demo d;ros::spin();}catch(const std::exception& e){ROS_FATAL_STREAM(e.what());return 1;}return 0;}
