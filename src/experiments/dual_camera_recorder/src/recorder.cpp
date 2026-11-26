#include "gray_video.hpp"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <boost/filesystem.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <regex>
#include <thread>

namespace fs=boost::filesystem;
using Image=sensor_msgs::ImageConstPtr;
using Clock=std::chrono::steady_clock;
struct Camera {
    std::string name,topic,frame;
    unsigned width,height,fps;
};
const Camera cameras[]={
    {"ov13855","/ov13855/image_raw","ov13855_optical_frame",1280,960,15},
    {"ov9281","/usb_cam/image_raw","usb_camera_optical_frame",1280,800,30}
};
static void csv_text(std::ostream& out,const std::string& value) {
    out << '"';
    for (char c:value) { if (c=='"') out << '"'; out << c; }
    out << '"';
}

struct Segment {
    const std::string stem;
    const uint64_t open_ns;
    std::atomic<bool> degraded{false}, timing_gap{false}, sealed{false};
    const std::string event_camera;
    std::atomic<int> pending_workers;
    std::mutex frame_mutex, event_mutex;
    std::ofstream frames,events;
    Segment(std::string base,uint64_t start,int count):stem(std::move(base)),open_ns(start),
        event_camera(count == 1 ? cameras[0].name : "both"),pending_workers(count) {
        for (const auto& suffix:{"_frames.csv","_events.csv","_ov13855.mkv","_ov9281.mkv"})
            if (fs::exists(stem+suffix)||fs::exists(stem+suffix+".partial"))
                throw std::runtime_error("录像文件已存在 "+stem+suffix);
        frames.exceptions(std::ios::badbit|std::ios::failbit);
        events.exceptions(std::ios::badbit|std::ios::failbit);
        frames.open(stem+"_frames.csv.partial"); events.open(stem+"_events.csv.partial");
        frames << "camera,frame_index,source_stamp_ns,pts_ms,source_seq\n";
        events << "event,camera,stamp_ns,detail\n";
        event("open",event_camera,start,"bag OPEN");
    }
    void event(const std::string& type,const std::string& camera,uint64_t ns,const std::string& detail) {
        std::lock_guard<std::mutex> lock(event_mutex);
        events << type << ',' << camera << ',' << ns << ',';
        csv_text(events,detail); events << std::endl;
    }
    void frame(const std::string& camera,unsigned index,const Image& img,int64_t pts) {
        std::lock_guard<std::mutex> lock(frame_mutex);
        frames << camera << ',' << index << ',' << img->header.stamp.toNSec() << ',' << pts << ',' << img->header.seq << '\n';
    }
    void workerClosed() {
        if (--pending_workers != 0) return;
        try {
            event(degraded?"incomplete":"finalized",event_camera,ros::Time::now().toNSec(),"");
            frames.close(); events.close();
            fs::rename(stem+"_events.csv.partial",stem+"_events.csv");
            fs::rename(stem+"_frames.csv.partial",stem+"_frames.csv");
            ROS_INFO_STREAM("录像已封存 "<<stem<<(degraded?"（存在缺帧或故障）":""));
        } catch(const std::exception& e) { degraded=true; ROS_ERROR_STREAM("录像索引收尾失败 "<<e.what()); }
        sealed=true;
    }
};
using Session=std::shared_ptr<Segment>;
struct Job { Session session; Image image; }; // A null image drains and closes this segment.

class Worker {
    Camera cam;
    std::mutex mutex;
    std::condition_variable cv;
    std::deque<Job> queue;
    bool stopping=false;
    std::unique_ptr<GrayVideo> video;
    unsigned count=0;
    int64_t previous=-1;
    uint32_t previous_seq=0;
    uint64_t previous_ns=0;
    bool failed=false;
    std::thread thread;
    void problem(const Session& s,const std::string& detail) {
        s->degraded=true;
        ROS_ERROR_STREAM(cam.name<<" 录像 "<<detail);
        try { s->event("error",cam.name,ros::Time::now().toNSec(),detail); } catch(const std::exception& e) { ROS_ERROR_STREAM(e.what()); }
    }
    void process(const Job& job) {
        const auto& s=job.session;
        const auto path=s->stem+"_"+cam.name+".mkv";
        if (!job.image) {
            try {
                if (video && !failed) {
                    video->finish();
                    video.reset();
                    fs::rename(path+".partial",path);
                } else if (!failed) problem(s,"本段没有有效图像");
                s->event("closed",cam.name,ros::Time::now().toNSec(),std::to_string(count));
            } catch(const std::exception& e) { problem(s,e.what()); }
            video.reset(); s->workerClosed(); failed=false; count=0; previous=-1; previous_ns=0;
            return;
        }
        if (failed) return;
        const auto& m=job.image;
        const uint64_t ns=m->header.stamp.toNSec();
        if (ns<s->open_ns) return;
        const int64_t pts=(ns-s->open_ns+500000)/1000000;
        try {
            if (pts<=previous) {
                s->degraded=true;
                s->event("nonmonotonic",cam.name,ns,std::to_string(pts));
                return;
            }
            if (count%cam.fps==0 && fs::space(fs::path(path).parent_path()).available < (uint64_t(1)<<30))
                throw std::runtime_error("剩余空间低于 1 GiB，停止本路录像");
            if (!video) video.reset(new GrayVideo(path+".partial",cam.width,cam.height,cam.fps));
            video->write(*m,pts);
            s->frame(cam.name,count,m,pts);
            if (count && (ns-previous_ns>1500000000ULL/cam.fps || m->header.seq!=previous_seq+1)) {
                const bool missing = m->header.seq != previous_seq+1;
                if (missing) s->degraded=true;
                else s->timing_gap=true;
                s->event(missing?"frame_gap":"timing_gap",cam.name,ns,"previous_ns="+std::to_string(previous_ns)+", seq="+
                         std::to_string(previous_seq)+"->"+std::to_string(m->header.seq));
                ROS_WARN_STREAM_THROTTLE(2,cam.name<<" 录像帧间隙或序号跳变");
            }
            ++count; previous=pts; previous_ns=ns; previous_seq=m->header.seq;
        } catch(const std::exception& e) { failed=true; problem(s,e.what()); }
    }
    void loop() {
        for (;;) {
            Job job;
            {
                std::unique_lock<std::mutex> lock(mutex);
                cv.wait(lock,[&]{return stopping||!queue.empty();});
                if (queue.empty()) return;
                job=std::move(queue.front()); queue.pop_front();
            }
            process(job);
        }
    }
public:
    explicit Worker(Camera cfg):cam(std::move(cfg)),thread(&Worker::loop,this) {}
    ~Worker() {
        { std::lock_guard<std::mutex> lock(mutex); stopping=true; }
        cv.notify_one(); thread.join();
    }
    bool enqueue(Session s,Image m) {
        std::lock_guard<std::mutex> lock(mutex);
        // Allow short eMMC writeback stalls without blocking camera callbacks.
        if (m && queue.size()>=4*cam.fps) return false;
        queue.push_back({std::move(s),std::move(m)}); cv.notify_one(); return true;
    }
};

class Recorder {
    ros::NodeHandle nh,priv{"~"};
    fs::path directory;
    std::regex allowed;
    int camera_count = 2;
    std::unique_ptr<Worker> workers[2];
    ros::Subscriber images[2],events,status;
    ros::Publisher ready_pub,state_pub;
    ros::WallTimer timer;
    Clock::time_point last[2]{},heartbeat{};
    bool lost[2]={false,false};
    unsigned dropped[2]={0,0};
    Session active, closing;
    std::string last_path,notice,last_state;
    void close(const std::string& why,bool failure) {
        if (!active) return;
        if (failure) active->degraded=true;
        recordEvent("close",active->event_camera,why);
        for (int i=0;i<camera_count;++i) {
            if (dropped[i]) recordEvent("queue_drops",cameras[i].name,std::to_string(dropped[i]));
            workers[i]->enqueue(active,{});
        }
        notice=why;
        closing=std::move(active);
    }
    void recordEvent(const std::string& type,const std::string& who,const std::string& note) {
        try { active->event(type,who,ros::Time::now().toNSec(),note); }
        catch (const std::exception& e) { active->degraded=true; ROS_ERROR_STREAM(e.what()); }
    }
    void path(const std_msgs::StringConstPtr& msg) {
        auto pos=msg->data.find(' ');
        if (pos==std::string::npos) return;
        auto action=msg->data.substr(0,pos); fs::path bag(msg->data.substr(pos+1));
        if (bag.parent_path()!=directory || !std::regex_match(bag.filename().string(),allowed)) return;
        if (action=="OPEN" && bag.string()!=last_path) {
            close("新包开始前未收到上一包 CLOSED",true);
            last_path=bag.string();
            auto base=bag; base.replace_extension("");
            try {
                active=std::make_shared<Segment>(base.string(),ros::Time::now().toNSec(),camera_count);
                dropped[0]=dropped[1]=0; lost[0]=lost[1]=false; heartbeat=Clock::now();
                notice.clear();
            } catch(const std::exception& e) { notice=e.what(); ROS_ERROR_STREAM(notice); }
        } else if ((action=="CLOSED"||action=="CLOSE_FAILED") && bag.string()==last_path)
            close(action,action!="CLOSED");
        publish();
    }
    void bagStatus(const std_msgs::StringConstPtr& m) {
        heartbeat=Clock::now();
        if (active && m->data.find("healthy=False")!=std::string::npos) close("主包写入失败",true);
    }
    void onImage(int i,const Image& m) {
        const auto& c=cameras[i];
        if (m->encoding!="mono8" || m->header.frame_id!=c.frame || m->width!=c.width || m->height!=c.height ||
            m->step<m->width || m->data.size()!=size_t(m->step)*m->height || m->header.stamp.isZero()) return;
        last[i]=Clock::now();
        if (active && !workers[i]->enqueue(active,m)) {
            active->degraded=true;
            if (++dropped[i]==1) recordEvent("queue_full",c.name,std::to_string(m->header.stamp.toNSec()));
        }
    }
    void publish() {
        auto now=Clock::now();
        bool ready_now=true;
        std::string state;
        for (int i=0;i<camera_count;++i) {
            bool fresh=last[i]!=Clock::time_point{} && now-last[i]<std::chrono::seconds(1);
            ready_now=ready_now&&fresh;
            state+=cameras[i].name+std::string(fresh?" 就绪 ":" 无新帧 ");
            if (active && lost[i]==fresh) {
                if (!fresh) active->degraded=true;
                recordEvent(fresh?"stream_recovered":"stream_lost",cameras[i].name,"");
                lost[i]=!fresh;
            }
        }
        if (active && now-heartbeat>std::chrono::seconds(5))
            close("录包器退出",true);
        std_msgs::String m;
        m.data=state+(active?"录制中 "+fs::path(last_path).filename().string()+(active->degraded?"（存在缺帧或故障）":active->timing_gap?"（部分帧间隔偏长）":""):
                       closing && !closing->sealed ? "视频封存中" :
                       notice=="CLOSED" && closing ? (closing->degraded ? "已封存（存在缺帧或故障，请查看事件表）" : closing->timing_gap?"视频已封存（部分帧间隔偏长）":"视频已封存") :
                       notice.empty()?"待录包":notice);
        state_pub.publish(m);
        if (m.data!=last_state) {
            const auto tmp=directory/"video_state.tmp";
            std::ofstream output(tmp.string());
            output << m.data << '\n'; output.close();
            boost::system::error_code ec;
            if (output) fs::rename(tmp,directory/"video_state.txt",ec);
            ROS_INFO_STREAM(m.data);
            last_state=m.data;
        }
        std_msgs::Bool ready; ready.data=ready_now;ready_pub.publish(ready);
    }
public:
    Recorder() {
        std::string dir;
        if (!priv.getParam("directory",dir)) throw std::runtime_error("missing ~directory");
        directory=fs::canonical(dir);
        const auto prefix = priv.param<std::string>("bag_prefix", "test_indoor3_4");
        if (prefix != "test_indoor3_4" && prefix != "test_indoor3_5")
            throw std::runtime_error("unknown recording profile");
        allowed = std::regex(prefix + R"(_[0-9]{6,}\.bag)");
        camera_count = priv.param("ov13855_only", false) ? 1 : 2;
        ready_pub=priv.advertise<std_msgs::Bool>("ready",1,true);
        state_pub=priv.advertise<std_msgs::String>("status",1,true);
        for (int i=0;i<camera_count;++i) {
            workers[i].reset(new Worker(cameras[i]));
            images[i]=nh.subscribe<sensor_msgs::Image>(cameras[i].topic,cameras[i].fps,[this,i](const Image& m){onImage(i,m);},
                                                       ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
        }
        events=nh.subscribe("/rc_record/bag_path",12,&Recorder::path,this);
        status=nh.subscribe("/rc_record/status",4,&Recorder::bagStatus,this);
        timer=nh.createWallTimer(ros::WallDuration(.25),[this](const ros::WallTimerEvent&){publish();});
    }
    ~Recorder() { close("录像器退出时主包仍打开",true); }
};
int main(int argc,char** argv) {
    ros::init(argc,argv,"indoor3_4_video");
    try { Recorder recorder;ros::spin(); } catch(const std::exception& e) {ROS_FATAL_STREAM(e.what());return 1;}
    return 0;
}
