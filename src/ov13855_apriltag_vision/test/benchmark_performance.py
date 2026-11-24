#!/usr/bin/env python3
"""Board-only isolated ROS benchmark. No flight/control nodes; restores camera mode."""
import os,time,json,signal,subprocess,socket,copy,statistics,csv,sys
from pathlib import Path
import yaml,numpy as np
ROOT = Path('/home/orangepi/orange')
if len(sys.argv) > 1:
    OUT = Path(sys.argv[1]).expanduser().resolve()
else:
    OUT = Path.home() / 'test_data' / 'ov13855_apriltag_vision' / time.strftime('performance_%Y%m%d_%H%M%S')
OUT.mkdir(parents=True, exist_ok=False)
PORT=11456
s=socket.socket();s.bind(('127.0.0.1',PORT));s.close()
if subprocess.run(['fuser','/dev/video11'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0: raise RuntimeError('Camera occupied')
os.environ.update(ROS_MASTER_URI=f'http://127.0.0.1:{PORT}',ROS_HOSTNAME='127.0.0.1')
owned=[]; handles=[]
def start(args,name):
 f=open(OUT/(name+'.log'),'w');handles.append(f)
 p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);owned.append(p);return p
def stop(p):
 if p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(8)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGTERM);p.wait(5)
 if p in owned:owned.remove(p)
def ticks(pid):
 try:
  x=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split();return int(x[11])+int(x[12])
 except (OSError,ValueError):return None
def system():
 x=list(map(int,Path('/proc/stat').read_text().splitlines()[0].split()[1:9]));return sum(x),x[3]+x[4]
def hardware():
 temp={p.parent.name+':'+(p.parent/'type').read_text().strip():int(p.read_text())/1000 for p in Path('/sys/class/thermal').glob('thermal_zone*/temp')}
 freq={p.parent.name:int(p.read_text()) for p in Path('/sys/devices/system/cpu/cpufreq').glob('policy*/scaling_cur_freq')}
 return temp,freq
cam=yaml.safe_load((ROOT/'src/ov13855_cam/config/ov13855.yaml').read_text());cal=yaml.safe_load((ROOT/'src/ov13855_cam/config/ov13855_2560x1440.yaml').read_text())
rows=[];samples=[];hz=os.sysconf('SC_CLK_TCK');master=None
try:
 master=start(['roscore','-p',str(PORT)],'master');time.sleep(2)
 import rospy,rosnode,rosgraph,xmlrpc.client
 from sensor_msgs.msg import Image
 from geometry_msgs.msg import PoseStamped
 from std_msgs.msg import Float64,String
 rospy.init_node('ov13855_performance_monitor',disable_signals=True)
 state={'active':False}
 def image(m):
  if state['active']:state['images'].append((time.monotonic(),m.header.stamp.to_sec(),m.width,m.height,m.encoding))
 def processing(m):
  if state['active']:state['processing'].append((time.monotonic(),m.data))
 def pose(m):
  if state['active']:state['poses'].append(time.monotonic())
 def status(m):
  if state['active']:state['statuses'].append(m.data)
 subs=[rospy.Subscriber('/ov13855/image_raw',Image,image,queue_size=1,buff_size=32*1024*1024),rospy.Subscriber('/ov13855_apriltag_vision/debug/processing_time_ms',Float64,processing,queue_size=100),rospy.Subscriber('/front_marker/pose',PoseStamped,pose,queue_size=100),rospy.Subscriber('/ov13855_apriltag_vision/status',String,status,queue_size=100)]
 def pid_for(name):
  uri=rosnode.get_api_uri(rosgraph.Master('/ov13855_performance_monitor'),name,skip_cache=True)
  return xmlrpc.client.ServerProxy(uri).getPid('/ov13855_performance_monitor')[2]
 def run(w,h,fps,vision,label):
  idx=len(rows);name=f'{idx:02d}_{w}x{h}_{fps}_{label}';cfg=copy.deepcopy(cam);cc=copy.deepcopy(cal)
  cc.update(image_width=w,image_height=h)
  for key,stride in [('camera_matrix',3),('projection_matrix',4)]:
   for j in range(stride):cc[key]['data'][j]*=w/2560;cc[key]['data'][stride+j]*=h/1440
  cp=OUT/(name+'_calibration.yaml');cp.write_text(yaml.safe_dump(cc));cfg.update(image_width=w,image_height=h,framerate=fps,camera_info_url='file://'+str(cp));conf=OUT/(name+'_camera.yaml');conf.write_text(yaml.safe_dump(cfg))
  processes=[];r={'name':name,'width':w,'height':h,'requested_fps':fps,'vision':vision,'label':label}
  state.update(active=False,images=[],processing=[],poses=[],statuses=[])
  try:
   processes.append(start(['roslaunch','ov13855_cam','ov13855.launch','config_file:='+str(conf)],name+'_camera'))
   if vision:processes.append(start(['roslaunch','ov13855_apriltag_vision','ov13855_apriltag_vision.launch'],name+'_vision'))
   deadline=time.monotonic()+20;pids={}
   while time.monotonic()<deadline:
    try:
     pids={'camera':pid_for('/ov13855_cam'),'monitor':os.getpid()}
     if vision:pids['vision']=pid_for('/ov13855_apriltag_vision_node')
     break
    except Exception:time.sleep(.25)
   if 'camera' not in pids or (vision and 'vision' not in pids):raise RuntimeError('node startup failed')
   time.sleep(5)
   r['device']=subprocess.run(['v4l2-ctl','-d','/dev/video11','--get-fmt-video'],capture_output=True,text=True).stdout
   starttime=time.monotonic();prevtime=starttime;prev={k:ticks(v) for k,v in pids.items()};ps=system();state['active']=True;local=[]
   for n in range(30):
    time.sleep(max(0,starttime+n+1-time.monotonic()));now=time.monotonic();cur={k:ticks(v) for k,v in pids.items()};sy=system();dt=now-prevtime
    a={'name':name,'elapsed':now-starttime,'system_cpu':100*(1-(sy[1]-ps[1])/(sy[0]-ps[0]))}
    for k in pids:a[k+'_cpu']=None if cur[k] is None or prev[k] is None else 100*(cur[k]-prev[k])/hz/dt
    a['temperature_c'],a['frequency_khz']=hardware();local.append(a);samples.append(a);prevtime=now;prev=cur;ps=sy
   state['active']=False;r['duration']=time.monotonic()-starttime
   for key in ['images','processing','poses','statuses']:r[key]=list(state[key])
   def rate(v):return (len(v)-1)/(v[-1]-v[0]) if len(v)>1 and v[-1]>v[0] else 0
   im=r['images'];r['image_hz']=rate([a[0] for a in im]);r['stamp_hz']=rate([a[1] for a in im]);r['image_count']=len(im);r['processing_hz']=len(r['processing'])/r['duration'];r['pose_hz']=len(r['poses'])/r['duration']
   r['interval_ms_p50_p95']=np.percentile(np.diff([a[0] for a in im])*1000,[50,95]).tolist() if len(im)>1 else []
   r['processing_ms_p50_p95']=np.percentile([a[1] for a in r['processing']],[50,95]).tolist() if r['processing'] else []
   r['actual_modes']=list({(a[2],a[3],a[4]) for a in im})
   for k in ['camera_cpu','vision_cpu','monitor_cpu','system_cpu']:
    values=[a[k] for a in local if a.get(k) is not None];r[k]={'mean':float(np.mean(values)),'p95':float(np.percentile(values,95))} if values else None
   r['success']=bool(im) and all(a[2:4]==(w,h) for a in im)
  except Exception as e:r['error']=str(e);r['success']=False
  finally:
   state['active']=False
   for p in reversed(processes):stop(p)
  rows.append(r);(OUT/'results.json').write_text(json.dumps(rows,indent=2));(OUT/'cpu_samples.json').write_text(json.dumps(samples,indent=2))
  print(json.dumps({k:v for k,v in r.items() if k not in ['images','processing','poses','statuses','device']}),flush=True)
 run(2560,1440,15,True,'baseline_start')
 for w,h in [(640,480),(1280,720),(1920,1080),(2560,1440),(2560,1920),(3840,2160),(4224,3136)]:
  for fps in [15,30]:
   for vision in [False,True]:run(w,h,fps,vision,'vision' if vision else 'camera')
 run(2560,1440,15,True,'baseline_end')
 first,last=rows[0]['image_hz'],rows[-1]['image_hz']
 if first and abs(last/first-1)>.1:run(2560,1440,15,True,'baseline_recheck')
finally:
 for p in list(reversed(owned)):
  if p is not master:stop(p)
 # Restore through the original ROS configuration, then release device.
 if master is not None and master.poll() is None:
  p=start(['roslaunch','ov13855_cam','ov13855.launch'],'restore_camera');time.sleep(5);stop(p)
 if master is not None:stop(master)
 for f in handles:f.close()
