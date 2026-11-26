#!/usr/bin/env python3
"""Manual 22-second camera-only acceptance on an isolated ROS master; never starts control."""
import os, time, subprocess, signal, json, threading, socket
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11455'
os.environ['ROS_HOSTNAME']='127.0.0.1'
# Refuse to reuse or terminate another master.
s=socket.socket()
try:s.bind(('127.0.0.1',11455))
finally:s.close()
processes=[]; logs=[]
def start(args,name):
 f=open('/tmp/ov13855_'+name+'.log','w');logs.append(f)
 p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
try:
 master=start(['roscore','-p','11455'],'master');time.sleep(2)
 import rospy, numpy as np
 from sensor_msgs.msg import Image,CameraInfo
 from geometry_msgs.msg import PoseStamped
 from std_msgs.msg import Float64,String
 rospy.init_node('ov13855_acceptance',disable_signals=True)
 stamps=[];info_stamps=set(); ages=[];sizes=set(); times=[];statuses=[];poses=[];receive=[]
 def image(m):
  stamps.append(m.header.stamp.to_nsec());receive.append(time.monotonic());ages.append((rospy.Time.now()-m.header.stamp).to_sec())
  sizes.add((m.width,m.height,m.encoding,m.header.frame_id))
  if len(stamps)==1:
   a=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width]
   print('FIRST_IMAGE',json.dumps({'mean':float(a.mean()),'std':float(a.std()),'min':int(a.min()),'max':int(a.max())}),flush=True)
   import cv2
   cv2.imwrite('/tmp/ov13855_first_frame.png',a)
 subscribers=[rospy.Subscriber('/ov13855/image_raw',Image,image,queue_size=1,buff_size=16000000),
  rospy.Subscriber('/ov13855/camera_info',CameraInfo,lambda m:info_stamps.add(m.header.stamp.to_nsec()),queue_size=100),
  rospy.Subscriber('/front_marker/pose',PoseStamped,lambda m:poses.append(m),queue_size=100),
  rospy.Subscriber('/ov13855_apriltag_vision/debug/processing_time_ms',Float64,lambda m:times.append(m.data),queue_size=100),
  rospy.Subscriber('/ov13855_apriltag_vision/status',String,lambda m:statuses.append(m.data),queue_size=100)]
 camera=start(['roslaunch','ov13855_cam','ov13855.launch'],'camera')
 vision=start(['roslaunch','ov13855_apriltag_vision','ov13855_apriltag_vision.launch'],'vision')
 time.sleep(22)
 report={'image_count':len(stamps),'sizes':list(sizes),'matched_info':sum(t in info_stamps for t in stamps),
  'received_hz':(len(receive)-1)/(receive[-1]-receive[0]) if len(receive)>1 else 0,
  'stamp_hz':(len(stamps)-1)*1e9/(stamps[-1]-stamps[0]) if len(stamps)>1 else 0,
  'age_ms_p50_p95':np.percentile(ages,[50,95]).tolist() if ages else [],
  'processing_ms_p50_p95':np.percentile(times,[50,95]).tolist() if times else [],
  'processed_count':len(times),'pose_count':len(poses),'last_status':statuses[-5:]}
 report['age_ms_p50_p95']=[x*1000 for x in report['age_ms_p50_p95']]
 print(json.dumps(report,indent=2),flush=True)
 with open('/tmp/ov13855_live_results.json','w') as f:json.dump(report,f,indent=2)
finally:
 for p in reversed(processes):
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 for p in reversed(processes):
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
 for f in logs:f.close()
