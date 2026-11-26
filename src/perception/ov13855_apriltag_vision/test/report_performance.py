import json,csv,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(sys.argv[1]);r=json.loads((p/'results.json').read_text());samples=json.loads((p/'cpu_samples.json').read_text())
fields=['name','width','height','requested_fps','vision','success','image_hz','stamp_hz','processing_hz','pose_hz','camera_cpu_mean','camera_cpu_p95','vision_cpu_mean','vision_cpu_p95','monitor_cpu_mean','system_cpu_mean','system_cpu_p95','processing_ms_p50','processing_ms_p95','max_temperature_c','cpu_frequency_khz_min','cpu_frequency_khz_max']
flat=[]
for a in r:
 b={k:a.get(k) for k in fields};ss=[x for x in samples if x['name']==a['name']]
 for k in ['camera_cpu','vision_cpu','monitor_cpu','system_cpu']:
  for stat in ['mean','p95']:b[k+'_'+stat]=(a.get(k) or {}).get(stat)
 for i,s in enumerate(['p50','p95']):b['processing_ms_'+s]=(a.get('processing_ms_p50_p95') or [None,None])[i]
 ts=[v for x in ss for v in x['temperature_c'].values()];fs=[v for x in ss for v in x['frequency_khz'].values()]
 b['max_temperature_c']=max(ts) if ts else None;b['cpu_frequency_khz_min']=min(fs) if fs else None;b['cpu_frequency_khz_max']=max(fs) if fs else None
 flat.append(b)
with (p/'summary.csv').open('w') as f:
 w=csv.DictWriter(f,fields,extrasaction='ignore');w.writeheader();w.writerows(flat)
with (p/'cpu_samples.csv').open('w') as f:
 keys=['name','elapsed','camera_cpu','vision_cpu','monitor_cpu','system_cpu','temperature_c','frequency_khz'];w=csv.DictWriter(f,keys);w.writeheader();w.writerows(samples)
fig,axes=plt.subplots(2,2,figsize=(14,9))
modes=[(640,480),(1280,720),(1920,1080),(2560,1440),(2560,1920),(3840,2160),(4224,3136)]
for j,fps in enumerate([15,30]):
 for vision in [False,True]:
  rows=[next((a for a in r if (a['width'],a['height'])==m and a['requested_fps']==fps and a['label']==('vision' if vision else 'camera')),None) for m in modes]
  label='Camera + AprilTag' if vision else 'Camera only'
  axes[0,j].plot(range(7),[a.get('image_hz',np.nan) if a else np.nan for a in rows],'-o',label=label)
  axes[1,j].plot(range(7),[sum((a.get(k) or {}).get('mean',0) for k in ['camera_cpu','vision_cpu']) if a else np.nan for a in rows],'-o',label=label)
 axes[0,j].axhline(fps,color='gray',linestyle=':',label='Requested fps');axes[0,j].set_title(f'Requested {fps} fps');axes[0,j].set_ylabel('Received image fps');axes[1,j].set_ylabel('Node CPU % (100% = one core)')
 for ax in axes[:,j]:ax.set_xticks(range(7));ax.set_xticklabels([f'{w}x{h}' for w,h in modes],rotation=35);ax.grid(alpha=.3);ax.legend()
fig.tight_layout();fig.savefig(p/'comparison.png',dpi=150)
text='# OV13855 帧率与 CPU 实测（2026-09-22）\n\n每组预热5秒，测量30秒；全部在板端，独立ROS master，不运行控制，不订阅标注图。NV12→mono8；检测最长边1280、4线程、quad_decimate=1.5。进程CPU 100%为一核，整机CPU归一化到全部核心。正式配置未修改，临时CameraInfo按宽高分别缩放，仅用于性能测试。\n\n|组合|实际图像Hz|检测Hz|Pose Hz|相机CPU均值/P95|视觉CPU均值/P95|整机CPU均值/P95|最高温度℃|\n|---|---:|---:|---:|---|---|---|---:|\n'
def cpu(a,k):
 v=a.get(k);return f"{v['mean']:.1f}/{v['p95']:.1f}%" if v else '—'
for a,b in zip(r,flat):text+=f"|{a['name']}|{a.get('image_hz',0):.2f}|{a.get('processing_hz',0):.2f}|{a.get('pose_hz',0):.2f}|{cpu(a,'camera_cpu')}|{cpu(a,'vision_cpu')}|{cpu(a,'system_cpu')}|{b['max_temperature_c']}|\n"
text+='\n原始results.json含每帧时间、尺寸、状态与处理时间；cpu_samples.json/CSV含每秒CPU/温度/频率；summary.csv含均值和P95；各camera.log记录实际协商caps，requested_fps不能等同于实测输出。无Pose的检测组不代表完成同等位姿负载。初次监测脚本使用错误ROS Master接口，失败组保留在相邻performance_20260922目录，不纳入本报告。\n'
(p/'REPORT.md').write_text(text)
print((p/'summary.csv').read_text())
