#!/usr/bin/env python3
"""Score one archived CH6 segment from indoor3_5; needs OpenCV contrib, numpy and ffmpeg."""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import numpy as np
import cv2

NS = 1_000_000_000


def write_csv(path, rows, columns):
    with path.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=columns)
        w.writeheader()
        w.writerows(rows)


def percentile(values, q):
    finite = [v for v in values if v is not None and math.isfinite(v)]
    return float(np.percentile(finite,q)) if finite else None


def markers(image, dictionary):
    # Native mono pixels for all measurements. Downsample only for finding markers in 13 MP stills.
    scale = min(1., 1920/image.shape[1])
    view = cv2.resize(image,None,fx=scale,fy=scale,interpolation=cv2.INTER_AREA) if scale<1 else image
    corners,ids,_ = cv2.aruco.detectMarkers(view,dictionary)
    result = {}
    if ids is not None:
        for idx,c in zip(ids.ravel(),corners):
            # The shelf labels are upright. Order their physical top and bottom edges in the image.
            ordered = c.reshape(4,2)[np.argsort(c.reshape(4,2)[:,1])]
            top = ordered[:2][np.argsort(ordered[:2,0])]
            bot = ordered[2:][np.argsort(ordered[2:,0])[::-1]]
            result[int(idx)] = np.float32(np.vstack((top,bot))/scale)
    return result


def edge_width(view, cx, cy, side, center):
    # Dark-to-light 10–90% transition. Run on the transpose for the top tape.
    widths = []
    for y in np.rint(cy+side*np.linspace(-.30,.30,11)).astype(int):
        mid = cx+side*center
        x0, x1 = math.floor(mid-.85*side), math.ceil(mid+.85*side)+1
        if not (0<=x0<x1<=view.shape[1] and 1<=y<view.shape[0]-1):
            continue
        v = np.median(view[y-1:y+2,x0:x1],axis=0)
        v = cv2.GaussianBlur(v[None,:],(5,1),.65).ravel()
        x = np.arange(len(v))
        allowed = np.flatnonzero(np.abs(x+x0-mid)<.30*side)
        if not len(allowed):
            continue
        peak = allowed[np.argmax(np.gradient(v)[allowed])]
        rel = (x-peak)/side
        lo = np.median(v[(rel>=-.39)&(rel<=-.23)])
        hi = np.median(v[(rel>=.23)&(rel<=.39)])
        if not hi-lo>=20:
            continue
        norm = (v-lo)/(hi-lo)
        roots = []
        for h in (.1,.5,.9):
            k = np.flatnonzero((norm[:-1]<=h)&(norm[1:]>h)&(abs(rel[:-1])<.5))
            if not len(k):
                break
            roots_at = k+(h-norm[k])/(norm[k+1]-norm[k])
            roots.append(roots_at[np.argmin(abs(roots_at-peak))])
        if len(roots)==3 and roots[0]<roots[1]<roots[2]:
            widths.append(roots[2]-roots[0])
    return percentile(widths,50) if len(widths)>=6 else None


def label(image, quad, decoder):
    x,y = quad.mean(axis=0)
    side = float(np.linalg.norm(quad-np.roll(quad,1,axis=0),axis=1).mean())
    dest = np.float32([[102,172],[202,172],[202,272],[102,272]])
    warp = cv2.getPerspectiveTransform(quad,dest)
    patch = cv2.warpPerspective(image,warp,(305,280),flags=cv2.INTER_CUBIC,borderValue=100)[:151,80:224].copy()
    lo,hi = np.percentile(patch,[5,95])
    norm = np.uint8(np.clip((patch.astype(float)-lo)*255/max(hi-lo,1),0,255))
    for candidate in (patch,norm,cv2.addWeighted(norm,6,cv2.GaussianBlur(norm,(0,0),3.4),-5,0)):
        values,_ = decoder.detectAndDecode(candidate)
        if values:
            break
    return dict(side_px=side,h_px=edge_width(image,x,y,side,-1.16),
                v_px=edge_width(image.T,y,x,side,-2.2),qr='|'.join(values))


def motion(before, after):
    """Center translation and row-dependent residual from a whole-frame similarity fit."""
    if before is None:
        return None,None,None
    p = cv2.goodFeaturesToTrack(before,220,.015,9)
    if p is None or len(p)<16:
        return None,None,None
    q,ok,_ = cv2.calcOpticalFlowPyrLK(before,after,p,None,winSize=(21,21),maxLevel=3)
    p,q = p[ok.ravel()==1].reshape(-1,2),q[ok.ravel()==1].reshape(-1,2)
    if len(p)<16:
        return None,None,None
    mat, used = cv2.estimateAffinePartial2D(p,q,method=cv2.RANSAC,ransacReprojThreshold=1.25)
    if mat is None or used.sum()<16:
        return None,None,None
    residual = 2*(q-(p@mat[:,:2].T+mat[:,2]))
    bands=[]
    for block in np.array_split(np.argsort(p[:,1]),4):
        if len(block)>=5:
            bands.append(np.median(residual[block],axis=0))
    spread = float(np.max(np.linalg.norm(np.asarray(bands)-np.median(bands,axis=0),axis=1))) if len(bands)==4 else None
    center = np.array([after.shape[1]/2,after.shape[0]/2])
    return mat,2*(mat@np.r_[center,1]-center),spread


def coverage(stamps, start, end):
    return len(stamps)>=2 and stamps[0]-start<=250_000_000 and end-stamps[-1]<=250_000_000 and max(np.diff(stamps))<=250_000_000


def score_target(records,point,leg):
    steady = [r for r in records if r['phase']=='steady']
    brake = [r for r in records if r['phase']=='brake']
    result = dict(id=records[0]['target'] if records else None,frames=len(steady),
                  tag_hits=sum(r['hit'] for r in steady),qr_hits=sum(r['correct'] for r in steady))
    for key in ('h_px','v_px','side_px','jitter_px'):
        values = [r[key] for r in steady if r[key] is not None]
        result[key] = dict(n=len(values),median=percentile(values,50),p95=percentile(values,95))
    t0 = point['t0_ns']
    a,b = t0+1_500_000_000,t0+5*NS
    odom = [r for r in leg.get('odom_samples',[]) if a<=r[0]<b]
    stable = coverage([r[0] for r in odom],a,b) and all(r[1]<=.15 and r[2]<=.1 and r[3]<=5 for r in odom)
    result.update(steady_odom=bool(stable), steady_complete=not point['steady_gaps'] and bool(point.get('score_end_ns') and point['score_end_ns']>=b),
                  max_speed_m_s=max((r[2] for r in odom),default=None))
    keys=('h_px','v_px','jitter_px')
    valid_ref = result['steady_complete'] and stable and len(steady)>0 and all(result[k]['n']>=.6*len(steady) for k in keys)
    result['recovery_s'] = '不可估计'
    if valid_ref and brake and not point['brake_gaps']:
        streak=0
        valid=0
        for r in brake:
            measurable=all(r[k] is not None for k in keys)
            valid+=measurable
            if measurable and all(r[k]<=result[k]['p95'] for k in keys):
                if streak==0:
                    first=r['stamp_ns']
                streak+=1
                if streak>=3:
                    result['recovery_s']=(first-t0)/NS
                    break
            else:
                streak=0
        else:
            if valid==len(brake):
                result['recovery_s']='>1.5'
    return result


def load_segment(run):
    manifest=json.loads((run/'manifest.json').read_text())
    info=manifest['video']['damper_test']
    stem=manifest['run_id'].split('/')[-1][7:]
    video=run/(stem+'_ov13855.mkv')
    with (run/(stem+'_frames.csv')).open() as f:
        frames=list(csv.DictReader(f))
    meta=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0','-show_streams','-show_packets','-show_entries','stream=codec_name,pix_fmt,width,height,time_base:packet=pts','-of','json',str(video)]))
    stream=meta['streams'][0]
    if (stream['width'],stream['height'],stream['codec_name'],stream['pix_fmt'],stream['time_base'])!=(1280,960,'ffvhuff','gray','1/1000'):
        raise ValueError('视频格式不符')
    if [int(r['pts_ms']) for r in frames]!=[int(p['pts']) for p in meta['packets']] or any(int(r['frame_index'])!=i for i,r in enumerate(frames)):
        raise ValueError('视频索引与帧 CSV 不匹配')
    return manifest, info, video, frames


def frame_metrics(video,frames,points,dictionary,decoder):
    features=[]; traces=[]
    before=last_delta=last_seq=last_ns=None
    tracks={}
    cmd=['ffmpeg','-v','error','-threads','2','-i',str(video),'-vsync','0','-enc_time_base','1:1000','-pix_fmt','gray','-f','rawvideo','-']
    with subprocess.Popen(cmd,stdout=subprocess.PIPE) as proc:
        for i,row in enumerate(frames):
            stamp=int(row['source_stamp_ns']); seq=int(row['source_seq'])
            pixels=proc.stdout.read(1280*960)
            if len(pixels)!=1280*960:
                raise ValueError('视频帧解码失败')
            im=np.frombuffer(pixels,np.uint8).reshape(960,1280)
            small=cv2.pyrDown(im)
            if last_seq is not None and (seq!=last_seq+1 or stamp-last_ns>100_000_000):
                before=last_delta=None
                tracks.clear()
            mat,delta,spread=motion(before,small)
            jitter=float(np.linalg.norm(delta-last_delta)/2) if delta is not None and last_delta is not None else None
            before,last_delta,last_seq,last_ns=small,delta,seq,stamp
            point=next(((p,l,goal) for p,l,goal in points if p['sent_ns']<=stamp and
                        stamp<(p['t0_ns']+5*NS if p['t0_ns'] is not None else round(l.get('end_ros',l['sent_ros'])*NS))),None)
            phase='other'; index=None; goal=None
            if point:
                p,leg,goal=point;index=p['index'];t0=p['t0_ns']
                phase='move' if t0 is None or stamp<t0 else ('brake' if stamp<t0+1_500_000_000 else 'steady')
            traces.append(dict(frame=i,pts_ms=int(row['pts_ms']),stamp_ns=stamp,point=index,phase=phase,
                               dx=None if delta is None else float(delta[0]),dy=None if delta is None else float(delta[1]),
                               jitter_px=jitter,band_residual_px=spread))
            if phase not in ('brake','steady'):
                tracks.clear()
                continue
            seen=markers(im,dictionary)
            loose_qr=None
            for idx in p['expected']:
                quad=seen.get(idx); detected=quad is not None
                if detected:
                    tracks[idx]=(stamp,quad)
                elif idx in tracks and mat is not None and stamp-tracks[idx][0]<=250_000_000:
                    anchor,old=tracks[idx]
                    quad=np.float32(old@mat[:,:2].T+2*mat[:,2])
                    tracks[idx]=(anchor,quad)
                else:
                    tracks.pop(idx,None)
                values=label(im,quad,decoder) if quad is not None else dict(side_px=None,h_px=None,v_px=None,qr='')
                expected='BOX-%02d'%idx
                if expected not in values['qr'].split('|'):
                    if loose_qr is None:
                        loose_qr,_=decoder.detectAndDecode(im)
                    if expected in loose_qr:
                        values['qr']=expected
                features.append(dict(goal=goal,point=index,target=idx,phase=phase,frame=i,stamp_ns=stamp,hit=detected,
                                     correct=('BOX-%02d'%idx in values['qr'].split('|')),jitter_px=jitter,**values))
        if proc.stdout.read(1) or proc.wait()!=0:
            raise ValueError('视频帧数异常')
    return features,traces


def point_results(run,points,features,dictionary,decoder):
    summary=[]
    photos=[]
    for p,leg,goal in points:
        item=dict(goal=goal,point=p['index'],t0_ns=p['t0_ns'], targets=[])
        for idx in p['expected']:
            group=[r for r in features if r['goal']==goal and r['point']==p['index'] and r['target']==idx]
            item['targets'].append(dict(score_target(group,p,leg),id=idx) if p['t0_ns'] is not None else dict(id=idx,frames=0,recovery_s='不可估计'))
        for photo in p['photos']:
            path=run/photo.get('file','')
            if photo['status']!='saved' or not path.is_file():
                photos.append(dict(point=p['index'],offset=photo['offset_s'],status=photo['status'] if photo['status']!='saved' else 'missing_file'))
                continue
            im=cv2.imread(str(path),cv2.IMREAD_UNCHANGED)
            if im is None or im.shape!=(3136,4224) or im.dtype!=np.uint8:
                raise ValueError('全分辨率照片不完整 '+str(path))
            seen=markers(im,dictionary)
            direct_qr=None
            for idx in p['expected']:
                value=label(im,seen[idx],decoder) if idx in seen else dict(qr='')
                expected='BOX-%02d'%idx
                if expected not in value['qr'].split('|'):
                    if direct_qr is None:
                        direct_qr,_=decoder.detectAndDecode(im)
                    if expected in direct_qr:
                        value['qr']=expected
                photos.append(dict(point=p['index'],offset=photo['offset_s'],status=photo['status'],target=idx,
                                   tag=idx in seen,qr=value['qr'],correct='BOX-%02d'%idx in value['qr'].split('|'),
                                   file=path.name,frame=photo['frame_index'],delta_ms=photo['delta_ms']))
        summary.append(item)
    return summary,photos


def write_report(out,manifest,info,points,summary,traces,candidates):
    lines=['# 四点减振评分','',info['kind']+' · '+manifest['run_id']+' · 减振球 '+str(info['hardness']),'','[原速回放](回放.mp4) · [逐帧指标](labels.csv) · [光流](flow.csv) · [全分辨率照片读码](photos.csv)','','|点|ID|Tag / 帧|QR / 帧|横向中位 / P95（px）|竖向中位 / P95（px）|黑框中位（px）|余振（s）|窗口与里程计|','|---|---|---|---|---|---|---|---|---|']
    def fmt(x):
        return '—' if x is None else ('%.2f'%x if isinstance(x,float) else str(x))
    for pt in summary:
        for t in pt['targets']:
            metrics=[fmt(t.get(k,{}).get('median'))+' / '+fmt(t.get(k,{}).get('p95')) for k in ('h_px','v_px')]
            flags='完整、停稳' if t.get('steady_complete') and t.get('steady_odom') else '窗口缺失或里程计未持续停稳'
            lines.append('| '+' | '.join(map(str,[pt['point'],t['id'],f"{t.get('tag_hits',0)}/{t['frames']}",f"{t.get('qr_hits',0)}/{t['frames']}",*metrics,fmt(t.get('side_px',{}).get('median')),fmt(t['recovery_s']),flags]))+' |')
    missing=sorted(set(range(1,5))-{p['point'] for p in summary})
    if missing:
        lines.append('\n本趟未到达的点位：'+', '.join(map(str,missing)))
    for p,leg,goal in points:
        moving=[r for r in traces if r['point']==p['index'] and r['phase']=='move']
        if moving:
            lines.append(f"\n前往第 {p['index']} 点的移动段：视频 PTS {moving[0]['pts_ms']/1000:.2f}–{moving[-1]['pts_ms']/1000:.2f} s。")
    lines+=['',f'移动段待回看 {len(candidates)} 帧，详见 [评分与候选时刻](score.json)。超过 3 px 的位移短周期变化或超过 2 px 的分带残差仅用于标记。未获得足够纸边、光流或有效稳态参考时显示不可估计。']
    (out/'总结.md').write_text('\n'.join(lines)+'\n')



def analyze(run,out):
    manifest,info,video,frames=load_segment(run)
    out.mkdir(parents=True,exist_ok=True)
    cv2.setNumThreads(1)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    decoder=cv2.wechat_qrcode_WeChatQRCode()
    points=[]
    for route in info['routes']:
        raw=json.loads((run/route['goal']).read_text())
        for point,leg in zip(route['points'],raw['legs']):
            points.append((point,leg,route['goal']))
    features,traces=frame_metrics(video,frames,points,dictionary,decoder)
    summary,photos=point_results(run,points,features,dictionary,decoder)
    write_csv(out/'labels.csv',features,['goal','point','target','phase','frame','stamp_ns','hit','correct','jitter_px','side_px','h_px','v_px','qr'])
    write_csv(out/'flow.csv',traces,['frame','pts_ms','stamp_ns','point','phase','dx','dy','jitter_px','band_residual_px'])
    write_csv(out/'photos.csv',photos,['point','offset','status','target','tag','qr','correct','file','frame','delta_ms'])
    candidates=[dict(frame=r['frame'],pts_ms=r['pts_ms'],point=r['point'],jitter_px=r['jitter_px'],band_residual_px=r['band_residual_px'])
                for r in traces if r['phase']=='move' and ((r['jitter_px'] or 0)>3 or (r['band_residual_px'] or 0)>2)]
    (out/'score.json').write_text(json.dumps(dict(run=manifest['run_id'],kind=info['kind'],hardness=info['hardness'],
        segment_complete=manifest['video']['complete'],missing_points=sorted(set(range(1,5))-{p['point'] for p in summary}),points=summary,motion_review=candidates),ensure_ascii=False,indent=2,allow_nan=False))
    # Preserve original timing (including capture gaps) on a player-friendly full-length proxy.
    subprocess.run(['ffmpeg','-v','error','-y','-i',str(video),'-fps_mode','passthrough','-c:v','libx264','-threads','3','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(out/'回放.mp4')],check=True)
    write_report(out,manifest,info,points,summary,traces,candidates)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('out',type=Path)
    args=parser.parse_args()
    analyze(args.run,args.out)
