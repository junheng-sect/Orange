#!/usr/bin/env python3
"""Export test12 DLL-minus-FAST-LIO2 differences from an unmodified Channel 6 bag."""
import argparse
from collections import Counter, deque
import csv
import itertools
import json
from pathlib import Path
import sys
import tempfile

import numpy as np
import rosbag

from dll_drift import BASE, DLL, HEALTH, MODE, ODOM, RELOC, RESULT, NS, TOPICS, Comparison

METRICS = ("dx_m", "dy_m", "dz_m", "horizontal_m", "yaw_deg")
POSITION = ("base_x", "base_y", "base_z", "dll_x", "dll_y", "dll_z")


def bag_events(bag):
    def scan_order(event):
        topic, msg, received = event
        if topic in (RESULT, RELOC):
            try:
                return round(json.loads(msg.data)["stamp"]*NS), 0
            except (KeyError, TypeError, ValueError):
                return received.to_nsec(), 0
        if topic == "/tf":
            return max((t.header.stamp.to_nsec() for t in msg.transforms), default=0), 1
        if topic == MODE:
            return received.to_nsec(), 0
        # A wall watchdog may publish STALE at the last scan's stamp after /clock
        # freezes. Keep the prior measured scan before that receipt-time tie.
        stale = topic == HEALTH and any(s.level == 3 for s in msg.status)
        return msg.header.stamp.to_nsec(), 2 if stale else 1

    events = bag.read_messages(topics=list(TOPICS))
    for _, group in itertools.groupby(events, key=lambda item:item.timestamp.to_nsec()):
        batch = list(group)
        yield from sorted(batch, key=scan_order) if len(batch) > 1 else batch


def read_rows(bag):
    comparison = Comparison()
    pending = deque()
    segment = 0
    previous = None
    seeds = {}
    rows = []

    def finish():
        stamp, _ = pending.popleft()
        row = comparison.measure(stamp)
        row["segment"] = segment
        frame = comparison.frames.get(stamp, {})
        for name, key in (("base", BASE), ("dll", DLL)):
            if key in frame:
                row.update(zip((name+"_"+axis for axis in "xyz"), frame[key][0]))
        rows.append(row)

    for topic, msg, received in bag_events(bag):
        now = received.to_nsec()
        while pending and now-pending[0][1] > .5*NS:
            finish()
        # Close old frames before replacing their session evidence.
        if topic == RESULT:
            try:
                value = json.loads(msg.data)
                if not isinstance(value,dict):
                    value = {}
            except ValueError:
                value = {}
            signature = (value.get("map"), value.get("stamp"), value.get("confirmed"), value.get("ok"),
                         value.get("transform"))
            if signature != previous:
                while pending:
                    finish()
                segment += 1
                previous = signature
                if value.get("confirmed") and value.get("ok"):
                    seeds[segment] = value
        if topic in (RELOC, MODE):
            try:
                info = json.loads(msg.data)
            except ValueError:
                info = {}
            if topic == RELOC and not info.get('valid'):
                until = round(float(info.get('stamp', 0))*NS)
                while pending and pending[0][0] < until:
                    finish()
            if topic == MODE and comparison.run and info.get('run') != comparison.run:
                while pending:
                    finish()
                segment += 1
        if topic == ODOM and msg.header.stamp.to_nsec() < comparison.latest_odom:
            while pending:
                finish()
            segment += 1
        try:
            comparison.feed(topic, msg, now)
        except (ValueError, KeyError, TypeError, AttributeError, OverflowError):
            comparison.reset()
        if topic == ODOM and (msg.header.frame_id, msg.child_frame_id) == ("drone_odom", "drone_base"):
            stamp = msg.header.stamp.to_nsec()
            if stamp > 0 and (not pending or pending[-1][0] != stamp):
                pending.append((stamp, now))
        # ROS time may freeze at bag end. A later watchdog event must not erase
        # a frame for which both simultaneous measurements were already valid.
        while pending and comparison.measure(pending[0][0])['valid']:
            finish()
    while pending:
        finish()
    return rows, seeds


def summary(rows):
    valid = [r for r in rows if r['valid']]
    stats = {}
    for name in ('horizontal_m','dz_m','yaw_deg'):
        vals = [abs(row[name]) for row in valid]
        key = name if name == 'horizontal_m' else 'abs_'+name
        stats[key] = dict(zip(('median','p95','max'), map(float,np.percentile(vals,[50,95,100])))) if vals else None
    gaps = []
    for (seg, ok), group in itertools.groupby(rows, key=lambda r:(r['segment'], r['valid'])):
        if not ok:
            group=list(group)
            gaps.append(dict(segment=seg, first_stamp_ns=group[0]['stamp_ns'],last_stamp_ns=group[-1]['stamp_ns'],
                             frames=len(group),reasons=dict(Counter(r['reason'] for r in group))))
    return dict(frames=len(rows), valid_frames=len(valid), valid_fraction=len(valid)/len(rows) if rows else None,
                statistics=stats, unavailable_intervals=gaps, direction='DLL minus FAST-LIO2 in drone_map',
                meaning='相对先验地图的漂移指标')


def checkpoints(bag, folder, rows, seeds):
    # Use the same multi-start local reference and motion checks as the A-B-A analysis.
    import rospkg
    import hashlib
    src = Path(__file__).resolve().parents[2]/'src/fastlio_dll/scripts'
    sys.path.insert(0,str(src))
    from aba_reference import static_cloud, register
    from analyze_aba import sensors, stops, initial_guess, window_pose, difference
    from drone_relocalization.geometry import quat_R, yaw, unpack
    from drone_relocalization.session import BagSession

    laser, received, coverage = sensors(bag)
    intervals, recorded = stops(bag)
    if not intervals:
        return dict(raw_coverage=coverage, stops=[])
    capture = BagSession(bag.filename)
    result = []
    try:
        pcd_hash = hashlib.sha256((folder/'map_world.pcd').read_bytes()).hexdigest()
        meta = json.loads((folder/'meta.json').read_text())
        crop = meta.get('shared_outdoor_crop',{})
        source = crop.get('source',{})
        manifest = json.loads((folder/'manifest.json').read_text())
        if manifest['sha256']['map_world.pcd'] != pcd_hash or manifest['sha256']['meta.json'] != hashlib.sha256((folder/'meta.json').read_bytes()).hexdigest():
            raise ValueError('ICP 地图哈希不符')
        # Bind the evaluation crop to the source map used for the recorded initialization.
        verified = set()
        calib = Path(rospkg.RosPack().get_path('fast_lio'))/'config/mid360s.yaml'
        for lo, hi in intervals:
            mid=(lo+hi)/2
            obj=dict(start_s=lo-laser[0],end_s=hi-laser[0],valid=False)
            result.append(obj)
            try:
                interval=[r for r in rows if lo<=r['stamp_ns']/NS<=hi]
                sessions={r['segment'] for r in interval if r['segment'] in seeds}
                if len(sessions)!=1:
                    raise ValueError('检查点跨会话或缺少初值')
                seed=seeds[next(iter(sessions))]
                original=Path(seed['map'])
                if original not in verified:
                    if crop.get('frame_id')!='drone_map' or any(hashlib.sha256((original/n).read_bytes()).hexdigest()!=source[k]
                            for n,k in [('map_world.pcd','map_sha256'),('meta.json','meta_sha256')]):
                        raise ValueError('ICP 子图与主定位地图坐标来源不一致')
                    verified.add(original)
                use=(laser>=mid-2.2)&(laser<=mid+2.2)
                held=recorded[(recorded[:,0]>=mid-2)&(recorded[:,0]<mid+2)]
                if (not use.any() or len(held)<30 or np.linalg.norm(np.ptp(held[:,1:4],axis=0))>.07
                        or min(abs(held[:,4:] @ held[len(held)//2,4:]))<np.cos(np.deg2rad(1))):
                    raise ValueError('检查点未稳定')
                cloud=static_cloud(bag,(min(received[use]),max(received[use])),mid-2,mid+2,calib)
                R,t=unpack(seed)
                pose=held[len(held)//2]
                pos=R@(capture.Rwc@pose[1:4]+capture.twc)+t
                ori=R@capture.Rwc@quat_R(pose[4:])
                hint=[*pos,np.rad2deg(yaw(ori))]
                pos,ori=initial_guess(bag,recorded,(mid-2,mid+2),None,None,hint)
                ref=register(cloud,folder,pos,ori)
                obj['reference']=ref
                if not ref['valid']:
                    raise ValueError(', '.join(ref['reasons']))
                obj['valid']=True
                for label,window in [('first',(lo,lo+1)),('last',(hi-1,hi))]:
                    values={}
                    for prefix in ('base','dll'):
                        def xyz(r):
                            if not r['valid' if prefix=='dll' else 'base_valid']:
                                return [np.nan]*3
                            return [r.get(prefix+'_'+axis,np.nan) for axis in 'xyz']
                        data=(np.array([r['stamp_ns']/NS for r in interval]),np.array([xyz(r) for r in interval]))
                        pt,counts=window_pose(data,window,laser)
                        values[prefix]=dict(coverage=counts,error=difference(pt,np.array(ref['pose']['translation'])))
                    obj[label]=values
            except (ValueError,OSError,KeyError) as exc:
                obj['reason']=str(exc)
    finally:
        capture.close()
    return dict(map=str(folder),pcd_sha256=pcd_hash,raw_coverage=coverage,stops=result)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bag',type=Path)
    ap.add_argument('output',type=Path,help='new output directory')
    ap.add_argument('--icp-map',type=Path,help='same-source outdoor crop; defaults to the recording manifest map')
    args=ap.parse_args()
    bagpath=args.bag.resolve()
    if args.output.exists():
        ap.error('output already exists')
    with rosbag.Bag(str(bagpath)) as bag:
        rows,seeds=read_rows(bag)
        if not rows:
            ap.error('bag has no usable /fastlio/odom_world frames')
        report=summary(rows)
        report.update(bag=str(bagpath),icp_checkpoints=dict(valid=False,reason="缺少同源裁图清单，可用 --icp-map 指定"))
        crop=args.icp_map
        manifest=bagpath.parent/'dll_map_manifest.json'
        try:
            if crop is None and manifest.is_file():
                crop=Path(json.loads(manifest.read_text())['map_dir'])
            if crop:
                report['icp_checkpoints']=checkpoints(bag,crop,rows,seeds)
        except (ValueError, KeyError, OSError) as exc:
            report['icp_checkpoints']=dict(valid=False,reason=str(exc))
    t0=rows[0]['stamp_ns']
    for row in rows:
        row['elapsed_s']=(row['stamp_ns']-t0)/NS
    for gap in report['unavailable_intervals']:
        gap['first_s']=(gap['first_stamp_ns']-t0)/NS
        gap['last_s']=(gap['last_stamp_ns']-t0)/NS
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(dir=str(args.output.parent)) as tmp:
        stage=Path(tmp)/'report'
        stage.mkdir()
        with (stage/'drift.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=['segment','stamp_ns','elapsed_s','base_valid','valid','state','reason','age_s',*POSITION,*METRICS])
            writer.writeheader()
            writer.writerows(rows)
        (stage/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
        stage.rename(args.output)
    print(f"{report['valid_frames']}/{report['frames']} valid frames: {args.output}")

if __name__=='__main__':
    main()
