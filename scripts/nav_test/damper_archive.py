"""Join a CH6 segment to route events using source timestamps. No ROS dependency."""
import bisect
import csv
import json
from pathlib import Path
import re
import shutil

NS = 1_000_000_000
EXPECTED = ((5,), (1, 2), (0, 1, 4), (3,))


def rows(path):
    if not path.is_file():
        return []
    with path.open(newline='') as f:
        return list(csv.DictReader(f))


def window_gaps(stamps, seq, start, end):
    i, j = bisect.bisect_left(stamps, start), bisect.bisect_left(stamps, end)
    if i == j:
        return [[start,end]]
    gaps = []
    if stamps[i]-start > 100_000_000:
        gaps.append([start,stamps[i]])
    for k in range(i+1,j):
        if stamps[k]-stamps[k-1]>100_000_000 or seq[k]!=seq[k-1]+1:
            gaps.append([stamps[k-1],stamps[k]])
    if end-stamps[j-1]>100_000_000:
        gaps.append([stamps[j-1],end])
    return gaps


def read_takeovers(bag):
    import rosbag
    edges = []
    flying = False
    with rosbag.Bag(str(bag)) as reader:
        for _,m,t in reader.read_messages(topics=['/mavros/state']):
            if not m.connected:
                continue
            if flying and m.mode != 'OFFBOARD':
                edges.append((m.header.stamp.to_nsec() or t.to_nsec()))
            flying = m.mode == 'OFFBOARD' and m.armed
    return edges


def segment_bounds(bag):
    events = rows(bag.with_name(bag.stem+'_events.csv'))
    opened = next((int(e['stamp_ns']) for e in events if e['event']=='open'),None)
    closed = next((int(e['stamp_ns']) for e in events if e['event']=='close'),None)
    if opened is not None and closed is not None:
        return opened, closed
    frames=rows(bag.with_name(bag.stem+'_frames.csv'))
    if frames:
        return int(frames[0]['source_stamp_ns']),int(frames[-1]['source_stamp_ns'])
    return None


def owner(stamp, segments):
    # If CH6 was off, retain the unmatched still once, with an explicit null frame.
    return min(segments,key=lambda b: max(segments[b][0]-stamp, stamp-segments[b][1], 0),default=None)


def supplement(bag, describe, takeovers):
    folder = bag.parent
    table = rows(bag.with_name(bag.stem+'_frames.csv'))
    stamps = [int(f['source_stamp_ns']) for f in table]
    seq = [int(f['source_seq']) for f in table]
    segments = {}
    for sibling in sorted(folder.glob('test_indoor3_5_*.bag')):
        bounds = segment_bounds(sibling)
        if bounds:
            segments[sibling] = bounds
    paths = set()
    label = folder/'damper_deg.txt'
    hardness = label.read_text().strip() if label.is_file() else '未标注'
    if label.is_file():
        paths.add(label)
    validation = folder/'validation_kind.txt'
    kind = validation.read_text().strip() if validation.is_file() else '实机任务'
    if validation.exists():
        paths.add(validation)
    result = dict(hardness=hardness, kind=kind, routes=[], takeover_ns=takeovers)
    for goal in sorted(folder.glob('goal_test_indoor3_5_*.json')):
        route = json.loads(goal.read_text())
        paths.add(goal)
        if not validation.exists() and route.get("real_output") is False:
            result["kind"] = "预览，无飞控输出"
        points=[]
        for leg in route['legs']:
            entry = leg.get('t0_ns')
            point = dict(index=leg['index'], expected=EXPECTED[leg['index']-1],
                         t0_ns=entry, sent_ns=round(leg['sent_ros']*NS), confirmed_ns=leg.get('confirmed_ns'),
                         score_end_ns=leg.get('score_end_ns'),depart_ns=leg.get('depart_ns'),photos=[])
            if entry is not None:
                point['brake_gaps'] = window_gaps(stamps,seq,entry,entry+1_500_000_000)
                point['steady_gaps'] = window_gaps(stamps,seq,entry+1_500_000_000,entry+5*NS)
                if bag in segments:
                    first,last=segments[bag]
                    for key,a,b in (('brake_gaps',entry,entry+1_500_000_000),('steady_gaps',entry+1_500_000_000,entry+5*NS)):
                        if first>a:
                            point[key].append([a,min(first,b)])
                        if last<b:
                            point[key].append([max(last,a),b])
            for photo in leg.get('photos',[]):
                item = dict(photo,frame_index=None,delta_ms=None)
                if item.get('stamp_ns') and segments:
                    primary = owner(item['stamp_ns'], segments)
                    item['segment'] = primary.stem
                    if primary != bag:
                        item['status'] = 'other_segment'
                        point['photos'].append(item)
                        continue
                name = item.get('file')
                if name and Path(name).name == name and (folder/name).is_file():
                    # Preserve even unrecorded stills; null index explicitly means no matching video frame.
                    paths.add(folder/name)
                    stamp = item['stamp_ns']
                    k = bisect.bisect_left(stamps,stamp)
                    near = [j for j in (k-1,k) if 0<=j<len(stamps)]
                    if near and bag in segments and segments[bag][0]<=stamp<=segments[bag][1]:
                        k = min(near,key=lambda j:abs(stamps[j]-stamp))
                        if abs(stamps[k]-stamp) <= 40_000_000:
                            item.update(frame_index=int(table[k]['frame_index']),delta_ms=(stamps[k]-stamp)/1e6)
                point['photos'].append(item)
            points.append(point)
        final = points[-1].get('score_end_ns') if len(points)==4 else None
        result['routes'].append(dict(goal=goal.name,end=route['end'],points=points,
                                     final_takeover_ns=next((t for t in takeovers if final and t>=final),None)))
    # Copy external diagnostics under the run, and freeze the shell log before any final trailer.
    snapshots = folder/'archive_diagnostics'
    log = folder/'fusion.log'
    sources = [folder/'nav_test.log']
    if log.exists():
        logroot = Path(__file__).resolve().parents[2]/'logs/fusion'
        for name in set(re.findall(r'(?:fusion_\d{8}_\d{6}\.log|core_watch_\d{8}_\d{6}\.csv)',log.read_text(errors='replace'))):
            sources.append(logroot/name)
    for source in sources:
        if source.is_file():
            snapshots.mkdir(exist_ok=True)
            copy = snapshots/source.name
            if not copy.exists():
                tmp = copy.with_suffix(copy.suffix+'.tmp')
                shutil.copyfile(source,tmp)
                tmp.replace(copy)
            paths.add(copy)
    for path in (folder/'timing').glob('**/*'):
        if path.is_file():
            paths.add(path)
    files=[]
    for path in sorted(paths):
        item=describe(path)
        if folder/'timing' in path.parents:
            item['name']=path.relative_to(folder).as_posix()
        files.append(item)
    return result,files
